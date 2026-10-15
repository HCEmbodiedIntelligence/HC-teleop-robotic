"""One durable job registry for HTTP, VR, local recording and raw capture."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import copy
import json
from pathlib import Path
import shutil
import threading
import time
import uuid

import yaml
from .artifact_state import atomic_json, fingerprint, publish_ready
from .topic_recorder import TopicRecorder
from .ros_recording_executor import RosRecordingExecutor


class RecordingService:
    def __init__(self, config, config_dir, ros_config=None, health=None):
        self.config = copy.deepcopy(config)
        self.config_dir = Path(config_dir)
        self.ros_config = copy.deepcopy(ros_config or {'enabled': False})
        self.health = health or (lambda: {})
        self.directory = self.resolve(config.get('directory', '../runtime/topic_recordings'))
        self.state_dir = self.resolve(config.get('job_state_dir', '../runtime/recording_jobs'))
        self.artifact_state_dir = self.resolve(config.get('artifact_state_dir', '../runtime/artifact_ready'))
        self.directory.mkdir(parents=True, exist_ok=True)
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.artifact_state_dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='recording-finalize')
        self.drainers = ThreadPoolExecutor(max_workers=config.get('max_pending_jobs', 8), thread_name_prefix='recording-drain')
        self.monitor = TopicRecorder({**config, 'format': 'mcap'}, self.config_dir)
        self.last = self.monitor
        self.active = None
        self.jobs = {}
        self.readers = {}
        self.file_locks = {}
        self.quality = {}
        self.raw = None
        self.closed = False
        self.binding_snapshot = None
        self.upload_service = None
        for path in self.state_dir.glob('*.json'):
            job = json.loads(path.read_text())
            self.jobs[job['recording_id']] = job
            if job['capture_state'] in {'starting', 'capturing', 'draining'}:
                job.update(capture_state='interrupted', artifact_state='interrupted', error='process stopped before source completion')
                self.save(job)
            elif job['artifact_state'] in {'source_complete', 'exporting', 'queued'} and job['workflow'] == 'hc':
                self.worker.submit(self.finish_hc, job)
            elif job['artifact_state'] in {'sealing', 'seal_queued'}:
                self.worker.submit(self.seal_raw, job)

    def resolve(self, value):
        path = Path(value).expanduser()
        return (path if path.is_absolute() else self.config_dir/path).resolve()

    def save(self, job):
        atomic_json(self.state_dir/(job['recording_id']+'.json'), job)

    @property
    def quality_rules(self):
        return self.config.get("gap_rules", [])

    def ipc_resources(self):
        # The runtime's idle subscription process supplies preflight health only.
        return self.monitor.ipc_resources()

    @property
    def path(self):
        return self.last.path

    def is_recording(self):
        with self.lock:
            return self.active is not None

    def record(self, event):
        with self.lock:
            return self.last.record(event) if self.active and self.active['workflow'] == 'hc' else False

    def precheck(self):
        if self.config.get('workflow', 'hc') == 'platform_raw':
            hub = self.raw_hub()
            result = hub.prepare_adapters() if not hub.ros_bridge else __import__(
                'middleware.core.capture.adapters', fromlist=['preflight']).preflight(hub.cfg, hub.reports, hub.clocks)
            return result
        from .ros_recording_executor import recording_subscriptions
        sources = recording_subscriptions(self.ros_config)
        health = self.health()
        issues = []
        for item in sources:
            mode = item.get('recording_mode', 'event' if item['topic'].startswith(('/action/', '/control/', '/status/')) else 'continuous')
            if item.get('required', True) and mode == 'continuous' and health.get(item['topic'], {}).get('state') != 'ok':
                issues.append({'topic': item['topic'], 'state': health.get(item['topic'], {}).get('state', 'no_data'),
                               'hz': health.get(item['topic'], {}).get('hz', 0), 'min_hz': item.get('min_hz', 1),
                               'reason': 'required continuous source is not fresh or below minimum rate'})
        return dict(ok=True, topics=[dict(item, **{k:v for k,v in health.get(item['topic'], {}).items() if k not in item}) for item in sources], ready=bool(sources) and not issues, issues=issues, topic_count=len(sources),
                    reason='' if sources and not issues else '录制源未就绪')

    def raw_hub(self):
        if self.raw is None:
            from .capture.config import validate
            from .capture.hub import CaptureHub
            path = self.resolve(self.config['capture_config_file'])
            cfg = validate(yaml.safe_load(path.read_text()))
            # Storage paths belong to the capture configuration file, not the job registry.
            for key in ('directory', 'export_directory'):
                if key in cfg['storage']:
                    value = Path(cfg['storage'][key]).expanduser()
                    cfg['storage'][key] = str((path.parent/value).resolve() if not value.is_absolute() else value)
            root = self.state_dir/'raw_engine'
            atomic_json(root/'capture-config.json', cfg)
            atomic_json(root/'capture-workflow.json', {'vr_target': 'raw', 'package_after_stop': False})
            self.raw = CaptureHub(root, self.ros_config)
        return self.raw

    def start(self, filename='', metadata=None):
        with self.lock:
            if self.closed:
                raise RuntimeError('recording service is shutting down')
            if self.active:
                return self.active['path']
            pending = sum(j['artifact_state'] in {'draining', 'source_complete', 'queued', 'exporting', 'seal_queued', 'sealing'} for j in self.jobs.values())
            if pending >= self.config.get('max_pending_jobs', 8):
                raise RuntimeError('后台处理积压达到上限')
            if shutil.disk_usage(self.directory).free < self.config.get('min_free_bytes', 268435456):
                raise RuntimeError('磁盘可用空间不足')
            check = self.precheck()
            if not check.get('ready', check.get('ok', False)):
                raise ValueError(str(check))
            ident = uuid.uuid4().hex
            workflow = self.config.get('workflow', 'hc')
            snapshot = copy.deepcopy(self.config)
            job = dict(recording_id=ident, workflow=workflow, profile_id=(metadata or {}).get('profile_id', ''),
                       config_fingerprint=fingerprint({'recording': snapshot, 'subscriptions': self.ros_config.get('subscriptions', [])}),
                       config=snapshot, metadata=copy.deepcopy(metadata or {}), capture_state='starting',
                       artifact_state='pending', artifacts=[], native_session_id=None, path='', error=None,
                       capture_started_at=datetime.now(timezone.utc).isoformat(),
                       mcap_artifact_id=uuid.uuid4().hex, lerobot_artifact_id=uuid.uuid4().hex)
            if self.binding_snapshot:
                try:
                    job['binding'] = self.binding_snapshot()
                except Exception:
                    pass  # No configured binding must not prevent local capture.
            self.jobs[ident] = job
            self.save(job)
            try:
                if workflow == 'platform_raw':
                    hub = self.raw_hub()
                    hub.start({**(metadata or {}), 'hc_recording_id': ident})
                    session = hub.session
                    job.update(path=str(session.path), native_session_id=session.id,
                               config_fingerprint=fingerprint(session.cfg), config_snapshot=session.cfg)
                    self.readers[ident] = session
                else:
                    if self.config.get('format', 'mcap') != 'mcap':
                        from .lerobot_export import ensure_export_available
                        ensure_export_available()
                    recorder = TopicRecorder({**self.config, 'format': 'mcap'}, self.config_dir)
                    basename = Path(filename).name if filename else 'recording.mcap'
                    if basename != filename and filename:
                        raise ValueError('filename must be a basename')
                    path = recorder.start(ident+'_'+basename, {**(metadata or {}), 'recording_id': ident})
                    executor = RosRecordingExecutor(self.ros_config, recorder)
                    executor.start()
                    self.last = recorder
                    self.readers[ident] = (recorder, executor)
                    from types import SimpleNamespace
                    from .data_quality import DataQuality
                    self.quality[ident] = DataQuality(SimpleNamespace(recorder=recorder, config={'data_quality': {'scope': 'segment'}}, recording_ros=executor, emit=lambda event: None))
                    job['path'] = path
                job.update(capture_state='capturing')
                self.save(job)
                self.active = job
                if workflow == 'platform_raw':
                    threading.Thread(target=self.watch_raw, args=(job, session), daemon=True, name='raw-completion').start()
                return job['path']
            except Exception as exc:
                reader = self.readers.pop(ident, None)
                if workflow == 'hc' and reader:
                    reader[1].stop()
                    reader[0].stop()
                job.update(capture_state='failed', artifact_state='failed', error=str(exc))
                self.save(job)
                raise

    def watch_raw(self, job, session):
        # Fixed-duration and source failures may end capture without a UI stop.
        session.closed.wait()
        with self.lock:
            if self.active is job and not self.closed:
                self.stop()

    def stop(self):
        with self.lock:
            job = self.active
            if not job:
                return self.status()
            if job['workflow'] == 'hc':
                quality = self.quality.get(job['recording_id'])
                if quality and quality.active_segment:
                    quality.mark('segment', source='stop')
                self.readers[job['recording_id']][0]._accepting_event.clear()
            else:
                self.raw.request_stop()
            job.update(capture_state='draining', artifact_state='draining', capture_ended_at=datetime.now(timezone.utc).isoformat())
            self.save(job)
            self.active = None
            self.drainers.submit(self.drain, job)
            return self.status()

    def drain(self, job):
        try:
            reader = self.readers[job['recording_id']]
            if job['workflow'] == 'hc':
                recorder, executor = reader
                source_status = executor.status()
                executor.stop()
                result = recorder.stop()
                if source_status.get('error'):
                    raise RuntimeError(source_status['error'])
                if not result['messages']:
                    raise RuntimeError('empty recording; source contains no messages')
                if result.get('error'):
                    raise RuntimeError(result['error'])
                job.update(capture_state='source_complete', artifact_state='source_complete')
                self.save(job)
                self.worker.submit(self.finish_hc, job)
            else:
                reader.stop()
                manifest = json.loads((Path(job['path'])/'manifest.json').read_text())
                if manifest['state'] != 'completed':
                    raise RuntimeError('raw source failed: '+str(manifest.get('errors')))
                job.update(capture_state='source_complete', artifact_state='awaiting_binding')
                self.save(job)
                if job.get('binding'):
                    self.request_raw_seal(job['recording_id'], job['binding'])
        except Exception as exc:
            job.update(capture_state='failed', artifact_state='failed', error=str(exc))
            self.save(job)
        finally:
            self.readers.pop(job['recording_id'], None)
            self.quality.pop(job['recording_id'], None)

    def finish_hc(self, job):
        try:
            path = Path(job['path'])
            fmt = job['config'].get('format', 'mcap')
            # Retained MCAP is independently complete, including for LeRobot-only jobs.
            publish_ready(self.artifact_state_dir, job, path.parent, [path.name], 'hc_mcap', job['mcap_artifact_id'])
            job['artifacts'] = [job['mcap_artifact_id']]
            if fmt != 'mcap':
                job['artifact_state'] = 'exporting'
                self.save(job)
                recorder = TopicRecorder(job['config'], self.config_dir)
                recorder.path = path
                recorder._active_metadata = job['metadata']
                completed_root = self.directory/'lerobot'/path.stem
                completion = completed_root/'.hc-export-complete.json'
                if completion.exists() and json.loads(completion.read_text()).get('source') == str(path.resolve()):
                    recorder._lerobot_path = str(completed_root)
                    recorder._export_state = 'completed'
                else:
                    recorder._start_lerobot_export()
                while recorder.status()['export_state'] == 'exporting':
                    time.sleep(.1)
                result = recorder.status()
                if result['export_state'] != 'completed':
                    raise RuntimeError(result['export_error'])
                root = Path(result['lerobot_path'])
                publish_ready(self.artifact_state_dir, job, root,
                              [p.relative_to(root).as_posix() for p in root.rglob('*') if p.is_file()],
                              'hc_lerobot', job['lerobot_artifact_id'], source_artifact_id=job['mcap_artifact_id'])
                job['artifacts'].append(job['lerobot_artifact_id'])
            job.update(artifact_state='ready', error=None)
        except Exception as exc:
            job.update(artifact_state='failed', error=str(exc))
        self.save(job)

    def request_raw_seal(self, recording_id, binding_snapshot):
        with self.lock:
            job = self.jobs[recording_id]
            keys = {'binding_id', 'platform_base_url', 'robot_id', 'collection_task_id'}
            if set(binding_snapshot) != keys or any(not isinstance(binding_snapshot[k], str) or not binding_snapshot[k].strip() for k in keys):
                raise ValueError('binding requires only binding_id/platform_base_url/robot_id/collection_task_id; no credentials')
            if job.get('seal_job_id'):
                if job['binding'] != binding_snapshot:
                    raise ValueError('sealed binding is immutable; create a new derived job')
                if job['artifact_state'] != 'failed':
                    return dict(job_id=job['seal_job_id'], state=job['artifact_state'])
            if job['workflow'] != 'platform_raw' or job['capture_state'] != 'source_complete':
                raise ValueError('raw source is not complete')
            job.update(binding=copy.deepcopy(binding_snapshot), seal_job_id=job.get('seal_job_id', uuid.uuid4().hex),
                       raw_artifact_id=job.get('raw_artifact_id', uuid.uuid4().hex), artifact_state='seal_queued')
            self.save(job)
            self.worker.submit(self.seal_raw, job)
            return dict(job_id=job['seal_job_id'], state='seal_queued')

    def seal_raw(self, job):
        try:
            import subprocess
            import sys
            import tempfile
            source = Path(job['path'])
            binding = job['binding']
            # Native identity requires local_robot_id and is preserved unchanged.
            native = json.loads((source/'session.json').read_text())
            local_robot_id = native.get('metadata', {}).get('capture_context', {}).get('local_robot_id') or job['profile_id']
            atomic_json(source/'capture-binding.json', {**binding, 'local_robot_id': local_robot_id})
            job['artifact_state'] = 'sealing'
            self.save(job)
            output = self.directory/'sealed'/job['raw_artifact_id']
            output.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryFile(mode='w+') as errors:
                process = subprocess.Popen([sys.executable, '-m', 'middleware.core.capture.job_process',
                    'recording', str(source), str(output), 'online'],
                    cwd=Path(__file__).resolve().parents[2], stdout=subprocess.PIPE, stderr=errors, text=True)
                result = None
                for line in process.stdout:
                    event = json.loads(line)
                    if event['kind'] == 'result':
                        result = event['value']
                    elif event['kind'] == 'error':
                        job['error'] = event['value']
                if process.wait() or result is None:
                    raise RuntimeError(job.get('error') or 'raw sealing process failed')
            marker = json.loads((output/'recording-complete.json').read_text())
            from .capture.completion import inventory, canonical
            import hashlib
            assets = marker['recording_upload']['assets']
            if hashlib.sha256(canonical(assets)).hexdigest() != marker['content_sha256']:
                raise ValueError('raw marker integrity mismatch')
            actual = {x['path']: x for x in inventory(output) if x['path'] != 'recording-complete.json'}
            if set(actual) != {x['path'] for x in assets} or any(actual[x['path']]['sha256'] != x['sha256'] or actual[x['path']]['size_bytes'] != x['size'] for x in assets):
                raise ValueError('raw sealed assets changed')
            publish_ready(self.artifact_state_dir, job, output,
                          [p.relative_to(output).as_posix() for p in output.rglob('*') if p.is_file()],
                          'platform_raw', job['raw_artifact_id'], binding_id=binding['binding_id'])
            job.update(artifact_state='ready', artifacts=[job['raw_artifact_id']], error=None)
        except Exception as exc:
            job.update(artifact_state='failed', error=str(exc))
        self.save(job)

    def status(self):
        with self.lock:
            if self.active and self.active['workflow'] == 'hc':
                self.quality[self.active['recording_id']].check()
            return {**self.last.status(), 'recording': self.active is not None,
                    'workflow': self.active['workflow'] if self.active else self.config.get('workflow', 'hc'),
                    'path': self.active['path'] if self.active else str(self.last.path or ''),
                    'raw_status': self.raw.session.status() if self.raw and self.raw.session else None,
                    'recording_id': self.active['recording_id'] if self.active else None,
                    'jobs': copy.deepcopy(list(self.jobs.values()))}

    def mark_latest(self, *args, **kwargs):
        return self.last.mark_latest(*args, **kwargs)

    def __getattr__(self, name):
        # Existing listing/download and marker APIs keep their HC compatibility.
        return getattr(self.last, name)

    def shutdown(self):
        self.stop()
        self.closed = True
        self.drainers.shutdown(wait=True)
        self.worker.shutdown(wait=True)
        if self.raw:
            self.raw.shutdown()

    def assert_file_idle(self, path):
        path = Path(path).resolve()
        with self.lock:
            for job in self.jobs.values():
                source = Path(job['path']).resolve()
                derived = self.directory/'lerobot'/source.stem
                if (source == path or derived == path) and job['artifact_state'] in {'pending', 'draining', 'source_complete', 'exporting', 'queued', 'sealing', 'seal_queued'}:
                    raise ValueError('文件仍由录制或后台任务占用')

    def file_operation(self, filename):
        from contextlib import contextmanager
        @contextmanager
        def hold():
            if Path(filename).name != filename:
                raise ValueError('invalid filename')
            # This process lock coordinates local readers/delete. Artifact uploads additionally
            # require the shared C lock protocol before ready files can be deleted.
            from .recording_datasets import resolve_dataset
            target = resolve_dataset(self.directory, filename)
            with self.lock:
                file_lock = self.file_locks.setdefault(str(target.resolve()), threading.RLock())
            with file_lock:
                self.assert_file_idle(self.directory/filename)
                yield
        return hold()

    def delete_recording(self, filename):
        with self.file_operation(filename):
            from .recording_datasets import resolve_dataset
            path = resolve_dataset(self.directory, filename).resolve()
            self.assert_file_idle(path)
            descriptors = []
            for description in self.artifact_state_dir.glob('*.json'):
                item = json.loads(description.read_text())
                if any((Path(item['root'])/f['path']).resolve() == path or (Path(item['root'])/f['path']).resolve().is_relative_to(path) for f in item['files']):
                    descriptors.append(item)
            from contextlib import ExitStack
            with ExitStack() as stack:
                if descriptors:
                    try:
                        from humanoid_upload.artifacts import ArtifactLocks
                    except ImportError as exc:
                        raise ValueError('删除封存产物需要安装共享文件锁依赖 upload_client') from exc
                    locks = ArtifactLocks(self.artifact_state_dir)
                    for item in sorted(descriptors, key=lambda d: d['artifact_id']):
                        if self.upload_service and self.upload_service.enabled:
                            self.upload_service.cancel_artifact(item['artifact_id'])
                        stack.enter_context(locks.delete(item['artifact_id']))
                result = self.last.delete_recording(filename)
                for item in descriptors:
                    job = self.jobs.get(item['recording_id'])
                    if job:
                        job['artifact_state'] = 'deleted'
                        self.save(job)
                return result

    def register_derived(self, source, output, operation):
        source_job = next((j for j in self.jobs.values() if Path(j['path']).name == source), None)
        ident, artifact = uuid.uuid4().hex, uuid.uuid4().hex
        job = dict(recording_id=ident, workflow='hc', profile_id=source_job['profile_id'] if source_job else 'unknown',
                   config_fingerprint=source_job['config_fingerprint'] if source_job else fingerprint({'imported_source': source}),
                   capture_state='source_complete', artifact_state='ready', artifacts=[artifact], path=str(self.directory/output),
                   derivation=operation, source=source, recovery_completeness='unknown' if operation == 'recover' else None)
        publish_ready(self.artifact_state_dir, job, self.directory, [output], 'hc_mcap', artifact,
                      source_artifact_id=source_job.get('mcap_artifact_id') if source_job else None)
        self.jobs[ident] = job
        self.save(job)

    def mark_current_or_latest(self, source='manual'):
        with self.lock:
            if self.active and self.active['workflow'] == 'platform_raw':
                event = self.readers[self.active['recording_id']].mark(source=source)
                return dict(recording=True, filename=self.active['native_session_id'], marked=True, event=event)
            if self.active:
                self.quality[self.active['recording_id']].mark(source=source)
            return self.last.mark_current_or_latest(source)

    def retry(self, recording_id):
        with self.lock:
            job = self.jobs[recording_id]
            if job['capture_state'] != 'source_complete' or job['artifact_state'] != 'failed':
                raise ValueError('only failed finalization with a complete source can retry')
            job['artifact_state'] = 'queued' if job['workflow'] == 'hc' else 'seal_queued'
            self.save(job)
            self.worker.submit(self.finish_hc if job['workflow'] == 'hc' else self.seal_raw, job)
            return {'job_id': recording_id, 'state': job['artifact_state']}

    def list_recordings(self):
        result = self.last.list_recordings()
        for item in result:
            job = next((j for j in self.jobs.values() if Path(j['path']).name == item.get('filename')), None)
            if job:
                item.update(recording_id=job['recording_id'], capture_state=job['capture_state'],
                            artifact_state=job['artifact_state'], artifacts=job['artifacts'])
                item['is_current'] = job['capture_state'] in {'starting', 'capturing', 'draining'}
        return result

    def delete_artifact(self, identity):
        import re
        if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', identity):
            raise ValueError('invalid artifact identity')
        description = self.artifact_state_dir/(identity+'.json')
        if description.is_symlink():
            raise ValueError('artifact descriptor is a symlink')
        item = json.loads(description.read_text())
        root = Path(item['root'])
        if item['artifact_id'] != identity or not root.is_absolute() or not root.resolve().is_relative_to(self.directory):
            raise ValueError('artifact root is outside the data directory')
        paths = []
        for entry in item['files']:
            relative = Path(entry['path'])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('invalid artifact member')
            path = root/relative
            if any(p.is_symlink() for p in (path, *path.parents)):
                raise ValueError('artifact symlink is forbidden')
            self.assert_file_idle(path)
            paths.append(path)
        from humanoid_upload.artifacts import ArtifactLocks
        if self.upload_service and self.upload_service.enabled:
            self.upload_service.cancel_artifact(identity)
        from contextlib import ExitStack
        with ExitStack() as stack:
            with self.lock:
                local_locks = [self.file_locks.setdefault(str(path.resolve()), threading.RLock()) for path in sorted(paths)]
            for local_lock in local_locks:
                stack.enter_context(local_lock)
            stack.enter_context(ArtifactLocks(self.artifact_state_dir).delete(identity))
            for path in paths:
                path.unlink(missing_ok=True)
            # Leave unrelated data and the permanent tombstone untouched.
            for directory in sorted({p.parent for p in paths}, key=lambda p: len(p.parts), reverse=True):
                if directory != self.directory:
                    try:
                        directory.rmdir()
                    except OSError:
                        pass
            job = self.jobs.get(item['recording_id'])
            if job:
                job['artifact_state'] = 'deleted'
                self.save(job)
        return {'artifact_id': identity, 'deleted': True}
