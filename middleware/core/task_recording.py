"""Serialize all capture starts and durably freeze each recording's upload target."""
from __future__ import annotations

import copy
import json
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from humanoid_upload.common import UploadError, atomic_json, read_json

from .capture_formats import SOURCE_FORMATS, artifact_formats, artifact_identity, upload_formats, job_artifact_formats
from .background_process import BackgroundProcess, BackgroundProcessError, BackgroundProcessStopped


def now():
    return datetime.now(timezone.utc).isoformat()


from .correction_support import CorrectionSupport


class TaskRecorder(CorrectionSupport):
    def __init__(self, recorder, platform):
        self.recorder, self.platform = recorder, platform
        if platform.upload.artifact_process.closed:
            platform.upload.artifact_process = BackgroundProcess('hc-artifact-worker')
        self.recorder.task_managed = True
        self.recorder.upload_service = platform.upload
        self.default_lerobot_task = recorder.lerobot_task
        self.prepare_recording = None
        self.capture_health = getattr(platform, 'capture_health', None)
        self.directory = recorder.directory
        self.state_dir = platform.directory / 'task-recordings'
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.finalizer = ThreadPoolExecutor(max_workers=1, thread_name_prefix='recording-seal')
        self.jobs = {}
        self.active = None
        self.pending = set()
        self.closing = threading.Event()
        for path in sorted(self.state_dir.glob('*.json')):
            job = read_json(path)
            self.jobs[job['recording_id']] = job
            if job['state'] in ('starting', 'armed', 'recording', 'stopping'):
                job.update(state='interrupted', error='录制中断，原始数据已保留；请检查完整性后处理')
                self.save(job)
            elif job['state'] in ('finalizing', 'upload_pending'):
                self.submit(job)
            elif job.get('format') == 'both' and not job.get('artifacts') and job['state'] in ('ready', 'queued'):
                job.update(state='finalizing')
                self.save(job)
                self.submit(job)

    def __getattr__(self, name):
        return getattr(self.recorder, name)

    def save(self, job):
        atomic_json(self.state_dir / (job['recording_id'] + '.json'), job)

    def set_selection(self, selection):
        with self.lock:
            if self.active or self.recorder.is_recording() or self.corrections:
                raise UploadError('RECORDING_ACTIVE', '请停止当前录制后修改任务')
            return self.platform.set_recording_selection(selection)

    def set_upload_format(self, value):
        with self.lock:
            if self.active or self.recorder.is_recording() or self.corrections:
                raise UploadError('RECORDING_ACTIVE', '请停止当前录制后修改上传格式')
            return self.platform.set_upload_format(value)

    @staticmethod
    def upload_format(job):
        return job.get('upload_format') or ('mcap' if job['format'] == 'both' else job['format'])

    def prepare_and_check(self, snapshots, writer):
        """Wake recorder-owned ROS inputs before validating their live data."""
        warmup = self.prepare_recording is not None
        if warmup:
            self.prepare_recording()
        deadline = time.monotonic() + (2.0 if warmup else 0.0)
        from .capture_requirements import check_device
        while True:
            try:
                health = self.capture_health() if self.capture_health else {}
                for snapshot in snapshots:
                    check_device(snapshot, self.platform.config, writer, health)
                return
            except UploadError as error:
                if error.code != 'CAPTURE_REQUIRED_DATA_MISSING' or time.monotonic() >= deadline:
                    raise
                time.sleep(.05)

    def start(self, filename='', metadata=None, selection=None, *, _prepared_writer=None):
        with self.lock:
            if self.corrections and _prepared_writer is None:
                raise ValueError('correction capture owns recording; exit the intervention session first')
            writer = _prepared_writer or self.recorder
            selection = copy.deepcopy(selection if selection is not None else self.platform.recording_selection())
            if not isinstance(selection, dict) or selection.get('mode') not in ('local', 'platform'):
                raise UploadError('RECORDING_SELECTION_REQUIRED', '请选择平台任务或“暂不关联平台任务”')
            request_id = selection.get('request_id') or uuid4().hex
            intent = {k: v for k, v in selection.items() if k != 'request_id'}
            pending_states = ('preparing', 'prepared', 'prepare_failed')
            job = next((j for j in self.jobs.values() if j.get('request_id') == request_id), None)
            if job:
                if job['intent'] != intent or job['requested_filename'] != filename:
                    raise UploadError('RECORDING_REQUEST_CONFLICT', '重复启动请求的任务或文件名发生变化')
                if job['state'] in ('failed', 'interrupted', 'deleted'):
                    raise UploadError('RECORDING_INTERRUPTED', job.get('error') or '原录制需要处理')
                if job['state'] not in pending_states:
                    return job['path']
            else:
                # Browser refresh/controller restart may lose its request UUID.
                # An unfinished identical start still owns the original recording ID.
                job = next((j for j in self.jobs.values() if j['state'] in pending_states
                            and j['intent'] == intent and j['requested_filename'] == filename), None)
            if self.active or self.recorder.is_recording():
                raise UploadError('RECORDING_ACTIVE', '已有录制正在进行，请先停止')
            upload_format = self.upload_format(job) if job is not None else self.platform.state['upload_format']
            if upload_format in ('lerobot', 'both') and writer.save_format not in ('lerobot', 'both'):
                raise UploadError('UPLOAD_FORMAT_UNAVAILABLE', '上传 LeRobot 需要在话题录制页将本地保存方式设为 LeRobot 或 MCAP + LeRobot')
            if writer.save_format == 'lerobot' and upload_format != 'lerobot':
                raise UploadError('UPLOAD_FORMAT_UNAVAILABLE', '本地保存格式为 LeRobot 时，上传格式只能选择 LeRobot')
            if job is None:
                identity = uuid4().hex
                chosen = filename or ('ros2_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + identity[:8])
                chosen = Path(chosen).name
                if not chosen.endswith('.mcap'): chosen += '.mcap'
                job = dict(recording_id=identity, request_id=request_id, intent=intent,
                           requested_filename=filename, chosen_filename=chosen,
                           path=str((self.directory / '.lerobot_sources' if writer.save_format == 'lerobot' else self.directory / '.bundle_sources' if writer.save_format == 'mcap_mp4' else self.directory) / chosen),
                           state='preparing', error=None, binding=None, started_at=None, ended_at=None,
                           format=writer.save_format, upload_format=upload_format, video_fps=writer.lerobot_fps, metadata=copy.deepcopy(metadata or {}),
                           local_robot=None, upload_requested=False)
                # fsync before preflight/prepare: no network request can precede identity.
                self.save(job)
                self.jobs[identity] = job
            identity = job['recording_id']
            inputs_prepared = False
            try:
                if job['format'] != writer.save_format:
                    raise UploadError('CAPTURE_FORMAT_MISMATCH', '准备重试期间录制格式已变化，请恢复原格式')
                if intent['mode'] == 'platform':
                    source_format = SOURCE_FORMATS[upload_formats(job)[0]]
                    if job['format'] not in ('mcap', 'lerobot', 'both', 'mcap_mp4'):
                        raise UploadError('CAPTURE_FORMAT_UNSUPPORTED', '该保存方式尚未接入平台任务')
                    if not job.get('preparation'):
                        checked = self.platform.preflight(intent, source_format=source_format)
                        plan = checked['task'].get('capture_plan')
                        if not plan:
                            raise UploadError('CAPTURE_PLAN_REQUIRED', '任务采集方案待配置；不会使用默认方案或本地模式')
                        revision = intent.get('expected_plan_revision', plan['revision'])
                        if revision != plan['revision']:
                            raise UploadError('CAPTURE_REVISION_CONFLICT', '任务计划已变更；请刷新并明确重新选择方案')
                        job['preparation'] = {'checked': checked, 'command': {
                            'expected_plan_revision': revision, 'recording_id': identity,
                            'source_format': source_format}}
                        job['local_robot'] = checked['local_robot']
                        self.save(job)
                    preparation = job['preparation']
                    if self.upload_format(job) == 'both' and not job.get('lerobot_preparation'):
                        checked = self.platform.preflight(intent, source_format='LEROBOT_V3')
                        first, second = preparation['checked']['task']['capture_plan'], checked['task']['capture_plan']
                        if any(first[k] != second[k] for k in ('revision', 'scheme', 'output', 'mapping_version', 'mapping_sha256')):
                            raise UploadError('CAPTURE_REVISION_CONFLICT', '两种格式必须使用同一任务方案与设备映射；请刷新任务后重试')
                        job['lerobot_preparation'] = {'checked': checked, 'command': {
                            'expected_plan_revision': preparation['command']['expected_plan_revision'],
                            'recording_id': artifact_identity(job, 'lerobot'), 'source_format': 'LEROBOT_V3'}}
                        self.save(job)
                    if self.platform.identity_provider() != preparation['checked']['local_robot']:
                        raise UploadError('LOCAL_ROBOT_MISMATCH', '准备重试期间设备配置发生变化，请恢复原配置')
                    if not job.get('capture_snapshot'):
                        job['capture_snapshot'] = self.platform.prepare(preparation['checked'], preparation['command'])
                        job['state'] = 'prepared'
                        self.save(job)
                    snapshots_to_check = [job['capture_snapshot']]
                    if self.upload_format(job) == 'both':
                        snapshots = job.setdefault('capture_snapshots', {'MCAP': job['capture_snapshot']})
                        if 'LEROBOT_V3' not in snapshots:
                            other = job['lerobot_preparation']
                            snapshots['LEROBOT_V3'] = self.platform.prepare(other['checked'], other['command'])
                            self.save(job)
                        snapshots_to_check.append(snapshots['LEROBOT_V3'])
                    self.prepare_and_check(snapshots_to_check, writer)
                    inputs_prepared = True
                    if not job['binding']:
                        job['binding'] = self.platform.bind(preparation['checked'])
                        self.save(job)
                else:
                    try:
                        job['local_robot'] = self.platform.identity_provider()
                    except UploadError:
                        job['local_robot'] = dict(local_robot_id='unconfigured', profile_id='unconfigured',
                            config_fingerprint='0'*64, serial_no=None, model_version_id=None)
            except UploadError as error:
                job.update(state='prepared' if job.get('capture_snapshot') else 'prepare_failed',
                           error=error.action, code=error.code)
                self.save(job)
                raise
            binding = job['binding']
            capture_metadata = {**job['metadata'], 'recording_id': identity,
                                'platform_task': json.dumps(binding, ensure_ascii=False) if binding else ''}
            capture_metadata.pop('capture_snapshot', None)
            capture_metadata.pop('capture_snapshots', None)
            if job.get('capture_snapshot'):
                capture_metadata['capture_snapshot'] = json.dumps(job['capture_snapshot'], ensure_ascii=False)
            if job.get('capture_snapshots'):
                capture_metadata['capture_snapshots'] = json.dumps(job['capture_snapshots'], ensure_ascii=False)
            try:
                # After this transition a crash is an interrupted capture, not a
                # permission to start again over possibly existing raw bytes.
                job.update(state='starting', started_at=now(), error=None, code=None)
                self.save(job)
                if not inputs_prepared and self.prepare_recording is not None:
                    self.prepare_recording()
                writer.lerobot_task = binding['task']['name'] if binding else self.default_lerobot_task
                job['path'] = writer.start(job['chosen_filename'], capture_metadata)
                job['state'] = 'armed' if _prepared_writer else 'recording'
                if _prepared_writer:
                    job['started_at'] = None
                else:
                    self.active = identity
                self.save(job)
                return job['path']
            except Exception:
                job.update(state='failed', error='录制启动失败；请检查采集配置和保存目录')
                self.save(job)
                raise

    def stop(self):
        with self.lock:
            if not self.active:
                return self.status()
            job = self.jobs[self.active]
            job['state'] = 'stopping'
            self.save(job)
            result = self.recorder.stop()
            job.update(ended_at=now(), state='finalizing')
            if result.get('error'):
                job.update(state='interrupted', error='录制未完整关闭，请检查原始文件')
            self.active = None
            self.save(job)
            if job['state'] == 'finalizing':
                self.submit(job)
            return self.status()

    def submit(self, job):
        if job['recording_id'] not in self.pending:
            self.pending.add(job['recording_id'])
            self.finalizer.submit(self.finish, job['recording_id'])

    def finish(self, identity):
        try:
            if self.closing.is_set():
                return
            with self.lock:
                job = copy.deepcopy(self.jobs[identity])
            source = Path(job['path'])
            if job.get('capture_snapshot'):
                from .topic_recorder import read_recording_metadata
                metadata = read_recording_metadata(source)
                if (metadata.get('recording_id') != identity or
                        json.loads(metadata.get('capture_snapshot', 'null')) != job['capture_snapshot'] or
                        json.loads(metadata.get('capture_snapshots', '{}')) != job.get('capture_snapshots', {})):
                    raise UploadError('CAPTURE_SNAPSHOT_MISMATCH', '源 MCAP 与已保存录制快照不一致，不能封存或改用默认方案')
            errors = []
            for fmt in job_artifact_formats(job):
                if fmt in job.get('deleted_formats', []):
                    continue
                try:
                    state = self.finish_artifact(job, fmt)
                    if state is None:
                        return
                    result = {'artifact_id': artifact_identity(job, fmt), 'state': state, 'error': None}
                except UploadError as error:
                    result = {'artifact_id': artifact_identity(job, fmt), 'state': 'needs_attention',
                              'error': error.action, 'code': error.code}
                    errors.append(result)
                with self.lock:
                    self.jobs[identity].setdefault('artifacts', {})[fmt] = result
                    self.save(self.jobs[identity])
            with self.lock:
                current = self.jobs[identity]
                states = [entry['state'] for entry in current['artifacts'].values()]
                current.update(state='needs_attention' if errors else 'queued' if 'queued' in states else 'ready',
                               error='；'.join(item['error'] for item in errors) if errors else None,
                               code=errors[0]['code'] if errors else None)
                self.save(current)
        except BackgroundProcessStopped:
            # Keep finalizing/upload_pending durable so restart can resume the
            # same recording and upload identity after bounded shutdown.
            pass
        except BackgroundProcessError:
            with self.lock:
                self.jobs[identity].update(state='needs_attention', code='ARTIFACT_WORKER_FAILED',
                    error='后台整理进程异常，原始数据已保留，可重试整理')
                self.save(self.jobs[identity])
        except UploadError as error:
            with self.lock:
                self.jobs[identity].update(state='needs_attention', error=error.action, code=error.code)
                self.save(self.jobs[identity])
        except Exception:
            with self.lock:
                self.jobs[identity].update(state='needs_attention', error='整理失败，数据已保留，请检查文件后重试')
                self.save(self.jobs[identity])
        finally:
            with self.lock:
                self.pending.discard(identity)

    def finish_artifact(self, job, fmt):
        if self.closing.is_set():
            return None
        if fmt == 'lerobot':
            source = Path(job['path'])
            target = Path(job.get('dataset_path') or self.directory / 'lerobot' / source.stem)
            from .export_progress import progress_path
            import time
            # The converter is the existing recorder's independent process.
            # Wait on its durable progress, not the next recording's settings.
            while not job.get('adopted_dataset'):
                if self.closing.is_set():
                    return None  # Durable finalizing state will resume on next start.
                progress_file = progress_path(target)
                progress = read_json(progress_file) if progress_file.exists() else {}
                if progress.get('state') == 'completed': break
                if progress.get('state') == 'error':
                    raise UploadError('EXPORT_FAILED', 'LeRobot 整理失败，原始 MCAP 已保留')
                if progress.get('state') == 'exporting':
                    import os
                    try: os.kill(int(progress.get('pid', 0)), 0)
                    except (OSError, ValueError):
                        raise UploadError('EXPORT_INTERRUPTED', '转换进程已停止，源数据已保留，请恢复转换后重试') from None
                if progress.get('state') != 'exporting':
                    raise UploadError('EXPORT_INTERRUPTED', '转换尚未完成，原始 MCAP 已保留；请恢复转换后重试')
                time.sleep(0.2)
        from .artifact_jobs import seal_task_artifact
        upload = self.platform.upload
        cfg = upload._config(job['binding']['binding_id']) if job.get('binding') else upload.cfg
        return upload.artifact_process.call(seal_task_artifact, job, fmt,
            str(self.directory), str(upload.state_dir), cfg)

    def upload_recording(self, identity, selection=None, source_format=None, new_upload=False):
        with self.lock:
            requested = identity
            job = self.jobs.get(identity)
            if job is None:
                job = next((j for j in self.jobs.values() if Path(j['path']).name == identity
                            or 'lerobot:' + Path(j['path']).stem == identity
                            or 'bundle:' + Path(j['path']).stem == identity
                            or any(artifact_identity(j, fmt) == identity for fmt in job_artifact_formats(j))), None)
            if job is None:
                job = self.adopt_history(identity)
            identity = job['recording_id']
            formats = upload_formats(job)
            if requested != identity:
                formats = ('lerobot',) if requested.startswith('lerobot:') or requested.endswith('-lerobot') else ('mcap_mp4',) if requested.startswith('bundle:') or requested.endswith('-mcap_mp4') else ('mcap',)
            if set(formats) & set(job.get('deleted_formats', [])):
                raise UploadError('ARTIFACT_MISSING', '该格式的本地文件已删除')
            if job['state'] in ('preparing', 'prepared', 'prepare_failed', 'starting', 'armed', 'recording', 'stopping', 'interrupted', 'failed', 'deleted'):
                raise UploadError('RECORDING_NOT_READY', '录制尚未完整关闭，原始数据已保留')
            if identity in self.pending:
                raise UploadError('ARTIFACT_BUSY', '正在整理本段数据，请稍后重试')
            if new_upload:
                format_by_source = {value: key for key, value in SOURCE_FORMATS.items()}
                if source_format not in format_by_source:
                    raise UploadError('UPLOAD_FORMAT_INVALID', '请选择有效的上传格式')
                if not selection or selection.get('mode') != 'platform':
                    raise UploadError('TASK_REQUIRED', '请选择目标任务')
                binding = self.platform.freeze(selection, remember=False, source_format=source_format)
                if binding['local_robot'].get('profile_id') != job['local_robot'].get('profile_id'):
                    raise UploadError('LOCAL_ROBOT_MISMATCH', '历史录制的机器人配置与当前配置不同')
                fmt = format_by_source[source_format]
                source_id = job.get('source_recording_id', identity)
                existing = next((j for j in self.jobs.values()
                    if j.get('source_recording_id', j['recording_id']) == source_id
                    and j.get('binding') and j['binding']['binding_id'] == binding['binding_id']
                    and self.upload_format(j) == fmt), None)
                if existing:
                    job = existing
                else:
                    source = Path(job['path'])
                    dataset = Path(job.get('dataset_path') or self.directory / 'lerobot' / source.stem)
                    if fmt == 'lerobot':
                        from .export_progress import progress_path
                        progress = read_json(progress_path(dataset)) if progress_path(dataset).exists() else {}
                        if not (dataset / 'meta/info.json').is_file() or progress.get('state') in ('exporting', 'error'):
                            raise UploadError('EXPORT_INCOMPLETE', 'LeRobot 数据集未完成，不能上传')
                    elif not source.is_file():
                        raise UploadError('MCAP_INCOMPLETE', '本地 MCAP 文件不存在')
                    job = copy.deepcopy(job)
                    job.update(recording_id=uuid4().hex, request_id=uuid4().hex,
                        source_recording_id=source_id, binding=binding, intent=copy.deepcopy(selection),
                        upload_format=fmt, upload_requested=True, state='upload_pending', error=None)
                    for field in ('artifacts', 'capture_snapshot', 'capture_snapshots', 'deleted_formats'):
                        job.pop(field, None)
                    if fmt == 'lerobot':
                        job.update(dataset_path=str(dataset), adopted_dataset=True)
                    self.save(job)
                    self.jobs[job['recording_id']] = job
                identity = job['recording_id']
                formats = (fmt,)
            if not job['binding']:
                if not selection or selection.get('mode') != 'platform':
                    raise UploadError('TASK_REQUIRED', '请为这条历史录制明确选择任务')
                checked = None
                for fmt in formats:
                    checked = self.platform.preflight(selection, remember=False, source_format=SOURCE_FORMATS[fmt])
                binding = self.platform.bind(checked, remember=False)
                # A manual association uploads the files explicitly selected here.
                binding['auto_upload'] = False
                if binding['local_robot'] != job['local_robot']:
                    raise UploadError('LOCAL_ROBOT_MISMATCH', '历史录制的机器人配置不同，不能自动改绑')
                descriptor_path = self.platform.upload.state_dir / (job['recording_id'] + '.json')
                if descriptor_path.exists():
                    descriptor = read_json(descriptor_path)
                    if descriptor.get('binding_id'):
                        raise UploadError('TASK_BINDING_MISMATCH', '已封存归属不能改变')
                # One durable intent, then finish() reconciles descriptor/queue.
                job.update(binding=binding, upload_requested=True, state='upload_pending')
                self.save(job)
            elif selection and (selection.get('task_id') != job['binding']['collection_task_id']
                                or selection.get('connection_id') != job['binding']['connection_id']):
                raise UploadError('TASK_BINDING_MISMATCH', '录制已固定任务，不能改投另一任务')
            job['upload_formats'] = sorted(set(job.get('upload_formats', [])) | set(formats))
            missing = False
            for fmt in formats:
                queued = next((j for j in self.platform.upload.queue.jobs()
                               if j.get('local_recording_id') == artifact_identity(job, fmt)), None)
                if queued:
                    self.platform.upload.queue.retry(queued['id'])
                else:
                    missing = True
            self.save(job)
            if missing:
                job.update(upload_requested=True, state='upload_pending', error=None)
                self.save(job)
                self.submit(job)
            # Explicit manual upload may enable the worker, but never the automatic switch.
            self.platform.state['worker_enabled'] = True
            self.platform.save()
            self.platform.upload.start()
            return {'ok': True, 'recording_id': job['recording_id']}

    def adopt_history(self, identifier):
        from .recording_datasets import resolve_dataset
        from .topic_recorder import read_recording_metadata
        target = resolve_dataset(self.directory, identifier)
        dataset = target if identifier.startswith('lerobot:') else None
        if dataset:
            from .export_progress import progress_path, read_progress
            if read_progress(progress_path(dataset)).get('state') in ('exporting', 'error'):
                raise UploadError('EXPORT_INCOMPLETE', '历史数据集尚未完整整理，源文件已保留')
            # 1. Exact match by dataset name
            target = next((p for p in (self.directory / '.lerobot_sources' / (dataset.name+'.mcap'),
                                      self.directory / (dataset.name+'.mcap')) if p.is_file()), None)
            # 2. Match standard timestamp pattern (teleop_YYYYMMDD_HHMMSS)
            if target is None:
                import re
                match = re.search(r'teleop_\d{8}_\d{6}', dataset.name)
                if match:
                    base = match.group(0)
                    target = next((p for p in (self.directory / '.lerobot_sources' / (base+'.mcap'),
                                              self.directory / (base+'.mcap')) if p.is_file()), None)
            # 3. Match candidate stem before trailing suffixes (e.g. _sync_fixed, _full_timeline)
            if target is None and '_' in dataset.name:
                parts = dataset.name.split('_')
                for i in range(len(parts) - 1, 0, -1):
                    cand = '_'.join(parts[:i])
                    target = next((p for p in (self.directory / '.lerobot_sources' / (cand+'.mcap'),
                                              self.directory / (cand+'.mcap')) if p.is_file()), None)
                    if target:
                        break

            # 4. Fallback if source MCAP is truly missing/deleted: use dataset metadata directly
            if target is None:
                info_path = dataset / 'meta/info.json'
                if not info_path.is_file():
                    raise UploadError('LEGACY_OWNERSHIP_UNKNOWN', '缺少历史数据集的源 MCAP 或元数据，无法确认设备与采集时间')
                info = read_json(info_path)
                local = self.platform.identity_provider()
                robot_type = info.get('robot_type')
                if robot_type and robot_type != local.get('profile_id'):
                    raise UploadError('LOCAL_ROBOT_MISMATCH', '历史数据集的机器人配置不匹配')
                stamp = lambda ns: datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat()
                started_at, ended_at = None, None
                sync_file = dataset / 'meta/teleop_sync.json'
                if sync_file.is_file():
                    try:
                        sync_data = read_json(sync_file)
                        if sync_data.get('grid_start_log_time_ns') and sync_data.get('grid_end_log_time_ns'):
                            started_at = stamp(sync_data['grid_start_log_time_ns'])
                            ended_at = stamp(sync_data['grid_end_log_time_ns'])
                        elif sync_data.get('origin_log_time_ns'):
                            started_at = stamp(sync_data['origin_log_time_ns'])
                            ended_at = stamp(sync_data['origin_log_time_ns'] + int(1e9))
                    except Exception:
                        pass
                if not started_at or not ended_at:
                    mtime = dataset.stat().st_mtime
                    started_at = datetime.fromtimestamp(mtime, timezone.utc).isoformat()
                    ended_at = datetime.fromtimestamp(mtime + 1, timezone.utc).isoformat()

                identity = uuid4().hex
                job = dict(recording_id=identity, request_id=uuid4().hex, intent={'mode': 'local'},
                           requested_filename=dataset.name, path=str(dataset), state='ready', error=None,
                           binding=None, started_at=started_at, ended_at=ended_at,
                           format='lerobot', metadata={'profile_id': robot_type or local.get('profile_id')},
                           local_robot=local, upload_requested=False,
                           dataset_path=str(dataset), adopted_dataset=True)
                self.jobs[identity] = job
                self.save(job)
                return job

        if not target.is_file() or target.suffix != '.mcap':
            raise UploadError('LEGACY_RECORDING_UNSUPPORTED', '该历史格式需要原来的封存/上传流程')
        if target.is_symlink() or any(p.is_symlink() for p in target.parents):
            raise UploadError('UNSAFE_PATH')
        metadata = read_recording_metadata(target)
        local = self.platform.identity_provider()
        if metadata.get('profile_id') and metadata.get('profile_id') != local.get('profile_id'):
            raise UploadError('LOCAL_ROBOT_MISMATCH', '历史录制的机器人配置不匹配')
        from mcap.reader import make_reader
        with target.open('rb') as stream:
            summary = make_reader(stream).get_summary()
        if not summary or not summary.statistics or not summary.statistics.message_count:
            raise UploadError('MCAP_INCOMPLETE', '历史录制不完整或没有可确认的采集时间')
        statistics = summary.statistics
        stamp = lambda ns: datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat()
        binding = None
        if metadata.get('platform_task'):
            try:
                binding = json.loads(metadata['platform_task'])
            except Exception:
                binding = None
        identity = uuid4().hex
        job = dict(recording_id=identity, request_id=uuid4().hex, intent={'mode': 'platform' if binding else 'local'},
                   requested_filename=dataset.name if dataset else target.name, path=str(target), state='ready', error=None,
                   binding=binding, started_at=stamp(statistics.message_start_time),
                   ended_at=stamp(statistics.message_end_time), format='lerobot' if dataset else 'mcap', metadata=metadata,
                   local_robot=local, upload_requested=False)
        if dataset:
            info = read_json(dataset/'meta/info.json')
            if info.get('robot_type') != local['profile_id']:
                raise UploadError('LOCAL_ROBOT_MISMATCH', '历史数据集的机器人配置不匹配')
            job.update(dataset_path=str(dataset), adopted_dataset=True)
        self.jobs[identity] = job
        self.save(job)
        return job

    def status(self):
        with self.lock:
            return {**self.recorder.status(), 'task_recording': copy.deepcopy(self.jobs.get(self.active)),
                    'recording': self.is_recording(),
                    'recording_selection': self.platform.recording_selection(),
                    'corrections': self.corrections.status() if self.corrections else None}

    def is_recording(self):
        return self.recorder.is_recording() or bool(self.corrections and self.corrections.active)

    def records(self):
        with self.lock:
            return [copy.deepcopy(job) for job in self.jobs.values() if job['state'] not in ('armed', 'discarded')]

    def upload_records(self):
        rows = []
        for job in self.records():
            # This table is an upload history, so only expose the format(s)
            # frozen for upload. Other locally saved artifacts remain visible
            # in the local recording list and must not look queued here.
            for fmt in upload_formats(job):
                artifact = job.get('artifacts', {}).get(fmt, {})
                row = {**job, **artifact, 'recording_id': artifact_identity(job, fmt), 'format': fmt}
                row['upload_target'] = ('lerobot:' + Path(job['path']).stem if fmt == 'lerobot' else
                                        'bundle:' + Path(job['path']).stem if fmt == 'mcap_mp4' else Path(job['path']).name)
                row['display_name'] = (Path(job.get('dataset_path') or job['path']).stem
                                       if fmt == 'lerobot' else Path(job['path']).stem
                                       if fmt == 'mcap_mp4' else Path(job['path']).name)
                rows.append(row)
        return rows

    def list_recordings(self, format_filter='all', identifier='', compact=False):
        files = self.recorder.list_recordings(format_filter, identifier, compact)
        with self.lock:
            hidden = {Path(job['path']).name for job in self.jobs.values() if job['state'] == 'armed'}
            files = [item for item in files if item['filename'] not in hidden]
            by_identifier = {}
            for job in self.jobs.values():
                source = Path(job['path'])
                by_identifier[source.name] = job
                by_identifier['lerobot:' + source.stem] = job
                by_identifier['bundle:' + source.stem] = job
            for item in files:
                identifier = item.get('id') or item['filename']
                job = by_identifier.get(identifier)
                if job:
                    if job['state'] in ('starting', 'recording', 'stopping'):
                        item['is_current'] = True
                    projection = copy.deepcopy(job)
                    fmt = 'lerobot' if identifier.startswith('lerobot:') else 'mcap_mp4' if identifier.startswith('bundle:') else 'mcap'
                    projection.update(projection.get('artifacts', {}).get(fmt, {}))
                    projection['artifact_id'] = artifact_identity(job, fmt)
                    item['platform_recording'] = projection
        return files

    def delete_recording(self, filename):
        with self.lock:
            from .recording_datasets import resolve_dataset
            job = next((job for job in self.jobs.values() if Path(job['path']).name == filename
                        or 'lerobot:' + Path(job['path']).stem == filename
                        or 'bundle:' + Path(job['path']).stem == filename), None)
            try:
                target = resolve_dataset(self.directory, filename)
            except FileNotFoundError:
                if job and (job.get('started_at') is None or job.get('state') in ('prepared', 'prepare_failed')):
                    job.update(state='deleted', error=None)
                    (self.state_dir / f"{job['recording_id']}.json").unlink(missing_ok=True)
                    self.jobs.pop(job['recording_id'], None)
                    return {'deleted': filename, 'freed_bytes': 0}
                raise
            for queued_job in self.platform.upload.queue.jobs():
                root = Path(queued_job['root'])
                if root == target or any(root/a['path'] == target for a in queued_job['manifest']['assets']):
                    raise ValueError('录制仍被上传队列引用，不能删除')
            if job:
                if job['state'] in ('starting', 'armed', 'recording', 'stopping'):
                    raise ValueError('录制尚未结束，不能删除')
                ids = {artifact_identity(job, fmt) for fmt in job_artifact_formats(job)}
                queued = any(j.get('local_recording_id') in ids for j in self.platform.upload.queue.jobs())
                if queued or job['recording_id'] in self.pending:
                    raise ValueError('录制有上传记录或仍在整理，不能删除；请先处理该录制作业')
            result = self.recorder.delete_recording(filename)
            if job:
                fmt = 'lerobot' if filename.startswith('lerobot:') else 'mcap_mp4' if filename.startswith('bundle:') else 'mcap'
                job['deleted_formats'] = sorted(set(job.get('deleted_formats', [])) | {fmt})
                job.setdefault('artifacts', {})[fmt] = {'artifact_id': artifact_identity(job, fmt), 'state': 'deleted'}
                if set(job['deleted_formats']) == set(job_artifact_formats(job)):
                    job.update(state='deleted', error=None)
                self.save(job)
            return result

    def close(self):
        if self.corrections: self.stop_corrections()
        self.stop()
        self.closing.set()
        self.platform.upload.artifact_process.close()
        self.finalizer.shutdown(wait=True)
        self.recorder.shutdown()

    shutdown = close
