"""Serialize all capture starts and durably freeze each recording's upload target."""
from __future__ import annotations

import copy
import json
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from humanoid_upload.bundle import checksum
from humanoid_upload.common import UploadError, atomic_json, read_json


def now():
    return datetime.now(timezone.utc).isoformat()


class TaskRecorder:
    def __init__(self, recorder, platform):
        self.recorder, self.platform = recorder, platform
        self.recorder.task_managed = True
        self.recorder.upload_service = platform.upload
        self.default_lerobot_task = recorder.lerobot_task
        self.prepare_recording = None
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
            if job['state'] in ('starting', 'recording', 'stopping'):
                job.update(state='interrupted', error='录制中断，原始数据已保留；请检查完整性后处理')
                self.save(job)
            elif job['state'] in ('finalizing', 'upload_pending'):
                self.submit(job)

    def __getattr__(self, name):
        return getattr(self.recorder, name)

    def save(self, job):
        atomic_json(self.state_dir / (job['recording_id'] + '.json'), job)

    def set_selection(self, selection):
        with self.lock:
            if self.active or self.recorder.is_recording():
                raise UploadError('RECORDING_ACTIVE', '请停止当前录制后修改任务')
            return self.platform.set_recording_selection(selection)

    def set_upload_format(self, value):
        with self.lock:
            if self.active or self.recorder.is_recording():
                raise UploadError('RECORDING_ACTIVE', '请停止当前录制后修改上传格式')
            return self.platform.set_upload_format(value)

    @staticmethod
    def upload_format(job):
        return job.get('upload_format') or ('lerobot' if job['format'] == 'lerobot' else 'mcap')

    def start(self, filename='', metadata=None, selection=None):
        with self.lock:
            selection = copy.deepcopy(selection if selection is not None else self.platform.recording_selection())
            request_id = selection.get('request_id') or uuid4().hex
            intent = {k: v for k, v in selection.items() if k != 'request_id'}
            for job in self.jobs.values():
                if job['request_id'] == request_id:
                    if job['intent'] != intent or job['requested_filename'] != filename:
                        raise UploadError('RECORDING_REQUEST_CONFLICT', '重复启动请求的任务或文件名发生变化')
                    if job['state'] in ('failed', 'interrupted'):
                        raise UploadError('RECORDING_INTERRUPTED', job.get('error', '原录制需要处理'))
                    return job['path']
            if self.active or self.recorder.is_recording():
                raise UploadError('RECORDING_ACTIVE', '已有录制正在进行，请先停止')
            # Network validation is inside the same critical section as the local
            # start and durable snapshot. Failure never silently becomes local mode.
            upload_format = self.platform.state['upload_format']
            if upload_format == 'lerobot' and self.recorder.save_format == 'mcap':
                raise UploadError('UPLOAD_FORMAT_UNAVAILABLE', '上传 LeRobot 需要在话题录制页将本地保存方式设为 LeRobot 或 MCAP + LeRobot')
            binding = self.platform.freeze(selection)
            try:
                local = self.platform.identity_provider()
            except UploadError:
                if binding: raise
                local = dict(local_robot_id='unconfigured', profile_id='unconfigured', config_fingerprint='0'*64, serial_no=None, model_version_id=None)
            identity = uuid4().hex
            chosen = filename or ('ros2_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + identity[:8])
            job = dict(recording_id=identity, request_id=request_id, intent=intent,
                       requested_filename=filename, path=str((self.directory / '.lerobot_sources' if self.recorder.save_format == 'lerobot' else self.directory) / (Path(chosen).name if chosen.endswith('.mcap') else Path(chosen).name + '.mcap')), state='starting', error=None,
                       binding=binding, started_at=now(), ended_at=None,
                       format=self.recorder.save_format, upload_format=upload_format, metadata=copy.deepcopy(metadata or {}),
                       local_robot=local, upload_requested=False)
            self.jobs[identity] = job
            self.save(job)
            capture_metadata = {**(metadata or {}), 'recording_id': identity,
                                'platform_task': json.dumps(binding, ensure_ascii=False) if binding else ''}
            try:
                if self.prepare_recording is not None: self.prepare_recording()
                self.recorder.lerobot_task = binding['task']['name'] if binding else self.default_lerobot_task
                job['path'] = self.recorder.start(chosen, capture_metadata)
                job['state'] = 'recording'
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
            with self.lock:
                job = copy.deepcopy(self.jobs[identity])
            source = Path(job['path'])
            if self.upload_format(job) == 'lerobot':
                target = Path(job.get('dataset_path') or self.directory / 'lerobot' / source.stem)
                from .export_progress import progress_path
                import time
                # The converter is the existing recorder's independent process.
                # Wait on its durable progress, not the next recording's settings.
                while not job.get('adopted_dataset'):
                    if self.closing.is_set():
                        return  # Durable finalizing state will resume on next start.
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
                root = target
                from .recording_datasets import dataset_files
                files = dataset_files(root)
                kind = 'hc_lerobot'
            else:
                root, files, kind = source.parent, [source], 'hc_mcap'
            descriptor_path = self.platform.upload.state_dir / (identity + '.json')
            if descriptor_path.exists():
                descriptor = read_json(descriptor_path)
                # A crash may occur after the durable job acquired its first
                # task, before the local-only descriptor was updated. Recover
                # only that transition; an existing task is immutable.
                if job['binding']:
                    binding_id = job['binding']['binding_id']
                    if descriptor.get('binding_id') not in (None, binding_id):
                        raise UploadError('TASK_BINDING_MISMATCH', '已封存归属不能改变')
                    if not descriptor.get('binding_id'):
                        descriptor.update(binding_id=binding_id,
                                          ownership_snapshot=job['binding']['task']['ownership'])
                        atomic_json(descriptor_path, descriptor)
            else:
                descriptor = dict(schema_version='hc-artifact-ready/v1', recording_id=identity,
                    artifact_id=identity, kind=kind, root=str(root),
                    files=[{'path': str(p.relative_to(root)), **checksum(p)} for p in files],
                    profile_id=job['local_robot']['profile_id'],
                    config_fingerprint=job['local_robot']['config_fingerprint'],
                    source_artifact_id=None, native_session_id=None, native_completion_marker=None,
                    binding_id=job['binding']['binding_id'] if job['binding'] else None,
                    capture_started_at=job['started_at'], capture_ended_at=job['ended_at'],
                    completed_at=now())
                if job['binding']:
                    descriptor['ownership_snapshot'] = job['binding']['task']['ownership']
                # Validate closure/inventory even for local-only data before marking ready.
                from humanoid_upload.artifacts import inspect_ready
                inspect_ready(descriptor, self.platform.upload.cfg)
                atomic_json(descriptor_path, descriptor)
            upload = job.get('upload_requested') or bool(job['binding'] and job['binding']['auto_upload'])
            with self.lock:
                current = self.jobs[identity]
                current.update(state='upload_pending' if upload else 'ready', error=None)
                self.save(current)
            if upload:
                self.platform.upload.enqueue(identity, job['binding']['binding_id'])
                with self.lock:
                    self.jobs[identity]['state'] = 'queued'
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

    def upload_recording(self, identity, selection=None):
        with self.lock:
            job = self.jobs.get(identity)
            if job is None:
                job = next((j for j in self.jobs.values() if Path(j['path']).name == identity
                            or 'lerobot:' + Path(j['path']).stem == identity), None)
            if job is None:
                job = self.adopt_history(identity)
            identity = job['recording_id']
            if job['state'] in ('starting', 'recording', 'stopping', 'interrupted', 'failed', 'deleted'):
                raise UploadError('RECORDING_NOT_READY', '录制尚未完整关闭，原始数据已保留')
            if identity in self.pending:
                raise UploadError('ARTIFACT_BUSY', '正在整理本段数据，请稍后重试')
            if not job['binding']:
                if not selection or selection.get('mode') != 'platform':
                    raise UploadError('TASK_REQUIRED', '请为这条历史录制明确选择任务')
                binding = self.platform.freeze(selection, remember=False,
                    source_format='LEROBOT_V3' if self.upload_format(job) == 'lerobot' else 'MCAP')
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
            queued = next((j for j in self.platform.upload.queue.jobs()
                           if j.get('local_recording_id') == job['recording_id']), None)
            if queued:
                self.platform.upload.queue.retry(queued['id'])
            else:
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
            target = next((p for p in (self.directory / '.lerobot_sources' / (dataset.name+'.mcap'),
                                      self.directory / (dataset.name+'.mcap')) if p.is_file()), None)
            if target is None:
                raise UploadError('LEGACY_OWNERSHIP_UNKNOWN', '缺少历史数据集的源 MCAP，无法确认设备与采集时间')
        if not target.is_file() or target.suffix != '.mcap':
            raise UploadError('LEGACY_RECORDING_UNSUPPORTED', '该历史格式需要原来的封存/上传流程')
        if target.is_symlink() or any(p.is_symlink() for p in target.parents):
            raise UploadError('UNSAFE_PATH')
        for queued in self.platform.upload.queue.jobs():
            root = Path(queued['root'])
            if root == dataset or any(root/a['path'] == target for a in queued['manifest']['assets']):
                raise UploadError('TASK_BINDING_MISMATCH', '该录制已有上传队列，请在上传列表重试原作业')
        for path in self.platform.upload.state_dir.glob('*.json'):
            descriptor = read_json(path)
            root = Path(descriptor.get('root', ''))
            if descriptor.get('binding_id') and (root == dataset or any(
                    root/a['path'] == target for a in descriptor.get('files', []))):
                raise UploadError('TASK_BINDING_MISMATCH', '该录制已经封存，原归属不能改变')
        metadata = read_recording_metadata(target)
        local = self.platform.identity_provider()
        if metadata.get('profile_id') != local['profile_id'] or metadata.get('platform_task'):
            raise UploadError('LEGACY_OWNERSHIP_UNKNOWN', '历史归属或设备不能自动确认，请使用原录制服务处理')
        from mcap.reader import make_reader
        with target.open('rb') as stream:
            summary = make_reader(stream).get_summary()
        if not summary or not summary.statistics or not summary.statistics.message_count:
            raise UploadError('MCAP_INCOMPLETE', '历史录制不完整或没有可确认的采集时间')
        statistics = summary.statistics
        stamp = lambda ns: datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat()
        identity = uuid4().hex
        job = dict(recording_id=identity, request_id=uuid4().hex, intent={'mode': 'local'},
                   requested_filename=target.name, path=str(target), state='ready', error=None,
                   binding=None, started_at=stamp(statistics.message_start_time),
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
                    'recording_selection': self.platform.recording_selection()}

    def records(self):
        with self.lock:
            return [copy.deepcopy(job) for job in self.jobs.values()]

    def list_recordings(self):
        files = self.recorder.list_recordings()
        with self.lock:
            for item in files:
                identifier = item.get('id') or item['filename']
                job = next((job for job in self.jobs.values() if
                    Path(job['path']).name == identifier or
                    'lerobot:' + Path(job['path']).stem == identifier), None)
                if job:
                    item['platform_recording'] = copy.deepcopy(job)
        return files

    def delete_recording(self, filename):
        with self.lock:
            from .recording_datasets import resolve_dataset
            target = resolve_dataset(self.directory, filename)
            for queued_job in self.platform.upload.queue.jobs():
                root = Path(queued_job['root'])
                if root == target or any(root/a['path'] == target for a in queued_job['manifest']['assets']):
                    raise ValueError('录制仍被上传队列引用，不能删除')
            job = next((job for job in self.jobs.values() if Path(job['path']).name == filename
                        or 'lerobot:' + Path(job['path']).stem == filename), None)
            if job:
                queued = any(j.get('local_recording_id') == job['recording_id'] for j in self.platform.upload.queue.jobs())
                if queued or job['recording_id'] in self.pending:
                    raise ValueError('录制有上传记录或仍在整理，不能删除；请先处理该录制作业')
            result = self.recorder.delete_recording(filename)
            if job:
                job.update(state='deleted', error=None)
                self.save(job)
            return result

    def close(self):
        self.stop()
        self.closing.set()
        self.finalizer.shutdown(wait=True)
        self.recorder.shutdown()

    shutdown = close
