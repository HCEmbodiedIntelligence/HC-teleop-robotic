"""Optional upload orchestration. No robot, ROS, or manager imports."""
import copy
import hashlib
import threading
import time
from pathlib import Path

from humanoid_upload.artifacts import ArtifactLocks, artifact_id
from humanoid_upload.common import DEFAULTS, UploadError, atomic_json, canonical, read_json, validate_config
from humanoid_upload.queue import Queue, public_job
from humanoid_upload.worker import Worker


class UploadService:
    def __init__(self, config, *, artifact_state_dir, data_directory, request_raw_seal=None):
        if not isinstance(config, dict) or type(config.get('enabled', False)) is not bool:
            raise UploadError('CONFIG_INVALID')
        self.enabled = config.get('enabled', False)
        self.state_dir = Path(artifact_state_dir).resolve()
        self.data_directory = str(Path(data_directory).resolve())
        self.request_raw_seal = request_raw_seal
        self._guard = threading.RLock()
        self._stop = threading.Event()
        self._thread = None
        self.error = None
        self.scan_errors = {}
        self.cfg = {**DEFAULTS, 'auto_enqueue': False, **{k: v for k, v in config.items() if k != 'enabled'},
                    'data_directory': self.data_directory}
        self.queue = None
        self.bindings = {}
        self.default_binding_id = None
        if not self.enabled:
            return
        import importlib
        import humanoid_upload.artifacts
        import humanoid_upload.queue
        import humanoid_upload.worker
        importlib.reload(humanoid_upload.artifacts)
        importlib.reload(humanoid_upload.queue)
        importlib.reload(humanoid_upload.worker)
        # Validate scheduling/rates even when no binding exists or bindings are restored.
        # Placeholders only satisfy config structure; they never enter the active config.
        unbound = dict(platform_base_url='https://unbound.invalid', credential_ref='env:UNBOUND',
                       robot_id='unbound', local_robot_id='unbound', task_id='unbound')
        checked = validate_config({**self.cfg, **{k: self.cfg.get(k) or v for k, v in unbound.items()}})
        self.cfg.update({k: v for k, v in checked.items() if k not in self.binding_fields})
        queue_path = Path(self.cfg['queue_directory']).resolve()
        data_path = Path(self.data_directory)
        if queue_path == data_path or queue_path.is_relative_to(data_path) or data_path.is_relative_to(queue_path):
            raise UploadError('QUEUE_INSIDE_DATA')
        self.queue = Queue(queue_path)
        self.locks = ArtifactLocks(self.state_dir)
        self.binding_file = self.queue.directory / 'bindings.json'
        if self.binding_file.exists():
            state = read_json(self.binding_file)
            try:
                bindings, default = state['bindings'], state['default_binding_id']
                if not isinstance(bindings, dict) or default not in bindings:
                    raise ValueError()
                for identity, binding in bindings.items():
                    if not isinstance(binding, dict) or set(binding) != set(self.binding_fields):
                        raise ValueError()
                    normalized = validate_config({**self.cfg, **binding})
                    if (hashlib.sha256(canonical(binding)).hexdigest() != identity or
                            any(normalized[k] != binding[k] for k in self.binding_fields)):
                        raise ValueError()
            except (KeyError, TypeError, ValueError):
                raise UploadError('BINDING_STORE_INVALID', '检查持久绑定文件；保留旧队列，不自动重绑') from None
            self.bindings, self.default_binding_id = bindings, default
        elif all(self.cfg.get(k) for k in ('platform_base_url', 'credential_ref', 'robot_id', 'local_robot_id', 'task_id')):
            self.set_binding({k: self.cfg[k] for k in self.binding_fields})

    binding_fields = ('platform_base_url', 'credential_ref', 'robot_id', 'local_robot_id', 'task_id')

    def require_enabled(self):
        if not self.enabled:
            raise UploadError('UPLOAD_DISABLED', '上传未启用；本地录制仍可使用')

    def set_binding(self, values):
        self.require_enabled()
        if not isinstance(values, dict) or set(values) != set(self.binding_fields):
            raise UploadError('BINDING_INVALID', '仅接受平台地址、凭据引用、平台及本地机器人和任务 ID')
        with self._guard:
            cfg = validate_config({k: v for k, v in {**self.cfg, **values}.items() if k != 'artifact_state_dir'})
            binding = {k: cfg[k] for k in self.binding_fields}
            identity = hashlib.sha256(canonical(binding)).hexdigest()
            bindings = {**self.bindings, identity: binding}
            atomic_json(self.binding_file, {'bindings': bindings, 'default_binding_id': identity})
            self.bindings, self.default_binding_id = bindings, identity
            return self.binding_snapshot()

    def binding_snapshot(self, binding_id=None):
        self.require_enabled()
        with self._guard:
            identity = binding_id or self.default_binding_id
            if identity not in self.bindings:
                raise UploadError('BINDING_REQUIRED', '先选择平台机器人和任务')
            value = self.bindings[identity]
            return {'binding_id': identity, 'platform_base_url': value['platform_base_url'],
                    'robot_id': value['robot_id'], 'collection_task_id': value['task_id']}

    def _config(self, binding_id=None):
        self.require_enabled()
        with self._guard:
            identity = binding_id or self.default_binding_id
            if identity not in self.bindings:
                raise UploadError('BINDING_REQUIRED')
            cfg = validate_config({**self.cfg, **self.bindings[identity]})
            return {**cfg, 'artifact_state_dir': str(self.state_dir), 'binding_id': identity}

    def enqueue(self, identity, binding_id=None):
        self.require_enabled()
        path = self.state_dir / (artifact_id(identity) + '.json')
        if path.is_symlink():
            raise UploadError('UNSAFE_PATH')
        descriptor = read_json(path)
        if descriptor.get('artifact_id') != identity:
            raise UploadError('IDENTITY_MISMATCH')
        frozen = descriptor.get('binding_id')
        previous = next((job for job in self.queue.jobs() if job.get('artifact_id') == identity), None)
        if previous is not None:
            frozen = previous['frozen'].get('binding_id')
        if frozen and binding_id and frozen != binding_id:
            raise UploadError('TASK_BINDING_MISMATCH')
        cfg = self._config(frozen or binding_id)
        return public_job(self.queue.enqueue_artifact(descriptor, cfg))

    def seal_raw(self, recording_id, binding_id=None):
        self.require_enabled()
        if self.request_raw_seal is None:
            raise UploadError('RAW_SEAL_UNAVAILABLE', '录制封存服务尚未接入')
        return self.request_raw_seal(artifact_id(recording_id), self.binding_snapshot(binding_id))

    def scan(self):
        self.require_enabled()
        errors, count = {}, 0
        if self.cfg['auto_enqueue']:
            for path in sorted(self.state_dir.glob('*.json')):
                try:
                    if path.is_symlink():
                        raise UploadError('UNSAFE_PATH')
                    descriptor = read_json(path)
                    if not descriptor.get('binding_id'):
                        continue  # Never assign historical files to a new page default.
                    self.enqueue(path.stem, descriptor['binding_id'])
                    count += 1
                except UploadError as error:
                    errors[path.stem] = error.code
                except (OSError, ValueError, TypeError, KeyError):
                    errors[path.stem] = 'ARTIFACT_INVALID'
        with self._guard:
            self.scan_errors = errors
        return {'enqueued': count, 'errors': errors}

    def step(self, scan=True):
        self.require_enabled()
        if scan:
            self.scan()
        # Frozen cfg overrides page defaults for every existing job.
        jobs = self.queue.jobs(due=True)
        if jobs:
            cfg = {**DEFAULTS, **self.cfg, **jobs[0]['frozen']}
            with self.queue.worker_lock():
                Worker(self.queue, cfg, should_stop=self._stop.is_set).tick()

    def start(self):
        if not self.enabled or (self._thread and self._thread.is_alive()):
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name='hc-upload', daemon=True)
        self._thread.start()

    def _run(self):
        next_scan = 0
        while not self._stop.is_set():
            try:
                scan = time.monotonic() >= next_scan
                if scan:
                    next_scan = time.monotonic() + self.cfg['scan_seconds']
                self.step(scan=scan)
                self.error = None
            except UploadError as error:
                self.error = error.code
            except Exception:
                self.error = 'UPLOAD_SERVICE_ERROR'  # No credentials or transport bodies in UI/logs.
            self._stop.wait(min(self.cfg['scan_seconds'], self.cfg['poll_seconds']))

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        # A timed-out network read still owns its lease; never claim it is released.
        return {'stopped': not self._thread or not self._thread.is_alive()}

    def artifact_usage(self, identity):
        self.require_enabled()
        return {'artifact_id': identity, **self.locks.usage(identity),
                'jobs': [j['id'] for j in self.queue.jobs() if j.get('artifact_id') == identity]}

    def cancel_artifact(self, identity):
        self.require_enabled()
        artifact_id(identity)
        for job in self.queue.jobs():
            if job.get('artifact_id') == identity:
                self.queue.cancel(job['id'])
        return self.artifact_usage(identity)

    def jobs(self):
        if not self.queue:
            return []
        result = []
        for job in self.queue.jobs():
            item = public_job(job)
            committed = job.get('server_state') == 'COMMITTED'
            deleted = bool(job.get('artifact_id') and self.locks.tombstone(job['artifact_id']).exists())
            invalid = job.get('code') in {'ASSET_INTEGRITY', 'ASSET_MISSING', 'LOCAL_CONTENT_CHANGED',
                                        'EXPORT_OUTSIDE_DATA_DIRECTORY', 'SYMLINK_ASSET', 'UNSAFE_PATH'}
            item.update(capture_state='completed', artifact_state='deleted' if deleted else 'invalid' if invalid else 'ready',
                        upload_state='submitted' if committed else job['state'],
                        processing_state=job.get('recording_status') or job.get('processing_status') or 'UNKNOWN',
                        quality_state=('NOT_EVALUATED' if job.get('recording_status') in {'AWAITING_SLICE', 'SLICED'}
                                       else job.get('quality_status') or 'UNKNOWN'),
                        cancel_requested=self.queue.cancelled(job['id']))
            result.append(item)
        return result

    def status(self):
        return {'enabled': self.enabled, 'running': bool(self._thread and self._thread.is_alive()),
                'error': self.error, 'scan_errors': copy.deepcopy(self.scan_errors),
                'default_binding': self.binding_snapshot() if self.default_binding_id else None,
                'binding_values': copy.deepcopy(self.bindings.get(self.default_binding_id) or
                    {key: self.cfg.get(key, '') for key in self.binding_fields}),
                'auto_enqueue': bool(self.cfg['auto_enqueue']), 'jobs': self.jobs()}


def main():
    """Optional separate HTTP process; shared application may instead embed this service."""
    import argparse
    import asyncio
    from aiohttp import web
    from .upload_api import register_upload_routes
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, help='JSON with uploads, artifact_state_dir, data_directory')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8091)
    args = parser.parse_args()
    value = read_json(args.config)
    service = UploadService(value['uploads'], artifact_state_dir=value['artifact_state_dir'],
                            data_directory=value['data_directory'])
    app = web.Application()
    register_upload_routes(app, service)
    async def start(_app):
        await asyncio.to_thread(service.start)
    async def stop(_app):
        await asyncio.to_thread(service.stop)
    app.on_startup.append(start)
    app.on_cleanup.append(stop)
    web.run_app(app, host=args.host, port=args.port)


if __name__ == '__main__':
    main()
