"""Upload handoff for main's MCAP recorder; independent of ROS and Motion.

Persist lifecycle times and binding before capture. Only closed sources and
completed LeRobot exports become immutable ready descriptors. Hashing runs off
recording/UI threads. Unfinished captures are never inferred complete on restart.
"""
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
import json
import threading
import uuid

from .artifact_state import atomic_json, fingerprint, publish_ready
from .export_progress import progress_path, read_progress


class RecordingArtifacts:
    def __init__(self, config, config_dir, directory):
        def resolve(key, default):
            path = Path(config.get(key, default)).expanduser()
            return (config_dir / path).resolve() if not path.is_absolute() else path.resolve()
        self.state_dir = resolve('artifact_state_dir', '../runtime/artifact_state')
        self.job_dir = resolve('upload_job_state_dir', '../runtime/recording_uploads')
        self.directory = Path(directory).resolve()
        self.config = dict(config)
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.jobs = {}
        self.recovery_errors = {}
        if self.job_dir.exists():
            for path in self.job_dir.glob('*.json'):
                try:
                    if path.is_symlink():
                        raise ValueError('job state is a symlink')
                    job = json.loads(path.read_text())
                    # A storage-root change must not pick up another recorder's jobs.
                    if job.get('directory') != str(self.directory):
                        continue
                    identity = job['recording_id']
                    if uuid.UUID(identity).hex != identity or path.stem != identity:
                        raise ValueError('invalid recording identity')
                    if job['format'] not in ('mcap', 'lerobot', 'both') or not isinstance(job['artifacts'], dict):
                        raise ValueError('invalid recording state')
                    for key in ('source', 'destination'):
                        if not Path(job[key]).resolve().is_relative_to(self.directory):
                            raise ValueError('recording path outside storage root')
                    datetime.fromisoformat(job['capture_started_at'])
                    if job.get('state') == 'capturing':
                        job['state'] = 'interrupted'
                        atomic_json(path, job)
                    self.jobs[identity] = job
                except (OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
                    self.recovery_errors[path.name] = str(exc)
        self.start_worker()

    def save(self, job):
        atomic_json(self.job_dir / (job['recording_id'] + '.json'), job)
        self.jobs[job['recording_id']] = job

    def begin(self, source, save_format, metadata, binding=None):
        try:
            from humanoid_upload.artifacts import ArtifactLocks  # noqa: F401
        except ImportError:
            return None  # Local capture remains available before install.sh --upload.
        snapshot = metadata.get('capture_snapshot')
        if isinstance(snapshot, str): snapshot = json.loads(snapshot)
        if snapshot:
            from humanoid_upload.capture_plan import validate_snapshot
            snapshot = validate_snapshot(snapshot, recording_id=metadata.get('recording_id'))
        identity = snapshot['recording_id'] if snapshot else uuid.uuid4().hex
        job = dict(recording_id=identity, directory=str(self.directory), source=str(source),
                   destination=str(self.directory / 'lerobot' / Path(source).stem),
                   format=save_format, profile_id=metadata.get('profile_id') or 'unknown',
                   config_fingerprint=fingerprint({'recording': self.config, 'profile': metadata}),
                   capture_started_at=datetime.now(timezone.utc).isoformat(),
                   binding=binding, state='capturing', artifacts={})
        if snapshot:
            job['capture_snapshot'] = snapshot
        with self.lock:
            self.save(job)
        return identity

    def mark_started(self, identity):
        if identity is not None:
            with self.lock:
                job = self.jobs[identity]
                job['capture_started_at'] = datetime.now(timezone.utc).isoformat()
                self.save(job)

    def close(self, identity, error=None, ended_at=None):
        if identity is None:
            return
        with self.lock:
            job = self.jobs[identity]
            job['capture_ended_at'] = ended_at or datetime.now(timezone.utc).isoformat()
            job['state'] = 'failed' if error else 'closed'
            job['error'] = error
            self.save(job)
        self.start_worker()

    def start_worker(self):
        with self.lock:
            if self.stop_event.is_set() or (self.thread and self.thread.is_alive()):
                return
            if not any(j['state'] == 'closed' for j in self.jobs.values()):
                return
            self.thread = threading.Thread(target=self._run, name='hc-artifact-handoff', daemon=True)
            self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            self.poll()
            with self.lock:
                pending = any(j['state'] == 'closed' and not (
                    (j['format'] == 'mcap' and j['artifacts'].get('hc_mcap') in ('ready', 'deleted')) or
                    (j['format'] == 'lerobot' and j['artifacts'].get('hc_lerobot') in ('ready', 'deleted')) or
                    (j['format'] == 'both' and all(j['artifacts'].get(k) in ('ready', 'deleted') for k in ('hc_mcap', 'hc_lerobot')))
                ) for j in self.jobs.values())
                if not pending:
                    self.thread = None
                    return
            self.stop_event.wait(1)

    def poll(self):
        with self.lock:
            for job in self.jobs.values():
                if job['state'] != 'closed':
                    continue
                kinds = ['hc_mcap'] if job['format'] == 'mcap' else ['hc_lerobot']
                if job['format'] == 'both':
                    kinds = ['hc_mcap', 'hc_lerobot']
                for kind in kinds:
                    if job['artifacts'].get(kind) in ('ready', 'deleted'):
                        continue
                    identity = job['recording_id'] + ('-mcap' if kind == 'hc_mcap' else '-lerobot')
                    try:
                        from humanoid_upload.artifacts import ArtifactLocks
                        with ArtifactLocks(self.state_dir).read(identity):
                            if kind == 'hc_mcap':
                                source = Path(job['source'])
                                with source.open('rb') as stream:
                                    magic = b'\x89MCAP0\r\n'
                                    if stream.read(8) != magic:
                                        raise ValueError('MCAP header is incomplete')
                                    stream.seek(-8, 2)
                                    if stream.read(8) != magic:
                                        raise ValueError('MCAP footer is incomplete')
                                root, paths = source.parent, [source.name]
                            else:
                                root = Path(job['destination'])
                                if read_progress(progress_path(root)).get('state') != 'completed':
                                    continue
                                from .recording_datasets import dataset_files, inspect_dataset
                                inspect_dataset(root)
                                paths = [p.relative_to(root).as_posix() for p in dataset_files(root)]
                            publish_ready(self.state_dir, job, root, paths, kind, identity,
                                          source_artifact_id=job['recording_id'] + '-mcap' if kind == 'hc_lerobot' and job['format'] == 'both' else None)
                        job['artifacts'][kind] = 'ready'
                        job.pop('error', None)
                    except Exception as exc:
                        if getattr(exc, 'code', None) == 'ARTIFACT_DELETED':
                            job['artifacts'][kind] = 'deleted'
                        else:
                            job['error'] = str(exc)
                    self.save(job)

    def annotate(self, items):
        # Do not block UI on the worker's content hashing lock.
        jobs = sorted(self.jobs.values(), key=lambda job: job['capture_started_at'])
        for item in items:
            dataset = item.get('id', '').startswith('lerobot:')
            for job in reversed(jobs):
                name = Path(job['destination'] if dataset else job['source']).name
                if name != item.get('filename'):
                    continue
                kind = 'hc_lerobot' if dataset else 'hc_mcap'
                if job['format'] == 'lerobot' and not dataset:
                    continue
                item['artifact_id'] = job['recording_id'] + ('-lerobot' if dataset else '-mcap')
                item['upload_artifact_state'] = job['artifacts'].get(kind, job['state'])
                item['upload_error'] = job.get('error')
                break
        return items

    @contextmanager
    def deleting(self, target):
        from contextlib import ExitStack
        target = Path(target).resolve()
        with self.lock, ExitStack() as leases:
            matched = []
            for job in self.jobs.values():
                for kind, key, suffix in [('hc_mcap', 'source', '-mcap'), ('hc_lerobot', 'destination', '-lerobot')]:
                    if Path(job[key]).resolve() != target:
                        continue
                    from humanoid_upload.artifacts import ArtifactLocks
                    leases.enter_context(ArtifactLocks(self.state_dir).delete(job['recording_id'] + suffix))
                    matched.append((job, kind))
            yield
            for job, kind in matched:
                job['artifacts'][kind] = 'deleted'
                self.save(job)

    def shutdown(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
