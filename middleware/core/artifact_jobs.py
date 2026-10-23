"""Disk-heavy finalization and upload operations, executed in managed children."""
from datetime import datetime, timezone
from pathlib import Path

from humanoid_upload.bundle import checksum
from humanoid_upload.common import Limiter, UploadError, atomic_json, read_json
from humanoid_upload.queue import Queue, public_job
from .capture_formats import artifact_identity, artifact_snapshot, upload_formats


def now():
    return datetime.now(timezone.utc).isoformat()


def enqueue_ready(descriptor, cfg):
    return public_job(Queue(cfg['queue_directory']).enqueue_artifact(descriptor, cfg))


def upload_tick(cfg):
    from humanoid_upload.worker import Worker
    from .background_process import stopping
    queue = Queue(cfg['queue_directory'])
    with queue.worker_lock():
        Worker(queue, cfg, should_stop=stopping).tick()


def seal_task_artifact(job, fmt, directory, state_dir, cfg):
    directory, state_dir = Path(directory), Path(state_dir)
    identity = artifact_identity(job, fmt)
    source = Path(job['path'])
    snapshot = artifact_snapshot(job, fmt)
    if fmt == 'lerobot':
        target = Path(job.get('dataset_path') or directory / 'lerobot' / source.stem)
        root = target
        if snapshot:
            from humanoid_upload.capture_plan import validate_lerobot_info
            snapshot = artifact_snapshot(job, fmt)
            presence = validate_lerobot_info(snapshot, read_json(root / 'meta/info.json'))
            context = {'schema_version': 'hc-capture-context/v1', 'recording_id': identity,
                       'capture_snapshot': snapshot, 'feature_presence': presence}
            context_path = root / 'capture-context.json'
            if context_path.exists() and read_json(context_path) != context:
                raise UploadError('CAPTURE_SNAPSHOT_MISMATCH', '已有 LeRobot 采集快照不能覆盖')
            if not context_path.exists(): atomic_json(context_path, context)
        from .recording_datasets import dataset_files
        files = dataset_files(root)
        kind = 'hc_lerobot'
    elif fmt == 'mcap_mp4':
        from .video_bundle import prepare
        from .recording_datasets import dataset_files
        # A new upload has its own session/ownership marker. Retain the
        # source package unchanged and give retries the same new directory.
        bundle_name = (source.stem + '-' + job['recording_id']
                       if job.get('source_recording_id') else source.stem)
        root = prepare(source, directory / 'bundles' / bundle_name, {**job, 'capture_snapshot': snapshot},
                       read_bytes_per_second=cfg['read_bytes_per_second'])
        if not job.get('binding'):
            return 'ready'  # The original package can acquire its first task later.
        files, kind = dataset_files(root), 'platform_raw'
    else:
        root, files, kind = source.parent, [source], 'hc_mcap'
        correction = source.with_suffix('.correction.json')
        if correction.is_file(): files.append(correction)
        session_capture = source.with_suffix('.session.json')
        if session_capture.is_file(): files.append(session_capture)
    descriptor_path = state_dir / (identity + '.json')
    if descriptor_path.exists():
        descriptor = read_json(descriptor_path)
        if descriptor.get('capture_snapshot') != snapshot:
            raise UploadError('CAPTURE_SNAPSHOT_MISMATCH', '已封存方案快照不能改变或由当前默认值补写')
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
        limiter = Limiter(cfg['read_bytes_per_second'])
        descriptor = dict(schema_version='hc-artifact-ready/v1', recording_id=identity,
            artifact_id=identity, kind=kind, root=str(root),
            files=[{'path': str(p.relative_to(root)), **checksum(p, limiter)} for p in files],
            profile_id=job['local_robot']['profile_id'],
            config_fingerprint=job['local_robot']['config_fingerprint'],
            source_artifact_id=job['recording_id'] if identity != job['recording_id'] else None, native_session_id=job['recording_id'] if fmt == 'mcap_mp4' else None, native_completion_marker='recording-complete.json' if fmt == 'mcap_mp4' else None,
            binding_id=job['binding']['binding_id'] if job['binding'] else None,
            capture_started_at=job['started_at'], capture_ended_at=job['ended_at'],
            completed_at=now())
        if job['binding']:
            descriptor['ownership_snapshot'] = job['binding']['task']['ownership']
        if snapshot:
            descriptor['capture_snapshot'] = snapshot
        # Validate closure/inventory even for local-only data before marking ready.
        from humanoid_upload.artifacts import inspect_ready
        inspect_ready(descriptor, cfg)
        atomic_json(descriptor_path, descriptor)
    upload = ((job.get('upload_requested') and fmt in job.get('upload_formats', upload_formats(job)))
              or bool(job['binding'] and job['binding']['auto_upload'] and fmt in upload_formats(job)))
    if upload:
        enqueue_ready(descriptor, cfg)
    return 'queued' if upload else 'ready'
