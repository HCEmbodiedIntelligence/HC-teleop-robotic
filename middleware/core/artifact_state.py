"""Persistent internal readiness records; never a platform upload manifest."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import uuid


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex)
    try:
        with temporary.open('x') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        fd = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        temporary.unlink(missing_ok=True)


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()).hexdigest()


def publish_ready(state_dir, job, root, paths, kind, artifact_id, *, source_artifact_id=None, binding_id=None):
    original = Path(root)
    if any(p.is_symlink() for p in (original, *original.parents)):
        raise ValueError('artifact root is a symlink')
    root = original.resolve(strict=True)
    files = []
    for value in sorted(paths):
        relative = Path(value)
        if relative.is_absolute() or '..' in relative.parts or not relative.parts:
            raise ValueError('invalid artifact path')
        file = root / relative
        if any((root / Path(*relative.parts[:i])).is_symlink() for i in range(1, len(relative.parts)+1)):
            raise ValueError('artifact symlink is forbidden')
        if not file.resolve(strict=True).is_relative_to(root) or not file.is_file() or file.name.endswith(('.tmp', '.partial', '.part', '.incomplete')) or file.name == 'FAILED.json':
            raise ValueError('incomplete artifact file')
        before = file.stat()
        digest = hashlib.sha256()
        with file.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                digest.update(block)
        after = file.stat()
        if after.st_size == 0:
            raise ValueError('empty artifact asset')
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError('artifact changed while hashing')
        files.append(dict(path=relative.as_posix(), size_bytes=after.st_size, sha256=digest.hexdigest()))
    if not files:
        raise ValueError('empty artifact')
    if kind == 'platform_raw' and not (root/'recording-complete.json').is_file():
        raise ValueError('raw bundle is not sealed')
    result = dict(schema_version='hc-artifact-ready/v1', recording_id=job['recording_id'],
                  artifact_id=artifact_id, kind=kind, root=str(root), files=files,
                  profile_id=job['profile_id'], config_fingerprint=job['config_fingerprint'],
                  source_artifact_id=source_artifact_id, native_session_id=job.get('native_session_id'),
                  native_completion_marker='recording-complete.json' if kind == 'platform_raw' else None,
                  binding_id=binding_id, completed_at=datetime.now(timezone.utc).isoformat())
    for key in ('capture_started_at', 'capture_ended_at'):
        if job.get(key):
            result[key] = job[key]
    if binding_id is None and job.get('binding'):
        result['binding_id'] = job['binding']['binding_id']
    target = Path(state_dir)/(artifact_id+'.json')
    if target.exists():
        previous = json.loads(target.read_text())
        if any(previous.get(k) != v for k, v in result.items() if k != 'completed_at'):
            raise ValueError('artifact identity cannot change content')
        return previous
    atomic_json(target, result)
    return result
