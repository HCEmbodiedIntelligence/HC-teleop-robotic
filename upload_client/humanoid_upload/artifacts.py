"""HC ready descriptors and cross-process read/delete leases (Linux flock)."""
import fcntl
import hashlib
import re
import stat
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path

from .bundle import checksum, finalized_mp4, safe_path
from .common import Limiter, UploadError, atomic_json, canonical, read_json, validate


def artifact_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', value):
        raise UploadError('ARTIFACT_ID_INVALID')
    return value


class ArtifactLocks:
    def __init__(self, state_dir):
        self.directory = Path(state_dir) / '.locks'
        self.directory.mkdir(parents=True, exist_ok=True)

    def tombstone(self, identity):
        return self.directory / (artifact_id(identity) + '.deleted.json')

    @contextmanager
    def _lock(self, identity, exclusive):
        path = self.directory / (artifact_id(identity) + '.lock')
        with path.open('a') as stream:
            try:
                fcntl.flock(stream, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)
            except BlockingIOError:
                raise UploadError('ARTIFACT_BUSY', '上传仍占用文件，等待释放后重试', retry=True) from None
            try:
                yield
            finally:
                fcntl.flock(stream, fcntl.LOCK_UN)

    @contextmanager
    def read(self, identity):
        with self._lock(identity, False):
            if self.tombstone(identity).exists():
                raise UploadError('ARTIFACT_DELETED')
            yield

    @contextmanager
    def delete(self, identity):
        with self._lock(identity, True):
            atomic_json(self.tombstone(identity), {'artifact_id': identity, 'deleted': True})
            yield

    def usage(self, identity):
        try:
            with self._lock(identity, True):
                return {'reading': False, 'deleted': self.tombstone(identity).exists()}
        except UploadError as error:
            if error.code != 'ARTIFACT_BUSY':
                raise
            return {'reading': True, 'deleted': self.tombstone(identity).exists()}


def inspect_ready(descriptor, cfg):
    if not isinstance(descriptor, dict) or descriptor.get('schema_version') != 'hc-artifact-ready/v1':
        raise UploadError('ARTIFACT_SCHEMA_INVALID')
    required = {'schema_version', 'recording_id', 'artifact_id', 'kind', 'root', 'files',
                'profile_id', 'config_fingerprint', 'source_artifact_id', 'native_session_id',
                'native_completion_marker', 'binding_id', 'completed_at'}
    if not required <= descriptor.keys() or descriptor.keys() - required - {'capture_started_at', 'capture_ended_at'}:
        raise UploadError('ARTIFACT_SCHEMA_INVALID')
    artifact_id(descriptor.get('artifact_id'))
    artifact_id(descriptor.get('recording_id'))
    kind = descriptor.get('kind')
    if kind not in {'hc_mcap', 'hc_lerobot', 'platform_raw'}:
        raise UploadError('ARTIFACT_KIND_INVALID')
    for key in ('profile_id', 'config_fingerprint', 'completed_at'):
        if not isinstance(descriptor.get(key), str) or not descriptor[key]:
            raise UploadError('ARTIFACT_SCHEMA_INVALID')
    root = Path(descriptor.get('root', ''))
    if not root.is_absolute() or any(p.is_symlink() for p in (root, *root.parents)):
        raise UploadError('UNSAFE_PATH')
    if not root.is_dir() or not root.resolve().is_relative_to(Path(cfg['data_directory']).resolve()):
        raise UploadError('EXPORT_OUTSIDE_DATA_DIRECTORY')
    entries = descriptor.get('files')
    if not isinstance(entries, list) or not entries or len(entries) > 4096:
        raise UploadError('ASSET_INVENTORY')
    paths, inventory = set(), []
    limiter = Limiter(cfg['read_bytes_per_second'])
    for item in entries:
        if not isinstance(item, dict):
            raise UploadError('ASSET_INVENTORY')
        rel = item.get('path')
        safe_path(rel)
        if rel in paths:
            raise UploadError('ASSET_INVENTORY')
        paths.add(rel)
        path = root / rel
        for component in (path, *path.parents):
            if component == root:
                break
            if component.is_symlink():
                raise UploadError('SYMLINK_ASSET')
        if any(p.endswith(('.tmp', '.part', '.incomplete')) or p.startswith('.export-') or p == 'FAILED.json' for p in path.parts):
            raise UploadError('EXPORT_INCOMPLETE')
        if not path.exists() or not stat.S_ISREG(path.stat().st_mode):
            raise UploadError('ASSET_MISSING')
        facts = checksum(path, limiter)
        if type(item.get('size_bytes')) is not int or item['size_bytes'] <= 0 or any(item.get(k) != facts[k] for k in ('size_bytes', 'sha256')):
            raise UploadError('ASSET_INTEGRITY')
        if rel.endswith('.mp4'):
            finalized_mp4(path)
        if rel.endswith('.parquet'):
            with path.open('rb') as stream:
                if stream.read(4) != b'PAR1':
                    raise UploadError('PARQUET_INCOMPLETE')
                stream.seek(-4, 2)
                if stream.read(4) != b'PAR1':
                    raise UploadError('PARQUET_INCOMPLETE')
        inventory.append({'path': rel, **facts})
    inventory.sort(key=lambda x: x['path'])
    if kind == 'hc_mcap':
        mcaps = [p for p in paths if p.endswith('.mcap')]
        if len(mcaps) != 1:
            raise UploadError('MCAP_INVENTORY_INVALID')
        with (root / mcaps[0]).open('rb') as stream:
            magic = b'\x89MCAP0\r\n'
            if stream.read(8) != magic:
                raise UploadError('MCAP_INCOMPLETE')
            stream.seek(-8, 2)
            if stream.read(8) != magic:
                raise UploadError('MCAP_INCOMPLETE')
    if kind == 'hc_lerobot':
        if 'meta/info.json' not in paths or not any(p.endswith('.parquet') for p in paths):
            raise UploadError('LEROBOT_INCOMPLETE')
        info = read_json(root / 'meta/info.json')
        if not isinstance(info, dict) or not info.get('codebase_version') or not info.get('features') or info.get('total_frames', 0) <= 0:
            raise UploadError('LEROBOT_METADATA_INVALID')
        # LeRobot is a dedicated dataset directory; missing media/shards are not optional.
        actual = set()
        for path in root.rglob('*'):
            if path.is_symlink() or not (path.is_dir() or path.is_file()):
                raise UploadError('SYMLINK_OR_SPECIAL_ASSET')
            if path.is_file():
                actual.add(path.relative_to(root).as_posix())
        if actual != paths:
            raise UploadError('ASSET_INVENTORY')
    return root, inventory


def manifest_for_ready(descriptor, cfg, client_id):
    root, inventory = inspect_ready(descriptor, cfg)
    if descriptor.get('binding_id') not in (None, cfg.get('binding_id')):
        raise UploadError('TASK_BINDING_MISMATCH')
    if descriptor['kind'] == 'platform_raw':
        marker_path = descriptor.get('native_completion_marker')
        if marker_path not in ('recording-complete.json', str(root / 'recording-complete.json')):
            raise UploadError('RAW_COMPLETION_REQUIRED')
        from .recording_bundle import manifest_for_recording
        result = manifest_for_recording(root, cfg, client_id)
        marker = result[1]
        if marker['schema_version'] != 'openarm-recording-complete/v2':
            raise UploadError('RAW_COMPLETION_REQUIRED')
        if descriptor.get('native_session_id') != marker['session_id']:
            raise UploadError('IDENTITY_MISMATCH')
        if {a['path'] for a in result[2]['assets']} != {a['path'] for a in inventory}:
            raise UploadError('ASSET_INVENTORY')
        return result
    if descriptor.get('native_completion_marker') is not None:
        raise UploadError('ARTIFACT_SCHEMA_INVALID')
    try:
        start, end = [datetime.fromisoformat(descriptor[k].replace('Z', '+00:00'))
                      for k in ('capture_started_at', 'capture_ended_at')]
        if not start.tzinfo or not end.tzinfo or end <= start:
            raise ValueError()
    except (KeyError, ValueError, TypeError, AttributeError):
        raise UploadError('CAPTURE_TIME_REQUIRED', '需要录制方提供真实采集起止时间，不能使用导出完成时间') from None
    digest = hashlib.sha256(canonical(descriptor)).hexdigest()
    marker = dict(export_id=descriptor['artifact_id'], session_id=descriptor['recording_id'],
                  robot_id=cfg['robot_id'], collection_task_id=cfg['task_id'], content_sha256=digest)
    lerobot = descriptor['kind'] == 'hc_lerobot'
    manifest = dict(schema_version='robot-ingest/v1', client_upload_id=client_id,
                    collection_task_id=cfg['task_id'], robot_id=cfg['robot_id'],
                    source_format='LEROBOT_V3' if lerobot else 'MCAP',
                    source_format_version=read_json(root/'meta/info.json')['codebase_version'] if lerobot else '0',
                    capture_mode='PRESEGMENTED' if lerobot else 'CONTINUOUS',
                    capture_started_at=descriptor['capture_started_at'], capture_ended_at=descriptor['capture_ended_at'],
                    cameras=[], assets=[dict(a, asset_id=f'asset-{i:03d}', role='RAW', metadata={},
                        media_type='video/mp4' if a['path'].endswith('.mp4') else 'application/octet-stream')
                        for i, a in enumerate(inventory)],
                    format_metadata={'hc_artifact': {k: descriptor.get(k) for k in (
                        'schema_version', 'artifact_id', 'recording_id', 'kind', 'profile_id',
                        'config_fingerprint', 'source_artifact_id', 'native_session_id')}})
    validate(manifest, 'robot-ingest-manifest')
    return root, marker, manifest
