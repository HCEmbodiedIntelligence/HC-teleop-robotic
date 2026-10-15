"""Sealed exports: provenance, byte inventory and durable A-to-D notification."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import uuid

from .catalog import read_json
from .profile import CONTRACT, schema_check
from .storage import atomic_json


def utc(ns=None):
    return datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat() if ns is not None else datetime.now(timezone.utc).isoformat()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode()


def _crc_table():
    table = []
    for value in range(256):
        for _ in range(8):
            value = (value >> 1) ^ (0xC96C5795D7870F42 if value & 1 else 0)
        table.append(value)
    return table


CRC_TABLE = _crc_table()


def inventory(root):
    result = []
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise ValueError('Symlink export asset')
        if not path.is_file() or path == root / 'export-complete.json':
            continue
        rel = path.relative_to(root).as_posix()
        if '\\' in rel or ':' in rel or any(p.startswith('.export-') or p.endswith(('.tmp', '.part', '.incomplete')) or p == 'FAILED.json' for p in rel.split('/')):
            raise ValueError('Incomplete or unsafe export asset: ' + rel)
        sha, crc, size = hashlib.sha256(), (1 << 64) - 1, 0
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                size += len(block)
                sha.update(block)
                for byte in block:
                    crc = CRC_TABLE[(crc ^ byte) & 255] ^ (crc >> 8)
        if not size:
            raise ValueError('Empty export asset: ' + rel)
        result.append({'path': rel, 'size_bytes': size, 'sha256': sha.hexdigest(),
                       'crc64': str(crc ^ ((1 << 64) - 1))})
    return result


def bind_offline_session(path, *, robot_id, collection_task_id):
    """Explicit, one-time attribution for a completed offline session; no globals."""
    path = Path(path)
    snapshot = read_json(path / 'session.json')
    if read_json(path / 'manifest.json')['state'] != 'completed':
        raise ValueError('Only a completed offline session can be bound')
    context = snapshot['metadata']['capture_context']
    if context.get('robot_id') or context.get('collection_task_id'):
        raise ValueError('Capture already has frozen attribution')
    if not all(isinstance(v, str) and v.strip() for v in (robot_id, collection_task_id)):
        raise ValueError('Robot and task identity required')
    binding = {'robot_id': robot_id, 'collection_task_id': collection_task_id}
    # Exclusive creation prevents conflicting rebinds. A crash may require E to
    # inspect an incomplete binding; it can never produce a complete export.
    with (path / 'capture-binding.json').open('x') as stream:
        stream.write(canonical(binding).decode())
        stream.flush()
        os.fsync(stream.fileno())
    sync_directory(path)
    return binding


def context_for_export(path, snapshot, manifest, annotations):
    context = dict(snapshot['metadata']['capture_context'])
    if not context.get('collection_task_id') or not context.get('robot_id'):
        binding = read_json(path / 'capture-binding.json', {})
        if set(binding) != {'robot_id', 'collection_task_id'}:
            raise ValueError('Offline capture requires explicit frozen task/robot binding')
        context.update(binding)
    if manifest['state'] != 'completed' or 'raw.mcap' not in {c['path'] for c in manifest['committed']}:
        raise ValueError('Capture is not successfully committed')
    context.update(session_id=snapshot['session_id'], export_id=str(uuid.uuid4()),
                   capture_started_at=utc(snapshot['started_unix_ns']), capture_ended_at=utc(manifest['ended_unix_ns']),
                   episodes=[{'episode_index': i, 'source_episode_id': e['source_episode_id'],
                              'start_frame': e['start_frame'], 'end_frame': e['end_frame'], 'task': e['task'],
                              'outcome': e['outcome'], 'export_qualified': True} for i, e in enumerate(annotations['episodes'])],
                   invalid_intervals=[{'start_frame': e['start_frame'], 'end_frame': e['end_frame'],
                                       'reason': e.get('reason') or 'manually_marked_invalid'} for e in annotations['invalid_intervals']])
    schema_check(context, 'capture-context')
    if context['capture_ended_at'] <= context['capture_started_at']:
        raise ValueError('Capture end must follow start')
    if len({e['source_episode_id'] for e in context['episodes']}) != len(context['episodes']):
        raise ValueError('Duplicate source episode identity')
    return context


def components(row, kind, cfg):
    result = {}
    for ident, source in cfg['sources'].items():
        if source['kind'] == kind and source['required']:
            data = row[kind][ident]['components']
            for i, axis in enumerate(source['named_axes']):
                result[axis['name']] = {key: values[i] for key, values in data.items()}
    return [result[a['name']] for a in cfg['capture']['profile']['axes']]


def mapping(row, cfg, context, ep, frame, bindings):
    actions, states = components(row, 'action', cfg), components(row, 'state', cfg)
    receives = [d['evidence']['receive_time_ns'] for d in actions] + [d['receive_ns'] for d in states]
    cameras = {}
    for camera in context['profile']['cameras']:
        ident = camera['id']
        image, binding = row['images'][ident], bindings[ident]
        receives.append(image['receive_time_ns'])
        cameras[ident] = {'source_frame': image['rgb']['source_seq'], 'capture_ns': image['rgb']['exposure_midpoint_ns'],
                          'receive_ns': image['receive_time_ns'], 'pts': binding['pts'], 'time_base': binding['time_base']}
    result = {'session_id': context['session_id'], 'source_episode_id': context['episodes'][ep]['source_episode_id'],
              'source_frame': row['target_frame_index'], 'episode_index': ep, 'frame_index': frame,
              'clock_epoch': row['clock_epoch'], 'target_ns': row.get('association_time_ns',row['target_time_ns']), 'receive_ns': max(receives),
              'action_source_ns': [d['source_ns'] for d in actions], 'action_valid': [d['valid'] for d in actions],
              'state_brackets_ns': [d['brackets_ns'] for d in states], 'state_valid': [d['valid'] for d in states],
              'cameras': cameras}
    schema_check(result, 'source-mapping')
    if not all(result['action_valid'] + result['state_valid']):
        raise ValueError('Invalid named vector')
    for axis, stamp in zip(context['profile']['axes'], result['action_source_ns']):
        if not 0 <= result['target_ns'] - stamp <= context['profile']['validity'][axis['group']]['max_age_ms'] * 1_000_000:
            raise ValueError('Noncausal or expired action')
    if any(not left <= result['target_ns'] <= right for left, right in result['state_brackets_ns']):
        raise ValueError('Invalid state bracket')
    return result


def validate_tables(root, context):
    """Check closed Parquet and camera metadata before a marker can exist."""
    import numpy as np
    import pyarrow as pa
    import pyarrow.parquet as pq
    info = read_json(root / 'meta/info.json')
    profile = context['profile']
    from .holobrain_fields import SCHEMA, axes, image_key
    public_fields = info.get('recording_field_schema') == SCHEMA
    output_axes = axes(profile) if public_fields else profile['axes']
    names = [a['name'] for a in output_axes]
    for key in ('action', 'observation.state'):
        expected={'dtype':'float32','shape':[len(names)],'names':names}
        if public_fields:
            expected['units']=[a['unit'] for a in output_axes]
        if info['features'][key] != expected:
            raise ValueError('Feature identity differs from profile')
    for camera in profile['cameras']:
        key=image_key(camera['id']) if public_fields else 'observation.images.'+camera['id']
        if info['features'][key]['shape'] != [camera['height'], camera['width'], 3]:
            raise ValueError('Camera shape differs from profile')
    total = 0
    for ep in pq.read_table(root / 'meta/episodes').to_pylist():
        table = pq.read_table(root / info['data_path'].format(chunk_index=ep['data/chunk_index'], file_index=ep['data/file_index']))
        if len(table) != ep['length']:
            raise ValueError('Parquet row count mismatch')
        for key in ('action', 'observation.state'):
            if table.schema.field(key).type.value_type != pa.float32():
                raise ValueError('Vector storage must be float32')
            values = np.asarray(table[key].to_pylist())
            if values.shape != (len(table), len(names)) or not np.isfinite(values).all():
                raise ValueError('Invalid vector dimensions or values')
        total += len(table)
    if total != info['total_frames']:
        raise ValueError('Total row count mismatch')


def sync_directory(path):
    fd = os.open(path, os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def seal(root, context):
    validate_tables(root, context)
    assets = inventory(root)
    marker = dict(schema_version='openarm-export-complete/v1', contract_version=CONTRACT,
                  status='COMPLETE', completed_at=utc(), assets=assets,
                  content_sha256=hashlib.sha256(canonical(assets)).hexdigest(),
                  **{k: context[k] for k in ('export_id', 'session_id', 'collection_task_id', 'robot_id', 'synthetic')})
    schema_check(marker, 'export-complete')
    for item in assets:
        with (root / item['path']).open('rb') as stream:
            os.fsync(stream.fileno())
    atomic_json(root / 'export-complete.json', marker)
    for path in sorted((p for p in root.rglob('*') if p.is_dir()), reverse=True):
        sync_directory(path)
    sync_directory(root)
    return marker
