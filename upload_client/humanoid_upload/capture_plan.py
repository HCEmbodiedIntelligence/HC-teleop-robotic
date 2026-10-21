"""Validate and carry server-owned capture snapshots, never resolve defaults."""
import copy
import hashlib
import re
from datetime import datetime

from .common import UploadError, canonical


REFERENCE_FIELDS = ('plan_revision', 'plan_sha256', 'snapshot_sha256', 'recording_id')
OWNERSHIP_FIELDS = ('organization_id', 'project_id', 'region_code', 'collection_task_id', 'dataset_id')
# These columns are produced by the LeRobot writer, not device subscriptions.
# Actual emitted metadata is still validated before the artifact is sealed.
LEROBOT_GENERATED_FEATURES = {
    key: {'dtype': 'float32' if key == 'timestamp' else 'int64', 'shape': [1], 'names': None}
    for key in ('timestamp', 'frame_index', 'episode_index', 'index', 'task_index')
}
SNAPSHOT_FIELDS = set(OWNERSHIP_FIELDS) | {
    'schema_version', 'robot_id', 'recording_id', 'plan_revision', 'plan_sha256',
    'scheme', 'mapping_version', 'mapping_sha256', 'source_format', 'effective_adapter',
    'output', 'output_contract_sha256', 'source', 'prepared_at', 'snapshot_sha256',
}


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def require(condition):
    if not condition:
        raise ValueError('invalid snapshot')


def validate_snapshot(value, **expected):
    """Validate transport integrity and frozen identity, not server authorization."""
    try:
        require(isinstance(value, dict) and set(value) == SNAPSHOT_FIELDS)
        require(value['schema_version'] == 'capture-recording-snapshot/v1')
        for key in (*OWNERSHIP_FIELDS, 'robot_id', 'recording_id', 'source'):
            require(isinstance(value[key], str) and value[key].strip())
        for key in ('plan_revision', 'mapping_version'):
            require(type(value[key]) is int and value[key] > 0)
        for key in ('plan_sha256', 'mapping_sha256', 'output_contract_sha256', 'snapshot_sha256'):
            require(isinstance(value[key], str) and re.fullmatch('[a-f0-9]{64}', value[key]))
        scheme = value['scheme']
        require(set(scheme) == {'scheme_id', 'version', 'content_sha256'})
        require(isinstance(scheme['scheme_id'], str) and scheme['scheme_id'])
        require(type(scheme['version']) is int and scheme['version'] > 0)
        require(re.fullmatch('[a-f0-9]{64}', scheme['content_sha256']))
        require(value['source_format'] in ('MCAP', 'LEROBOT_V3', 'CAPTURE_BUNDLE'))
        adapter = value['effective_adapter']
        require(isinstance(adapter, dict) and adapter['source_format'] == value['source_format'])
        require(isinstance(adapter['configuration' if value['source_format'] != 'LEROBOT_V3' else 'feature_sources'], dict))
        output = value['output']
        require(output['frequency_policy'] == 'PRESERVE' and output['frequency_hz'] > 0)
        require(isinstance(output['features'], dict) and output['features'])
        require(datetime.fromisoformat(value['prepared_at'].replace('Z', '+00:00')).tzinfo)
        require(digest(output) == value['output_contract_sha256'])
        require(digest({k: v for k, v in value.items() if k != 'snapshot_sha256'}) == value['snapshot_sha256'])
    except (KeyError, ValueError, TypeError, AttributeError):
        raise UploadError('CAPTURE_SNAPSHOT_INVALID', '服务端录制快照不完整或摘要不一致；保留原录制 ID 并检查平台协议') from None
    if any(value.get(key) != item for key, item in expected.items()):
        raise UploadError('CAPTURE_SNAPSHOT_MISMATCH', '录制快照的任务、版本、格式或归属不一致；不能改投或使用默认方案')
    return copy.deepcopy(value)


def reference(snapshot):
    validate_snapshot(snapshot)
    return {key: snapshot[key] for key in REFERENCE_FIELDS}


def validate_lerobot_info(snapshot, info):
    """Compare actual definitions; optional absent features stay absent."""
    snapshot = validate_snapshot(snapshot, source_format='LEROBOT_V3')
    output, adapter = snapshot['output'], snapshot['effective_adapter']
    if info.get('fps') != output['frequency_hz']:
        raise UploadError('CAPTURE_FREQUENCY_MISMATCH', f"实际 {info.get('fps')} Hz 与方案 {output['frequency_hz']} Hz 不一致；不自动重采样")
    if info.get('codebase_version') != adapter.get('codebase_version', 'v3.0'):
        raise UploadError('CAPTURE_FORMAT_MISMATCH', 'LeRobot 版本与任务方案不一致')
    features = info.get('features', {})
    presence = {}
    for key, wanted in output['features'].items():
        source = adapter['feature_sources'].get(key, key)
        actual = features.get(source)
        presence[key] = actual is not None
        if actual is None:
            if wanted.get('required', True):
                raise UploadError('CAPTURE_REQUIRED_DATA_MISSING', f'缺少方案必需字段：{key}（来源 {source}）')
            continue
        for field in ('dtype', 'shape', 'names'):
            if wanted.get(field) is not None and actual.get(field) != wanted[field]:
                raise UploadError('CAPTURE_STRUCTURE_MISMATCH', f'{key}.{field} 与任务方案不一致')
        # Older info.json lacks units; never infer evidence from dimensions.
        if actual.get('units') is not None and actual['units'] != wanted.get('units'):
            raise UploadError('CAPTURE_STRUCTURE_MISMATCH', f'{key}.units 与任务方案不一致')
    return presence
