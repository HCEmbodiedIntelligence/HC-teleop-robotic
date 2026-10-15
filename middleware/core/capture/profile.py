"""Versioned profile resolution; no device serials, global config or user paths."""
import copy
import json
import re
from pathlib import Path

import jsonschema

CONTRACT = 'openarm-capture/v1'
CONTRACTS = Path(__file__).with_name('contracts')


def schema_check(value, name):
    schema = json.loads((CONTRACTS / (name + '.schema.json')).read_text())
    jsonschema.Draft202012Validator(schema, format_checker=jsonschema.FormatChecker()).validate(value)


def openarm_profile():
    return json.loads((CONTRACTS / 'openarmx-v1.json').read_text())


def robot_profile(robot_type, driver_parameters, gripper_parameters, cameras):
    """Describe standard runtime outputs using the robot's deployed configuration.

    Motion joints already use logical radians at the driver ABI. Grippers declare
    their SI position units. No vendor names, axis count or camera IDs are assumed.
    The existing upload contract currently supports a 30 Hz derived dataset.
    """
    axes = []
    for group, instances, key in (
            ('arm', driver_parameters, 'joint_names'),
            ('gripper', gripper_parameters, 'gripper_names')):
        for params in instances:
            names = params.get(key, [])
            units = params.get('position_units', ['rad'] * len(names) if group == 'arm' else [])
            if not names or len(units) != len(names):
                raise ValueError('机器人驱动必须声明轴名称和对应的位置单位')
            axes.extend({'name': name, 'unit': unit, 'group': group}
                        for name, unit in zip(names, units))
    required = [camera for camera in cameras if camera['enabled'] and camera['required']]
    if not required:
        raise ValueError('请在机器人相机配置中启用至少一个必需图像来源')
    for camera in required:
        if any(type(camera.get(key)) is not int or camera[key] < 1 for key in ('width', 'height')):
            raise ValueError(camera['id'] + ': 请在机器人相机配置中填写图像宽度和高度')
    invalidations = ['pause', 'input_loss', 'clutch_release', 'rebind',
                     'clock_epoch_change', 'source_restart']
    profile = {'profile_version': 'robot-config-v1', 'robot_type': robot_type, 'fps': 30,
               'axes': axes,
               'cameras': [{'id': c['id'], 'width': c['width'], 'height': c['height'],
                            'calibration_ref': c.get('calibration_ref') or 'unknown:not-provided'}
                           for c in required],
               'action_stage': 'sent', 'command_boundary': 'motion_or_teleop_to_driver_request',
               'validity': {group: {'mode': 'periodic', 'max_age_ms': age,
                                    'invalidate_on': list(invalidations)}
                            for group, age in [('arm', 100), ('gripper', 150)]}}
    schema_check(profile, 'profile')
    if len({axis['name'] for axis in axes}) != len(axes):
        raise ValueError('机器人关节与夹爪轴名称不能重复')
    return profile


def validate_capture_config(cfg):
    profile = cfg['capture']['profile']
    schema_check(profile, 'profile')
    names = [a['name'] for a in profile['axes']]
    cameras = [c['id'] for c in profile['cameras']]
    if len(set(names)) != len(names) or len(set(cameras)) != len(cameras):
        raise ValueError('Duplicate profile identity')
    # G0 source-mapping schema fixes time_base to 1/30, including generic profiles.
    if profile['fps'] != 30 or cfg['dataset']['fps'] != profile['fps']:
        raise ValueError('openarm-capture/v1 requires 30 Hz; other rates need a contract revision')
    if profile['profile_version'] == 'openarmx-v1' and profile != openarm_profile():
        raise ValueError('OpenArm profile differs from frozen G0; request E revision')
    for kind in ('state', 'action'):
        actual = [a for s in cfg['sources'].values() if s['kind'] == kind and s['required']
                  for a in s.get('named_axes', [])]
        if len(actual) != len(names) or {a['name']: a['unit'] for a in actual} != {a['name']: a['unit'] for a in profile['axes']}:
            raise ValueError('Every profile axis needs exactly one named ' + kind + ' source')
        for axis in actual:
            expected = next(a for a in profile['axes'] if a['name'] == axis['name'])
            rule = profile['validity'][expected['group']]
            if axis['max_age_ms'] != rule['max_age_ms'] or axis.get('mode', 'periodic') != rule['mode']:
                raise ValueError('Axis validity differs from profile')
    actual_cameras = {n for n, s in cfg['sources'].items() if s['kind'] == 'rgbd' and s['required']}
    if actual_cameras != set(cameras):
        raise ValueError('Required cameras differ from profile')


def resolve_capture_config(profile, driver_parameters, gripper_parameters, camera_sources, *, alignment, settings=None):
    """Consume *resolved* runtime ros__parameters and manager camera sources.

    E resolves plugin parameters first. Lists support multiple driver instances.
    The caller explicitly chooses measured association tolerances (milliseconds).
    """
    from .config import validate
    cfg = {'capture': {'profile': copy.deepcopy(profile)}, 'dataset': {'fps': profile['fps']},
           'alignment': copy.deepcopy(alignment), 'sources': copy.deepcopy(camera_sources)}
    for section, value in (settings or {}).items():
        if section not in {'dataset', 'buffers', 'storage'}:
            raise ValueError('Unsupported recording policy: ' + section)
        cfg[section] = copy.deepcopy(value)
    axes = {a['name']: a for a in profile['axes']}
    for group, instances, names_key, topic_prefix in (
            ('arm', driver_parameters, 'joint_names', 'platform_joint_'),
            ('gripper', gripper_parameters, 'gripper_names', 'platform_gripper_')):
        for i, params in enumerate(instances):
            names = [n for n in params[names_key] if n in axes]
            if not names:
                continue
            units = params.get('position_units', ['rad'] * len(params[names_key]))
            if any(units[params[names_key].index(n)] != axes[n]['unit'] for n in names):
                raise ValueError('Runtime units differ from profile')
            for kind, suffix in (('state', 'state_topic'), ('action', 'command_topic')):
                ident = f'{group}_{i}_{kind}'
                if ident in cfg['sources']:
                    raise ValueError('机器人相机与标准数值来源 ID 冲突: ' + ident)
                cfg['sources'][ident] = {
                    'kind': kind, 'transport': 'ros_joint_state', 'required': True,
                    'topic': params[topic_prefix + suffix], 'timestamp_semantics': 'send' if kind == 'action' else 'read',
                    'action_stage': 'sent', 'named_axes': [dict(axes[n], **{
                        k: profile['validity'][axes[n]['group']][k] for k in ('max_age_ms', 'mode')}) for n in names],
                    'fields': {'position': {'semantics': 'continuous', 'units': [axes[n]['unit'] for n in names],
                                            'coordinate_frame': 'joint'}}}
    return validate(cfg)


def freeze_context(cfg, metadata):
    allowed = {'local_robot_id', 'robot_id', 'collection_task_id', 'model_sha256', 'configuration_sha256', 'synthetic'}
    if set(metadata) - allowed:
        raise ValueError('Unknown capture identity field (credentials must never enter a bundle)')
    value = copy.deepcopy(metadata)
    for key in ('local_robot_id', 'model_sha256', 'configuration_sha256'):
        if not isinstance(value.get(key), str) or not value[key]:
            raise ValueError('Missing frozen capture field: ' + key)
    for key in ('model_sha256', 'configuration_sha256'):
        if not re.fullmatch('[a-f0-9]{64}', value[key]):
            raise ValueError(key + ' must hash the resolved source snapshot')
    if type(value.get('synthetic')) is not bool:
        raise ValueError('Capture must explicitly declare synthetic')
    for key in ('robot_id', 'collection_task_id'):
        if key in value and (not isinstance(value[key], str) or not value[key].strip()):
            raise ValueError('Invalid capture identity: ' + key)
    if ('robot_id' in value) != ('collection_task_id' in value):
        raise ValueError('Provide both robot/task identity or leave both offline')
    value['profile'] = copy.deepcopy(cfg['capture']['profile'])
    value['contract_version'] = CONTRACT
    value['physical_sync'] = 'unknown'
    return value
