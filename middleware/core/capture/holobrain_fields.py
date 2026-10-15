"""Public recording fields, shared by the live MCAP writer and dataset writer.

Only identities/order are specified here. Values retain their actual robot units;
no HC-TJ calibration, gripper normalization or fabricated observation is applied.
"""
SCHEMA = 'holobrain-recording-fields/v1'
STATE_TOPIC = 'io_teleop/joint_states'
ACTION_TOPIC = 'io_teleop/joint_cmd'
# The deployed D435 has the historical ID camera_hand (camera manual, section 4).
# Other camera positions are not inferred from their device model.
CAMERAS = {'camera_hand': 'head', 'camera_front': 'head'}
AXES = [(f'openarmx_{side}_joint{i}', f'Joint{i}_{suffix}')
        for side, suffix in [('right', 'R'), ('left', 'L')] for i in range(1, 8)]
AXES.insert(7, ('right_gripper', 'R_ban'))
AXES.append(('left_gripper', 'L_ban'))


def camera_name(ident):
    return CAMERAS.get(ident, ident)


def image_key(ident, depth=False):
    return 'observation.images.' + camera_name(ident) + ('_depth' if depth else '')


def camera_topic(ident, depth=False):
    name = camera_name(ident)
    camera = name if name.startswith('camera_') else 'camera_' + name
    return '/io_teleop/' + camera + ('/depth' if depth else '/color')


def axes(profile):
    original = profile['axes']
    by_name = {a['name']: a for a in original}
    if set(by_name) == {name for name, _ in AXES}:
        return [{**by_name[source], 'source_name': source, 'name': name} for source, name in AXES]
    return [{**a, 'source_name': a['name']} for a in original]


def vector(row, kind, cfg):
    from .completion import components
    physical = cfg['capture']['profile']['axes']
    by_name = {a['name']: p for a, p in zip(physical, components(row, kind, cfg))}
    public = axes(cfg['capture']['profile'])
    parts = [by_name[a['source_name']] for a in public]
    if not all(p['valid'] for p in parts):
        raise ValueError('Invalid named vector cannot be exported')
    return [p['values'] for p in parts], [a['name'] for a in public]


def source_timestamp(row, cfg):
    if cfg['alignment'].get('state_reference')=='primary_exposure_midpoint':
        stamp=row.get('association_time_ns')
        if type(stamp) is not int or stamp<=0:
            raise ValueError('Primary RGB exposure midpoint is missing')
        return stamp
    # The reference column means the primary RGB's actual source header time,
    # not frame_index/fps, the alignment target or the file write time.
    images = row.get('images', {})
    primary = next((ident for ident in images if camera_name(ident) == 'head'), None)
    if not primary:
        primary = cfg['alignment'].get('primary_camera')
    if not primary and images:
        primary = sorted(images)[0]
    if primary:
        rgb = images[primary]['rgb']
        for key in ('header_timestamp_ns', 'source_timestamp_ns', 'capture_time_ns'):
            if key in rgb:
                return int(rgb[key])
        raise ValueError('Primary RGB is missing its source timestamp')
    return int(row['target_time_ns'])


def _depth_scale(source):
    import math
    from .depth_source import validate_profile

    if source.get('depth_source') is not None:
        try:
            return validate_profile(source['depth_source'])['scale_m_per_unit']
        except (ValueError, TypeError, KeyError):
            return None
    value = source.get('depth_scale')
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0:
        return value
    return None


def description(cfg):
    profile = cfg.get('capture', {}).get('profile') if cfg.get('capture') else None
    cameras = [ident for ident, source in cfg['sources'].items() if source['kind'] == 'rgbd']
    keys = [image_key(ident, depth) for ident in cameras for depth in ((False, True) if cfg['sources'][ident].get('depth_enabled',True) else (False,))]
    if len(keys) != len(set(keys)):
        raise ValueError('Camera IDs collide in the public recording field namespace')
    return {'schema_version': SCHEMA,
            'axes': axes(profile) if profile else [],
            'cameras': [{'source_id': ident, 'rgb': image_key(ident), 'depth': image_key(ident, True) if source.get('depth_enabled',True) else None,
                         'rgb_topic': camera_topic(ident), 'depth_topic': camera_topic(ident, True) if source.get('depth_enabled',True) else None,
                         'depth_scale_m_per_unit': _depth_scale(source) if source.get('depth_enabled',True) else None}
                        for ident, source in cfg['sources'].items() if source['kind'] == 'rgbd'],
            'state_topic': STATE_TOPIC, 'action_topic': ACTION_TOPIC,
            'source_timestamp': ('configured primary RGB exposure midpoint in ROS nanoseconds; state/action associated at this time'
                                 if cfg.get('alignment',{}).get('state_reference')=='primary_exposure_midpoint' else
                                 'head RGB, otherwise configured primary RGB; original source header nanoseconds'),
            'derived_fields': 'observation.holobrain.* requires actual robot calibration and preprocessing'}
