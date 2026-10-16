"""Generate raw capture settings from the current public robot and camera APIs."""
from pathlib import Path
from xml.etree import ElementTree

import yaml

from .robot_api import RobotApiProfile, ENDPOINTS
from .capture.config import validate
from .capture.profile import robot_profile


def capture_config(profile_dir, cameras_file=None):
    from humanoid_camera.configuration import validate_cameras
    from humanoid_camera.settings import resolved_document
    from humanoid_camera.depth_source import installed_driver_version, realsense_ros_depth_profile
    from .capture.originals import camera_topics

    api = RobotApiProfile(profile_dir)
    path = Path(cameras_file) if cameras_file else api.directory / 'cameras.yaml'
    cameras = [resolved_document(c, c) for c in validate_cameras(yaml.safe_load(path.read_text())['cameras'])]
    mimic = {j.get('name') for j in ElementTree.fromstring(api.urdf_path.read_bytes()).findall('joint')
             if j.find('mimic') is not None}
    joints = [j for name, j in api.joints.items() if name not in mimic]
    arms = [j for j in joints if j['unit'] == 'rad']
    grippers = [j for j in joints if j['unit'] == 'm']
    driver = [dict(joint_names=[j['public_name'] for j in arms], position_units=[j['unit'] for j in arms])] if arms else []
    hand = [dict(gripper_names=[j['public_name'] for j in grippers], position_units=[j['unit'] for j in grippers])] if grippers else []
    profile = robot_profile(api.profile_id, driver, hand, cameras)
    sources = {}
    for c in cameras:
        if not c['enabled']:
            continue
        if c['backend'] != 'realsense' or not c.get('timestamp_alignment', True):
            raise ValueError(c['id'] + ': raw 录制需要提供曝光元数据的 RealSense 标准化输出')
        sources[c['id']] = dict(kind='rgbd', transport='ros_rgbd', required=c['required'], fps=c['fps'],
            camera_model=c['device_type'], topic=c['rgbd_topic'], metadata_topic=c['metadata_topic'],
            packet_topic='/' + c['namespace'] + '/normalized/capture',
            original_topics=camera_topics(c, bundled=True), depth_enabled=c.get('depth_enabled', True),
            require_exposure_metadata=True, exposure={'max_actual_us': c.get('max_actual_exposure_us', 5000)},
            camera_max_error_ms=c['camera_max_error_ms'], rgbd_max_midpoint_skew_ms=c['rgbd_max_midpoint_skew_ms'])
        depth_profile = realsense_ros_depth_profile(installed_driver_version(), c.get('depth_format'))
        if c.get('depth_enabled', True) and depth_profile is not None:
            sources[c['id']]['depth_source'] = depth_profile
    axes = [dict(axis, **{k: profile['validity'][axis['group']][k] for k in ('mode', 'max_age_ms')})
            for axis in profile['axes']]
    for kind, topic in [('state', ENDPOINTS['joint_states']), ('action', ENDPOINTS['joint_commands'])]:
        sources['robot_' + kind] = dict(kind=kind, transport='ros_joint_state', required=True, topic=topic,
            timestamp_semantics='read' if kind == 'state' else 'send', action_stage='sent', named_axes=axes,
            fields={'position': {'semantics': 'continuous', 'units': [axis['unit'] for axis in axes], 'coordinate_frame': 'joint'}})
    required = [c for c in cameras if c['enabled'] and c['required']]
    if not required:
        raise ValueError('主相机时间轴需要至少一台启用且必需的相机')
    primary = next((c['id'] for c in required if c['device_type'].lower().startswith('d435')), required[0]['id'])
    return validate(dict(capture={'profile': profile}, sources=sources,
        alignment=dict(primary_camera=primary, timeline_mode='primary_camera', state_reference='primary_exposure_midpoint',
                       camera_timing_reference='per_camera_rgb', derive_action_from_next_state=False, require_camera_sync=False),
        buffers={'retention_ms': 800},
        storage=dict(directory='sessions', export_directory='sealed', retain_original_ros=False, depth_video_encoding=False)))
