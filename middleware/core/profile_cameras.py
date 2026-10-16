"""One physical-camera configuration for launch, preview and HC recording."""
from pathlib import Path
import yaml


def apply_profile_cameras(config, profile_dir, *, simulation=False):
    path = Path(profile_dir) / 'cameras.yaml'
    if simulation or not config.get('camera', {}).get('enabled', False) or not path.is_file():
        return
    from humanoid_camera.configuration import validate_cameras
    document = yaml.safe_load(path.read_text())
    if document.get('schema_version') != 1:
        raise ValueError('profile cameras.yaml requires schema_version: 1')
    cameras = validate_cameras(document.get('cameras'))
    subscriptions = config['ros'].setdefault('subscriptions', [])
    # A physical profile replaces the generic X1/legacy camera topic selection.
    # Keep non-camera subscriptions and TF exactly as configured.
    subscriptions[:] = [s for s in subscriptions if not (
        s.get('type', '').startswith(('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
                                      'sensor_msgs/msg/CameraInfo', 'realsense2_camera_msgs/'))
        or '/cameras/' in s.get('topic', ''))]
    streams = []
    for camera in cameras:
        if not camera['enabled']:
            continue
        if camera['backend'] != 'realsense':
            raise ValueError('profile camera recording currently requires the RealSense provider')
        prefix = f"/{camera['namespace']}/{camera['camera_name']}"
        for stream, key in [('color', 'rgb_topic'), ('depth', 'depth_topic')]:
            if stream == 'depth' and not camera.get('depth_enabled', True):
                continue
            for topic, msg_type in [(camera[key], 'Image'), (prefix + '/' + stream + '/camera_info', 'CameraInfo')]:
                subscriptions.append(dict(topic=topic, type='sensor_msgs/msg/' + msg_type,
                    enabled=True, outputs=['record'], required=camera['required'],
                    recording_mode='continuous', max_hz=0., event_max_hz=0.))
        streams.append(dict(id=camera['id'], name=camera['id'], topic=camera['rgb_topic'],
                            type='sensor_msgs/msg/Image', enabled=True, stereo=False,
                            width=camera['width'], height=camera['height'], fps=camera['fps']))
    config['camera']['streams'] = streams
    config['camera']['enabled'] = bool(streams)
