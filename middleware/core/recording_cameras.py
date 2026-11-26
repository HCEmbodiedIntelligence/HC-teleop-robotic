"""RGB feature names and selected ROS inputs shared with the HC LeRobot exporter."""
import re


def camera_feature(topic):
    return 'observation.images.' + re.sub(r'[^a-zA-Z0-9_]', '_', topic.strip('/').removesuffix('/compressed'))


def recording_cameras(ros_config):
    """Describe current recording inputs without opening camera subscriptions."""
    if not isinstance(ros_config, dict):
        return []
    if '_recording_cameras' in ros_config:
        return ros_config['_recording_cameras']
    selected = {
        item['topic']: item['type'] for item in ros_config.get('subscriptions', ())
        if item.get('enabled', True) and 'record' in item.get('outputs', ())
        and item.get('type') in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
        and 'depth' not in item['topic'].lower()
    }
    features = {}
    for topic, kind in selected.items():
        if topic + '/compressed' in selected:
            continue
        feature = camera_feature(topic)
        # The exporter rejects colliding feature names. Do not advertise either input.
        features[feature] = None if feature in features else dict(feature=feature, topic=topic, type=kind)
    return [camera for camera in features.values() if camera is not None]
