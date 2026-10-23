"""Check configured capture inputs and observed health without driving devices."""
import re

from humanoid_upload.common import UploadError
from humanoid_upload.capture_plan import LEROBOT_GENERATED_FEATURES, validate_snapshot


NUMERIC_TOPICS = {
    'observation.left_arm.ee_pose': '/observation/left_arm/ee_pose',
    'observation.right_arm.ee_pose': '/observation/right_arm/ee_pose',
    'observation.headset.pose': '/observation/headset/pose',
    'observation.base.pose': '/observation/base/pose',
    'action.left_arm.ee_pose': '/action/left_arm/ee_pose',
    'action.right_arm.ee_pose': '/action/right_arm/ee_pose',
    'action.base.velocity': '/action/base/velocity',
}


def camera_feature(topic):
    return 'observation.images.' + re.sub(r'[^a-zA-Z0-9_]', '_', topic.strip('/').removesuffix('/compressed'))


def check_device(snapshot, config, recorder, health):
    snapshot = validate_snapshot(snapshot)
    output, adapter = snapshot['output'], snapshot['effective_adapter']
    subscriptions = {item['topic']: item for item in config.get('ros', {}).get('subscriptions', [])
                     if item.get('enabled', True) and 'record' in item.get('outputs', [])}
    required = {}
    if snapshot['source_format'] in ('MCAP', 'CAPTURE_BUNDLE'):
        settings = adapter['configuration']
        roles = {feature['role'] for feature in output['features'].values() if feature.get('required', True)}
        if ('state' in roles and not settings.get('state_topic') or
                'action' in roles and not settings.get('action_topic') and not settings.get('derive_action_from_next_state')):
            raise UploadError('CAPTURE_SOURCE_UNSUPPORTED', '方案必需的状态或动作缺少明确话题映射；请在平台修复实例映射')
        for key in ('state_topic', 'action_topic'):
            if settings.get(key): required[settings[key]] = 'sensor_msgs/msg/JointState'
        for camera in settings.get('cameras', []):
            if camera.get('required', True): required[camera['topic']] = camera['message_type']
        for name, source in settings.get('numeric_sources', {}).items():
            if output['features'].get(name, {}).get('required', True):
                required[source['topic']] = source['message_type']
    else:
        sources = {'observation.state': '/observation/joint_states', 'action': '/action/joint_states', **NUMERIC_TOPICS}
        sources.update({camera_feature(topic): topic for topic, item in subscriptions.items()
                        if item['type'] in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')
                        and 'depth' not in topic.lower()})
        generated = {'observation.source_time', 'observation.recording_time',
                     'observation.source_available', 'observation.source_valid',
                     'observation.source_age', *LEROBOT_GENERATED_FEATURES}
        for key, feature in output['features'].items():
            source = adapter['feature_sources'].get(key, key)
            if not feature.get('required', True) or source in generated:
                continue
            topic = sources.get(source)
            if topic is None:
                raise UploadError('CAPTURE_SOURCE_UNSUPPORTED', f'本机导出器无法采集 {key}（来源 {source}）；请检查实例映射')
            required[topic] = ('sensor_msgs/msg/JointState' if source in ('observation.state', 'action')
                               else subscriptions.get(topic, {}).get('type'))
        clock_topic = '/observation/joint_states'
        required.setdefault(clock_topic, 'sensor_msgs/msg/JointState')
        required.setdefault('/action/joint_states', 'sensor_msgs/msg/JointState')
    for topic, message_type in required.items():
        item = subscriptions.get(topic)
        if item is None:
            raise UploadError('CAPTURE_SOURCE_NOT_RECORDING', f'方案要求 {topic}，本机未启用该话题录制；选择方案不会启动设备发布')
        if message_type and item.get('type') != message_type:
            raise UploadError('CAPTURE_STRUCTURE_MISMATCH', f'{topic} 消息类型与方案不一致')
        observed = health.get(topic, {})
        if not observed.get('has_data') or observed.get('state') in ('no_data', 'error'):
            raise UploadError('CAPTURE_REQUIRED_DATA_MISSING', f'未检测到方案必需话题 {topic} 的实时数据；请检查设备发布')
        # Rates and local recording limits are runtime settings, not scheme gates.
        # Historical snapshots may contain a sample rate; never require it here.
