import json
from types import SimpleNamespace
import pytest

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from builtin_interfaces.msg import Time

from middleware.core.recording.dataset_topics import DatasetTopics


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class Node:
    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, cls, topic, callback, qos):
        return topic, callback

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=42)))


def bridge():
    return DatasetTopics(Node(), {'joint_groups': {
        'left_arm': ['Joint1_L', 'Joint2_L'], 'waist': ['leg_1', 'leg_2', 'zhi']}})


def test_joint_split_reorders_by_name_preserves_empty_arrays_and_header():
    b = bridge()
    msg = JointState(name=['Joint2_L', 'Joint1_L', 'zhi'], position=[2., 1., 3.])
    msg.header.stamp.sec = 7
    b.joints('action', msg)
    out = b.publishers['/action/left_arm/joint_states'].messages[0]
    assert list(out.position) == [1., 2.]
    assert not out.velocity and not out.effort
    assert out.header.stamp.sec == 7
    assert not b.publishers['/action/waist/joint_states'].messages


def test_pose_sides_and_frame_preserved():
    b = bridge()
    msg = PoseArray(poses=[Pose(), Pose()])
    msg.header.frame_id = 'zhi_Link'
    msg.poses[0].position.x = 1.
    msg.poses[1].position.x = 2.
    b.poses('observation', msg)
    assert b.publishers['/observation/right_arm/ee_pose'].messages[0].pose.position.x == 1.
    out = b.publishers['/observation/left_arm/ee_pose'].messages[0]
    assert out.pose.position.x == 2. and out.header.frame_id == 'zhi_Link'


def test_base_velocity_order_and_invalid_input():
    b = bridge()
    b.base(Float64MultiArray(data=[.3, .4, .5]))
    b.base(Float64MultiArray(data=[float('nan'), 0., 0.]))
    out = b.publishers['/action/base/velocity'].messages
    assert len(out) == 1
    assert (out[0].twist.angular.z, out[0].twist.linear.x, out[0].twist.linear.y) == (.3, .4, .5)
    assert out[0].header.stamp.sec == 42


def test_head_tracking_and_raw_coordinate_preservation():
    b = bridge()
    data = {'tracking': {'head': True}, 'poses': {'head': {
        'position': [1., 2., 3.], 'quaternion': [0., 0., 0., 1.]}}}
    b.head(String(data=json.dumps(data)))
    data['tracking']['head'] = False
    b.head(String(data=json.dumps(data)))
    b.head(String(data='invalid'))
    out = b.publishers['/observation/headset/pose'].messages
    assert len(out) == 1
    assert out[0].header.frame_id == 'vr_tracking'
    assert out[0].pose.position.y == 2.


def test_robot_head_actual_and_target_are_split_with_frame_preserved():
    b = bridge()
    sources = dict(b.subscriptions)
    msg = PoseArray(poses=[Pose(), Pose()])
    msg.header.frame_id = 'robot_base'
    msg.header.stamp.sec = 9
    msg.poses[0].position.x = 1.
    msg.poses[1].position.x = 2.

    sources['/status/head/tracking_poses'](msg)

    actual = b.publishers['/observation/head/pose'].messages[0]
    target = b.publishers['/action/head/pose'].messages[0]
    assert actual.pose.position.x == 1.
    assert target.pose.position.x == 2.
    assert actual.header.frame_id == target.header.frame_id == 'robot_base'
    assert actual.header.stamp.sec == target.header.stamp.sec == 9


def test_camera_metadata_is_relayed_unchanged_to_standard_topics():
    b = bridge()
    sources = dict(b.subscriptions)
    calibration = String(data='{"schema":"hc_camera_calibration/v1"}')
    extrinsics = String(data='{"schema":"hc_robot_camera_extrinsics/v1"}')

    sources['/io_teleop/camera_head/calibration'](calibration)
    sources['/io_teleop/camera_head/extrinsics'](extrinsics)

    assert b.publishers['/observation/cameras/head/calibration'].messages == [calibration]
    assert b.publishers['/observation/cameras/head/extrinsics'].messages == [extrinsics]


def test_native_force_and_retired_vibration_topics_are_not_relayed():
    b = bridge()
    sources = dict(b.subscriptions)

    assert '/io_teleop/state_ee_wrenches' not in sources
    assert '/observation/ee_wrenches' not in b.publishers
    assert '/io_teleop/vibration_feedback_exo' not in sources
    assert '/observation/hands/vibration_feedback' not in b.publishers


def test_config_offers_projected_topics_and_raw_vr_for_page_selection():
    from pathlib import Path
    from middleware.core.config import ConfigStore
    config = ConfigStore(Path(__file__).parents[1] / 'middleware/config.yaml').load()
    from middleware.core.recording.dataset_topics import load_joint_groups
    config['ros']['dataset_topics']['joint_groups'] = load_joint_groups(config, Path(__file__).parents[1] / 'middleware')
    b = DatasetTopics(Node(), config['ros']['dataset_topics'])
    # Presence in the topic catalogue does not force the page to record it.
    available = {i['topic'] for i in config['ros']['subscriptions']}
    assert set(b.publishers) <= available
    assert '/vrdata' in available


def test_no_projection_subscribes_to_its_own_output():
    b = bridge()
    assert not ({topic for topic, callback in b.subscriptions} & set(b.publishers))


@pytest.mark.parametrize('backend', ['v23', 'motion'])
def test_groups_follow_every_bundled_profile(backend, monkeypatch):
    from pathlib import Path
    import yaml
    from middleware.core.recording.dataset_topics import load_joint_groups
    monkeypatch.setenv('HC_CONTROL_BACKEND', backend)
    monkeypatch.delenv('HC_ROBOT_NAME', raising=False)
    monkeypatch.delenv('HC_ROBOT_CONFIG_ROOT', raising=False)
    root = Path(__file__).parents[1]
    config = yaml.safe_load((root / 'middleware/config.yaml').read_text())
    profiles = sorted(path.parent.name for path in (root / 'adapters/robots').glob('*/arm_teleop.yaml'))
    assert {'x1', 'openarmx'} <= set(profiles)
    for profile in profiles:
        config['robot_profiles']['active'] = profile
        groups = load_joint_groups(config, root / 'middleware')
        source = yaml.safe_load((root / 'adapters/robots' / profile / 'arm_teleop.yaml').read_text())
        for side in ('left', 'right'):
            expected = source['arms'][side]['joint_names']
            if backend == 'motion':
                expected = [f'{side}_joint_{i}' for i in range(1,len(expected)+1)]
            assert groups[side + '_arm'] == expected
        assert bool(groups.get('waist')) == bool(source.get('body', {}).get('waist_joint_names'))


def test_groups_follow_profile_without_waist(tmp_path, monkeypatch):
    from pathlib import Path
    import yaml
    from middleware.core.recording.dataset_topics import load_joint_groups

    root = Path(__file__).parents[1]
    config = yaml.safe_load((root / 'middleware/config.yaml').read_text())
    profile = tmp_path / 'arms_only'
    profile.mkdir()
    source = {'arms': {
        'left': {'joint_names': ['left_1', 'left_2']},
        'right': {'joint_names': ['right_1', 'right_2']},
    }, 'body': {'enabled': False, 'waist_joint_names': []}}
    (profile / 'arm_teleop.yaml').write_text(yaml.safe_dump(source))
    monkeypatch.setenv('HC_ROBOT_CONFIG_ROOT', str(tmp_path))
    config['robot_profiles']['active'] = 'arms_only'

    groups = load_joint_groups(config, root / 'middleware')
    assert groups == {
        'left_arm': ['left_1', 'left_2'],
        'right_arm': ['right_1', 'right_2'],
    }


def test_skips_construction_and_publish_when_subscription_count_is_zero():
    class CountingPublisher(Publisher):
        def __init__(self, count):
            super().__init__()
            self._count = count

        def get_subscription_count(self):
            return self._count

    class CountingNode(Node):
        def __init__(self, active_topics):
            self.active_topics = set(active_topics)

        def create_publisher(self, cls, topic, qos):
            return CountingPublisher(1 if topic in self.active_topics else 0)

    active = {'/action/left_arm/joint_states', '/observation/right_arm/ee_pose', '/action/head/pose'}
    b = DatasetTopics(
        CountingNode(active),
        {'joint_groups': {'left_arm': ['Joint1_L', 'Joint2_L'], 'waist': ['leg_1', 'leg_2', 'zhi']}},
    )

    js = JointState(name=['Joint1_L', 'Joint2_L', 'leg_1', 'leg_2', 'zhi'], position=[1., 2., 3., 4., 5.])
    b.joints('action', js)
    assert len(b.publishers['/action/left_arm/joint_states'].messages) == 1
    assert len(b.publishers['/action/waist/joint_states'].messages) == 0

    pa = PoseArray(poses=[Pose(), Pose()])
    b.poses('observation', pa)
    assert len(b.publishers['/observation/right_arm/ee_pose'].messages) == 1
    assert len(b.publishers['/observation/left_arm/ee_pose'].messages) == 0

    b.robot_head(pa)
    assert len(b.publishers['/action/head/pose'].messages) == 1
    assert len(b.publishers['/observation/head/pose'].messages) == 0

    b.base(Float64MultiArray(data=[0.1, 0.2, 0.3]))
    assert len(b.publishers['/action/base/velocity'].messages) == 0

    b.head(String(data='not-json'))
    assert len(b.publishers['/observation/headset/pose'].messages) == 0


def test_raw_subscription_skips_deserialization_until_subscriber_present(monkeypatch):
    import time
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.qos import qos_profile_sensor_data
    import rclpy.serialization
    from middleware.core.control.ros_executor import CachedWaitSetExecutor

    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '231')
    orig_deserialize = rclpy.serialization.deserialize_message
    deserialize_calls = []

    def tracking_deserialize(raw, cls):
        deserialize_calls.append(cls)
        return orig_deserialize(raw, cls)

    monkeypatch.setattr(rclpy.serialization, 'deserialize_message', tracking_deserialize)
    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = rclpy.create_node('test_dataset_raw_filter', context=context,
                             enable_rosout=False, start_parameter_services=False)
    executor = CachedWaitSetExecutor(context=context)
    executor.add_node(node)
    src_pub = node.create_publisher(JointState, '/observation/joint_states', qos_profile_sensor_data)
    b = DatasetTopics(node, {'joint_groups': {'left_arm': ['Joint1_L', 'Joint2_L']}})
    received = []
    sub = None
    try:
        # With zero downstream subscribers, raw bytes must not be deserialized.
        assert b.publishers['/observation/left_arm/joint_states'].get_subscription_count() == 0
        for _ in range(3):
            src_pub.publish(JointState(name=['Joint1_L', 'Joint2_L'], position=[0.5, 0.6]))
            executor.spin_once(timeout_sec=0.02)
        assert deserialize_calls == []

        # Once a subscriber connects, messages deserialize and split normally.
        sub = node.create_subscription(JointState, '/observation/left_arm/joint_states',
                                       received.append, 10)
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            src_pub.publish(JointState(name=['Joint2_L', 'Joint1_L'], position=[2.0, 1.0]))
            executor.spin_once(timeout_sec=0.02)
        assert received
        assert list(received[-1].position) == [1.0, 2.0]
        assert deserialize_calls
    finally:
        if sub is not None:
            node.destroy_subscription(sub)
        node.destroy_publisher(src_pub)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        context.try_shutdown()
