import json
from types import SimpleNamespace

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from builtin_interfaces.msg import Time

from middleware.core.dataset_topics import DatasetTopics


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


def test_camera_metadata_is_relayed_unchanged_to_standard_topics():
    b = bridge()
    sources = dict(b.subscriptions)
    calibration = String(data='{"schema":"hc_camera_calibration/v1"}')
    extrinsics = String(data='{"schema":"hc_robot_camera_extrinsics/v1"}')

    sources['/io_teleop/camera_head/calibration'](calibration)
    sources['/io_teleop/camera_head/extrinsics'](extrinsics)

    assert b.publishers['/observation/cameras/head/calibration'].messages == [calibration]
    assert b.publishers['/observation/cameras/head/extrinsics'].messages == [extrinsics]


def test_force_and_vibration_feedback_are_relayed_unchanged():
    b = bridge()
    sources = dict(b.subscriptions)
    wrenches = Float64MultiArray(data=list(range(12)))
    vibration = Float64MultiArray(data=list(range(10)))

    sources['/io_teleop/state_ee_wrenches'](wrenches)
    sources['/io_teleop/vibration_feedback_exo'](vibration)

    assert b.publishers['/observation/ee_wrenches'].messages == [wrenches]
    assert b.publishers['/observation/hands/vibration_feedback'].messages == [vibration]


def test_config_records_all_projected_topics_and_raw_vr():
    from pathlib import Path
    import yaml
    from middleware.core.config import ConfigStore
    config = ConfigStore(Path(__file__).parents[1] / 'middleware/config.yaml').load()
    from middleware.core.dataset_topics import load_joint_groups
    config['ros']['dataset_topics']['joint_groups'] = load_joint_groups(config, Path(__file__).parents[1] / 'middleware')
    b = DatasetTopics(Node(), config['ros']['dataset_topics'])
    recorded = {i['topic'] for i in config['ros']['subscriptions'] if i['enabled'] and 'record' in i['outputs']}
    assert set(b.publishers) <= recorded
    assert {'/vrdata', '/clock'} <= recorded


def test_no_projection_subscribes_to_its_own_output():
    b = bridge()
    assert not ({topic for topic, callback in b.subscriptions} & set(b.publishers))


def test_groups_follow_every_bundled_profile():
    from pathlib import Path
    import yaml
    from middleware.core.dataset_topics import load_joint_groups
    root = Path(__file__).parents[1]
    config = yaml.safe_load((root / 'middleware/config.yaml').read_text())
    profiles = sorted(path.parent.name for path in (root / 'adapters/robots').glob('*/arm_teleop.yaml'))
    assert {'x1', 'openarmx'} <= set(profiles)
    for profile in profiles:
        config['robot_profiles']['active'] = profile
        groups = load_joint_groups(config, root / 'middleware')
        source = yaml.safe_load((root / 'adapters/robots' / profile / 'arm_teleop.yaml').read_text())
        assert groups['left_arm'] == source['arms']['left']['joint_names']
        assert groups['right_arm'] == source['arms']['right']['joint_names']
        assert bool(groups.get('waist')) == bool(source.get('body', {}).get('waist_joint_names'))


def test_groups_follow_profile_without_waist(tmp_path, monkeypatch):
    from pathlib import Path
    import yaml
    from middleware.core.dataset_topics import load_joint_groups

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
