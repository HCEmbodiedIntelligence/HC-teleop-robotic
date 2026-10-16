"""Profile portability and fail-closed client input checks; no hardware access."""
from pathlib import Path
from types import MethodType, SimpleNamespace
import time

import pytest

pytest.importorskip('rclpy')
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String

from examples.robot_api_demo import (
    RobotDemo, load_profile, joint_values, make_target,
    STATE_TOPIC, READY_TOPIC, SOURCE_TOPIC, PERMIT_TOPIC,
)


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('robot,joint', [
    ('openarmx', 'left_joint_4'), ('x1', 'left_joint_4'),
])
def test_same_client_reads_each_profile_and_maps_by_name(robot, joint):
    profile = load_profile(ROOT / 'adapters/robots' / robot)
    names = profile['groups']['left_arm']
    feedback = JointState()
    feedback.name = list(reversed(names))
    feedback.position = [0.] * len(names)
    feedback.velocity = [0.] * len(names)
    args = SimpleNamespace(group='left_arm', joint=joint, delta=.02, positions=None)
    target = make_target(names, joint_values(feedback, True), args, profile['params'])
    assert dict(zip(names, target))[joint] == .02
    assert sum(target) == .02
    assert next(c for c in profile['channels'] if c['kind'] == 'move_j' and c['group'] == 'left_arm')['endpoint'] == (
        '/control/motion/left_arm/move_j')


def test_mapping_is_not_fixed_to_seven_joints_or_left_right_arms():
    args = SimpleNamespace(group='manipulator', joint='elbow', delta=.1, positions=None)
    assert make_target(['shoulder', 'elbow'], {'elbow': .2, 'shoulder': .3}, args, {}) == [
        .3, pytest.approx(.3)]


@pytest.mark.parametrize('target', [[0.], [float('nan'), 0.], [2., 0.]])
def test_invalid_length_nan_and_out_of_limit_targets_rejected(target):
    args = SimpleNamespace(group='arm', positions=target)
    with pytest.raises(ValueError):
        make_target(['a', 'b'], {}, args, {
            'group_lower_limits.arm': [-1., -1.], 'group_upper_limits.arm': [1., 1.]})


@pytest.mark.parametrize('names,velocity', [(['a', 'a'], [0., 0.]), (['a', 'b'], [])])
def test_invalid_feedback_cannot_be_used_for_motion(names, velocity):
    message = JointState(name=names, position=[0., 0.], velocity=velocity)
    with pytest.raises(ValueError):
        joint_values(message, require_velocity=True)


def client_state():
    message = JointState(name=['a'], position=[0.], velocity=[0.])
    message.header.stamp.sec = 100
    permit = Header(frame_id='motion:1')
    permit.stamp.sec = 100
    node = SimpleNamespace(
        profile={'params': {}}, state_topic=STATE_TOPIC, permit_topic=PERMIT_TOPIC,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        latest={topic: (value, time.monotonic()) for topic, value in {
            STATE_TOPIC: message, READY_TOPIC: Bool(data=True),
            SOURCE_TOPIC: String(data='motion'), PERMIT_TOPIC: permit,
        }.items()})
    node.recent = MethodType(RobotDemo.recent, node)
    return node


def test_motion_requires_ready_matching_names_source_and_fresh_permit():
    node = client_state()
    assert RobotDemo.gate_problem(node, ['a']) is None
    assert RobotDemo.gate_problem(node, ['missing']) is not None
    node.latest[READY_TOPIC][0].data = False
    assert 'hardware ready' in RobotDemo.gate_problem(node, ['a'])
    node = client_state()
    node.latest[SOURCE_TOPIC][0].data = 'vr'
    assert 'motion' in RobotDemo.gate_problem(node, ['a'])
    node = client_state()
    node.latest[PERMIT_TOPIC][0].stamp.sec = 99
    assert '许可' in RobotDemo.gate_problem(node, ['a'])
