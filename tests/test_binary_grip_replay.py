"""Drive replay of recorded public binary commands through native per-hand outputs."""
import json
from pathlib import Path
from unittest.mock import Mock

import pytest
from mcap.writer import Writer
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState

from middleware.core.robot_api import RobotApiProfile, SERVO_J_COMMANDED_STATE, SERVO_J_HAND_STATE
from middleware.core.ros_bridge import RosBridge
from middleware.core.topic_player import TopicPlayer

ROOT = Path(__file__).resolve().parents[1]
ARM = '/action/joint_states'
LEFT = '/action/left_hand/joint_states'
RIGHT = '/action/right_hand/joint_states'
KIND = 'sensor_msgs/msg/JointState'


def bag(path, events, robot):
    with path.open('wb') as f:
        w = Writer(f)
        w.start(profile='ros2')
        w.add_metadata('hc_teleop.session', {'profile_id': robot})
        sc = w.register_schema(KIND, 'ros2msg', b'')
        topics = {t: w.register_channel(t, 'cdr', sc) for t, _, _ in events}
        for i, (topic, names, positions) in enumerate(events, 1):
            msg = JointState(name=names, position=positions)
            msg.header.stamp.sec = 3
            msg.header.stamp.nanosec = i
            msg.header.frame_id = 'fixture'
            w.add_message(topics[topic], i, serialize_message(msg), i)
        w.finish()


def run(tmp_path, profile, events, sink):
    path = tmp_path/'grips.mcap'
    bag(path, events, profile.profile_id)
    meta = dict(robot_api_info=json.dumps(profile.describe_servo_j()))
    player = TopicPlayer(tmp_path, lambda: sink, active_profile_provider=lambda: profile.profile_id,
                         active_api_metadata_provider=lambda: meta)
    try:
        player.play(path.name)
        player._thread.join(3)
        return player.status()
    finally:
        player.stop(require_reset=False)


@pytest.mark.parametrize('robot', ['x1', 'HC_REALMAN', 'openarmx'])
def test_replay_each_side_once_in_original_command_order(tmp_path, robot):
    profile = RobotApiProfile(ROOT/'adapters/robots'/robot)
    left, right = (profile.binary_grips[g] for g in ('left_hand', 'right_hand'))
    arm = profile.servo_j_members['whole_body'][0]
    events = [(LEFT, left.native_names, left.opened), (ARM, [arm], [.1]),
              (SERVO_J_HAND_STATE, ['right_grip'], [1.]),
              (RIGHT, right.native_names, right.opened),
              (SERVO_J_COMMANDED_STATE, ['left_grip'], [1.]),
              (SERVO_J_COMMANDED_STATE, ['left_joint_1'], [.2]),
              (SERVO_J_COMMANDED_STATE, ['right_grip','left_grip'], [1., 0.])]
    sink = Mock()
    result = run(tmp_path, profile, events, sink)
    assert result['state'] == 'completed' and result['progress'] == 100.
    actual = [(c.args[0], deserialize_message(c.args[2], JointState)) for c in sink.publish_raw.call_args_list]
    assert [(t, m.header.stamp.nanosec) for t, m in actual] == [(ARM,2), (LEFT,5), (RIGHT,7), (LEFT,7)]
    for (_, msg), (grip, bit) in zip(actual[1:], [(left,1.), (right,1.), (left,0.)]):
        assert msg.header.frame_id == 'fixture'
        assert tuple(msg.name) == grip.command_names and tuple(msg.position) == grip.encode(bit)


def test_replay_preserves_native_other_side_without_quantizing_it(tmp_path):
    profile = RobotApiProfile(ROOT/'adapters/robots/x1')
    left, right = (profile.binary_grips[g] for g in ('left_hand', 'right_hand'))
    # This fine-grained intermediate finger posture has no published binary event on the right.
    fine = [(a+b)/2 for a, b in zip(right.opened, right.closed)]
    events = [(LEFT, left.native_names, left.closed), (ARM, ['Joint1_R'], [.1]),
              (RIGHT, right.native_names, fine), (SERVO_J_HAND_STATE, ['right_grip'], [0.]),
              (SERVO_J_COMMANDED_STATE, ['left_grip'], [1.])]
    sink = Mock()
    assert run(tmp_path, profile, events, sink)['state'] == 'completed'
    assert [c.args[0] for c in sink.publish_raw.call_args_list] == [ARM, RIGHT, LEFT]
    native = deserialize_message(sink.publish_raw.call_args_list[1].args[2], JointState)
    assert tuple(native.position) == tuple(fine)


def test_hand_only_commands_use_replay_fence_and_never_invent_an_arm_target(tmp_path):
    profile = RobotApiProfile(ROOT/'adapters/robots/x1')
    bridge = RosBridge({'enabled': True, 'subscriptions': [], 'legacy_control_profile': str(profile.directory)},
                       lambda *_: None)
    bridge._boundary = Mock(teleop=profile.teleop)
    bridge.receive_hardware_ready(True)
    # Observe real RosBridge.publish_raw admission and its replay-only queue; no ROS publishers run.
    pending = []
    bridge._enqueue = lambda cmd, args: pending.append((cmd, args)) is None
    events = [('/observation/joint_states', ['Joint1_R'], [.1]),
              (SERVO_J_COMMANDED_STATE, ['left_grip'], [1.])]
    assert run(tmp_path, profile, events, bridge)['state'] == 'completed'
    admitted = [(cmd, args) for cmd, args in pending if cmd == 'replay_raw']
    assert len(admitted) == 1 and admitted[0][1][0] == LEFT
    assert bridge.publish_raw(LEFT, KIND, admitted[0][1][2]) is False


def test_observation_only_x1_cannot_drive(tmp_path):
    profile = RobotApiProfile(ROOT/'adapters/robots/x1')
    sink = Mock()
    with pytest.raises(ValueError, match='recorded final actions'):
        run(tmp_path, profile, [(SERVO_J_HAND_STATE, ['left_grip'], [1.])], sink)
    sink.begin_replay.assert_not_called()
