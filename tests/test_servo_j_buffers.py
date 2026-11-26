"""The v23 transport can rename joints without duplicating their numeric buffers."""
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

pytest.importorskip('humanoid_motion_server.servo_j')
from sensor_msgs.msg import JointState
from middleware.core.servo_j import ProfileServoJ


def test_servoj_renames_in_place_and_feedback_preserves_driver_message():
    profile = Path(__file__).resolve().parents[1] / 'adapters/robots/openarmx'
    node, boundary = Mock(), Mock()
    boundary.bridge._lock = RLock()
    boundary.bridge.ownership = SimpleNamespace(active=False)
    adapter = ProfileServoJ(node, boundary, profile, node)
    adapter.server.state.get_subscription_count.return_value = 1
    message = JointState(name=['left_joint_2', 'left_joint_1'], position=[.2, .1],
                         velocity=[.01, .02])
    positions, velocities, efforts = message.position, message.velocity, message.effort
    adapter.command('left_arm', message)
    delivered = boundary.bridge.forward_live_command.call_args.args[1]
    assert delivered is message
    assert delivered.name == ['openarmx_left_joint2', 'openarmx_left_joint1']
    assert delivered.position is positions and delivered.velocity is velocities
    assert delivered.effort is efforts
    adapter.feedback(delivered, 'joints')
    public = adapter.server.state.publish.call_args.args[0]
    assert public.name == ['left_joint_2', 'left_joint_1'] and public.header is delivered.header
    assert public.position is positions and public.velocity is velocities and public.effort is efforts
    assert delivered.name == ['openarmx_left_joint2', 'openarmx_left_joint1']



def test_commanded_x1_projects_physical_targets_and_preserves_buffers():
    node, boundary = Mock(), Mock()
    profile = Path(__file__).resolve().parents[1] / 'adapters/robots/x1'
    adapter = ProfileServoJ(node, boundary, profile, node)
    publisher = adapter.commanded_pub
    publisher.publish.reset_mock()
    publisher.get_subscription_count.return_value = 0
    m = JointState(name=['Joint2_R', 'Joint3_R'], position=[.31, .27], velocity=[.1, .2])
    adapter.commanded(m)
    publisher.publish.assert_not_called()
    assert not adapter.commanded_layouts
    publisher.get_subscription_count.return_value = 1
    adapter.commanded(m)
    result = publisher.publish.call_args.args[0]
    assert result.name == ['right_joint_2', 'right_joint_3']
    assert result.position is m.position and result.header is m.header
    assert not result.velocity and not result.effort
    m = JointState(name=['Joint2_R', 'head_1', 'L_ban', 'leg_2', 'Joint3_R'],
                   position=[.31, .5, 1., float('nan'), .27])
    adapter.commanded(m)
    target = publisher.publish.call_args.args[0]
    assert target.name == ['right_joint_2', 'right_joint_3'] and list(target.position) == [.31, .27]
    for _ in range(2):
        m.position[0] = float('nan')
        adapter.commanded(m)
        assert publisher.publish.call_args.args[0].name == ['right_joint_3']
    hand = JointState(name=['R_ban', 'R_thumb_roll_joint'], position=[1., .2])
    adapter.commanded(hand)
    assert publisher.publish.call_args.args[0].name == ['right_grip']
    assert list(publisher.publish.call_args.args[0].position) == [1.]
    publisher.publish.reset_mock()
    adapter.commanded(JointState(name=['R_ban'], position=[1.]))
    publisher.publish.assert_not_called()


@pytest.mark.parametrize('profile_id', ['x1', 'HC_REALMAN', 'openarmx'])
def test_binary_grip_uses_native_endpoints_and_real_feedback(profile_id):
    profile = Path(__file__).resolve().parents[1] / 'adapters/robots' / profile_id
    node, boundary = Mock(), Mock()
    node.create_publisher.side_effect = lambda *args, **kwargs: Mock()
    boundary.bridge._lock = RLock()
    boundary.bridge.ownership.active = False
    adapter = ProfileServoJ(node, boundary, profile, node)
    measured, commanded = adapter.hand_state, adapter.commanded_pub
    measured.get_subscription_count.return_value = 1
    commanded.get_subscription_count.return_value = 1
    adapter.server.state.get_subscription_count.return_value = 1
    for group, grip in adapter.api.binary_grips.items():
        for bit in (0., 1.):
            request = JointState(name=[grip.name], position=[bit])
            request.header.stamp.sec = 27
            adapter.command(group, request)
            sent = boundary.bridge.forward_live_command.call_args.args[1]
            assert sent is request and sent.header.stamp.sec == 27
            assert sent.name is grip.command_names
            assert tuple(sent.position) == (grip.closed if bit else grip.opened) + ((bit,) if grip.marker else ())
            adapter.commanded(sent)
            public = commanded.publish.call_args.args[0]
            assert public.name == [grip.name] and list(public.position) == [bit] and public.header is sent.header
            # Deliberately report a different posture than the commanded one.
            opposite = grip.opened if bit else grip.closed
            sample = JointState(name=list(grip.native_names), position=list(opposite))
            if grip.marker:
                sample.name.append(grip.marker)
                sample.position.append(bit)
            sample.header.stamp.sec = 5
            if adapter.grips_in_body_feedback:
                sample.name.extend(['openarmx_left_joint1', 'openarmx_left_finger_joint2'])
                sample.position.extend([.2, .033])
                sample.velocity = [1. + i for i in range(len(sample.name))]
                sample.effort = [11. + i for i in range(len(sample.name))]
            adapter.feedback(sample, 'joints' if adapter.grips_in_body_feedback else 'hands')
            public = measured.publish.call_args.args[0]
            assert public.name == [grip.name] and list(public.position) == [1.-bit]
            assert not public.velocity and not public.effort and public.header is sample.header
            if adapter.grips_in_body_feedback:
                body = adapter.server.state.publish.call_args.args[0]
                i = sample.name.index('openarmx_left_joint1')
                assert body.name == ['left_joint_1'] and list(body.position) == [.2]
                assert list(body.velocity) == [sample.velocity[i]] and list(body.effort) == [sample.effort[i]]
        boundary.bridge.forward_live_command.reset_mock()
        adapter.command(group, JointState(name=[grip.name], position=[.2]))
        adapter.command(group, JointState(name=list(grip.native_names), position=list(grip.opened)))
        boundary.bridge.forward_live_command.assert_not_called()
        measured.publish.reset_mock()
        incomplete = JointState(name=[grip.marker or 'unrelated_joint'], position=[1.])
        with patch('middleware.core.servo_j.JointState') as result_type:
            adapter.feedback(incomplete, 'joints' if adapter.grips_in_body_feedback else 'hands')
            result_type.assert_not_called()
        measured.publish.assert_not_called()
