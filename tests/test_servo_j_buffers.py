"""The v23 transport can rename joints without duplicating their numeric buffers."""
from pathlib import Path
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

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
