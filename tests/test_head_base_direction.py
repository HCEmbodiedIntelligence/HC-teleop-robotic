"""Headset left/right turns must converge in chassis coordinates (no robot I/O)."""
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

from adapters.core.arm_teleop_math import quaternion_from_axis_angle
pytest.importorskip("rclpy")
pytest.importorskip("pybullet")
from adapters.core.arm_teleop_node import RobotArmTeleopNode


@pytest.mark.parametrize('angle', [-0.4, 0.4])
def test_head_follow_direction_and_alignment_converge(angle):
    config = yaml.safe_load((Path(__file__).parents[1] / 'adapters/robots/x1/arm_teleop.yaml').read_text())
    body_config = dict(config['body'])
    node = SimpleNamespace(
        body_config=body_config, control=config['control'],
        body=SimpleNamespace(head_pose=(np.zeros(3), quaternion_from_axis_angle([0, 1, 0], angle)), head_stamp=100.),
        neck_reference_head=(np.zeros(3), np.array([0., 0., 0., 1.])),
        base_yaw_alignment_offset=0., base_zero_pending=False,
        _wrap_angle=lambda x: math.atan2(math.sin(x), math.cos(x)),
        _compute_user_base_velocity=lambda *a, **k: (0.1, 0., 0.),
        _compute_hands_reach_assist_velocity=lambda *a: (0., 0.),
        _camera_heading_yaw=lambda: 0., base_pub=Mock(),
    )
    node._compute_head_orientation_velocity = lambda *a, **k: RobotArmTeleopNode._compute_head_orientation_velocity(node, *a, **k)
    initial = node._compute_head_orientation_velocity(100., forward_velocity=0.1)
    assert initial * angle > 0  # OpenXR +Y left turn -> chassis +Z left turn.
    assert node._compute_head_orientation_velocity(100.) == 0  # No movement intent.
    assert node._compute_head_orientation_velocity(101., forward_velocity=0.1) == 0  # Stale pose.
    for _ in range(800):
        RobotArmTeleopNode._publish_base_command(node, 100.)
    final = node._compute_head_orientation_velocity(100., forward_velocity=0.1)
    assert abs(final) < abs(initial) * 0.1
    assert abs(node.base_yaw_alignment_offset) <= abs(angle)

@pytest.mark.parametrize('angle', [-0.6, 0.6])
@pytest.mark.parametrize('forward', [-0.2, 0.2])
def test_headset_heading_rotates_translation_in_same_direction(angle, forward):
    config = yaml.safe_load((Path(__file__).parents[1] / 'adapters/robots/x1/arm_teleop.yaml').read_text())
    node = SimpleNamespace(
        body_config={**config['body'], 'head_orientation_assist_enabled': False}, control=config['control'],
        body=SimpleNamespace(head_pose=(np.zeros(3), quaternion_from_axis_angle([0, 1, 0], angle)), head_stamp=100.),
        neck_reference_head=(np.zeros(3), np.array([0., 0., 0., 1.])),
        base_yaw_alignment_offset=0., base_zero_pending=False, _monotonic=lambda:100.,
        _wrap_angle=lambda x: math.atan2(math.sin(x), math.cos(x)),
        _compute_user_base_velocity=lambda *a, **k: (forward, 0., 0.),
        _compute_hands_reach_assist_velocity=lambda *a: (0., 0.), base_pub=Mock(),
    )
    node._camera_heading_yaw=lambda:RobotArmTeleopNode._camera_heading_yaw(node)
    node._compute_head_orientation_velocity=lambda *a, **k:RobotArmTeleopNode._compute_head_orientation_velocity(node,*a,**k)
    RobotArmTeleopNode._publish_base_command(node,100.)
    yaw,vx,vy=node.base_pub.publish.call_args.args[0].data
    assert yaw == 0
    assert vx == pytest.approx(forward*math.cos(angle))
    assert vy == pytest.approx(forward*math.sin(angle))
    # Once chassis has aligned with the head, forward has no lateral component.
    node.base_yaw_alignment_offset=-angle
    assert node._camera_heading_yaw() == pytest.approx(0.)
    node._monotonic=lambda:101.
    assert node._camera_heading_yaw() == 0.
