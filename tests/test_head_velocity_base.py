"""Real chassis yaw velocity following needs headset input, not odometry."""
import math
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('pybullet')
from adapters.core.arm_teleop_math import quaternion_from_axis_angle
from adapters.core.arm_teleop_node import RobotArmTeleopNode

CONFIG = Path(__file__).parents[1] / 'adapters/robots/x1/arm_teleop.yaml'


def pose(yaw=0.):
    return np.zeros(3), quaternion_from_axis_angle([0, 1, 0], yaw)


@pytest.mark.parametrize('mode, expected', [('teleop', 'planar_velocity'), ('sim', 'pose'), (None, 'pose')])
def test_launch_mode_selects_real_velocity_or_simulated_pose(mode, expected):
    rclpy.init(domain_id=168)
    try:
        with patch.dict(os.environ):
            os.environ.pop('HC_TELEOP_MODE', None)
            if mode is not None:
                os.environ['HC_TELEOP_MODE'] = mode
            node = RobotArmTeleopNode(str(CONFIG), backend='v23')
        try:
            assert node.head_base_mode == expected
            assert (node.head_base_follower is None) == (expected == 'planar_velocity')
            assert node.head_velocity_max_angular == pytest.approx(.2)
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def make_node():
    config = yaml.safe_load(CONFIG.read_text())
    node = SimpleNamespace(
        body_config=config['body'], control=config['control'],
        body=SimpleNamespace(head_pose=pose(), head_stamp=100.),
        head_base_mode='yaw_velocity', head_base_follower=None,
        head_velocity_reference=None, last_head_velocity_time=None,
        head_velocity_max_angular=.2, head_base_state='idle',
        neck_reference_head=pose(1.), base_yaw_alignment_offset=0.,
        base_zero_pending=False, base_pub=Mock(), now=100.,
        _compute_user_base_velocity=lambda *a, **k: (0., 0., 0.),
        _compute_hands_reach_assist_velocity=lambda *a: (0., 0.),
        _wrap_angle=lambda a: math.atan2(math.sin(a), math.cos(a)),
    )
    node._monotonic = lambda: node.now
    for name in ('_compute_head_orientation_velocity', '_camera_heading_yaw', '_stop_base_if_pending'):
        method = getattr(RobotArmTeleopNode, name)
        setattr(node, name, lambda *a, _method=method, **k: _method(node, *a, **k))
    return node


def tick(node, time, angle, grip=True):
    node.now = time
    node.body.head_pose = pose(angle)
    node.body.head_stamp = time
    RobotArmTeleopNode._publish_base_command(node, time, left_clutch_active=grip)


@pytest.mark.parametrize('angle', [-1., 1.])
def test_stationary_head_turn_without_base_pose_has_correct_direction_and_cap(angle):
    node = make_node()
    tick(node, 100., 0.)
    tick(node, 100.01, angle)
    yaw, forward, lateral = node.base_pub.publish.call_args.args[0].data
    assert yaw * (-angle * node.body_config['yaw_direction']) > 0
    assert abs(yaw) == pytest.approx(.2)
    assert forward == lateral == 0.
    assert node.head_base_state == 'yaw_velocity'
    # Holding the new angle settles the command estimate instead of spinning forever.
    for step in range(1, 4001):
        tick(node, 100.01 + step * .01, angle)
    assert abs(node._compute_head_orientation_velocity(node.now)) < .001


def test_release_stops_and_regrip_uses_current_head_instead_of_neck_zero():
    node = make_node()
    tick(node, 100., .2)
    tick(node, 100.01, .6)
    assert abs(node.base_pub.publish.call_args.args[0].data[0]) > 0
    tick(node, 100.02, .6, grip=False)
    assert list(node.base_pub.publish.call_args.args[0].data) == [0., 0., 0.]
    assert node.head_velocity_reference is None
    assert node.last_head_velocity_time is None
    assert node.head_base_state == 'idle'
    node.base_pub.reset_mock()
    tick(node, 100.03, -.5)
    node.base_pub.publish.assert_not_called()
    np.testing.assert_allclose(node.head_velocity_reference[1], pose(-.5)[1])
    tick(node, 100.04, -.7)
    assert abs(node.base_pub.publish.call_args.args[0].data[0]) > 0


def test_stale_head_stops_and_fresh_head_recaptures_reference():
    node = make_node()
    tick(node, 100., 0.)
    tick(node, 100.01, .5)
    RobotArmTeleopNode._publish_base_command(node, 101., left_clutch_active=True)
    assert list(node.base_pub.publish.call_args.args[0].data) == [0., 0., 0.]
    assert node.head_base_state == 'waiting_head_pose'
    assert node.head_velocity_reference is None
    node.base_pub.reset_mock()
    tick(node, 101.01, .5)
    node.base_pub.publish.assert_not_called()


@pytest.mark.parametrize('elapsed', [.025, 10.])
def test_alignment_uses_elapsed_time_with_stall_bound(elapsed):
    node = make_node()
    tick(node, 100., 0.)
    tick(node, 100. + elapsed, 1.)
    yaw = node.base_pub.publish.call_args.args[0].data[0]
    expected_dt = min(elapsed, node.control['pose_timeout'])
    assert node.base_yaw_alignment_offset == pytest.approx(
        yaw * expected_dt * node.body_config['yaw_direction'])


def test_no_grip_prevents_head_rotation_but_keeps_manual_stick():
    node = make_node()
    node._compute_user_base_velocity = lambda *a, **k: (.1, 0., 0.)
    tick(node, 100., 1., grip=False)
    yaw, forward, lateral = node.base_pub.publish.call_args.args[0].data
    assert yaw == 0.
    assert forward == pytest.approx(.1)
    assert lateral == 0.
    assert node.head_velocity_reference is None


@pytest.mark.parametrize('mode', ['teleop', 'sim'])
def test_unknown_configured_mode_is_rejected_in_all_launch_modes(tmp_path, mode):
    config = yaml.safe_load(CONFIG.read_text())
    config['body']['head_pose_base']['teleop_mode'] = 'misspelled'
    config['robot']['urdf_path'] = str(CONFIG.parent / config['robot']['urdf_path'])
    path = tmp_path / 'invalid-mode.yaml'
    path.write_text(yaml.safe_dump(config))
    rclpy.init(domain_id=168)
    node = object.__new__(RobotArmTeleopNode)
    try:
        with patch.dict(os.environ, {'HC_TELEOP_MODE': mode}):
            with pytest.raises(ValueError, match='teleop_mode'):
                node.__init__(str(path), backend='v23')
    finally:
        node.destroy_node()
        rclpy.shutdown()
