"""Command-only planar tracking, coordinate signs and clutch safety."""
import math
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml
from adapters.core.arm_teleop_math import quaternion_from_axis_angle
from adapters.core.head_base_pose import HeadBaseVelocityFollower

CONFIG = yaml.safe_load((Path(__file__).parents[1] / 'adapters/robots/x1/arm_teleop.yaml').read_text())
LIMITS = (.05, .05, .2)


def head(x=0., z=0., yaw=0., height=0.):
    return np.array([x, height, z]), quaternion_from_axis_angle([0, 1, 0], yaw)


def follower():
    return HeadBaseVelocityFollower(CONFIG['body']['head_pose_base'],
                                    CONFIG['control']['head_axis_mapping'], CONFIG['body']['yaw_direction'])


@pytest.mark.parametrize('x,z,expected', [(0., .2, (1, 0)), (0., -.2, (-1, 0)),
                                          (-.2, 0., (0, 1)), (.2, 0., (0, -1))])
@pytest.mark.parametrize('heading', [0., .8, -1.2])
def test_planar_axes_follow_captured_head_heading(x, z, expected, heading):
    f = follower()
    f.update_velocity(head(yaw=heading), 0., LIMITS, .3)
    # Local forward/right -> OpenXR tracking space at the captured heading.
    xr_x = x * math.cos(heading) + z * math.sin(heading)
    xr_z = -x * math.sin(heading) + z * math.cos(heading)
    yaw, vx, vy = f.update_velocity(head(x=xr_x, z=xr_z, yaw=heading), .01, LIMITS, .3)
    assert yaw == pytest.approx(0.)
    assert np.sign(np.where(np.abs([vx, vy]) < 1e-10, 0., [vx, vy])).astype(int).tolist() == list(expected)
    assert abs(vx) <= .05 + 1e-12 and abs(vy) <= .05 + 1e-12


def test_combined_motion_settles_and_discard_estimate_on_long_gap():
    f = follower()
    f.update_velocity(head(), 0., LIMITS, .3)
    for i in range(1, 2501):
        cmd = f.update_velocity(head(x=-.1, z=.2, yaw=-.4), i * .01, LIMITS, .3)
        assert abs(cmd[0]) <= .2
        assert abs(cmd[1]) <= .05 + 1e-12 and abs(cmd[2]) <= .05 + 1e-12
    assert max(abs(v) for v in cmd) < 1e-5
    np.testing.assert_allclose(f.estimate[:2], [.2, .1], atol=.011)
    assert f.update_velocity(head(z=2.), 30., LIMITS, .3) == (0., 0., 0.)
    np.testing.assert_allclose(f.estimate, 0.)


def test_vertical_motion_does_not_drive_chassis():
    f = follower()
    f.update_velocity(head(), 0., LIMITS, .3)
    assert f.update_velocity(head(height=.2), .01, LIMITS, .3) == (0., 0., 0.)


def node():
    pytest.importorskip('rclpy')
    pytest.importorskip('pybullet')
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    n = SimpleNamespace(head_planar_follower=follower(), body_config=CONFIG['body'],
                        control=CONFIG['control'], body=SimpleNamespace(head_pose=head(), head_stamp=0.),
                        head_velocity_max_linear=.05, head_velocity_max_lateral=.05,
                        head_velocity_max_angular=.2, base_pub=Mock(), base_zero_pending=False,
                        _compute_user_base_velocity=Mock(return_value=(0.,0.,0.)))
    n._stop_base_if_pending = lambda: RobotArmTeleopNode._stop_base_if_pending(n)
    n.tick = lambda t, grip=True: RobotArmTeleopNode._publish_head_planar_velocity(n,t,grip)
    return n


@pytest.mark.parametrize('reason', ['release','stale','invalid'])
def test_release_or_invalid_input_stops_and_regrip_recaptures(reason):
    n = node()
    n.tick(0.)
    n.body.head_pose = head(z=.2)
    n.body.head_stamp = .01
    n.tick(.01)
    assert n.base_pub.publish.call_args.args[0].data[1] > 0
    if reason == 'invalid': n.body.head_pose = None
    n.tick(1. if reason == 'stale' else .02, reason != 'release')
    assert list(n.base_pub.publish.call_args.args[0].data) == [0.,0.,0.]
    assert n.head_planar_follower.target is None
    n.body.head_pose = head(z=.3)
    n.body.head_stamp = 2.
    n.base_pub.reset_mock()
    n.tick(2.)
    n.base_pub.publish.assert_not_called()


def test_stick_override_resets_target_and_returns_without_pullback():
    n = node()
    n.tick(0.)
    n.body.head_pose = head(z=.2)
    n.body.head_stamp = .01
    n.tick(.01)
    n._compute_user_base_velocity.return_value = (.03, -.02, .1)
    n.tick(.02)
    assert list(n.base_pub.publish.call_args.args[0].data) == [.1,.03,-.02]
    assert n.head_planar_follower.target is None
    n._compute_user_base_velocity.return_value = (0.,0.,0.)
    n.tick(.03)
    assert list(n.base_pub.publish.call_args.args[0].data) == [0.,0.,0.]


def test_unified_frame_tracking_loss_stops_immediately():
    n = node()
    from std_msgs.msg import String
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    n.tick(0.)
    n.body.head_pose = head(z=.2)
    n.body.head_stamp = .01
    n.tick(.01)
    n.enabled, n.stop_reason = True, ""
    message = String(data=json.dumps({"tracking": {"head": False}, "poses": {}, "inputs": {}}))
    RobotArmTeleopNode._vr_data_callback(n, message)
    assert n.body.head_pose is None
    assert n.head_planar_follower.target is None
    assert list(n.base_pub.publish.call_args.args[0].data) == [0., 0., 0.]
