"""Base yaw must not feed back into the headset-controlled neck target."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
from adapters.core.arm_teleop_math import quaternion_from_axis_angle
pytest.importorskip('rclpy')
pytest.importorskip('pybullet')
from adapters.core.arm_teleop_node import RobotArmTeleopNode


def node_for(angle, offset):
    return SimpleNamespace(
        neck_enabled=True, neck_joint_names=['head_1', 'head_2'],
        body=SimpleNamespace(head_pose=(np.zeros(3), quaternion_from_axis_angle([0, 1, 0], angle)), head_stamp=10.),
        neck_reference_head=(np.zeros(3), np.array([0., 0., 0., 1.])),
        base_yaw_alignment_offset=offset,
        body_config={'neck_max_velocity': 100., 'head_orientation_align_on_move_only': True},
        control={'pose_timeout': .5, 'rate_hz': 100., 'axis_mapping': [[0,0,1],[-1,0,0],[0,1,0]]},
        last_command={}, joint_state={}, initial_joints={}, joint_by_name={},
        _wrap_angle=lambda a: math.atan2(math.sin(a),math.cos(a)),
    )


@pytest.mark.parametrize('head_angle', [0., -.4, .4])
def test_base_rotation_never_changes_neck_target(head_angle):
    results=[]
    for base_angle in [0., -.7, .7]:
        node=node_for(head_angle,base_angle)
        command={}
        RobotArmTeleopNode._update_neck_joints(node,command,10.)
        results.append(command)
    assert results[0] == results[1] == results[2]
    if head_angle == 0.:
        assert results[0]['head_1'] == pytest.approx(0.)
    else:
        assert abs(results[0]['head_1']) == pytest.approx(abs(head_angle))


def test_stale_headset_does_not_update_neck():
    command={}
    RobotArmTeleopNode._update_neck_joints(node_for(.4,.7), command, 11.)
    assert command == {}


def test_base_head_following_still_uses_alignment():
    node=node_for(.4,0.)
    before=RobotArmTeleopNode._compute_head_orientation_velocity(node,10.,forward_velocity=.2)
    assert abs(before)>0
    # The configured mapping turns +Y headset yaw into -Z mapped yaw.
    node.base_yaw_alignment_offset=-.4
    after=RobotArmTeleopNode._compute_head_orientation_velocity(node,10.,forward_velocity=.2)
    assert after == pytest.approx(0.)
