import numpy as np
import pytest
from adapters.core.arm_teleop_math import clutch_base_axis_mapping, quaternion_from_axis_angle, quaternion_to_matrix, relative_target

@pytest.mark.parametrize('human_yaw',[0.,np.pi/2,-np.pi/2])
@pytest.mark.parametrize('base_yaw',[0.,np.pi/2,-np.pi/2])
def test_reclutch_preserves_forward_left_after_person_or_base_turn(human_yaw,base_yaw):
    mapping=np.array([[0,0,1],[-1,0,0],[0,1,0]])
    head=quaternion_from_axis_angle([0,1,0],human_yaw)
    root=quaternion_from_axis_angle([0,0,1],base_yaw)
    # Tilted chest task frame: conversion must still describe base-forward motion.
    from adapters.core.arm_teleop_math import quaternion_multiply
    task=quaternion_multiply(root,quaternion_from_axis_angle([0,1,0],.3))
    m=clutch_base_axis_mapping(mapping,head,root,task)
    for vr_axis,expected in [([0,0,1],[1,0,0]),([-1,0,0],[0,1,0]),([0,1,0],[0,0,1])]:
        room_delta=quaternion_to_matrix(head)@np.array(vr_axis)*.1
        target,_=relative_target(room_delta,head,[0,0,0],head,[0,0,0],[0,0,0,1],m,1.,1.,True)
        base_delta=quaternion_to_matrix(root).T@quaternion_to_matrix(task)@target
        np.testing.assert_allclose(base_delta,np.array(expected)*.1,atol=1e-7)
