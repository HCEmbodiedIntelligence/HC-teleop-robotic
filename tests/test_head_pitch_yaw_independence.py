import numpy as np
import pytest
from adapters.core.arm_teleop_math import mapped_relative_pitch, quaternion_from_axis_angle, quaternion_multiply
M=[[0,0,1],[-1,0,0],[0,1,0]]
def pose(yaw,pitch,roll=0.):
    return quaternion_multiply(quaternion_multiply(
        quaternion_from_axis_angle([0,1,0],yaw),
        quaternion_from_axis_angle([1,0,0],pitch)),
        quaternion_from_axis_angle([0,0,1],roll))
@pytest.mark.parametrize('reference_yaw',[0,np.pi/2,-np.pi/2])
@pytest.mark.parametrize('current_yaw',[0,np.pi/2,-np.pi/2,np.pi])
@pytest.mark.parametrize('pitch',[-.3,0.,.3])
def test_pitch_before_after_turn_and_reclutch(reference_yaw,current_yaw,pitch):
    assert mapped_relative_pitch(pose(current_yaw,pitch),pose(reference_yaw,0),M)==pytest.approx(pitch)
def test_reference_pitch_and_roll():
    assert mapped_relative_pitch(pose(np.pi/2,.35,.4),pose(0,.15,-.3),M)==pytest.approx(.2)
