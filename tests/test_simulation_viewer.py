"""Local display only consumes latest remote state and never issues robot commands."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tools.runtime.simulation_viewer import LatestState, LocalDisplay, load_profile, normalize_frame


def frame(value=1):
    return dict(positions={'elbow': value}, base_pose=[[1, 2, 3], [0, 0, 0, 2]])


def test_latest_frame_overwrites_and_stopping_discards_previous_state():
    state = LatestState()
    state.set_frame(normalize_frame(frame(0)))
    assert state.snapshot()[1] is None
    state.set_status(dict(state='running', profile_id='x1'))
    state.set_frame(normalize_frame(frame(1)))
    state.set_frame(normalize_frame(frame(2)))
    assert state.snapshot()[1]['positions'] == {'elbow': 2}
    state.set_status(dict(state='stopped', profile_id='x1'))
    assert state.snapshot()[1] is None


def test_switching_profiles_discards_frame():
    state = LatestState()
    state.set_status(dict(state='running', profile_id='x1'))
    state.set_frame(normalize_frame(frame()))
    state.set_status(dict(state='running', profile_id='other'))
    assert state.snapshot()[1] is None


@pytest.mark.parametrize('bad', [
    dict(positions={'elbow': float('nan')}, base_pose=[[1, 2, 3], [0, 0, 0, 1]]),
    dict(positions={}, base_pose=[[1, 2, 3], [0, 0, 0, 0]]),
    dict(positions={}, base_pose=[[1, 2], [0, 0, 0, 1]]),
])
def test_invalid_remote_frames_are_rejected(bad):
    with pytest.raises(ValueError):
        normalize_frame(bad)


def test_display_only_updates_known_local_joints_and_root_pose():
    gui = object.__new__(LocalDisplay)
    gui.p = Mock()
    gui.client = 3
    gui.robot = SimpleNamespace(robot_id=9)
    gui.joints = {'elbow': 4}
    data = frame()
    data['positions']['unknown'] = 5
    gui.apply(normalize_frame(data))
    gui.p.resetJointState.assert_called_once_with(9, 4, 1., targetVelocity=0, physicsClientId=3)
    gui.p.resetBasePositionAndOrientation.assert_called_once_with(9, (1., 2., 3.), (0., 0., 0., 1.), physicsClientId=3)
    gui.p.stepSimulation.assert_not_called()


def test_profile_uses_metadata_and_model_from_matching_package(tmp_path):
    (tmp_path / 'profile.yaml').write_text('id: sample\n')
    (tmp_path / 'arm_teleop.yaml').write_text('robot:\n  urdf_path: urdf/model.urdf\n')
    profile_id, config, urdf = load_profile(tmp_path)
    assert profile_id == 'sample'
    assert urdf == tmp_path / 'urdf/model.urdf'
    assert config['arm_teleop_path'] == str(tmp_path / 'arm_teleop.yaml')
