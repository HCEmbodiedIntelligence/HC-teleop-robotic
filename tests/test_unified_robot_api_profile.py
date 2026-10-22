"""Profile contract/conversion tests; the ROS acceptance suite tests execution."""
import copy
import json
import math
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from middleware.core.robot_api import RobotApiProfile, selected_profile
from middleware.core.robot_profiles import RobotProfileError, RobotProfileManager

ROOT = Path(__file__).resolve().parents[1]
PROFILES = [ROOT / 'adapters/robots' / n for n in ('openarmx', 'x1')]
PROFILES.append(ROOT / 'tests/fixtures/robot_api_six_dof')


def modified(tmp_path, mutate, file='robot_api.yaml'):
    target = tmp_path / 'profile'
    shutil.copytree(PROFILES[2], target)
    p = target / file
    data = yaml.safe_load(p.read_text())
    mutate(data)
    p.write_text(yaml.safe_dump(data, sort_keys=False))
    return target


@pytest.mark.parametrize('directory', PROFILES)
def test_profiles_resolve_names_home_model_and_auxiliaries(directory, tmp_path):
    api = RobotApiProfile(directory)
    info = api.describe()
    for side in ('left', 'right'):
        group = side + '_arm'
        names = info['groups'][group]['joint_names']
        assert names == [f'{side}_joint_{i}' for i in range(1, len(names)+1)]
        assert set(info['poses']['home'][group]) == set(names)
        target = info['poses']['home'][group]
        converted = api.convert(list(target), list(target.values()), to_model=True, group=group)
        assert dict(zip(converted['name'], converted['position'])) == {
            n: api.teleop['robot']['initial_joints'][n] for n in api.groups[group]}
    assert set(api.joints) == set(api.model)  # no dropped waist/head/gripper/wheel data
    assert len(api.groups['left_arm']) == (6 if directory == PROFILES[2] else 7)
    assert info['groups']['whole_body']['joint_states'] is None
    out = api.write_runtime(tmp_path)
    assert json.loads((out / 'robot_info.json').read_text()) == info
    params = yaml.safe_load((out / 'driver.yaml').read_text())[api.driver['kind']]['ros__parameters']
    assert '/internal/robot/joint_states' in params.values()
    assert '/internal/robot/joint_commands' in params.values()


def test_nonunit_negative_scale_offset_reordered_arrays_and_virtual_work(tmp_path):
    directory = modified(tmp_path, lambda c: c['joint_overrides'].update({
        'fixture_left_axis_1': {'scale': -2., 'offset_rad': .25},
        'fixture_left_axis_2': {'scale': .5, 'offset_rad': -.2}}))
    api = RobotApiProfile(directory)
    source = SimpleNamespace(name=['left_joint_2', 'left_joint_1'], position=[.4, .3],
                             velocity=[.8, -.4], effort=[2., 3.], header={'stamp':(123,456),'frame_id':'original'})
    model = api.convert_message(source, to_model=True, group='left_arm')
    assert model.name == ['fixture_left_axis_1','fixture_left_axis_2']
    assert model.position == pytest.approx([-.35, 0.])
    assert model.velocity == pytest.approx([.8, .4])
    assert model.effort == pytest.approx([-1.5, 4.])
    assert model.header == source.header
    assert sum(v*t for v,t in zip(model.velocity, model.effort)) == pytest.approx(
        sum(v*t for v,t in zip(source.velocity, source.effort)))
    public = api.convert_message(model)
    assert public.position == pytest.approx([.3, .4])
    assert public.effort == pytest.approx([3., 2.])
    lim = api.public['left_joint_1']['limits']
    assert (lim['lower'],lim['upper']) == pytest.approx((-1.125, 1.375))
    assert lim['velocity'] == pytest.approx(.3)
    assert lim['acceleration'] == pytest.approx(1.2)
    assert lim['jerk'] == pytest.approx(4.8)
    source.velocity = source.effort = []
    out = api.convert_message(source, to_model=True)
    assert not out.velocity and not out.effort


@pytest.mark.parametrize('override', [
    {'scale': 0.}, {'scale': float('nan')}, {'scale': float('inf')},
    {'offset_rad': float('nan')}, {'scale': True}, {'offset_m': 1.},
])
def test_invalid_transforms_rejected(tmp_path, override):
    directory = modified(tmp_path, lambda c: c['joint_overrides'].update({'fixture_left_axis_1':override}))
    with pytest.raises(RobotProfileError):
        RobotApiProfile(directory)


@pytest.mark.parametrize('mutate,file', [
    (lambda c: c['groups']['left_arm'].update(model_joints_from='os.system'), 'robot_api.yaml'),
    (lambda c: c['groups']['left_arm'].update(public_name_prefix='native_'), 'robot_api.yaml'),
    (lambda c: c['auxiliary_joints'].update(unknown='aux_joint_1'), 'robot_api.yaml'),
    (lambda c: c['auxiliary_joints'].update(fixture_left_axis_1='right_joint_1'), 'robot_api.yaml'),
    (lambda c: c['groups']['whole_body'].update(combine=['unknown']), 'robot_api.yaml'),
    (lambda c: c['robot']['initial_joints'].pop('fixture_left_axis_1'), 'arm_teleop.yaml'),
    (lambda c: c['joint_groups']['left_arm'].reverse(), 'motion/sdk.yaml'),
    (lambda c: c['humanoid_motion_control']['ros__parameters']['group_upper_limits.left_arm'].__setitem__(0,-3.), 'motion/motion.yaml'),
    (lambda c: c['channels'][0].update(endpoint='/wrong'), 'motion/channels.yaml'),
    (lambda c: c['groups'].update(left_arm=None), 'robot_api.yaml'),
    (lambda c: c['driver'].update(parameters=[]), 'robot_api.yaml'),
    (lambda c: c['humanoid_motion_control'].update(ros__parameters=None), 'motion/motion.yaml'),
])
def test_inconsistent_profiles_rejected(tmp_path, mutate, file):
    directory = modified(tmp_path, mutate, file)
    with pytest.raises(RobotProfileError):
        RobotApiProfile(directory)


@pytest.mark.parametrize('names,position,velocity,group', [
    (['left_joint_1','left_joint_1'], [0.,0.], [], 'left_arm'),
    (['left_joint_1'], [], [], 'left_arm'),
    (['right_joint_1'], [0.], [], 'left_arm'),
    (['fixture_left_axis_1'], [0.], [], 'left_arm'),
    (['left_joint_1'], [float('nan')], [], 'left_arm'),
    (['left_joint_1'], [0.], [0.,0.], 'left_arm'),
])
def test_malformed_and_cross_group_commands_rejected(names,position,velocity,group):
    with pytest.raises(RobotProfileError):
        RobotApiProfile(PROFILES[2]).convert(names, position, velocity, to_model=True, group=group)


def test_import_export_preserves_mapping_connections_and_references(tmp_path):
    directory = modified(tmp_path, lambda c: (
        c['joint_overrides'].update(fixture_left_axis_1={'scale':-1.5,'offset_rad':.2}),
        c['driver']['parameters']['plugin_parameters'].__setitem__(0,'state_topic=/changed/feedback')))
    original = RobotApiProfile(directory)
    payload = RobotProfileManager(directory.parent).export_archive(directory.name)
    manager = RobotProfileManager(tmp_path / 'imported')
    profile = manager.import_archive('imported_fixture', '', payload)
    assert profile['unified_api']['available']
    loaded = RobotApiProfile(profile['path'])
    assert loaded.config == original.config
    assert loaded.teleop == original.teleop
    assert loaded.sdk == original.sdk
    assert loaded.driver['parameters']['plugin_parameters'][0] == 'state_topic=/changed/feedback'
    assert loaded.public['left_joint_1']['scale'] == -1.5


def test_selection_uses_same_environment_precedence_and_relative_root(tmp_path):
    config = {'robot_profiles':{'active':'openarmx','root':'../adapters/robots'}}
    assert selected_profile(config,tmp_path,{}).name == 'openarmx'
    assert selected_profile(config,tmp_path,{'HC_ROBOT_NAME':'x1','HC_ROBOT_CONFIG_ROOT':'robots'}) == tmp_path/'robots/x1'
    with pytest.raises(RobotProfileError):
        selected_profile(config,tmp_path,{'HC_ROBOT_NAME':'../escape'})
