"""Batch model synchronization preserves the original X1 joint/FK results."""
import math
from pathlib import Path
import random
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

bullet = pytest.importorskip('pybullet')
pytest.importorskip('rclpy')
from adapters.core.arm_teleop_node import RobotArmTeleopNode


def fake_node():
    return SimpleNamespace(robot_id=7, physics_client=3,
        joint_state={'known': .25, 'unknown': .5, 'fixed': .75},
        joint_by_name={'known': 1, 'fixed': 2}, dof_by_joint={1: 0})


def test_missing_batch_api_uses_original_scalar_selection(monkeypatch):
    node = fake_node()
    monkeypatch.delattr(bullet, 'resetJointStatesMultiDof', raising=False)
    scalar = Mock()
    monkeypatch.setattr(bullet, 'resetJointState', scalar)
    RobotArmTeleopNode._sync_model(node)
    scalar.assert_called_once_with(7, 1, .25, physicsClientId=3)


@pytest.mark.parametrize('exception', [TypeError('unsupported signature'), NotImplementedError('unsupported API'), bullet.error('unsupported')])
def test_unsupported_batch_reapplies_selection_and_does_not_retry_each_tick(monkeypatch, exception):
    node = fake_node()
    batch, scalar = Mock(side_effect=exception), Mock()
    monkeypatch.setattr(bullet, 'resetJointStatesMultiDof', batch)
    monkeypatch.setattr(bullet, 'resetJointState', scalar)
    for _ in range(2):
        RobotArmTeleopNode._sync_model(node)
    batch.assert_called_once_with(7, [1], [[.25]], targetVelocities=[[0.]], physicsClientId=3)
    assert scalar.call_count == 2
    assert node._model_sync_bulk_supported is False


def test_unknown_fixed_and_empty_states_never_reset_other_joints(monkeypatch):
    node = fake_node()
    node.joint_state = {'unknown': .5, 'fixed': .75}
    batch, scalar = Mock(), Mock()
    monkeypatch.setattr(bullet, 'resetJointStatesMultiDof', batch)
    monkeypatch.setattr(bullet, 'resetJointState', scalar)
    RobotArmTeleopNode._sync_model(node)
    node.joint_state = {}
    RobotArmTeleopNode._sync_model(node)
    batch.assert_not_called()
    scalar.assert_not_called()


def test_pose_batch_keeps_root_semantics_deduplicates_and_falls_back_once(monkeypatch):
    node = SimpleNamespace(robot_id=7, physics_client=3, _link_pose=Mock(return_value=('p', 'q')))
    batch = Mock(side_effect=TypeError('unsupported'))
    monkeypatch.setattr(bullet, 'getLinkStates', batch)
    for _ in range(2):
        assert RobotArmTeleopNode._link_pose_batch(node, [-1, 2, 2, 3]) == {
            -1: ('p', 'q'), 2: ('p', 'q'), 3: ('p', 'q')}
    batch.assert_called_once_with(7, [2, 3], computeForwardKinematics=True, physicsClientId=3)
    assert node._link_pose.call_count == 6


def test_pose_batch_matches_scalar_and_reads_fresh_state_after_joint_motion():
    profile = Path(__file__).resolve().parents[1]/'adapters/robots/x1'
    robot = yaml.safe_load((profile/'arm_teleop.yaml').read_text())['robot']
    client = bullet.connect(bullet.DIRECT)
    try:
        body = bullet.loadURDF(str((profile/robot['urdf_path']).resolve()), robot['base_position'],
            robot['base_orientation'], useFixedBase=True, physicsClientId=client)
        node = SimpleNamespace(robot_id=body, physics_client=client)
        node._root_pose = lambda: RobotArmTeleopNode._root_pose(node)
        node._link_pose = lambda index: RobotArmTeleopNode._link_pose(node, index)
        count = bullet.getNumJoints(body, physicsClientId=client)
        indices = [-1, *range(count)]
        moving = next(index for index in range(count)
            if bullet.getJointInfo(body, index, physicsClientId=client)[2] == bullet.JOINT_REVOLUTE)
        for value in (0., .1, -.2):
            bullet.resetJointState(body, moving, value, physicsClientId=client)
            batch = RobotArmTeleopNode._link_pose_batch(node, indices)
            for index in indices:
                scalar = node._link_pose(index)
                assert np.array_equal(batch[index][0], scalar[0])
                assert np.array_equal(batch[index][1], scalar[1])
    finally:
        bullet.disconnect(physicsClientId=client)


def test_x1_100_random_feedback_groups_match_all_joint_states_and_link_poses():
    profile = Path(__file__).resolve().parents[1]/'adapters/robots/x1'
    robot = yaml.safe_load((profile/'arm_teleop.yaml').read_text())['robot']
    clients = [bullet.connect(bullet.DIRECT) for _ in range(2)]
    try:
        bodies = [bullet.loadURDF(str((profile/robot['urdf_path']).resolve()), robot['base_position'],
            robot['base_orientation'], useFixedBase=True, flags=bullet.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=client) for client in clients]
        info = [bullet.getJointInfo(bodies[0], i, physicsClientId=clients[0])
                for i in range(bullet.getNumJoints(bodies[0], physicsClientId=clients[0]))]
        by_name = {item[1].decode(): item[0] for item in info}
        dofs = {item[0]: number for number, item in enumerate(item for item in info if item[2] != bullet.JOINT_FIXED)}
        indices = list(dofs)
        assert all(info[index][2] == bullet.JOINT_REVOLUTE for index in indices)
        measured = [by_name[f'Joint{i}_{side}'] for side in ('R', 'L') for i in range(1, 8)]
        measured += [by_name[name] for name in ('leg_1', 'leg_2', 'zhi')]
        fixed = next(item[1].decode() for item in info if item[2] == bullet.JOINT_FIXED)
        rng = random.Random(20261116)

        def position(index):
            low, high = float(info[index][8]), float(info[index][9])
            if not math.isfinite(low+high) or low >= high:
                low, high = -math.pi, math.pi
            return rng.uniform(low, high)

        node = SimpleNamespace(robot_id=bodies[1], physics_client=clients[1],
                               joint_by_name=by_name, dof_by_joint=dofs)
        for _ in range(100):
            initial, velocity = [position(index) for index in indices], [rng.uniform(-1., 1.) for index in indices]
            for client, body in zip(clients, bodies):
                for index, value, speed in zip(indices, initial, velocity):
                    bullet.resetJointState(body, index, value, targetVelocity=speed, physicsClientId=client)
            for selection in (measured, indices):
                values = {info[index][1].decode(): position(index) for index in selection}
                values.update(unknown_joint=.123, **{fixed: .321})
                # This is the production scalar loop before the optimization.
                for name, value in values.items():
                    index = by_name.get(name)
                    if index is not None and index in dofs:
                        bullet.resetJointState(bodies[0], index, value, physicsClientId=clients[0])
                node.joint_state = values
                RobotArmTeleopNode._sync_model(node)
                states = [bullet.getJointStates(body, indices, physicsClientId=client)
                          for client, body in zip(clients, bodies)]
                assert states[0] == states[1]  # Includes selected/unselected joint velocities.
                for index in range(len(info)):
                    poses = [bullet.getLinkState(body, index, computeForwardKinematics=True, physicsClientId=client)
                             for client, body in zip(clients, bodies)]
                    assert np.array_equal(np.concatenate(poses[0]), np.concatenate(poses[1]))
        assert getattr(node, '_model_sync_bulk_supported', True)
    finally:
        for client in clients:
            bullet.disconnect(physicsClientId=client)
