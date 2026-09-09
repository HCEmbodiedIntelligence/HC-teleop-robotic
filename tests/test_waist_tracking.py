"""Offline FK/IK regressions using the X1 model, without commanding a robot."""
import os
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

try:
    import rclpy
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from adapters.core.arm_teleop_math import orientation_error
except ImportError:
    RobotArmTeleopNode = None


@unittest.skipIf(RobotArmTeleopNode is None, "ROS/PyBullet dependencies unavailable")
class WaistTrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Node currently uses the default context; isolate its DDS domain.
        rclpy.init(domain_id=167)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_head_target_converges_with_rotated_x1_base(self):
        path = Path(__file__).resolve().parents[1] / 'adapters/robots/x1/arm_teleop.yaml'
        with patch.dict(os.environ, {'HC_TELEOP_MODE': 'sim'}):
            node = RobotArmTeleopNode(str(path), backend='v23')
        try:
            for delta in ([0., 0., .1], [0., .1, 0.], [0., .08, .08],
                          [0., 0., -.1], [0., -.1, 0.], [0., -.08, -.08]):
                with self.subTest(head_delta=delta):
                    node.joint_state.update(node.initial_joints)
                    node.last_command.update(node.initial_joints)
                    node._sync_model()
                    node.body.head_pose = (np.zeros(3), np.array([0., 0., 0., 1.]))
                    node._engage_body()
                    node.body.head_pose = (np.array(delta), np.array([0., 0., 0., 1.]))
                    node._update_body_target()
                    initial_torso = node.body.reference_torso
                    mapped_target = node._world_pose(node._root_pose(), node.body.target_base)
                    self.assertAlmostEqual(
                        mapped_target[0][2] - initial_torso[0][2], delta[1] * .5,
                        places=6,
                    )
                    mapped_pitch = np.dot(
                        np.array([-1., 0., 0.]),
                        orientation_error(mapped_target[1], initial_torso[1]),
                    )
                    self.assertAlmostEqual(mapped_pitch, delta[2] * .8, places=5)
                    initial_head = node._link_pose(node.link_by_name['camera_head'])
                    for _ in range(400):
                        node._sync_model()
                        previous = np.array([node.joint_state[k] for k in node.body.joint_names])
                        command = {}
                        node._update_body(command)
                        q = np.array([command[k] for k in node.body.joint_names])
                        self.assertTrue(np.isfinite(q).all())
                        self.assertLessEqual(np.max(abs(q - previous)), .0035 + 1e-9)
                        node.joint_state.update(command)
                        node.last_command.update(command)
                    node._sync_model()
                    actual = node._link_pose(node.body.torso_index)
                    target = node._world_pose(node._root_pose(), node.body.target_base)
                    position_error = np.linalg.norm(target[0] - actual[0])
                    rotation_error = np.linalg.norm(orientation_error(target[1], actual[1]))
                    self.assertLess(position_error, .003)
                    self.assertLess(rotation_error, .003)
                    actual_head = node._link_pose(node.link_by_name['camera_head'])
                    head_in_torso = node._relative_pose(actual, actual_head)
                    target_head = node._world_pose(target, head_in_torso)
                    head_error = np.linalg.norm(target_head[0] - actual_head[0])
                    self.assertLess(head_error, .003)
                    if delta[2] == 0:
                        self.assertGreater((actual_head[0][2] - initial_head[0][2]) * delta[1], 0)
                    if delta[1] == 0:
                        # X1 starts at base yaw +90 degrees: forward is world +Y.
                        self.assertGreater((actual_head[0][1] - initial_head[0][1]) * delta[2], 0)
                    print('head delta', delta, 'head error m', head_error,
                          'torso errors m/rad', position_error, rotation_error)
        finally:
            node.destroy_node()
