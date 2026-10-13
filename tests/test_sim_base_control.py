import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation"))
try:
    from simulation.general_sim_robot_control_node_ros2 import SimRobotController
except ImportError:
    SimRobotController = None


@unittest.skipIf(SimRobotController is None, "ROS/PyBullet dependencies unavailable")
class SimBaseControlTests(unittest.TestCase):
    def test_feedback_uses_current_command_frame_pose(self):
        from builtin_interfaces.msg import Time
        node = SimpleNamespace(
            base=SimpleNamespace(control_pose=([1., -2., .1], [0., 0., 0., 2.])),
            base_pose_pub=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=4))),
        )
        SimRobotController.publish_base_pose(node)
        msg = node.base_pose_pub.publish.call_args.args[0]
        self.assertEqual(msg.pose.position.x, 1.)
        self.assertEqual(msg.pose.position.y, -2.)
        self.assertEqual(msg.pose.orientation.w, 1.)
        self.assertEqual(msg.header.stamp.sec, 4)
        self.assertEqual(msg.header.frame_id, "world")
        node.base.control_pose = ([float("nan"), 0., 0.], [0., 0., 0., 1.])
        node.base_pose_pub.reset_mock()
        SimRobotController.publish_base_pose(node)
        node.base_pose_pub.publish.assert_not_called()

    def make_node(self):
        node = object.__new__(SimRobotController)
        node.configs = {"base_command_mode": "velocity"}
        node.base_velocity = [0., 0., 0.]
        node.base_update_stamp = 10.
        node.base_command_stamp = 10.
        node._move_base = Mock()
        return node

    def test_velocity_is_integrated_once_per_tick(self):
        node = self.make_node()
        with patch("simulation.general_sim_robot_control_node_ros2.time.monotonic", return_value=10.):
            node.target_base_move_callback(SimpleNamespace(data=[0.4, 0.3, -0.2]))
        node._move_base.assert_not_called()
        with patch("simulation.general_sim_robot_control_node_ros2.time.monotonic", return_value=10.01):
            node._update_base_velocity()
        for actual, expected in zip(node._move_base.call_args.args, [0.004, 0.003, -0.002]):
            self.assertAlmostEqual(actual, expected)

    def test_timeout_stops_and_does_not_replay_motion(self):
        node = self.make_node()
        node.base_velocity = [0.4, 0.3, -0.2]
        with patch("simulation.general_sim_robot_control_node_ros2.time.monotonic", return_value=11.):
            node._update_base_velocity()
        node._move_base.assert_not_called()
        self.assertEqual(node.base_velocity, [0., 0., 0.])

    def test_recent_command_after_stall_has_bounded_step(self):
        node = self.make_node()
        node.base_velocity = [0.4, 0., 0.]
        node.base_command_stamp = 11.
        with patch("simulation.general_sim_robot_control_node_ros2.time.monotonic", return_value=11.):
            node._update_base_velocity()
        self.assertAlmostEqual(node._move_base.call_args.args[0], 0.02)

    def test_invalid_command_stops_motion(self):
        for data in ([float("nan"), 0., 0.], [0., 1.]):
            node = self.make_node()
            node.base_velocity = [0.4, 0., 0.]
            node.target_base_move_callback(SimpleNamespace(data=data))
            self.assertEqual(node.base_velocity, [0., 0., 0.])

    def test_physical_frame_motion_and_feedback_with_rotated_model(self):
        import math
        import threading
        import numpy as np
        import pybullet as p
        from robot_utils.robot_module import RobotBase
        client = p.connect(p.DIRECT)
        try:
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[.1,.1,.1], physicsClientId=client)
            robot_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=shape,
                basePosition=[1,2,0], baseOrientation=p.getQuaternionFromEuler([0,0,math.pi/2]),
                baseInertialFramePosition=[.02,0,0], physicsClientId=client)
            base = RobotBase(SimpleNamespace(robot_id=robot_id, physics_clent_id=client,
                                             bullet_lock=threading.RLock()))
            start = base.control_pose
            node = SimpleNamespace(base=base)
            SimRobotController._move_base(node, 0., .2, 0.)
            np.testing.assert_allclose(np.array(base.control_pose[0])-start[0], [0,.2,0], atol=1e-6)
            SimRobotController._move_base(node, 0., 0., .1)
            np.testing.assert_allclose(np.array(base.control_pose[0])-start[0], [-.1,.2,0], atol=1e-6)
            self.assertAlmostEqual(p.getEulerFromQuaternion(base.control_pose[1])[2], math.pi/2, places=5)
        finally:
            p.disconnect(client)
