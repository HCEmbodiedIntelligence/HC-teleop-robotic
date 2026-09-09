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
