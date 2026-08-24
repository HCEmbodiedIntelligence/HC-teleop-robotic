import unittest
from pathlib import Path

import numpy as np
import yaml


class ArmTeleopConfigTests(unittest.TestCase):
    @staticmethod
    def load_config():
        path = (
            Path(__file__).resolve().parents[1]
            / "adapters"
            / "robots"
            / "x1"
            / "arm_teleop.yaml"
        )
        return yaml.safe_load(path.read_text(encoding="utf-8"))

    def test_base_axis_remap_does_not_change_home_gesture_axis(self):
        config = self.load_config()

        self.assertEqual(config["body"]["stick_x_axis"], 2)
        self.assertEqual(config["body"]["stick_y_axis"], 3)
        self.assertEqual(config["control"]["home_gesture_axis"], 2)

    def test_collision_avoidance_is_disabled_for_uninterrupted_grasp_capture(self):
        control = self.load_config()["control"]
        self.assertFalse(control["collision_avoidance_enabled"])
        self.assertEqual(control["collision_min_distance"], 0.02)
        self.assertEqual(control["collision_home_tolerance"], 0.008)
        self.assertLess(
            control["collision_home_tolerance"],
            control["collision_min_distance"],
        )
        self.assertTrue(control["collision_latch_until_release"])
        self.assertEqual(control["collision_escape_epsilon"], 0.00001)
        self.assertFalse(control["collision_check_intra_arm"])
        self.assertTrue(control["collision_check_inter_arm"])
        self.assertTrue(control["collision_check_arm_body"])
        self.assertEqual(control["solver_reset_topic"], "/hc_teleop/solver_reset")

    def test_right_hand_angles_match_rs485_demo_motor_positions(self):
        right = self.load_config()["grippers"]["right"]
        mappings = [
            ((0.4, 1.121), (4096, 800)),
            ((-1.642, 0.045), (4096, 0)),
            ((0.0, 1.0), (4096, 0)),
            ((-0.13, -0.05), (4096, 0)),
            ((0.0, 2.5), (4096, 0)),
            ((0.0, 2.5), (4096, 0)),
            ((0.05, 0.13), (0, 4096)),
            ((0.0, 2.5), (4096, 0)),
            ((0.05, 0.13), (4096, 0)),
            ((0.0, 2.5), (4096, 0)),
        ]

        def motor_positions(joint_positions):
            result = []
            for position, (input_range, output_range) in zip(
                joint_positions, mappings
            ):
                ratio = (position - input_range[0]) / (
                    input_range[1] - input_range[0]
                )
                ratio = min(1.0, max(0.0, ratio))
                motor = output_range[0] + ratio * (
                    output_range[1] - output_range[0]
                )
                result.append(int(motor))
            return result

        self.assertEqual(
            motor_positions(right["finger_open"]),
            [3800, 4096, 4096, 1899, 4096, 4096, 4096, 4096, 0, 4096],
        )
        self.assertEqual(
            motor_positions(right["finger_closed"]),
            [3800, 4096, 66, 1899, 2300, 2300, 1932, 604, 2094, 408],
        )

    def test_robot_profile_uses_only_standard_command_topics(self):
        control = self.load_config()["control"]
        self.assertNotIn("command_smoothing", control)
        self.assertEqual(control["joint_state_topic"], "/hc_teleop/joint_states")
        self.assertEqual(control["command_topic"], "/hc_teleop/joint_cmd")

    def test_controller_mapping_is_right_handed_and_forward(self):
        mapping = np.asarray(self.load_config()["control"]["axis_mapping"])
        np.testing.assert_allclose(mapping @ [0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
        self.assertAlmostEqual(float(np.linalg.det(mapping)), 1.0)


if __name__ == "__main__":
    unittest.main()
