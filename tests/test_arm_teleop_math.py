import math
import unittest

import numpy as np

from adapters.core.arm_teleop_math import (
    adaptive_damping,
    binary_trigger_state,
    clamp_step,
    joystick_base_velocity,
    joystick_yaw_velocity,
    joint_limit_avoidance,
    mapped_relative_pitch,
    mapped_relative_yaw,
    ordered_homing_joint_names,
    orientation_error,
    quaternion_from_axis_angle,
    quaternion_multiply,
    relative_target,
    stabilize_pose,
    sticks_outward,
)


class ArmTeleopMathTests(unittest.TestCase):
    def test_trigger_state_is_recorded_as_binary_open_or_closed(self):
        self.assertEqual(binary_trigger_state(0.0), 0.0)
        self.assertEqual(binary_trigger_state(0.499), 0.0)
        self.assertEqual(binary_trigger_state(0.5), 1.0)
        self.assertEqual(binary_trigger_state(1.0), 1.0)

    def test_adaptive_damping_increases_near_singularity(self):
        self.assertAlmostEqual(adaptive_damping(0.2, 0.03, 0.3, 0.1), 0.03)
        self.assertAlmostEqual(adaptive_damping(0.0, 0.03, 0.3, 0.1), 0.3)
        self.assertGreater(
            adaptive_damping(0.04, 0.03, 0.3, 0.1),
            adaptive_damping(0.08, 0.03, 0.3, 0.1),
        )

    def test_joint_limit_avoidance_pushes_toward_safe_range(self):
        result = joint_limit_avoidance(
            [-0.99, 0.0, 0.99], [-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], 0.1
        )
        self.assertGreater(result[0], 0.0)
        self.assertEqual(result[1], 0.0)
        self.assertLess(result[2], 0.0)

    def test_controller_positive_z_maps_to_robot_forward(self):
        target, _ = relative_target(
            [0.1, 0.2, 0.3],
            [0, 0, 0, 1],
            [0, 0, 0],
            [0, 0, 0, 1],
            [1, 2, 3],
            [0, 0, 0, 1],
            [[0, 0, 1], [-1, 0, 0], [0, 1, 0]],
            1.0,
            1.0,
            False,
        )
        np.testing.assert_allclose(target, [1.3, 1.9, 3.2], atol=1e-7)

    def test_relative_orientation_is_applied(self):
        turn = quaternion_from_axis_angle([0, 0, 1], math.pi / 4)
        _, target = relative_target(
            [0, 0, 0],
            turn,
            [0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0],
            [0, 0, 0, 1],
            np.eye(3),
            1.0,
            1.0,
            True,
        )
        np.testing.assert_allclose(orientation_error(turn, target), [0, 0, 0], atol=1e-7)

    def test_openxr_yaw_maps_to_robot_z_but_head_roll_does_not(self):
        mapping = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
        yaw = quaternion_from_axis_angle([0, 1, 0], 0.4)
        roll = quaternion_from_axis_angle([0, 0, 1], 0.4)
        self.assertAlmostEqual(
            mapped_relative_yaw(yaw, [0, 0, 0, 1], mapping), 0.4, places=7
        )
        self.assertAlmostEqual(
            mapped_relative_yaw(roll, [0, 0, 0, 1], mapping), 0.0, places=7
        )

    def test_openxr_pitch_maps_to_robot_pitch(self):
        mapping = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
        pitch_down = quaternion_from_axis_angle([1, 0, 0], -0.3)
        roll = quaternion_from_axis_angle([0, 0, 1], 0.3)
        self.assertAlmostEqual(
            mapped_relative_pitch(pitch_down, [0, 0, 0, 1], mapping), 0.3, places=7
        )
        self.assertAlmostEqual(
            mapped_relative_pitch(roll, [0, 0, 0, 1], mapping), 0.0, places=7
        )

    def test_pose_stabilizer_holds_noise_and_smooths_real_motion(self):
        previous_position = np.asarray([1.0, 2.0, 3.0])
        previous_orientation = np.asarray([0.0, 0.0, 0.0, 1.0])
        held_position, held_orientation = stabilize_pose(
            [1.001, 2.0, 3.0],
            quaternion_from_axis_angle([0, 0, 1], 0.01),
            previous_position,
            previous_orientation,
            0.002,
            0.015,
            0.5,
        )
        np.testing.assert_allclose(held_position, previous_position)
        np.testing.assert_allclose(held_orientation, previous_orientation)

        moved_position, moved_orientation = stabilize_pose(
            [1.012, 2.0, 3.0],
            quaternion_from_axis_angle([0, 0, 1], 0.115),
            previous_position,
            previous_orientation,
            0.002,
            0.015,
            0.5,
        )
        np.testing.assert_allclose(moved_position, [1.005, 2.0, 3.0], atol=1e-9)
        np.testing.assert_allclose(
            orientation_error(moved_orientation, previous_orientation),
            [0.0, 0.0, 0.05],
            atol=1e-7,
        )

    def test_both_sticks_outward_gesture(self):
        self.assertTrue(sticks_outward(-0.9, 0.85, 0.8))
        self.assertFalse(sticks_outward(0.9, -0.85, 0.8))
        self.assertFalse(sticks_outward(-0.7, 0.85, 0.8))

    def test_homing_joint_order_includes_arms_and_waist_once(self):
        names = ordered_homing_joint_names(
            ["right_1", "shared"],
            ["left_1", "shared"],
            ["leg_1", "leg_2", "zhi"],
        )

        self.assertEqual(
            names,
            ["right_1", "shared", "left_1", "leg_1", "leg_2", "zhi"],
        )

    def test_left_stick_maps_to_forward_and_lateral_base_motion(self):
        self.assertEqual(
            joystick_base_velocity(0.05, -0.1, 0.12, 0.25, 0.2),
            (0.0, 0.0),
        )
        forward, lateral = joystick_base_velocity(0.0, 1.0, 0.12, 0.25, 0.2)
        self.assertAlmostEqual(forward, 0.25)
        self.assertAlmostEqual(lateral, 0.0)
        forward, lateral = joystick_base_velocity(-1.0, 0.0, 0.12, 0.25, 0.2)
        self.assertAlmostEqual(forward, 0.0)
        self.assertAlmostEqual(lateral, 0.2)
        _, lateral = joystick_base_velocity(1.0, 0.0, 0.12, 0.25, 0.2)
        self.assertAlmostEqual(lateral, -0.2)

    def test_right_stick_maps_to_yaw_velocity(self):
        # Deadzone suppression (< 0.08)
        self.assertEqual(joystick_yaw_velocity(0.05, 0.08, 0.45, -1.0), 0.0)
        self.assertEqual(joystick_yaw_velocity(-0.07, 0.08, 0.45, -1.0), 0.0)

        # Full left (-1.0) with yaw_direction=-1.0 -> positive counter-clockwise yaw (+0.45)
        yaw_left = joystick_yaw_velocity(-1.0, 0.08, 0.45, -1.0)
        self.assertAlmostEqual(yaw_left, 0.45)

        # Full right (+1.0) with yaw_direction=-1.0 -> negative clockwise yaw (-0.45)
        yaw_right = joystick_yaw_velocity(1.0, 0.08, 0.45, -1.0)
        self.assertAlmostEqual(yaw_right, -0.45)

        # Inverted direction (+1.0)
        yaw_left_inv = joystick_yaw_velocity(-1.0, 0.08, 0.45, 1.0)
        self.assertAlmostEqual(yaw_left_inv, -0.45)

        # Halfway deflection
        # (0.54 - 0.08) / (1.0 - 0.08) = 0.46 / 0.92 = 0.5
        yaw_half = joystick_yaw_velocity(-0.54, 0.08, 0.45, -1.0)
        self.assertAlmostEqual(yaw_half, 0.225)

    def test_joint_step_and_limits(self):
        result = clamp_step([2, -2], [0, 0], [-1, -0.05], [1, 1], 0.1)
        np.testing.assert_allclose(result, [0.1, -0.05])

    def test_quaternion_composition(self):
        first = quaternion_from_axis_angle([1, 0, 0], 0.2)
        identity = quaternion_multiply(first, [0, 0, 0, 1])
        np.testing.assert_allclose(first, identity, atol=1e-7)


if __name__ == "__main__":
    unittest.main()
