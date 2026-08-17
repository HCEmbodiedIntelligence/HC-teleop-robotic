from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin


PROJECT = Path(__file__).resolve().parents[1]
SOURCE = (
    PROJECT
    / "vendor/io_unicontroller_ros2/control_v23_reconstructed/src"
)
sys.path.insert(0, str(SOURCE))

from controller_v2_3 import ControllerV23, TargetTransform  # noqa: E402
from solve_ik import _solve_box_lsq  # noqa: E402


INITIAL = {
    "leg_1": 0.5, "leg_2": 1.2, "zhi": -0.6,
    "Joint1_R": -1.44, "Joint2_R": -0.77, "Joint3_R": 1.44,
    "Joint4_R": -1.57, "Joint5_R": 0.0, "Joint6_R": 0.0,
    "Joint7_R": 0.0, "Joint1_L": 1.44, "Joint2_L": -0.77,
    "Joint3_L": -1.44, "Joint4_L": -1.57, "Joint5_L": 0.0,
    "Joint6_L": 0.0, "Joint7_L": 0.0,
}


class V23ReconstructionTests(unittest.TestCase):
    def make_controller(self) -> ControllerV23:
        controller = ControllerV23(
            PROJECT / "robot_configs/hc_tj_description/controller_v23.yml"
        )
        names = controller.free_joint_names
        controller.update_joint_state(names, [INITIAL[name] for name in names])
        return controller

    def test_relative_jacobians_match_finite_difference(self):
        controller = self.make_controller()
        interface, q = controller.interface, controller.q.copy()
        epsilon = 1e-7
        worst = 0.0
        for task in controller.pose_tasks:
            current = interface.relative_pose(task.root, task.frame, q)
            analytic = interface.relative_jacobian(task.root, task.frame, q)
            numeric = np.empty_like(analytic)
            for column in range(interface.nv_free):
                velocity = np.zeros(interface.nv_free)
                velocity[column] = 1.0
                q_next = interface.integrate_free(q, velocity, epsilon)
                moved = interface.relative_pose(task.root, task.frame, q_next)
                numeric[:3, column] = (
                    moved.translation - current.translation
                ) / epsilon
                numeric[3:, column] = (
                    current.rotation
                    @ pin.log3(current.rotation.T @ moved.rotation)
                    / epsilon
                )
            worst = max(worst, float(np.max(np.abs(analytic - numeric))))
        self.assertLess(worst, 1e-6)

    def test_forward_target_converges_within_limits(self):
        controller = self.make_controller()
        targets = {}
        for task in controller.pose_tasks:
            pose = controller.interface.relative_pose(
                task.root, task.frame, controller.q
            )
            targets[(task.root, task.frame)] = TargetTransform(
                pose.translation.copy(), pose.rotation.copy()
            )
        right_key = (
            controller.pose_tasks[0].root, controller.pose_tasks[0].frame
        )
        start = targets[right_key].translation.copy()
        targets[right_key].translation[0] += 0.02
        controller.update_tf_targets(targets)
        for _ in range(400):
            names, positions = controller.step()
            self.assertLessEqual(
                float(np.max(np.abs(controller.last_result.velocity))),
                float(np.max(controller.velocity_limits)) + 1e-9,
            )
            controller.update_joint_state(names, positions)
        finish = controller.interface.relative_pose(
            *right_key, controller.q
        ).translation
        self.assertGreater(finish[0] - start[0], 0.018)
        self.assertLess(
            np.linalg.norm(targets[right_key].translation - finish), 0.002
        )

    def test_box_solver_can_release_active_bounds(self):
        matrix = np.array([[1.0, 1.8], [0.2, 1.0], [1.0, -0.3]])
        target = np.array([2.0, -1.0, 0.4])
        lower = np.array([-0.25, -0.4])
        upper = np.array([0.3, 0.2])
        velocity = _solve_box_lsq(
            matrix, target, 1e-5, lower, upper
        )
        self.assertTrue(np.all(velocity >= lower - 1e-12))
        self.assertTrue(np.all(velocity <= upper + 1e-12))
        gradient = (
            (matrix.T @ matrix + 1e-5 * np.eye(2)) @ velocity
            - matrix.T @ target
        )
        for index, value in enumerate(velocity):
            if lower[index] + 1e-9 < value < upper[index] - 1e-9:
                self.assertAlmostEqual(float(gradient[index]), 0.0, places=8)
            elif np.isclose(value, lower[index]):
                self.assertGreaterEqual(gradient[index], -1e-8)
            else:
                self.assertLessEqual(gradient[index], 1e-8)


if __name__ == "__main__":
    unittest.main()
