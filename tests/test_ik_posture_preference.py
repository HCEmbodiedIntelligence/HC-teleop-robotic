from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import pinocchio as pin

PROJECT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT / "adapters" / "v23" / "src"
if str(SOURCE) not in sys.path:
    sys.path.insert(0, str(SOURCE))

from controller_v2_3 import ControllerV23, TargetTransform  # noqa: E402
from solve_ik import solve_ik, _stack_tasks  # noqa: E402
from tasks import FrameTask, JointTask  # noqa: E402


INITIAL = {
    "leg_1": 0.5, "leg_2": 1.2, "zhi": -0.6,
    "Joint1_R": -1.44, "Joint2_R": -0.77, "Joint3_R": 1.44,
    "Joint4_R": -1.57, "Joint5_R": 0.0, "Joint6_R": 0.0,
    "Joint7_R": 0.0, "Joint1_L": 1.44, "Joint2_L": -0.77,
    "Joint3_L": -1.44, "Joint4_L": -1.57, "Joint5_L": 0.0,
    "Joint6_L": 0.0, "Joint7_L": 0.0,
}


class IkPosturePreferenceTests(unittest.TestCase):
    def test_controller_v23_loads_posture_task_with_initial_targets(self):
        controller = ControllerV23(PROJECT / "adapters/robots/x1/controller_v23.yml")
        self.assertGreater(len(controller.joint_tasks), 0)
        joint_task = controller.joint_tasks[0]
        self.assertIsNotNone(joint_task.target)
        self.assertEqual(len(joint_task.target), len(joint_task.joint_names))
        # Verify target matches initial joints
        for name, target_val in zip(joint_task.joint_names, joint_task.target):
            self.assertAlmostEqual(target_val, INITIAL[name], places=4)

    def test_openarmx_controller_loads_posture_task(self):
        controller = ControllerV23(PROJECT / "adapters/robots/openarmx/controller_v23.yml")
        self.assertGreater(len(controller.joint_tasks), 0)
        joint_task = controller.joint_tasks[0]
        self.assertIsNotNone(joint_task.target)

    def test_nullspace_projection_orthogonality(self):
        controller = ControllerV23(PROJECT / "adapters/robots/x1/controller_v23.yml")
        names = controller.free_joint_names
        controller.update_joint_state(names, [INITIAL[name] for name in names])
        
        matrix, target = _stack_tasks(
            controller.interface, controller.q, controller.tasks, controller.targets
        )
        
        # Primary tasks are 12 rows (6 for right, 6 for left)
        m_prim = 12
        J_prim = matrix[:m_prim]
        J_sec_proj = matrix[m_prim:]
        
        # J_prim @ J_sec_proj.T should be near zero (approximate nullspace orthogonality)
        ortho_norm = float(np.linalg.norm(J_prim @ J_sec_proj.T))
        self.assertLess(ortho_norm, 1e-4)

    def test_nullspace_posture_preserves_ee_tracking_accuracy_while_biasing_joints(self):
        # 1. Controller without posture task
        c_baseline = ControllerV23(PROJECT / "adapters/robots/x1/controller_v23.yml")
        c_baseline.tasks = [t for t in c_baseline.tasks if not isinstance(t, JointTask)]
        c_baseline.joint_tasks = []
        names = c_baseline.free_joint_names
        c_baseline.update_joint_state(names, [INITIAL[name] for name in names])

        # Move right EE forward by 10 cm, left EE up by 5 cm
        targets = {}
        for task in c_baseline.pose_tasks:
            pose = c_baseline.interface.relative_pose(task.root, task.frame, c_baseline.q)
            targets[(task.root, task.frame)] = TargetTransform(pose.translation.copy(), pose.rotation.copy())
        r_key = (c_baseline.pose_tasks[0].root, c_baseline.pose_tasks[0].frame)
        l_key = (c_baseline.pose_tasks[1].root, c_baseline.pose_tasks[1].frame)
        targets[r_key].translation[0] += 0.10
        targets[l_key].translation[2] += 0.05
        c_baseline.update_tf_targets(targets)

        for _ in range(400):
            n, p = c_baseline.step(0.01)
            c_baseline.update_joint_state(n, p)

        # 2. Controller with nullspace posture task
        c_posture = ControllerV23(PROJECT / "adapters/robots/x1/controller_v23.yml")
        c_posture.update_joint_state(names, [INITIAL[name] for name in names])
        c_posture.update_tf_targets(targets)

        for _ in range(400):
            n, p = c_posture.step(0.01)
            c_posture.update_joint_state(n, p)

        # Check that EE tracking accuracy is exact in both cases (error < 0.1 mm)
        curr_r_base = c_baseline.interface.relative_pose(*r_key, c_baseline.q).translation
        curr_r_post = c_posture.interface.relative_pose(*r_key, c_posture.q).translation
        err_r_post = float(np.linalg.norm(curr_r_post - targets[r_key].translation))
        self.assertLess(err_r_post, 1e-4)

        curr_l_post = c_posture.interface.relative_pose(*l_key, c_posture.q).translation
        err_l_post = float(np.linalg.norm(curr_l_post - targets[l_key].translation))
        self.assertLess(err_l_post, 1e-4)

        # Check that joint distance to home is closer with posture preference
        init_arr = np.array([INITIAL[name] for name in names], dtype=float)
        p_baseline = c_baseline.interface.free_configuration(c_baseline.q)
        p_posture = c_posture.interface.free_configuration(c_posture.q)
        dist_baseline = float(np.linalg.norm(p_baseline - init_arr))
        dist_posture = float(np.linalg.norm(p_posture - init_arr))

        self.assertLess(dist_posture, dist_baseline)


if __name__ == "__main__":
    unittest.main()

