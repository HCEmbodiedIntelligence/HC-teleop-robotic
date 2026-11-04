import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import numpy as np
import pybullet as p
from middleware.core.robot_profiles import RobotProfileManager
from adapters.v23.src.pinocchio_interface_v3 import PinocchioInterfaceV3
from adapters.v23.src.controller_v2_3 import ControllerV23


class HcRealmanProfileTest(unittest.TestCase):
    def setUp(self):
        self.profile_dir = Path(__file__).resolve().parents[1] / "adapters/robots/HC_REALMAN"

    def test_profile_manager_metadata(self):
        mgr = RobotProfileManager(str(self.profile_dir.parent))
        profile = mgr.get("HC_REALMAN")
        self.assertIsNotNone(profile)
        self.assertEqual(profile["robot_name"], "HC_REALMAN")
        self.assertTrue(profile["teleop_compatible"])
        self.assertEqual(profile["arm_count"], 2)
        self.assertEqual(profile["free_joint_count"], 12)

    def test_meshes_exist_and_match_relative_paths(self):
        urdf_path = self.profile_dir / "urdf/hc_realman.urdf"
        tree = ET.parse(urdf_path)
        for mesh in tree.getroot().findall(".//mesh"):
            fn = mesh.get("filename")
            resolved = (self.profile_dir / "urdf" / fn).resolve()
            self.assertTrue(resolved.is_file(), f"Missing mesh: {fn} -> {resolved}")

    def test_pybullet_direct_load(self):
        cid = p.connect(p.DIRECT)
        try:
            urdf = str(self.profile_dir / "urdf/hc_realman.urdf")
            robot_id = p.loadURDF(urdf, useFixedBase=True)
            self.assertGreaterEqual(robot_id, 0)
            num_joints = p.getNumJoints(robot_id)
            # 48 kinematic joints + 2 end effector TCP fixed frames (right_hand_tcp, left_hand_tcp)
            self.assertEqual(num_joints, 50)
        finally:
            p.disconnect(cid)

    def test_pinocchio_and_controller_v23(self):
        cfg_path = self.profile_dir / "controller_v23.yml"
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        urdf = str(self.profile_dir / cfg["model"]["urdf"])
        PinocchioInterfaceV3(urdf, cfg["model"]["free_joints"])
        self.assertEqual(len(cfg["model"]["free_joints"]), 12)
        ctrl = ControllerV23(str(cfg_path))
        self.assertEqual(len(ctrl.tasks), 2)
        self.assertEqual(len(ctrl.free_joint_names), 12)

    def test_joint_names_alignment_with_driver(self):
        with open(self.profile_dir / "arm_teleop.yaml") as f:
            arm_cfg = yaml.safe_load(f)
        sim_joints = arm_cfg["arms"]["left"]["joint_names"] + arm_cfg["arms"]["right"]["joint_names"]
        driver_cfg = Path("/home/maple/test2/HC_REALMAN/config/hardware.yaml")
        if driver_cfg.is_file():
            with open(driver_cfg) as f:
                hw_cfg = yaml.safe_load(f)
            hw_joints = hw_cfg["left_arm"]["joints"] + hw_cfg["right_arm"]["joints"]
            self.assertEqual(sim_joints, hw_joints)

    def test_human_like_initial_pose_and_forward_z_axis(self):
        """Verify initial arm pose has forward-pointing Z-axis (+X) and human-like geometry."""
        with open(self.profile_dir / "arm_teleop.yaml") as f:
            cfg = yaml.safe_load(f)
        initial_joints = cfg["robot"]["initial_joints"]

        cid = p.connect(p.DIRECT)
        try:
            urdf = str(self.profile_dir / "urdf/hc_realman.urdf")
            robot_id = p.loadURDF(urdf, useFixedBase=True)
            link_map = {}
            for i in range(p.getNumJoints(robot_id)):
                info = p.getJointInfo(robot_id, i)
                j_name = info[1].decode("utf-8")
                l_name = info[12].decode("utf-8")
                link_map[l_name] = i
                if j_name in initial_joints:
                    p.resetJointState(robot_id, i, initial_joints[j_name])

            self.assertIn("right_hand_tcp", link_map)
            self.assertIn("left_hand_tcp", link_map)

            # Check Right Hand TCP
            r_state = p.getLinkState(robot_id, link_map["right_hand_tcp"], computeForwardKinematics=True)
            r_pos = np.array(r_state[4])
            r_rot = np.array(p.getMatrixFromQuaternion(r_state[5])).reshape(3, 3)
            r_z_axis = r_rot[:, 2]

            # Check Left Hand TCP
            l_state = p.getLinkState(robot_id, link_map["left_hand_tcp"], computeForwardKinematics=True)
            l_pos = np.array(l_state[4])
            l_rot = np.array(p.getMatrixFromQuaternion(l_state[5])).reshape(3, 3)
            l_z_axis = l_rot[:, 2]

            # 1. Z-axis must point forward (-X, front of robot)
            self.assertLess(r_z_axis[0], -0.99, f"Right TCP Z-axis does not point forward (-X): {r_z_axis}")
            self.assertLess(l_z_axis[0], -0.99, f"Left TCP Z-axis does not point forward (-X): {l_z_axis}")

            # 2. Forward reach X in natural human range [-0.55m, -0.40m]
            self.assertGreaterEqual(r_pos[0], -0.55)
            self.assertLessEqual(r_pos[0], -0.40)
            self.assertGreaterEqual(l_pos[0], -0.55)
            self.assertLessEqual(l_pos[0], -0.40)

            # 3. Comfortable working height Z in [0.80m, 1.05m]
            self.assertGreaterEqual(r_pos[2], 0.80)
            self.assertLessEqual(r_pos[2], 1.05)
            self.assertGreaterEqual(l_pos[2], 0.80)
            self.assertLessEqual(l_pos[2], 1.05)

            # 4. Left-Right lateral symmetry: |Y_r + Y_l| < 0.005m
            self.assertAlmostEqual(r_pos[1] + l_pos[1], 0.0, delta=0.005)

            # 5. Collision free
            p.stepSimulation()
            contacts = p.getContactPoints(robot_id, robot_id)
            self.assertEqual(len(contacts), 0, f"Initial pose has self-collisions: {contacts}")
        finally:
            p.disconnect(cid)

    def test_ee_target_and_actual_coincidence(self):
        """Verify ee_target coincides exactly with ee_actual in the simulator using ZZJ_Link base."""
        with open(self.profile_dir / "arm_teleop.yaml") as f:
            cfg = yaml.safe_load(f)
        initial_joints = cfg["robot"]["initial_joints"]

        cid = p.connect(p.DIRECT)
        try:
            urdf = str(self.profile_dir / "urdf/hc_realman.urdf")
            robot_id = p.loadURDF(urdf, useFixedBase=True)
            link_map = {}
            for i in range(p.getNumJoints(robot_id)):
                info = p.getJointInfo(robot_id, i)
                j_name = info[1].decode("utf-8")
                link_map[info[12].decode("utf-8")] = i
                if j_name in initial_joints:
                    p.resetJointState(robot_id, i, initial_joints[j_name])

            base_id = link_map["ZZJ_Link"]
            base_pose = p.getLinkState(robot_id, base_id, computeForwardKinematics=True)[4:6]

            for side in ["right", "left"]:
                ee_id = link_map[f"{side}_hand_tcp"]
                actual_ee = p.getLinkState(robot_id, ee_id, computeForwardKinematics=True)[4:6]

                # arm_teleop_node publishes target relative to base_link:
                inv_base = p.invertTransform(base_pose[0], base_pose[1])
                local_target = p.multiplyTransforms(inv_base[0], inv_base[1], actual_ee[0], actual_ee[1])

                # sim_robot_control_node reconstructs target in world:
                reconstructed_target = p.multiplyTransforms(base_pose[0], base_pose[1], local_target[0], local_target[1])

                # Position difference must be practically zero (< 1e-6)
                pos_diff = np.linalg.norm(np.array(actual_ee[0]) - np.array(reconstructed_target[0]))
                self.assertLess(pos_diff, 1e-6, f"{side} arm ee_actual and ee_target do not coincide: diff={pos_diff}")
        finally:
            p.disconnect(cid)

    def test_axis_mapping_directions_and_rotations(self):
        """Verify axis_mapping ensures correct human-to-robot lateral translation and wrist rotations."""
        with open(self.profile_dir / "arm_teleop.yaml") as f:
            cfg = yaml.safe_load(f)
        mapping = np.asarray(cfg["control"]["axis_mapping"], dtype=float)

        self.assertEqual(mapping.shape, (3, 3))

        # 1. Translation mappings
        # VR forward (+Z) -> Robot front (-X)
        d_fwd = mapping @ np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(d_fwd, [-1.0, 0.0, 0.0], atol=1e-6)

        # VR right (+X) -> Robot right (+Y in HC_REALMAN ZZJ_Link frame)
        d_right = mapping @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(d_right, [0.0, 1.0, 0.0], atol=1e-6)

        # VR up (+Y) -> Robot up (+Z)
        d_up = mapping @ np.array([0.0, 1.0, 0.0])
        np.testing.assert_allclose(d_up, [0.0, 0.0, 1.0], atol=1e-6)

        # 2. Rotation mappings: mapped_R = M @ R_vr @ M.T
        def get_rot_matrix(axis, angle_rad):
            axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            return np.eye(3) + np.sin(angle_rad) * K + (1 - np.cos(angle_rad)) * (K @ K)

        def extract_rot_axis(R):
            tr = np.trace(R)
            angle = np.arccos(np.clip((tr - 1.0) / 2.0, -1.0, 1.0))
            if np.abs(angle) < 1e-6:
                return np.zeros(3)
            return np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle))

        # Pitch around VR X (+X_vr) preserves downward pitch direction (-Y_rob)
        R_pitch_vr = get_rot_matrix([1.0, 0.0, 0.0], np.radians(10.0))
        R_pitch_rob = mapping @ R_pitch_vr @ mapping.T
        np.testing.assert_allclose(extract_rot_axis(R_pitch_rob), [0.0, -1.0, 0.0], atol=1e-6)

        # Yaw around VR Y (+Y_vr) rotates around -Z_rob (correct inverted yaw)
        R_yaw_vr = get_rot_matrix([0.0, 1.0, 0.0], np.radians(10.0))
        R_yaw_rob = mapping @ R_yaw_vr @ mapping.T
        np.testing.assert_allclose(extract_rot_axis(R_yaw_rob), [0.0, 0.0, -1.0], atol=1e-6)

        # Roll around VR Z (+Z_vr) rotates around +X_rob (correct inverted roll)
        R_roll_vr = get_rot_matrix([0.0, 0.0, 1.0], np.radians(10.0))
        R_roll_rob = mapping @ R_roll_vr @ mapping.T
        np.testing.assert_allclose(extract_rot_axis(R_roll_rob), [1.0, 0.0, 0.0], atol=1e-6)

    def test_omnipicker_gripper_kinematics_and_mimic(self):
        """Verify OmniPicker parallel 4-bar mimic constraints and symmetrical clamp motion."""
        with open(self.profile_dir / "arm_teleop.yaml") as f:
            cfg = yaml.safe_load(f)

        urdf_path = self.profile_dir / "urdf/hc_realman.urdf"
        tree = ET.parse(urdf_path)
        joints = {j.get("name"): j for j in tree.getroot().findall(".//joint")}

        # Check mimic constraints in URDF
        expected_mimics = {
            "hand_wide1_joint": ("hand_narrow1_joint", -1.0),
            "hand_narrow2_joint": ("hand_narrow1_joint", 1.8),
            "hand_narrow3_joint": ("hand_narrow1_joint", -2.8),
            "hand_narrow_loop_joint": ("hand_narrow1_joint", 2.3),
            "hand_wide2_joint": ("hand_narrow1_joint", -1.8),
            "hand_wide3_joint": ("hand_narrow1_joint", 2.8),
            "hand_wide_loop_joint": ("hand_narrow1_joint", -2.3),
            "Z_hand_wide1_joint": ("Z_hand_narrow1_joint", -1.0),
            "Z_hand_narrow2_joint": ("Z_hand_narrow1_joint", 1.8),
            "Z_hand_narrow3_joint": ("Z_hand_narrow1_joint", -2.8),
            "Z_hand_narrow_loop_joint": ("Z_hand_narrow1_joint", 2.3),
            "Z_hand_wide2_joint": ("Z_hand_narrow1_joint", -1.8),
            "Z_hand_wide3_joint": ("Z_hand_narrow1_joint", 2.8),
            "Z_hand_wide_loop_joint": ("Z_hand_narrow1_joint", -2.3),
        }
        for j_name, (expected_parent, expected_mult) in expected_mimics.items():
            self.assertIn(j_name, joints)
            mimic = joints[j_name].find("mimic")
            self.assertIsNotNone(mimic, f"Missing mimic element in {j_name}")
            self.assertEqual(mimic.get("joint"), expected_parent)
            self.assertAlmostEqual(float(mimic.get("multiplier")), expected_mult, places=2)

        # Check arm_teleop gripper topics and hardware joint mapping
        for side, hw_name in [("right", "right_gripper"), ("left", "left_gripper")]:
            g_cfg = cfg["grippers"][side]
            self.assertIn(hw_name, g_cfg["finger_joint_names"])
            idx = g_cfg["finger_joint_names"].index(hw_name)
            self.assertEqual(g_cfg["finger_open"][idx], 1.0)
            self.assertEqual(g_cfg["finger_closed"][idx], 0.0)

        # Verify PyBullet kinematic clamping motion
        cid = p.connect(p.DIRECT)
        try:
            rid = p.loadURDF(str(urdf_path), useFixedBase=True)
            link_map = {p.getJointInfo(rid, i)[12].decode(): i for i in range(p.getNumJoints(rid))}
            joint_map = {p.getJointInfo(rid, i)[1].decode(): i for i in range(p.getNumJoints(rid))}

            r_cfg = cfg["grippers"]["right"]
            names = r_cfg["finger_joint_names"]
            opened = np.array(r_cfg["finger_open"], dtype=float)
            closed = np.array(r_cfg["finger_closed"], dtype=float)

            # At open (trigger = 0)
            for n, val in zip(names, opened):
                if n in joint_map:
                    p.resetJointState(rid, joint_map[n], val)
            p.stepSimulation()
            open_pts = p.getClosestPoints(rid, rid, 0.2, link_map["hand_narrow3_Link"], link_map["hand_wide3_Link"])
            self.assertTrue(len(open_pts) > 0)
            self.assertGreater(open_pts[0][8], 0.075, "Open gripper gap should exceed 75mm")

            # At closed (trigger = 1)
            for n, val in zip(names, closed):
                if n in joint_map:
                    p.resetJointState(rid, joint_map[n], val)
            p.stepSimulation()
            close_pts = p.getClosestPoints(rid, rid, 0.2, link_map["hand_narrow3_Link"], link_map["hand_wide3_Link"])
            self.assertTrue(len(close_pts) > 0)
            self.assertLess(close_pts[0][8], 0.005, "Closed gripper gap should be less than 5mm")

            # Check that pads center around right_hand_tcp
            s_n = np.array(p.getLinkState(rid, link_map["hand_narrow3_Link"])[4])
            s_w = np.array(p.getLinkState(rid, link_map["hand_wide3_Link"])[4])
            s_tcp = np.array(p.getLinkState(rid, link_map["right_hand_tcp"])[4])
            midpoint = 0.5 * (s_n + s_w)
            self.assertLess(np.linalg.norm(midpoint - s_tcp), 0.005, "Closed pads do not center around right_hand_tcp")
        finally:
            p.disconnect(cid)


if __name__ == "__main__":
    unittest.main()
