import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
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
            self.assertEqual(num_joints, 48)
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


if __name__ == "__main__":
    unittest.main()
