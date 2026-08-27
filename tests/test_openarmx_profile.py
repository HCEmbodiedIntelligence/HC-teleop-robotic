import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import yaml

from middleware.core.robot_profiles import RobotProfileManager


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "adapters" / "robots" / "openarmx"


class OpenArmXProfileTests(unittest.TestCase):
    def test_profile_joint_order_and_standard_topics(self):
        controller = yaml.safe_load(
            (PROFILE / "controller_v23.yml").read_text(encoding="utf-8")
        )
        teleop = yaml.safe_load(
            (PROFILE / "arm_teleop.yaml").read_text(encoding="utf-8")
        )
        expected = (
            teleop["arms"]["right"]["joint_names"]
            + teleop["arms"]["left"]["joint_names"]
        )
        self.assertEqual(controller["model"]["free_joints"], expected)
        self.assertEqual(
            controller["ros_interface"]["sub_topic"]["joint_state"],
            "/hc_teleop/joint_states",
        )
        self.assertEqual(
            teleop["control"]["command_topic"], "/hc_teleop/joint_cmd_vr"
        )

    def test_urdf_meshes_are_self_contained(self):
        urdf = PROFILE / "urdf" / "openarmx_robot.urdf"
        tree = ElementTree.parse(urdf)
        filenames = [mesh.attrib["filename"] for mesh in tree.findall(".//mesh")]
        self.assertTrue(filenames)
        self.assertFalse(any(name.startswith("package://") for name in filenames))
        for filename in filenames:
            self.assertTrue((urdf.parent / filename).resolve().is_file(), filename)

    def test_vr_axes_match_openarmx_calibration(self):
        teleop = yaml.safe_load(
            (PROFILE / "arm_teleop.yaml").read_text(encoding="utf-8")
        )
        mapping = np.asarray(teleop["control"]["axis_mapping"], dtype=float)
        expected = np.asarray(
            [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        )
        np.testing.assert_allclose(mapping, expected)
        np.testing.assert_allclose(mapping @ mapping.T, np.eye(3))
        self.assertAlmostEqual(float(np.linalg.det(mapping)), 1.0)

    def test_ready_archive_round_trips_through_profile_importer(self):
        archive = (PROFILE / "openarmx_profile.zip").read_bytes()
        with tempfile.TemporaryDirectory() as temp_dir:
            imported = RobotProfileManager(temp_dir).import_archive(
                "openarmx_test",
                "OpenArmX Test",
                archive,
                "openarmx_profile.zip",
            )
            self.assertEqual(imported["robot_name"], "openarmx")
            self.assertEqual(imported["free_joint_count"], 14)
            self.assertEqual(imported["arm_count"], 2)
            self.assertTrue(imported["teleop_compatible"])


if __name__ == "__main__":
    unittest.main()
