import unittest
from pathlib import Path
from xml.etree import ElementTree

import yaml

from middleware.profile_cli import load_selection


class ProjectLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]

    def test_code_has_only_two_product_areas(self):
        for relative in (
            "middleware/core",
            "adapters/core",
            "adapters/nodes",
            "adapters/v23",
            "adapters/robots/x1",
            "interfaces",
            "simulation",
        ):
            self.assertTrue((self.root / relative).is_dir(), relative)
        for obsolete in (
            "adapter",
            "hc_teleop_adapter",
            "hc_teleop_middleware",
            "robot_configs",
            "scripts",
            "vendor",
        ):
            self.assertFalse((self.root / obsolete).exists(), obsolete)

    def test_runtime_does_not_depend_on_x1_source_workspace(self):
        runtime_files = [
            self.root / "adapters/start.sh",
            self.root / "middleware/start.sh",
            self.root / "run_generic_controller.sh",
        ]
        text = "\n".join(path.read_text(encoding="utf-8") for path in runtime_files)
        self.assertNotIn("hc_io_suit", text)
        self.assertNotIn("HC_X1/install", text)
        self.assertNotIn("/io_teleop", text)

        launcher = (self.root / "run.sh").read_text(encoding="utf-8")
        self.assertNotIn("hc_io_suit", launcher)
        self.assertNotIn("HC_X1/install", launcher)
        for topic in (
            "joint_states",
            "joint_cmd",
            "target_joint_from_vr",
            "target_finger_joints",
            "target_ee_poses",
            "target_gripper_status",
            "target_base_move",
        ):
            self.assertIn(
                f"/io_teleop/{topic}:=/hc_teleop/{topic}", launcher
            )

    def test_single_product_launcher_has_sim_and_teleop_modes(self):
        launcher = (self.root / "run.sh").read_text(encoding="utf-8")
        self.assertIn('"${PROJECT_ROOT}/middleware/start.sh"', launcher)
        self.assertIn('"${PROJECT_ROOT}/adapters/start.sh"', launcher)
        self.assertIn("general_sim_robot_control_node_ros2.py", launcher)
        self.assertIn("sim|teleop", launcher)
        for obsolete in (
            "start_teleop.sh",
            "start_real_robot_teleop.sh",
            "run_sim_teleop.sh",
            "run_simulator.sh",
        ):
            self.assertFalse((self.root / obsolete).exists(), obsolete)

    def test_dashboard_exposes_vr_exoskeleton_command_switch(self):
        html = (
            self.root / "middleware" / "core" / "static" / "index.html"
        ).read_text(encoding="utf-8")
        javascript = (
            self.root / "middleware" / "core" / "static" / "app.js"
        ).read_text(encoding="utf-8")
        config = yaml.safe_load(
            (self.root / "middleware" / "config.yaml").read_text(encoding="utf-8")
        )

        self.assertIn("状态监控", html)
        self.assertIn('id="sourceVr"', html)
        self.assertIn('id="sourceExoskeleton"', html)
        self.assertIn('id="jointMonitorRows"', html)
        self.assertIn("/api/teleop/source", javascript)
        self.assertEqual(
            config["ros"]["command_mux"]["output_topic"],
            "/hc_teleop/joint_cmd",
        )
        self.assertEqual(
            config["ros"]["command_mux"]["control_source_topic"],
            "/hc_teleop/control_source",
        )

    def test_simulator_updates_actual_flange_markers(self):
        source = (
            self.root / "simulation" / "general_sim_robot_control_node_ros2.py"
        ).read_text(encoding="utf-8")
        self.assertIn("self.actual_ee_frame_ids = [", source)
        self.assertIn(
            "self.actual_ee_frame_ids[index] = debug_draw_pose(",
            source,
        )
        self.assertIn("self.target_ee_frame_ids = [None] * len(self.arms)", source)

    def test_middleware_selection_exists_in_adapter_robot_store(self):
        config_path = self.root / "middleware" / "config.yaml"
        profile_id, manager = load_selection(config_path)
        self.assertEqual(manager.root, (self.root / "adapters" / "robots").resolve())
        self.assertIn(profile_id, [profile["id"] for profile in manager.list()])

        metadata = yaml.safe_load(
            (manager.root / profile_id / "profile.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(metadata["id"], profile_id)

    def test_x1_urdf_mesh_references_are_self_contained(self):
        profile = self.root / "adapters" / "robots" / "x1"
        urdf = profile / "urdf" / "hc_tj_robot.urdf"
        document = ElementTree.parse(urdf)
        filenames = {
            mesh.get("filename", "") for mesh in document.findall(".//mesh")
        }
        self.assertTrue(filenames)
        for filename in filenames:
            self.assertTrue((urdf.parent / filename).resolve().is_file(), filename)
        self.assertFalse((profile / "mesh").exists())

        simulation = yaml.safe_load(
            (profile / "vr_configs.yml").read_text(encoding="utf-8")
        )
        self.assertEqual(simulation["urdf_path"], "urdf/hc_tj_robot.urdf")
        self.assertEqual(len(simulation["arms"]), 2)


if __name__ == "__main__":
    unittest.main()
