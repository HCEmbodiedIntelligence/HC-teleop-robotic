import tempfile
import unittest
from pathlib import Path

import yaml

from middleware.core.robot_profiles import (
    RobotProfileError,
    RobotProfileManager,
    STANDARD_TOPICS,
)


URDF = b"""<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base"/>
  <link name="torso"/>
  <link name="right_shoulder"/>
  <link name="right_hand"/>
  <link name="left_shoulder"/>
  <link name="left_hand"/>
  <joint name="waist" type="revolute">
    <parent link="base"/><child link="torso"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="right_mount" type="fixed">
    <parent link="torso"/><child link="right_shoulder"/>
  </joint>
  <joint name="right_joint" type="revolute">
    <parent link="right_shoulder"/><child link="right_hand"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="1" velocity="2"/>
  </joint>
  <joint name="left_mount" type="fixed">
    <parent link="torso"/><child link="left_shoulder"/>
  </joint>
  <joint name="left_joint" type="revolute">
    <parent link="left_shoulder"/><child link="left_hand"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="1" velocity="2"/>
  </joint>
</robot>
"""


def io_yaml(*, right_ee=2):
    return yaml.safe_dump(
        {
            "urdf_path": "old/model.urdf",
            "robot_name": "old_name",
            "base_pose": {
                "position": [0, 0, 0],
                "orientation": [0, 0, 0, 1],
            },
            "arms": [
                {"joint_index": [2], "ee_index": right_ee, "rest_j_pos": [0]},
                {"joint_index": [4], "ee_index": 4, "rest_j_pos": [0]},
            ],
            "folding_waist": {
                "joint_index": [0],
                "rest_j_pos": [0],
                "base": -1,
                "cmd_ee": 0,
            },
            "controller_indices": {"cmd_ee": [2, 4], "base": [1, 3]},
        },
        sort_keys=False,
    ).encode()


class RobotProfileTests(unittest.TestCase):
    def test_imports_io_yaml_and_generates_standard_v23_controller(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            profile = manager.import_profile(
                "test_robot",
                "Test Robot",
                "robot.urdf",
                URDF,
                "vr_configs.yml",
                io_yaml(),
            )

            self.assertEqual(profile["schema"], "hc-robot-config-v1")
            self.assertEqual(profile["joint_count"], 5)
            self.assertEqual(profile["free_joint_count"], 3)
            profile_path = Path(directory) / "test_robot"
            io_config = yaml.safe_load(
                (profile_path / "vr_configs.yml").read_text(encoding="utf-8")
            )
            controller = yaml.safe_load(
                (profile_path / "controller_v23.yml").read_text(encoding="utf-8")
            )
            teleop = yaml.safe_load(
                (profile_path / "arm_teleop.yaml").read_text(encoding="utf-8")
            )
            self.assertEqual(io_config["robot_name"], "test_robot")
            self.assertEqual(io_config["urdf_path"], "urdf/robot.urdf")
            self.assertEqual(
                controller["model"]["free_joints"],
                ["right_joint", "left_joint", "waist"],
            )
            self.assertEqual(
                controller["ros_interface"]["sub_topic"]["joint_state"],
                STANDARD_TOPICS["joint_state"],
            )
            self.assertEqual(
                controller["ros_interface"]["sub_topic"]["reset"],
                STANDARD_TOPICS["solver_reset"],
            )
            self.assertEqual(
                controller["ros_interface"]["pub_topic"]["reset_ack"],
                STANDARD_TOPICS["solver_reset_ack"],
            )
            self.assertEqual(len(controller["task"]["pose"]), 3)
            self.assertEqual(
                teleop["arms"]["right"]["joint_names"], ["right_joint"]
            )
            self.assertEqual(teleop["body"]["waist_joint_names"], ["waist"])
            self.assertEqual(teleop["robot"]["urdf_path"], "urdf/robot.urdf")
            self.assertTrue(profile["teleop_compatible"])

    def test_rejects_out_of_range_io_joint_index_without_creating_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            with self.assertRaisesRegex(RobotProfileError, "outside"):
                manager.import_profile(
                    "bad",
                    "Bad",
                    "robot.urdf",
                    URDF,
                    "vr_configs.yml",
                    io_yaml(right_ee=99),
                )
            self.assertFalse((Path(directory) / "bad").exists())

    def test_imported_v23_topics_and_urdf_path_are_normalized(self):
        source = {
            "ros_interface": {
                "sub_topic": {"joint_state": "/wrong"},
                "pub_topic": {"joint_target": "/wrong"},
            },
            "control": {"dt": 0.01},
            "model": {
                "urdf": "/tmp/wrong.urdf",
                "free_joints": ["right_joint", "left_joint"],
            },
            "task": {
                "pose": [
                    [["right_shoulder", "right_hand"], 5.0, 1.0],
                    [["left_shoulder", "left_hand"], 5.0, 1.0],
                ],
                "axis": [],
                "joint": [],
            },
            "limit": {"velocity": [180, 180]},
        }
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            manager.import_profile(
                "v23_robot",
                "V23 Robot",
                "robot.urdf",
                URDF,
                "controller.yml",
                yaml.safe_dump(source).encode(),
            )
            saved = yaml.safe_load(
                (Path(directory) / "v23_robot" / "controller_v23.yml").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(saved["model"]["urdf"], "urdf/robot.urdf")
            self.assertEqual(
                saved["ros_interface"]["pub_topic"]["joint_target"],
                STANDARD_TOPICS["joint_target"],
            )
            self.assertEqual(
                saved["ros_interface"]["sub_topic"]["reset"],
                STANDARD_TOPICS["solver_reset"],
            )
            self.assertEqual(
                saved["ros_interface"]["pub_topic"]["reset_ack"],
                STANDARD_TOPICS["solver_reset_ack"],
            )

    def test_rejects_duplicate_profile_without_overwriting_it(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            arguments = (
                "test_robot",
                "Test",
                "robot.urdf",
                URDF,
                "vr_configs.yml",
                io_yaml(),
            )
            manager.import_profile(*arguments)
            with self.assertRaisesRegex(RobotProfileError, "already exists"):
                manager.import_profile(*arguments, overwrite=False)
            # Default overwrite=True successfully updates the profile
            updated = manager.import_profile(*arguments, overwrite=True)
            self.assertEqual(updated["id"], "test_robot")

    def test_rejects_malformed_v23_limit_and_control_sections(self):
        source = {
            "control": "invalid",
            "model": {"urdf": "old.urdf", "free_joints": ["right_joint"]},
            "task": {
                "pose": [[["right_shoulder", "right_hand"], 5.0, 1.0]],
                "axis": [],
                "joint": [],
            },
            "limit": [],
        }
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            with self.assertRaisesRegex(RobotProfileError, "limit must be an object"):
                manager.import_profile(
                    "invalid_v23",
                    "Invalid",
                    "robot.urdf",
                    URDF,
                    "controller.yml",
                    yaml.safe_dump(source).encode(),
                )
            self.assertFalse((Path(directory) / "invalid_v23").exists())

            source["limit"] = {"velocity": [180]}
            with self.assertRaisesRegex(RobotProfileError, "control must be an object"):
                manager.import_profile(
                    "invalid_control",
                    "Invalid",
                    "robot.urdf",
                    URDF,
                    "controller.yml",
                    yaml.safe_dump(source).encode(),
                )

    def test_imports_zip_archive_with_urdf_yaml_and_meshes(self):
        import io
        import zipfile

        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, "w") as zf:
            zf.writestr("my_archive_robot/urdf/robot.urdf", URDF)
            zf.writestr("my_archive_robot/vr_configs.yml", io_yaml())
            zf.writestr("my_archive_robot/mesh/link.stl", b"dummy stl mesh content")

        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            profile = manager.import_archive(
                "my_archive_robot",
                "My Archive Robot",
                zip_buffer.getvalue(),
                "my_archive_robot.zip",
            )
            self.assertEqual(profile["id"], "my_archive_robot")
            self.assertEqual(profile["schema"], "hc-robot-config-v1")
            profile_path = Path(directory) / "my_archive_robot"
            self.assertTrue((profile_path / "urdf" / "robot.urdf").is_file())
            self.assertTrue((profile_path / "controller_v23.yml").is_file())
            self.assertTrue((profile_path / "mesh" / "link.stl").is_file())

    def test_archive_duplicate_respects_overwrite_flag(self):
        import io
        import zipfile

        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, "w") as zf:
            zf.writestr("archive_bot/urdf/robot.urdf", URDF)
            zf.writestr("archive_bot/vr_configs.yml", io_yaml())
        payload = zip_buffer.getvalue()
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            manager.import_archive("archive_bot", "Archive", payload)
            with self.assertRaisesRegex(RobotProfileError, "already exists"):
                manager.import_archive(
                    "archive_bot", "Archive", payload, overwrite=False
                )
            updated = manager.import_archive(
                "archive_bot", "Archive Updated", payload, overwrite=True
            )
            self.assertEqual(updated["display_name"], "Archive Updated")

    def test_delete_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            manager.import_profile(
                "del_robot", "Delete Robot", "robot.urdf", URDF, "vr_configs.yml", io_yaml()
            )
            self.assertTrue((Path(directory) / "del_robot").is_dir())
            manager.delete_profile("del_robot")
            self.assertFalse((Path(directory) / "del_robot").exists())
            with self.assertRaisesRegex(RobotProfileError, "does not exist"):
                manager.delete_profile("del_robot")


    def test_import_archive_with_arm_teleop_only(self):
        import io
        import zipfile

        teleop_yaml = yaml.safe_dump(
            {
                "robot": {
                    "urdf_path": "urdf/robot.urdf",
                    "base_position": [0.0, 0.0, 0.0],
                    "base_orientation": [0.0, 0.0, 0.0, 1.0],
                    "initial_joints": {
                        "right_joint": 0.5,
                        "left_joint": -0.5,
                    },
                },
                "arms": {
                    "right": {
                        "base_link": "base",
                        "generic_task_base_link": "right_shoulder",
                        "ee_link": "right_hand",
                        "joint_names": ["right_joint"],
                    },
                    "left": {
                        "base_link": "base",
                        "generic_task_base_link": "left_shoulder",
                        "ee_link": "left_hand",
                        "joint_names": ["left_joint"],
                    },
                },
            }
        )

        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, "w") as zf:
            zf.writestr("simple_bot/urdf/robot.urdf", URDF)
            zf.writestr("simple_bot/arm_teleop.yaml", teleop_yaml)

        with tempfile.TemporaryDirectory() as directory:
            manager = RobotProfileManager(directory)
            profile = manager.import_archive(
                "simple_bot",
                "Simple Bot",
                zip_buffer.getvalue(),
                "simple_bot.zip",
            )
            self.assertEqual(profile["id"], "simple_bot")
            self.assertEqual(profile["schema"], "arm-teleop-v1")
            self.assertEqual(profile["link_count"], 6)
            self.assertEqual(profile["joint_count"], 5)
            self.assertEqual(profile["movable_joint_count"], 3)
            self.assertEqual(profile["free_joint_count"], 2)
            self.assertEqual(profile["task_count"], 2)
            self.assertEqual(profile["arm_count"], 2)

            bot_dir = Path(directory) / "simple_bot"
            self.assertTrue((bot_dir / "controller_v23.yml").is_file())
            self.assertTrue((bot_dir / "vr_configs.yml").is_file())
            self.assertTrue((bot_dir / "profile.yaml").is_file())

            vr_cfg = yaml.safe_load((bot_dir / "vr_configs.yml").read_text())
            self.assertEqual(vr_cfg["controller_indices"]["base"], [-1, -1])
            self.assertEqual(vr_cfg["controller_indices"]["cmd_ee"], ["right_hand", "left_hand"])
            self.assertEqual(vr_cfg["arms"][0]["joint_index"], ["right_joint"])
            self.assertEqual(vr_cfg["arms"][0]["ee_index"], "right_hand")

    def test_ensure_profile_files_on_get(self):
        teleop_yaml = yaml.safe_dump(
            {
                "robot": {
                    "urdf_path": "urdf/robot.urdf",
                    "initial_joints": {"right_joint": 0.0, "left_joint": 0.0},
                },
                "arms": {
                    "right": {
                        "base_link": "base",
                        "generic_task_base_link": "right_shoulder",
                        "ee_link": "right_hand",
                        "joint_names": ["right_joint"],
                    },
                    "left": {
                        "base_link": "base",
                        "generic_task_base_link": "left_shoulder",
                        "ee_link": "left_hand",
                        "joint_names": ["left_joint"],
                    },
                },
            }
        )
        with tempfile.TemporaryDirectory() as directory:
            bot_dir = Path(directory) / "manual_bot"
            (bot_dir / "urdf").mkdir(parents=True)
            (bot_dir / "urdf" / "robot.urdf").write_bytes(URDF)
            (bot_dir / "arm_teleop.yaml").write_text(teleop_yaml)

            manager = RobotProfileManager(directory)
            profile = manager.get("manual_bot")
            self.assertEqual(profile["link_count"], 6)
            self.assertEqual(profile["joint_count"], 5)
            self.assertTrue((bot_dir / "controller_v23.yml").is_file())
            self.assertTrue((bot_dir / "vr_configs.yml").is_file())
            self.assertTrue((bot_dir / "profile.yaml").is_file())


if __name__ == "__main__":
    unittest.main()

