import tempfile
import unittest
from pathlib import Path

import yaml

from middleware.core.config import ConfigError, ConfigStore, validate_config


class ConfigTests(unittest.TestCase):
    def test_image_transport_qos_is_preserved_and_validated(self):
        item = {"topic": "/camera", "type": "sensor_msgs/msg/Image", "qos_reliability": "reliable"}
        value = validate_config({"ros": {"subscriptions": [item]}})
        self.assertEqual(value["ros"]["subscriptions"][0]["qos_reliability"], "reliable")
        item["qos_reliability"] = "invalid"
        with self.assertRaisesRegex(ConfigError, "qos_reliability"):
            validate_config({"ros": {"subscriptions": [item]}})

    def test_defaults_are_added(self):
        value = validate_config({"server": {"port": 9000}})
        self.assertEqual(value["server"]["port"], 9000)
        self.assertEqual(value["vr"]["pose_port"], 5005)
        self.assertEqual(value["robot_profiles"]["active"], "x1")
        self.assertEqual(
            value["robot_profiles"]["root"], "../adapters/robots"
        )
        self.assertEqual(value["vr"]["data_topic"], "/vrdata")
        self.assertEqual(value["ros"]["domain_id"], 14)
        self.assertEqual(value["ros"]["recording"]["directory"], "runtime/topic_recordings")
        self.assertEqual(value["ros"]["command_mux"]["source"], "vr")
        self.assertEqual(
            value["ros"]["command_mux"]["exoskeleton_topic"],
            "/control/exoskeleton/joint_states",
        )
        self.assertEqual(
            value["ros"]["command_mux"]["control_source_topic"],
            "/control/source",
        )

    def test_command_mux_requires_distinct_topics_and_known_source(self):
        with self.assertRaisesRegex(ConfigError, "source must be vr or exoskeleton"):
            validate_config({"ros": {"command_mux": {"source": "unknown"}}})
        with self.assertRaisesRegex(ConfigError, "topics must be different"):
            validate_config(
                {
                    "ros": {
                        "command_mux": {
                            "vr_topic": "/same",
                            "exoskeleton_topic": "/same",
                        }
                    }
                }
            )

    def test_legacy_vr_topics_are_removed(self):
        value = validate_config(
            {
                "vr": {
                    "pose_topics": {"head": "/old"},
                    "input_topics": {"left": "/old"},
                    "event_topic": "/old",
                }
            }
        )
        self.assertNotIn("pose_topics", value["vr"])
        self.assertNotIn("input_topics", value["vr"])
        self.assertNotIn("event_topic", value["vr"])

    def test_invalid_subscription_is_rejected(self):
        with self.assertRaises(ConfigError):
            validate_config(
                {"ros": {"subscriptions": [{"topic": "no-slash", "type": "bad"}]}}
            )

    def test_invalid_robot_profile_section_is_rejected(self):
        with self.assertRaisesRegex(ConfigError, "robot_profiles must be an object"):
            validate_config({"robot_profiles": []})

    def test_domain_id_validation(self):
        value = validate_config({"ros": {"domain_id": 42}})
        self.assertEqual(value["ros"]["domain_id"], 42)
        with self.assertRaisesRegex(ConfigError, "ros.domain_id"):
            validate_config({"ros": {"domain_id": 300}})
        with self.assertRaisesRegex(ConfigError, "ros.domain_id"):
            validate_config({"ros": {"domain_id": -1}})

    def test_subscription_description_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ConfigStore(Path(directory) / "config.yaml")
            value = store.load()
            value["ros"]["subscriptions"] = [{
                "topic": "/observation/test", "type": "sensor_msgs/msg/JointState",
                "description": "左臂反馈：用于对比 <目标> 与实际动作", "outputs": ["record"]}]
            store.save(value)
            saved = ConfigStore(store.path).load()
            self.assertEqual(saved["ros"]["subscriptions"][0]["description"],
                             value["ros"]["subscriptions"][0]["description"])

    def test_store_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ConfigStore(Path(directory) / "config.yaml")
            value = store.load()
            value["camera"]["enabled"] = True
            value["ros"]["domain_id"] = 15
            store.save(value)
            saved = ConfigStore(store.path).load()
            self.assertTrue(saved["camera"]["enabled"])
            self.assertEqual(saved["ros"]["domain_id"], 15)

    def test_head_depth_is_enabled_for_recording(self):
        path = Path(__file__).resolve().parents[1] / "middleware" / "config.yaml"
        config = validate_config(yaml.safe_load(path.read_text(encoding="utf-8")))
        subscriptions = {
            item["topic"]: item for item in config["ros"]["subscriptions"]
        }

        depth = subscriptions["/observation/cameras/head/depth/compressed"]
        self.assertEqual(depth["type"], "sensor_msgs/msg/CompressedImage")
        self.assertTrue(depth["enabled"])
        self.assertIn("record", depth["outputs"])
        self.assertEqual(depth["max_hz"], 0.0)

    def test_joint_angles_are_streamed_to_dashboard_without_limiting_recording(self):
        path = Path(__file__).resolve().parents[1] / "middleware" / "config.yaml"
        config = validate_config(yaml.safe_load(path.read_text(encoding="utf-8")))
        subscriptions = {
            item["topic"]: item for item in config["ros"]["subscriptions"]
        }

        for topic in ("/action/joint_states", "/observation/joint_states"):
            item = subscriptions[topic]
            self.assertIn("websocket", item["outputs"])
            self.assertIn("record", item["outputs"])
            self.assertEqual(item["max_hz"], 0.0)
            self.assertEqual(item["event_max_hz"], 20.0)

    def test_selected_isaac_camera_is_enabled_without_duplicate_legacy_topics(self):
        path = Path(__file__).resolve().parents[1] / "middleware" / "config.yaml"
        config = validate_config(yaml.safe_load(path.read_text(encoding="utf-8")))
        subscriptions = {
            item["topic"]: item for item in config["ros"]["subscriptions"]
        }

        self.assertTrue(subscriptions["/observation/cameras/head/color/compressed"]["enabled"])
        self.assertNotIn("/io_teleop/camera_head/color", subscriptions)
        self.assertNotIn("/io_teleop/camera_head/depth", subscriptions)
        for topic in ("/cameras/eye/color", "/observation/cameras/wrist_left/color", "/observation/cameras/wrist_right/color"):
            self.assertFalse(subscriptions[topic]["enabled"])
            compressed = subscriptions[topic + "/compressed"]
            self.assertTrue(compressed["enabled"])
            self.assertEqual(compressed["type"], "sensor_msgs/msg/CompressedImage")
            self.assertIn("record", compressed["outputs"])
        self.assertTrue(subscriptions["/observation/cameras/overhead/color/compressed"]["enabled"])
        self.assertFalse(subscriptions["/cameras/Bfront/color"]["enabled"])

    def test_recording_save_modes_and_lerobot_options(self):
        for mode in ("mcap", "lerobot", "both"):
            cfg = validate_config({"ros": {"recording": {"format": mode, "lerobot_fps": 15, "lerobot_task": "夹取箱子"}}})
            self.assertEqual(cfg["ros"]["recording"]["format"], mode)
            self.assertEqual(cfg["ros"]["recording"]["lerobot_task"], "夹取箱子")
        for settings in ({"format": "csv"}, {"lerobot_fps": 0}, {"lerobot_fps": 61},
                         {"lerobot_fps": 15.5}, {"lerobot_fps": True}, {"lerobot_task": " "}):
            with self.assertRaises(ConfigError):
                validate_config({"ros": {"recording": settings}})

    def test_multiple_camera_streams_are_normalized(self):
        value = validate_config(
            {
                "camera": {
                    "enabled": True,
                    "streams": [
                        {"id": "head", "name": "Head", "topic": "camera/head"},
                        {"id": "left_wrist", "topic": "/camera/left"},
                    ],
                }
            }
        )
        streams = value["camera"]["streams"]
        self.assertEqual([item["id"] for item in streams], ["head", "left_wrist"])
        self.assertEqual(streams[0]["topic"], "/camera/head")
        self.assertEqual(streams[1]["name"], "left_wrist")

        with self.assertRaisesRegex(ConfigError, "duplicate camera stream id"):
            validate_config(
                {
                    "camera": {
                        "streams": [
                            {"id": "same", "topic": "/one"},
                            {"id": "same", "topic": "/two"},
                        ]
                    }
                }
            )

    def test_invalid_camera_encoder_settings_are_rejected(self):
        with self.assertRaisesRegex(ConfigError, "camera fps"):
            validate_config({"camera": {"fps": 0}})
        with self.assertRaisesRegex(ConfigError, "camera codec"):
            validate_config({"camera": {"codec": "AV1"}})
        with self.assertRaisesRegex(ConfigError, r"streams\[0\] width"):
            validate_config(
                {
                    "camera": {
                        "streams": [
                            {"id": "head", "topic": "/camera/head", "width": 0}
                        ]
                    }
                }
            )


if __name__ == "__main__":
    unittest.main()
