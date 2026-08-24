import tempfile
import unittest
from pathlib import Path

import yaml

from middleware.core.config import ConfigError, ConfigStore, validate_config


class ConfigTests(unittest.TestCase):
    def test_defaults_are_added(self):
        value = validate_config({"server": {"port": 9000}})
        self.assertEqual(value["server"]["port"], 9000)
        self.assertEqual(value["vr"]["pose_port"], 5005)
        self.assertEqual(value["robot_profiles"]["active"], "x1")
        self.assertEqual(
            value["robot_profiles"]["root"], "../adapters/robots"
        )
        self.assertEqual(value["vr"]["data_topic"], "/vrdata")
        self.assertEqual(value["ros"]["recording"]["directory"], "runtime/topic_recordings")

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

        depth = subscriptions["/hc_teleop/camera_head/depth/compressed"]
        self.assertEqual(depth["type"], "sensor_msgs/msg/CompressedImage")
        self.assertTrue(depth["enabled"])
        self.assertIn("record", depth["outputs"])
        self.assertEqual(depth["max_hz"], 0.0)

    def test_default_camera_recording_avoids_raw_and_duplicate_streams(self):
        path = Path(__file__).resolve().parents[1] / "middleware" / "config.yaml"
        config = validate_config(yaml.safe_load(path.read_text(encoding="utf-8")))
        subscriptions = {
            item["topic"]: item for item in config["ros"]["subscriptions"]
        }

        self.assertTrue(subscriptions["/hc_teleop/camera_head/color/compressed"]["enabled"])
        self.assertNotIn("/io_teleop/camera_head/color", subscriptions)
        self.assertNotIn("/io_teleop/camera_head/depth", subscriptions)
        self.assertFalse(subscriptions["/cameras/eye/color"]["enabled"])
        self.assertTrue(subscriptions["/hc_teleop/camera_overhead/color/compressed"]["enabled"])
        self.assertFalse(subscriptions["/cameras/Bfront/color"]["enabled"])


if __name__ == "__main__":
    unittest.main()
