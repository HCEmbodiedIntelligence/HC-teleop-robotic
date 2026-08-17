import tempfile
import unittest
from pathlib import Path

from hc_teleop_middleware.config import ConfigError, ConfigStore, validate_config


class ConfigTests(unittest.TestCase):
    def test_defaults_are_added(self):
        value = validate_config({"server": {"port": 9000}})
        self.assertEqual(value["server"]["port"], 9000)
        self.assertEqual(value["vr"]["pose_port"], 5005)

    def test_invalid_subscription_is_rejected(self):
        with self.assertRaises(ConfigError):
            validate_config(
                {"ros": {"subscriptions": [{"topic": "no-slash", "type": "bad"}]}}
            )

    def test_store_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ConfigStore(Path(directory) / "config.yaml")
            value = store.load()
            value["camera"]["enabled"] = True
            store.save(value)
            self.assertTrue(ConfigStore(store.path).load()["camera"]["enabled"])


if __name__ == "__main__":
    unittest.main()
