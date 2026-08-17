import unittest
from pathlib import Path

import yaml


class ArmTeleopConfigTests(unittest.TestCase):
    def test_base_axis_remap_does_not_change_home_gesture_axis(self):
        path = Path(__file__).resolve().parents[1] / "arm_teleop.yaml"
        config = yaml.safe_load(path.read_text(encoding="utf-8"))

        self.assertEqual(config["body"]["stick_x_axis"], 3)
        self.assertEqual(config["body"]["stick_y_axis"], 2)
        self.assertEqual(config["control"]["home_gesture_axis"], 2)


if __name__ == "__main__":
    unittest.main()
