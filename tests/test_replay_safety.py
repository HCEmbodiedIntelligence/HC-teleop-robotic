from __future__ import annotations

import unittest

from middleware.core.ros_bridge import RosBridge


class ReplaySafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = RosBridge(
            {
                "enabled": True,
                "domain_id": 14,
                "command_mux": {
                    "enabled": True,
                    "source": "vr",
                    "vr_topic": "/hc_teleop/joint_cmd_vr",
                    "exoskeleton_topic": "/hc_teleop/joint_cmd_exoskeleton",
                    "output_topic": "/hc_teleop/joint_cmd",
                    "control_source_topic": "/hc_teleop/control_source",
                },
                "subscriptions": [],
            },
            lambda _event, _outputs: None,
        )

    def test_final_command_rejects_generic_publish(self):
        self.assertFalse(
            self.bridge.publish(
                "/hc_teleop/joint_cmd",
                "sensor_msgs/msg/JointState",
                {"name": ["joint1"], "position": [0.1]},
            )
        )

    def test_replay_requires_readiness_and_disables_live_output_afterwards(self):
        self.assertFalse(self.bridge.begin_replay())
        self.bridge._hardware_ready = True
        self.assertTrue(self.bridge.begin_replay())
        self.assertTrue(
            self.bridge.publish_raw(
                "/hc_teleop/joint_cmd",
                "sensor_msgs/msg/JointState",
                b"serialized",
            )
        )
        command, _payload = self.bridge._commands.get_nowait()
        self.assertEqual(command, "replay_raw")
        self.bridge.end_replay(require_reset=True)
        status = self.bridge.status()["command_mux"]
        self.assertFalse(status["replay_active"])
        self.assertFalse(status["output_enabled"])
        self.assertFalse(
            self.bridge.publish_raw(
                "/hc_teleop/joint_cmd",
                "sensor_msgs/msg/JointState",
                b"serialized",
            )
        )


if __name__ == "__main__":
    unittest.main()
