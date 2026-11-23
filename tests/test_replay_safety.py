from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

from middleware.core.control.ros_bridge import RosBridge


class ReplaySafetyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.bridge = RosBridge(
            {
                "enabled": True,
                "domain_id": 14,
                "command_mux": {
                    "enabled": True,
                    "source": "vr",
                    "vr_topic": "/control/vr/joint_states",
                    "exoskeleton_topic": "/control/exoskeleton/joint_states",
                    "output_topic": "/action/joint_states",
                    "control_source_topic": "/control/source",
                },
                "subscriptions": [],
            },
            lambda _event, _outputs: None,
        )
        self.bridge._replay_home_publisher = Mock()
        self.bridge._replay_home_publisher.get_subscription_count.return_value = 1

    def test_final_command_rejects_generic_publish(self):
        self.assertFalse(
            self.bridge.publish(
                "/action/joint_states",
                "sensor_msgs/msg/JointState",
                {"name": ["joint1"], "position": [0.1]},
            )
        )

    def test_replay_requires_readiness_and_disables_live_output_afterwards(self):
        self.assertFalse(self.bridge.begin_replay())
        self.bridge.receive_hardware_ready(True)
        self.assertTrue(self.bridge.begin_replay())
        self.assertTrue(
            self.bridge.publish_raw(
                "/action/joint_states",
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
                "/action/joint_states",
                "sensor_msgs/msg/JointState",
                b"serialized",
            )
        )

    def test_hardware_smoother_output_and_non_ros_event_schemas_rejected(self):
        self.bridge.receive_hardware_ready(True)
        self.assertTrue(self.bridge.begin_replay())
        self.assertFalse(
            self.bridge.publish(
                "/control/joint_cmd",
                "sensor_msgs/msg/JointState",
                {"name": ["joint1"], "position": [0.1]},
            )
        )
        self.assertFalse(
            self.bridge.publish_raw(
                "/control/joint_cmd",
                "sensor_msgs/msg/JointState",
                b"serialized",
            )
        )
        self.assertFalse(
            self.bridge.publish_raw(
                "/events/recording/action_provenance",
                "teleop_event",
                b"{}",
            )
        )

    def test_return_home_requires_completed_replay_and_dispatches_once(self):
        self.bridge.receive_hardware_ready(True)
        with self.assertRaises(ValueError):
            self.bridge.request_replay_home()
        self.assertTrue(self.bridge.begin_replay())
        with self.assertRaises(ValueError):
            self.bridge.request_replay_home()
        self.bridge.end_replay()
        self.assertTrue(self.bridge.request_replay_home())
        with self.assertRaises(ValueError):
            self.bridge.request_replay_home()
        publisher = Mock()
        with patch.object(self.bridge, '_publisher', return_value=publisher) as lookup:
            self.bridge._drain_commands(Mock(), {}, Mock(), Mock())
        lookup.assert_called_once()
        self.assertEqual(lookup.call_args.args[2], '/teleop/arm/home')
        self.assertTrue(publisher.publish.call_args.args[0].data)
        self.assertTrue(self.bridge.status()['command_mux']['output_enabled'])

    def test_emergency_stop_fences_queued_replay_home(self):
        self.bridge.receive_hardware_ready(True)
        self.assertTrue(self.bridge.begin_replay())
        self.bridge.end_replay()
        self.assertTrue(self.bridge.request_replay_home())
        self.bridge.emergency_stop('/control/emergency_stop', 'test stop')
        with patch.object(self.bridge, '_publisher', return_value=Mock()) as lookup:
            self.bridge._drain_commands(Mock(), {}, Mock(), Mock())
        self.assertNotIn('/teleop/arm/home', [call.args[2] for call in lookup.call_args_list])
        self.assertFalse(self.bridge.status()['command_mux']['output_enabled'])

    def test_replay_home_rejects_hardware_not_ready(self):
        self.bridge.receive_hardware_ready(True)
        self.assertTrue(self.bridge.begin_replay())
        self.bridge.end_replay()
        self.bridge.receive_hardware_ready(False)
        with self.assertRaises(ValueError):
            self.bridge.request_replay_home()
        self.assertTrue(self.bridge._commands.empty())

    def test_replay_home_waits_for_subscriber_without_consuming_request(self):
        self.bridge.receive_hardware_ready(True)
        self.assertTrue(self.bridge.begin_replay())
        self.bridge.end_replay()
        count = self.bridge._replay_home_publisher.get_subscription_count
        count.return_value = 0
        with self.assertRaisesRegex(ValueError, '节点尚未连接'):
            self.bridge.request_replay_home()
        count.return_value = 1
        self.assertTrue(self.bridge.request_replay_home())

    def test_raw_replay_cannot_impersonate_feedback_safety_or_solver(self):
        self.bridge.receive_hardware_ready(True)
        self.assertTrue(self.bridge.begin_replay())
        for topic in ("/observation/joint_states", "/observation/hands/joint_states",
                      "/status/solver/reset_ack", "/control/solver/reset",
                      "/control/source", "/control/teleop/enabled",
                      "/control/motion/permit", "/events/teleop/clutch", "/vrdata"):
            with self.subTest(topic=topic):
                self.assertFalse(self.bridge.publish_raw(topic, "sensor_msgs/msg/JointState", b"old"))
        self.assertTrue(self.bridge._commands.empty())


if __name__ == "__main__":
    unittest.main()
