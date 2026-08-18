from __future__ import annotations

import io
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock

from mcap.writer import Writer as McapWriter
from mcap.well_known import MessageEncoding, SchemaEncoding
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

from hc_teleop_middleware.topic_player import TopicPlayer
from hc_teleop_middleware.topic_recorder import _get_msg_def


class TopicPlayerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.dir_path = Path(self.temp_dir.name)
        self.published_messages: list[tuple[str, str, bytes]] = []

        self.mock_ros = MagicMock()
        def mock_publish_raw(topic: str, msg_type: str, raw_data: bytes):
            self.published_messages.append((topic, msg_type, raw_data))
        self.mock_ros.publish_raw = mock_publish_raw

        # Create a sample MCAP recording
        self.test_mcap_path = self.dir_path / "test_recording.mcap"
        self._create_test_mcap(self.test_mcap_path)

        self.player = TopicPlayer(self.dir_path, lambda: self.mock_ros)

    def tearDown(self) -> None:
        self.player.stop()
        self.temp_dir.cleanup()

    def _create_test_mcap(self, path: Path) -> None:
        with path.open("wb") as stream:
            writer = McapWriter(stream)
            writer.start(profile="ros2")
            schema_data = _get_msg_def("sensor_msgs/msg/JointState")
            schema_id = writer.register_schema(
                "sensor_msgs/msg/JointState", SchemaEncoding.ROS2, schema_data
            )
            chan_state = writer.register_channel(
                "/hc_teleop/joint_states", MessageEncoding.CDR, schema_id
            )
            chan_cmd = writer.register_channel(
                "/hc_teleop/joint_cmd", MessageEncoding.CDR, schema_id
            )

            t0 = 1_000_000_000
            for i in range(5):
                msg = JointState()
                msg.name = ["joint1", "joint2"]
                msg.position = [float(i) * 0.1, float(i) * 0.2]
                raw = bytes(serialize_message(msg))
                # 10ms intervals
                writer.add_message(chan_state, t0 + i * 10_000_000, raw, t0 + i * 10_000_000)
                writer.add_message(chan_cmd, t0 + i * 10_000_000 + 1_000, raw, t0 + i * 10_000_000 + 1_000)
            writer.finish()

    def test_player_play_and_complete(self) -> None:
        status = self.player.play("test_recording.mcap", speed=10.0)
        self.assertEqual(status["state"], "playing")
        self.assertEqual(status["filename"], "test_recording.mcap")
        self.assertEqual(status["total_messages"], 10)

        # Wait for completion
        for _ in range(50):
            st = self.player.status()
            if st["state"] == "completed":
                break
            time.sleep(0.02)

        self.assertEqual(self.player.status()["state"], "completed")
        self.assertEqual(len(self.published_messages), 10)

    def test_player_topic_remapping(self) -> None:
        # Remap /hc_teleop/joint_states -> /hc_teleop/target_joint
        remap = {"/hc_teleop/joint_states": "/hc_teleop/target_joint"}
        self.player.play(
            "test_recording.mcap",
            speed=10.0,
            topic_remap=remap,
            selected_topics=["/hc_teleop/target_joint"],
        )

        for _ in range(50):
            st = self.player.status()
            if st["state"] == "completed":
                break
            time.sleep(0.02)

        self.assertEqual(len(self.published_messages), 5)
        for topic, msg_type, raw in self.published_messages:
            self.assertEqual(topic, "/hc_teleop/target_joint")

    def test_player_pause_resume_and_stop(self) -> None:
        self.player.play("test_recording.mcap", speed=0.1)
        time.sleep(0.05)
        self.assertEqual(self.player.pause()["state"], "paused")
        self.assertEqual(self.player.status()["state"], "paused")

        self.assertEqual(self.player.resume()["state"], "playing")
        self.assertEqual(self.player.status()["state"], "playing")

        self.assertEqual(self.player.stop()["state"], "idle")
        self.assertEqual(self.player.status()["state"], "idle")


if __name__ == "__main__":
    unittest.main()
