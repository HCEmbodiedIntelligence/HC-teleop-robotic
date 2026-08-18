import json
import tempfile
import unittest
from pathlib import Path

from mcap.reader import make_reader
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from hc_teleop_middleware.topic_recorder import TopicRecorder


class TopicRecorderTests(unittest.TestCase):
    def test_records_ros_events_as_mcap(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = TopicRecorder(
                {"directory": "recordings"}, root
            )
            recorder.start()
            msg = String(data="test_payload")
            recorder.record(
                {
                    "kind": "ros_message",
                    "topic": "/hc_teleop/test",
                    "msg_type": "std_msgs/msg/String",
                    "_raw": bytes(serialize_message(msg)),
                    "payload": {"data": "test_payload"},
                }
            )
            recorder.stop()

            self.assertIsNotNone(recorder.path)
            self.assertTrue(recorder.path.name.endswith(".mcap"))
            with recorder.path.open("rb") as f:
                reader = make_reader(f)
                messages = list(reader.iter_messages())
                self.assertEqual(len(messages), 1)
                schema, channel, message = messages[0]
                self.assertEqual(channel.topic, "/hc_teleop/test")
                self.assertEqual(schema.name, "std_msgs/msg/String")
            self.assertEqual(recorder.status()["messages"], 1)

    def test_dynamic_start_stop_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = TopicRecorder(
                {"directory": "recordings"}, root
            )
            self.assertFalse(recorder.status()["recording"])
            path_str = recorder.start("custom_run.mcap")
            self.assertTrue(recorder.status()["recording"])
            recorder.record({"kind": "event", "data": 123})
            status = recorder.stop()
            self.assertFalse(status["recording"])
            self.assertEqual(status["messages"], 1)
            self.assertTrue(Path(path_str).is_file())
            self.assertTrue(path_str.endswith(".mcap"))

    def test_list_and_delete_recordings(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = TopicRecorder(
                {"directory": "recordings"}, root
            )
            # Create two recordings
            p1 = recorder.start("run_1.mcap")
            recorder.record({"kind": "event", "idx": 1})
            recorder.stop()

            p2 = recorder.start("run_2.mcap")
            recorder.record({"kind": "event", "idx": 2})

            files = recorder.list_recordings()
            self.assertEqual(len(files), 2)
            # run_2 is actively recording
            r2_item = next(f for f in files if f["filename"] == "run_2.mcap")
            self.assertTrue(r2_item["is_current"])

            # Cannot delete active file
            with self.assertRaises(ValueError):
                recorder.delete_recording("run_2.mcap")

            # Can delete completed file
            recorder.delete_recording("run_1.mcap")
            self.assertFalse(Path(p1).exists())
            self.assertEqual(len(recorder.list_recordings()), 1)

            recorder.stop()
            recorder.delete_recording("run_2.mcap")
            self.assertEqual(len(recorder.list_recordings()), 0)


if __name__ == "__main__":
    unittest.main()
