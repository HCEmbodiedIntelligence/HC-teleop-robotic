import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from mcap.reader import make_reader
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from middleware.core.topic_recorder import TopicRecorder


class TopicRecorderTests(unittest.TestCase):
    def test_unchanged_mcap_inspection_is_cached(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TopicRecorder({"directory": "recordings"}, Path(directory))
            recorder.directory.mkdir(parents=True)
            (recorder.directory / 'cached.mcap').write_bytes(b'cached')
            metadata = {
                'duration_sec': 1.0, 'duration_human': '1.0s', 'message_count': 2,
                'topic_count': 1, 'channels': [], 'avg_rate_hz': 2.0,
                'profile_id': '', 'profile_display_name': '', 'robot_name': '',
                'controller_config': '', 'free_joints': [],
            }
            with patch('middleware.core.topic_recorder._inspect_mcap_file', return_value=metadata) as inspect:
                recorder.list_recordings('mcap')
                recorder.list_recordings('mcap')
            inspect.assert_called_once()

    def test_batch_lerobot_conversion_is_queued_in_one_process(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TopicRecorder({"directory": "recordings", "lerobot_fps": 20}, Path(directory))
            recorder.directory.mkdir(parents=True)
            (recorder.directory / 'one.mcap').write_bytes(b'one')
            (recorder.directory / 'two.mcap').write_bytes(b'two')

            class Process:
                pid = 31415

            with patch('middleware.core.lerobot_export.ensure_export_available'), \
                    patch('middleware.core.topic_recorder.subprocess.Popen', return_value=Process()) as spawn:
                result = recorder.convert_recordings_to_lerobot(['one.mcap', 'two.mcap'])

            self.assertEqual(result['accepted'], ['one.mcap', 'two.mcap'])
            self.assertEqual(result['pid'], 31415)
            spawn.assert_called_once()
            spec_path = Path(spawn.call_args.args[0][-1])
            spec = json.loads(spec_path.read_text())
            self.assertEqual([Path(item['source']).name for item in spec['items']], ['one.mcap', 'two.mcap'])
            self.assertEqual([item['fps'] for item in spec['items']], [20, 20])
    def test_records_16bit_png_depth_as_ros2_cdr(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TopicRecorder({"directory": "recordings"}, Path(directory))
            recorder.start("depth.mcap")
            depth = CompressedImage(
                format="16UC1; png",
                data=b"\x89PNG\r\n\x1a\nsynthetic-depth",
            )
            recorder.record(
                {
                    "kind": "ros_message",
                    "topic": "/io_teleop/camera_head/depth",
                    "msg_type": "sensor_msgs/msg/CompressedImage",
                    "_raw": bytes(serialize_message(depth)),
                }
            )
            recorder.stop()

            with recorder.path.open("rb") as stream:
                messages = list(make_reader(stream).iter_messages())
            self.assertEqual(len(messages), 1)
            schema, channel, message = messages[0]
            self.assertEqual(channel.topic, "/io_teleop/camera_head/depth")
            self.assertEqual(schema.name, "sensor_msgs/msg/CompressedImage")
            restored = deserialize_message(message.data, CompressedImage)
            self.assertEqual(restored.format, "16UC1; png")
            self.assertTrue(bytes(restored.data).startswith(b"\x89PNG"))

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

    def test_records_and_lists_robot_profile_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TopicRecorder({"directory": "recordings"}, Path(directory))
            recorder.start(
                "openarm_session.mcap",
                {
                    "profile_id": "openarmx",
                    "profile_display_name": "OpenArmX Bimanual",
                    "robot_name": "openarmx",
                    "controller_config": "controller_v23.yml",
                    "free_joints": '["openarmx_right_joint1"]',
                },
            )
            recorder.record({"kind": "event", "data": 1})
            recorder.stop()

            item = recorder.list_recordings()[0]
            self.assertEqual(item["profile_id"], "openarmx")
            self.assertEqual(item["profile_display_name"], "OpenArmX Bimanual")
            self.assertEqual(item["robot_name"], "openarmx")
            self.assertEqual(item["free_joints"], ["openarmx_right_joint1"])

            with recorder.path.open("rb") as stream:
                metadata = list(make_reader(stream).iter_metadata())
            self.assertEqual(len(metadata), 1)
            self.assertEqual(metadata[0].name, "hc_teleop.session")
            self.assertEqual(metadata[0].metadata["profile_id"], "openarmx")

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

            mark = recorder.mark_current_or_latest("test")
            self.assertEqual(mark["filename"], "run_2.mcap")
            self.assertTrue(mark["recording"])
            r2_item = next(f for f in recorder.list_recordings() if f["filename"] == "run_2.mcap")
            self.assertTrue(r2_item["marked"])
            self.assertEqual(r2_item["mark_source"], "test")

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
            self.assertFalse((recorder.directory / ".run_2.mcap.mark.json").exists())


if __name__ == "__main__":
    unittest.main()
