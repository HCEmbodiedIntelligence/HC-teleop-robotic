import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from middleware.core.topic_recorder import TopicRecorder


class LeRobotRecordingTests(unittest.TestCase):
    def test_missing_environment_does_not_start_partial_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TopicRecorder({'directory': directory, 'format': 'lerobot'}, Path(directory))
            with patch('middleware.core.lerobot_export.ensure_export_available', side_effect=RuntimeError('install environment')):
                with self.assertRaisesRegex(RuntimeError, 'install environment'):
                    recorder.start()
            self.assertFalse(recorder.status()['recording'])
            self.assertIsNone(recorder.path)

    def test_export_failure_keeps_source_and_reports_error(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = TopicRecorder({'directory': directory, 'format': 'both'}, root)
            recorder.path = root / 'session.mcap'
            recorder.path.write_bytes(b'source retained')
            process = Mock()
            process.poll.return_value = 2
            process.returncode = 2
            with patch('middleware.core.topic_recorder.subprocess.Popen', return_value=process) as launch:
                recorder._start_lerobot_export()
            self.assertIn('middleware.core.lerobot_export', launch.call_args.args[0])
            status = recorder.status()
            self.assertEqual(status['export_state'], 'error')
            self.assertIn('原始 MCAP 已保留', status['export_error'])
            self.assertEqual(recorder.path.read_bytes(), b'source retained')

    def test_completed_export_has_dataset_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            recorder = TopicRecorder({'directory': directory, 'format': 'both'}, root)
            recorder.path = root / 'episode.mcap'
            process = Mock()
            process.poll.return_value = None
            with patch('middleware.core.topic_recorder.subprocess.Popen', return_value=process):
                recorder._start_lerobot_export()
            self.assertEqual(recorder.status()['export_state'], 'exporting')
            dataset = Path(recorder._lerobot_path)
            (dataset / 'meta').mkdir(parents=True)
            (dataset / 'meta/info.json').write_text(json.dumps({'codebase_version': 'v3.0'}))
            process.poll.return_value = 0
            process.returncode = 0
            result = recorder.status()
            self.assertEqual(result['export_state'], 'completed')
            self.assertEqual(result['lerobot_path'], str(dataset))
            self.assertIsNone(result['export_error'])
