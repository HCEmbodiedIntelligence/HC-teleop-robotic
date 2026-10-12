import json
from pathlib import Path
from unittest.mock import patch
from middleware.core.export_progress import ExportProgress, progress_path, read_progress
from middleware.core.topic_recorder import TopicRecorder


def test_atomic_throttled_monotonic_progress(tmp_path):
    path = progress_path(tmp_path / 'lerobot/episode')
    progress = ExportProgress(path)
    progress.update('读取 MCAP', 10, 10, 100, '条消息', force=True)
    progress.update('读取 MCAP', 20, 20, 100, '条消息')
    assert read_progress(path)['percent'] == 10
    progress.update('编码视频', 50, 1, 3, '路视频', force=True)
    other_process = ExportProgress(path)
    other_process.update('校验', 40, force=True)
    assert read_progress(path)['percent'] == 50
    assert not path.with_suffix('.json.tmp').exists()
    other_process.update('失败', 0, state='error', error='bad source', force=True)
    assert read_progress(path)['error'] == 'bad source'


def test_pending_visible_before_directory_exists_and_failure_persists(tmp_path):
    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path)
    destination = tmp_path / 'lerobot/episode'
    path = progress_path(destination)
    progress = ExportProgress(path)
    progress.update('编码视频', 70, 2, 3, '路视频', force=True)
    entries = recorder.list_recordings()
    assert len(entries) == 1 and entries[0]['is_current']
    assert entries[0]['id'] == 'lerobot:episode'
    assert entries[0]['export_progress']['percent'] == 70
    assert not destination.exists()
    progress.update('失败', 70, state='error', error='encoding failed', force=True)
    assert recorder.list_recordings()[0]['export_error'] == 'encoding failed'
    (destination / 'meta').mkdir(parents=True)
    (destination / 'meta/info.json').write_text(json.dumps({'total_frames': 1, 'total_episodes': 1, 'fps': 15, 'features': {}}))
    completed = recorder.list_recordings()
    assert len(completed) == 1 and not completed[0]['is_current']


def test_exited_worker_is_shown_as_interrupted(tmp_path):
    destination = tmp_path / 'lerobot/episode'
    ExportProgress(progress_path(destination)).update('编码视频', 70, force=True)
    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path)
    with patch('os.kill', side_effect=ProcessLookupError):
        entry = recorder.list_recordings()[0]
    assert entry['export_state'] == 'error'
    assert 'MCAP' in entry['export_error']


def test_successful_cli_retry_replaces_cached_managed_failure(tmp_path):
    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path)
    destination = tmp_path / 'lerobot/episode'
    recorder._lerobot_path = str(destination)
    recorder._export_state, recorder._export_error = 'error', 'original failure'
    progress = ExportProgress(progress_path(destination))
    progress.update('编码视频', 70, force=True)
    status = recorder.status()
    assert status['export_state'] == 'exporting' and status['export_error'] is None
    (destination / 'meta').mkdir(parents=True)
    (destination / 'meta/info.json').write_text('{}')
    progress.update('完成', 100, state='completed', force=True)
    status = recorder.status()
    assert status['export_state'] == 'completed' and status['export_error'] is None
