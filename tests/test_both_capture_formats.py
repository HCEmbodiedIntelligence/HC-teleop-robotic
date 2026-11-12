"""One physical recording, two immutable format receipts and independent retries."""
import copy
from pathlib import Path

import pytest

from humanoid_upload.capture_plan import reference
from humanoid_upload.common import UploadError
from middleware.core.capture_formats import lerobot_snapshot
from middleware.core.topic_recorder import read_recording_metadata
from middleware.core.export_progress import ExportProgress, progress_path
from test_platform_task_recording import choice, finish
from test_shared_capture_plans import environment, recorder_for, record, server


@pytest.fixture(autouse=True)
def shared_output(server, monkeypatch):
    # One published multi-format scheme has one output contract and plan digest.
    task = server.task
    def common(task_id, fmt):
        result = task(task_id, fmt)
        base = task(task_id, 'MCAP')['capture_plan']
        result['capture_plan'].update(output=base['output'], plan_sha256=base['plan_sha256'])
        return result
    monkeypatch.setattr(server, 'task', common)


def test_both_uploads_two_formats_and_retries_without_duplicate(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server, 'both')
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    recorder.set_upload_format('both')
    try:
        source = record(recorder, choice(platform, 'A', 'both-start'))
        job = recorder.records()[0]
        queued = platform.upload.queue.jobs()
        assert len(server.snapshots) == len(queued) == 2
        assert {q['manifest']['source_format'] for q in queued} == {'MCAP', 'LEROBOT_V3'}
        assert {q['local_recording_id'] for q in queued} == {job['recording_id'], job['recording_id']+'-lerobot'}
        assert all(q['manifest']['capture_plan'] == reference(q['capture_snapshot']) for q in queued)
        assert lerobot_snapshot(read_recording_metadata(Path(source)))['recording_id'] == job['recording_id']+'-lerobot'
        before = copy.deepcopy([q['manifest'] for q in queued])
        platform.upload.step(scan=False)
        platform.upload.step(scan=False)
        recorder.upload_recording(job['recording_id'])
        finish(recorder)
        platform.upload.step(scan=False)
        platform.upload.step(scan=False)
        assert len(server.uploads) == 2
        assert [q['manifest'] for q in platform.upload.queue.jobs()] == before
        assert recorder.start(selection=choice(platform, 'A', 'both-start')) == source
    finally:
        recorder.close()


def test_both_export_failure_keeps_mcap_queued_and_can_retry_lerobot(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server, 'both')
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    recorder.set_upload_format('both')
    convert = recorder.recorder._start_lerobot_export
    def failed_export():
        target = recorder.directory/'lerobot'/recorder.path.stem
        ExportProgress(progress_path(target)).update('failed', 0, state='error', force=True)
    monkeypatch.setattr(recorder.recorder, '_start_lerobot_export', failed_export)
    try:
        recorder.start(selection=choice(platform, 'A'))
        recorder.record({'topic': '/test', 'payload': {'synthetic': True}})
        recorder.stop()
        finish(recorder)
        job = recorder.records()[0]
        assert job['state'] == 'needs_attention'
        assert job['artifacts']['mcap']['state'] == 'queued'
        assert job['artifacts']['lerobot']['code'] == 'EXPORT_FAILED'
        assert len(platform.upload.queue.jobs()) == 1
        convert()
        recorder.upload_recording('lerobot:'+Path(job['path']).stem)
        finish(recorder)
        assert len(platform.upload.queue.jobs()) == 2
        assert recorder.records()[0]['state'] == 'queued'
    finally:
        recorder.close()


def test_both_does_not_start_when_second_adapter_unavailable(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server, 'both')
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    recorder.set_upload_format('both')
    preflight = platform.preflight
    def check(*args, **kwargs):
        if kwargs.get('source_format') == 'LEROBOT_V3':
            raise UploadError('CAPTURE_FORMAT_UNSUPPORTED')
        return preflight(*args, **kwargs)
    monkeypatch.setattr(platform, 'preflight', check)
    try:
        with pytest.raises(UploadError, match='CAPTURE_FORMAT_UNSUPPORTED'):
            recorder.start(selection=choice(platform, 'A'))
        assert not recorder.is_recording()
        assert not list(recorder.directory.glob('*.mcap'))
        assert not server.snapshots
    finally:
        recorder.close()
