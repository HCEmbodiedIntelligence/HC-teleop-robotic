"""Main recorder handoff regression: no ROS or external network required."""
import json
from pathlib import Path

import pytest
from mcap.writer import Writer
from middleware.core.recording_artifacts import RecordingArtifacts
from middleware.core.export_progress import ExportProgress, progress_path
from middleware.core.upload_service import UploadService
from humanoid_upload.artifacts import ArtifactLocks
from humanoid_upload.common import UploadError


def manager(tmp_path):
    data = tmp_path / 'data'
    data.mkdir(exist_ok=True)
    cfg = {'artifact_state_dir': str(tmp_path/'ready'), 'upload_job_state_dir': str(tmp_path/'jobs')}
    obj = RecordingArtifacts(cfg, tmp_path, data)
    obj.shutdown()  # Drive handoff deterministically with poll().
    return obj


def source(obj, name='episode.mcap'):
    path = obj.directory / name
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema(name='example', encoding='jsonschema', data=b'{}')
        channel = writer.register_channel(topic='/example', message_encoding='json', schema_id=schema)
        writer.add_message(channel_id=channel, log_time=100, publish_time=100, data=b'{}')
        writer.finish()
    return path


def service(obj, tmp_path):
    return UploadService({'enabled': True, 'auto_enqueue': True,
        'queue_directory': str(tmp_path/'queue'), 'platform_base_url': 'http://127.0.0.1:12345',
        'credential_ref': 'env:TEST_UPLOAD_TOKEN', 'robot_id': 'robot', 'local_robot_id': 'local', 'task_id': 'first'},
        artifact_state_dir=obj.state_dir, data_directory=obj.directory)


def test_auto_enqueue_freezes_capture_binding_and_survives_restart(tmp_path, monkeypatch):
    monkeypatch.setenv('TEST_UPLOAD_TOKEN', 'synthetic-token')
    obj = manager(tmp_path)
    upload = service(obj, tmp_path)
    binding = upload.binding_snapshot()
    identity = obj.begin(source(obj), 'mcap', {'profile_id': 'x1'}, binding)
    obj.poll()
    assert not list(obj.state_dir.glob('*.json'))  # Never enqueue an open capture.
    upload.set_binding(dict(platform_base_url='http://127.0.0.1:12345', credential_ref='env:TEST_UPLOAD_TOKEN', robot_id='robot', local_robot_id='local', task_id='second'))
    obj.close(identity)
    obj.poll()
    ready = json.loads((obj.state_dir/(identity+'-mcap.json')).read_text())
    assert ready['binding_id'] == binding['binding_id']
    assert ready['capture_started_at'] <= ready['capture_ended_at']
    assert upload.scan()['errors'] == {}
    assert upload.queue.jobs()[0]['frozen']['task_id'] == 'first'
    recovered = service(obj, tmp_path)
    recovered.scan()
    assert len(recovered.queue.jobs()) == 1


def test_unbound_capture_not_automatically_rebound(tmp_path, monkeypatch):
    monkeypatch.setenv('TEST_UPLOAD_TOKEN', 'synthetic-token')
    obj = manager(tmp_path)
    identity = obj.begin(source(obj), 'mcap', {'profile_id': 'x1'})
    obj.close(identity)
    obj.poll()
    upload = service(obj, tmp_path)
    assert upload.scan()['enqueued'] == 0
    upload.enqueue(identity+'-mcap')  # Explicit historical selection is supported.
    assert len(upload.queue.jobs()) == 1


def test_incomplete_writer_and_interrupted_capture_not_ready(tmp_path):
    obj = manager(tmp_path)
    path = obj.directory/'broken.mcap'
    path.write_bytes(b'\x89MCAP0\r\nnot-closed')
    identity = obj.begin(path, 'mcap', {})
    obj.close(identity)
    obj.poll()
    assert not list(obj.state_dir.glob('*.json'))
    assert 'footer' in obj.jobs[identity]['error']
    live = obj.begin(source(obj), 'mcap', {})
    recovered = manager(tmp_path)
    assert recovered.jobs[live]['state'] == 'interrupted'
    assert not (obj.state_dir/(live+'-mcap.json')).exists()


def test_delete_cannot_race_uploader_or_republish(tmp_path):
    obj = manager(tmp_path)
    path = source(obj)
    identity = obj.begin(path, 'mcap', {})
    obj.close(identity)
    obj.poll()
    locks = ArtifactLocks(obj.state_dir)
    with locks.read(identity+'-mcap'):
        with pytest.raises(UploadError, match='ARTIFACT_BUSY'):
            with obj.deleting(path):
                path.unlink()
    assert path.exists()
    with obj.deleting(path):
        path.unlink()
    obj.poll()
    assert locks.tombstone(identity+'-mcap').exists()
    assert obj.jobs[identity]['artifacts']['hc_mcap'] == 'deleted'


def test_both_formats_have_distinct_ids_wait_for_export(tmp_path):
    obj = manager(tmp_path)
    identity = obj.begin(source(obj), 'both', {'profile_id':'x1'})
    obj.close(identity)
    obj.poll()
    assert (obj.state_dir/(identity+'-mcap.json')).exists()
    assert not (obj.state_dir/(identity+'-lerobot.json')).exists()
    destination = Path(obj.jobs[identity]['destination'])
    (destination/'meta').mkdir(parents=True)
    (destination/'meta/info.json').write_text(json.dumps(dict(total_frames=1,fps=15,total_episodes=1,features={})))
    # Completion marker, not merely meta/info.json, gates handoff.
    obj.poll()
    assert not (obj.state_dir/(identity+'-lerobot.json')).exists()
    ExportProgress(progress_path(destination)).update('done',100,state='completed',force=True)
    obj.poll()
    ready = json.loads((obj.state_dir/(identity+'-lerobot.json')).read_text())
    assert ready['recording_id'] == identity
    assert ready['source_artifact_id'] == identity+'-mcap'


def test_closed_source_recovered_after_restart(tmp_path):
    obj = manager(tmp_path)
    identity = obj.begin(source(obj), 'mcap', {'profile_id':'x1'})
    obj.close(identity)
    recovered = manager(tmp_path)
    recovered.poll()
    assert (obj.state_dir/(identity+'-mcap.json')).exists()


def test_real_recorder_publishes_closed_capture_and_deletion_lease(tmp_path):
    from middleware.core.topic_recorder import TopicRecorder
    recorder = TopicRecorder({'directory': str(tmp_path/'data'),
        'artifact_state_dir': str(tmp_path/'ready'),
        'upload_job_state_dir': str(tmp_path/'jobs')}, tmp_path)
    try:
        recorder.start('recorded.mcap', {'profile_id': 'x1'})
        # A zero-message, normally closed recording still has a valid MCAP footer.
        recorder.stop()
        recorder._artifacts.poll()
        records = recorder.list_recordings()
        item = next(row for row in records if row['filename'] == 'recorded.mcap')
        assert item['upload_artifact_state'] == 'ready'
        locks = ArtifactLocks(recorder.artifact_state_dir)
        with locks.read(item['artifact_id']):
            with pytest.raises(UploadError, match='ARTIFACT_BUSY'):
                recorder.delete_recording('recorded.mcap')
        recorder.delete_recording('recorded.mcap')
        assert not recorder.path.exists()
    finally:
        recorder.shutdown()


def test_missing_upload_install_does_not_block_local_capture(tmp_path, monkeypatch):
    import sys
    obj = manager(tmp_path)
    with monkeypatch.context() as patch:
        patch.setitem(sys.modules, 'humanoid_upload.artifacts', None)
        assert obj.begin(source(obj), 'mcap', {}) is None
        obj.close(None)
    assert obj.jobs == {}


def test_damaged_handoff_state_does_not_break_local_recording(tmp_path):
    obj = manager(tmp_path)
    obj.job_dir.mkdir(parents=True, exist_ok=True)
    (obj.job_dir/'damaged.json').write_text('{broken')
    recovered = manager(tmp_path)
    assert 'damaged.json' in recovered.recovery_errors
    identity = recovered.begin(source(recovered), 'mcap', {})
    recovered.mark_started(identity)
    recovered.close(identity)
    recovered.poll()
    assert (recovered.state_dir/(identity+'-mcap.json')).exists()
