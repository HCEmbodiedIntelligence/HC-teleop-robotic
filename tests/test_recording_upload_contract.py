"""Run with C's installed humanoid_upload package; no network/robot calls."""
import json
from pathlib import Path
import pytest

artifacts = pytest.importorskip('humanoid_upload.artifacts')
from humanoid_upload.common import UploadError
from middleware.core.artifact_state import publish_ready
from middleware.core.recording_service import RecordingService


def test_delete_holds_c_protocol_and_leaves_tombstone(tmp_path):
    cfg=dict(directory=str(tmp_path/'data'),job_state_dir=str(tmp_path/'jobs'),artifact_state_dir=str(tmp_path/'ready'))
    recorder=RecordingService(cfg,tmp_path)
    path=recorder.directory/'source.mcap'
    path.write_bytes(b'fixture for lease only')
    job=dict(recording_id='recording',profile_id='test',config_fingerprint='hash',path=str(path),capture_state='source_complete',artifact_state='ready',artifacts=['artifact'])
    recorder.jobs['recording']=job
    publish_ready(recorder.artifact_state_dir,job,recorder.directory,[path.name],'hc_mcap','artifact')
    locks=artifacts.ArtifactLocks(recorder.artifact_state_dir)
    try:
        with locks.read('artifact'):
            with pytest.raises(UploadError) as error:recorder.delete_recording(path.name)
            assert error.value.code=='ARTIFACT_BUSY'
            assert path.exists() and not locks.tombstone('artifact').exists()
        recorder.delete_recording(path.name)
        assert not path.exists()
        assert locks.tombstone('artifact').exists()
        with pytest.raises(UploadError) as error:
            with locks.read('artifact'):pass
        assert error.value.code=='ARTIFACT_DELETED'
    finally:recorder.shutdown()


def test_lerobot_identifier_cannot_bypass_upload_lease(tmp_path):
    cfg=dict(directory=str(tmp_path/'data'),job_state_dir=str(tmp_path/'jobs'),artifact_state_dir=str(tmp_path/'ready'))
    recorder=RecordingService(cfg,tmp_path)
    root=recorder.directory/'lerobot'/'episode';(root/'meta').mkdir(parents=True)
    (root/'meta/info.json').write_text('{}')
    job=dict(recording_id='recording',profile_id='test',config_fingerprint='hash')
    publish_ready(recorder.artifact_state_dir,job,root,['meta/info.json'],'hc_lerobot','artifact')
    locks=artifacts.ArtifactLocks(recorder.artifact_state_dir)
    try:
        with locks.read('artifact'):
            with pytest.raises(UploadError):recorder.delete_recording('lerobot:episode')
        assert root.exists()
        recorder.delete_recording('lerobot:episode')
        assert not root.exists() and locks.tombstone('artifact').exists()
    finally:recorder.shutdown()
