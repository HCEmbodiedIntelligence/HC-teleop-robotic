import copy
import json
from pathlib import Path
import threading
import time
from unittest.mock import patch

import pytest
from mcap.reader import make_reader
from middleware.core.recording_service import RecordingService
from middleware.core.artifact_state import publish_ready
from middleware.core.config import DEFAULT_CONFIG, validate_config, ConfigError


def config(root):
    return dict(directory=str(root/'data'), job_state_dir=str(root/'jobs'), artifact_state_dir=str(root/'ready'),
                min_free_bytes=1, format='mcap', max_pending_jobs=4)


def service(root):
    ros = {'enabled': False, 'subscriptions': [dict(topic='/observation/joint_states', type='sensor_msgs/msg/JointState', outputs=['record']),
                                               dict(topic='/action/joint_states', type='sensor_msgs/msg/JointState', outputs=['record'])]}
    return RecordingService(config(root), root, ros, lambda: {'/observation/joint_states': {'state':'ok'}})


def wait_for(predicate):
    deadline = time.monotonic()+15
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.02)


def test_continuous_recording_isolates_queues_and_persists_ready(tmp_path):
    recorder = service(tmp_path)
    gate = threading.Event()
    original = recorder.finish_hc
    def delayed(job):
        gate.wait(10)
        original(job)
    recorder.finish_hc = delayed
    try:
        first = recorder.start(metadata={'profile_id':'test'})
        assert recorder.start() == first
        recorder.record({'kind':'event', 'payload':{'segment':1}})
        recorder.mark_current_or_latest('test')
        recorder.stop()
        second = recorder.start(metadata={'profile_id':'other'})
        assert second != first
        recorder.record({'kind':'event', 'payload':{'segment':2}})
        recorder.stop()
        assert not list((tmp_path/'ready').glob('*.json'))
        gate.set()
        wait_for(lambda: all(j['artifact_state']=='ready' for j in recorder.jobs.values()))
        descriptions = [json.loads(p.read_text()) for p in (tmp_path/'ready').glob('*.json')]
        assert len(descriptions)==2
        assert len({d['recording_id'] for d in descriptions})==2
        assert {d['profile_id'] for d in descriptions}=={'test','other'}
        assert all(d['capture_started_at'] <= d['capture_ended_at'] <= d['completed_at'] for d in descriptions)
        for path, segment in [(first,1),(second,2)]:
            with open(path,'rb') as stream:
                messages=list(make_reader(stream).iter_messages())
            assert messages
            # JSON events are serialized as String CDR by the existing recorder.
            assert any(str(segment).encode() in m.data for _,_,m in messages)
    finally:
        gate.set()
        recorder.shutdown()
    restored=service(tmp_path)
    assert len(restored.jobs)==2
    assert all(j['artifact_state']=='ready' for j in restored.jobs.values())
    restored.shutdown()


def test_preflight_events_do_not_require_motion(tmp_path):
    recorder=service(tmp_path)
    try:
        assert recorder.precheck()['ready']
        recorder.health=lambda: {}
        assert not recorder.precheck()['ready']
        with pytest.raises(ValueError): recorder.start()
    finally: recorder.shutdown()


def test_disk_limit_prevents_new_recording(tmp_path):
    recorder=service(tmp_path)
    try:
        recorder.config['min_free_bytes']=10**30
        with pytest.raises(RuntimeError,match='磁盘'):recorder.start()
        assert not recorder.jobs
    finally:recorder.shutdown()


def test_ready_rejects_symlink_and_changed_identity(tmp_path):
    root=tmp_path/'data';root.mkdir()
    (root/'source').write_bytes(b'one')
    (root/'link').symlink_to(root/'source')
    job=dict(recording_id='recording',profile_id='test',config_fingerprint='hash')
    with pytest.raises(ValueError, match='symlink'):
        publish_ready(tmp_path/'ready',job,root,['link'],'hc_mcap','artifact')
    publish_ready(tmp_path/'ready',job,root,['source'],'hc_mcap','artifact')
    (root/'source').write_bytes(b'two')
    with pytest.raises(ValueError,match='identity'):
        publish_ready(tmp_path/'ready',job,root,['source'],'hc_mcap','artifact')


def test_raw_binding_is_frozen_and_async(tmp_path):
    recorder=service(tmp_path)
    job=dict(recording_id='r',workflow='platform_raw',capture_state='source_complete',artifact_state='awaiting_binding')
    recorder.jobs['r']=job
    binding=dict(binding_id='b',platform_base_url='https://example.invalid',robot_id='robot',collection_task_id='task')
    try:
        with patch.object(recorder.worker,'submit') as submit:
            result=recorder.request_raw_seal('r',binding)
            assert result['state']=='seal_queued'
            assert recorder.request_raw_seal('r',binding)['job_id']==result['job_id']
            assert submit.call_count==1
            with pytest.raises(ValueError,match='immutable'):
                recorder.request_raw_seal('r',{**binding,'collection_task_id':'changed'})
    finally:recorder.shutdown()


def test_workflow_and_motion_validation():
    cfg=copy.deepcopy(DEFAULT_CONFIG)
    cfg['ros']['command_mux']['source']='motion'
    value=validate_config(cfg)
    assert value['ros']['recording']['workflow']=='hc'
    cfg['ros']['recording'].update(workflow='platform_raw',capture_config_file='')
    with pytest.raises(ConfigError):validate_config(cfg)


def test_empty_source_never_publishes_ready(tmp_path):
    recorder=service(tmp_path)
    try:
        recorder.start()
        recorder.stop()
        wait_for(lambda: next(iter(recorder.jobs.values()))['artifact_state']=='failed')
        assert not list((tmp_path/'ready').glob('*.json'))
    finally:recorder.shutdown()


def test_crashed_capture_is_interrupted_on_restart(tmp_path):
    from middleware.core.artifact_state import atomic_json
    cfg=config(tmp_path)
    atomic_json(Path(cfg['job_state_dir'])/'r.json',dict(recording_id='r',capture_state='capturing',artifact_state='pending'))
    recorder=service(tmp_path)
    try:
        assert recorder.jobs['r']['capture_state']=='interrupted'
        assert not list((tmp_path/'ready').glob('*.json'))
    finally:recorder.shutdown()


def test_raw_auto_completion_uses_common_stop(tmp_path):
    from types import SimpleNamespace
    recorder = service(tmp_path)
    closed = threading.Event()
    closed.set()
    job = {'recording_id': 'raw'}
    recorder.active = job
    try:
        with patch.object(recorder, 'stop') as stop:
            recorder.watch_raw(job, SimpleNamespace(closed=closed))
            stop.assert_called_once_with()
        recorder.active = None
        with patch.object(recorder, 'stop') as stop:
            recorder.watch_raw(job, SimpleNamespace(closed=closed))
            stop.assert_not_called()
    finally:
        recorder.active = None
        recorder.shutdown()


def test_failed_publication_retries_same_identity(tmp_path):
    recorder = service(tmp_path)
    try:
        with patch('middleware.core.recording_service.publish_ready', side_effect=OSError('injected full disk')):
            recorder.start(metadata={'profile_id': 'test'})
            recorder.record({'kind': 'event', 'payload': {'synthetic': True}})
            recorder.stop()
            wait_for(lambda: next(iter(recorder.jobs.values()))['artifact_state'] == 'failed')
        job = next(iter(recorder.jobs.values()))
        identity = job['mcap_artifact_id']
        assert job['capture_state'] == 'source_complete'
        assert Path(job['path']).is_file()
        recorder.retry(job['recording_id'])
        wait_for(lambda: job['artifact_state'] == 'ready')
        assert job['artifacts'] == [identity]
        assert len(list(recorder.artifact_state_dir.glob('*.json'))) == 1
    finally:
        recorder.shutdown()


def test_backlog_blocks_new_segment_without_reusing_source(tmp_path):
    recorder = service(tmp_path)
    recorder.config['max_pending_jobs'] = 1
    gate = threading.Event()
    original = recorder.finish_hc
    def delayed(job):
        gate.wait(10)
        original(job)
    recorder.finish_hc = delayed
    try:
        source = recorder.start(metadata={'profile_id': 'test'})
        recorder.record({'kind': 'event', 'payload': {'synthetic': True}})
        recorder.stop()
        with pytest.raises(RuntimeError, match='积压'):
            recorder.start()
        assert len(recorder.jobs) == 1
        assert Path(source).exists()
    finally:
        gate.set()
        recorder.shutdown()
