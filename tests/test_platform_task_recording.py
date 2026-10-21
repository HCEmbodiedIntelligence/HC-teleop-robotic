"""Real MCAP files, durable queues and isolated platform HTTP simulations."""
import copy
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from middleware.core.platform_connection import PlatformConnection
from middleware.core.task_recording import TaskRecorder
from middleware.core.topic_recorder import TopicRecorder, read_recording_metadata
from humanoid_upload.common import UploadError, atomic_json, read_json
from capture_contract_support import task_document, snapshot_document

LOCAL = dict(local_robot_id='real-local-robot', profile_id='test-profile', config_fingerprint='a'*64,
             serial_no=None, model_version_id=None)


def setup(tmp_path, monkeypatch, *, legacy=None):
    cfg = {'ros': {'recording': {'format': 'mcap', 'directory': str(tmp_path/'data')}},
           'uploads': {'queue_directory': str(tmp_path/'queue'), **(legacy or {})}}
    service = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
    service.config['ros']['subscriptions'] = [{'topic': '/test', 'type': 'sensor_msgs/msg/JointState', 'outputs': ['record']}]
    service.capture_health = lambda: {'/test': {'has_data': True, 'hz': 15, 'state': 'ok'}}
    # Tests advance the worker explicitly, so no timing-dependent real platform access.
    monkeypatch.setattr(service.upload, 'start', lambda: None)
    def request(connection, method, endpoint, payload=None, **kw):
        if endpoint == 'connections:redeem':
            return dict(token='hcri_'+'a'*32+'.'+'b'*64, robot_id='robot', robot_name='Robot',
                organization_id='org', ingest_identity_id='identity', platform_name='Platform', local_robot=LOCAL)
        if endpoint == 'me':
            return dict(robot_id='robot', robot_name='Robot', organization_id='org', platform_name='Platform', local_robot=LOCAL)
        if endpoint.startswith('tasks/'):
            task = endpoint.split('/')[1].split(':')[0]
            document = task_document(task, payload['source_format'])
            return snapshot_document(document, payload) if endpoint.endswith(':prepare-recording') else document
        return {'items': [], 'total': 0, 'page': 1, 'page_size': 20, 'projects': []}
    monkeypatch.setattr(service, 'request', request)
    return cfg, service


def connected(tmp_path, monkeypatch):
    cfg, service = setup(tmp_path, monkeypatch)
    service.connect('http://platform.invalid/#robot-connect='+'c'*48)
    return cfg, service, TaskRecorder(TopicRecorder(cfg['ros']['recording'], tmp_path), service)


def choice(service, task, request_id=None):
    result = dict(mode='platform', connection_id=service.state['current'], task_id=task, expected_plan_revision=1)
    if request_id: result['request_id'] = request_id
    return result


def finish(recorder):
    deadline = time.monotonic()+10
    while recorder.pending:
        assert time.monotonic() < deadline, recorder.records()
        time.sleep(.02)


def capture(recorder, selection):
    path = recorder.start(metadata={'profile_id': LOCAL['profile_id']}, selection=selection)
    for i in range(3): recorder.record({'topic': '/test', 'payload': {'frame': i}, 'stamp_ns': time.time_ns()})
    recorder.stop()
    finish(recorder)
    return path


def test_single_link_saved_secret_and_retry_request_survive_restart(tmp_path, monkeypatch):
    cfg, service = setup(tmp_path, monkeypatch)
    requests = []
    original = service.request
    def request(*args, **kwargs):
        requests.append(copy.deepcopy(args[3]))
        return original(*args, **kwargs)
    monkeypatch.setattr(service, 'request', request)
    link = 'http://platform.invalid/#robot-connect='+'c'*48
    service.connect(link)
    first = service.state['current']
    assert service.status()['auto_upload'] is True
    secret_path = next((tmp_path/'queue/credentials').glob('*.token'))
    token = secret_path.read_text()
    assert secret_path.stat().st_mode & 0o777 == 0o600
    assert token not in json.dumps(service.status())
    assert token not in (tmp_path/'queue/connections.json').read_text()
    restored = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
    monkeypatch.setattr(restored.upload, 'start', lambda: None)
    monkeypatch.setattr(restored, 'request', request)
    restored.connect(link)
    assert restored.state['current'] == first
    assert requests[0]['request_id'] == requests[1]['request_id']
    assert len(list((tmp_path/'queue/credentials').glob('*'))) == 1
    def fail(*a, **kw): raise UploadError('NETWORK_UNCERTAIN', retry=True)
    monkeypatch.setattr(restored, 'request', fail)
    with pytest.raises(UploadError): restored.connect('http://other.invalid/#robot-connect='+'d'*48)
    assert restored.state['current'] == first


def test_two_windows_a_b_queued_independently_and_restart_preserves_ownership(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    try:
        a = choice(service, 'A', 'request-a')
        b = choice(service, 'B', 'request-b')
        first = capture(recorder, a)
        second = capture(recorder, b)
        jobs = service.upload.queue.jobs()
        assert [j['frozen']['task_id'] for j in jobs] == ['A', 'B']
        assert [j['manifest']['format_metadata']['ownership_snapshot']['dataset_id'] for j in jobs] == ['dataset-A', 'dataset-B']
        assert json.loads(read_recording_metadata(Path(first))['platform_task'])['collection_task_id'] == 'A'
        assert json.loads(read_recording_metadata(Path(second))['platform_task'])['collection_task_id'] == 'B'
        assert 'hcri_' not in Path(first).read_bytes().decode(errors='ignore')
        snapshot = copy.deepcopy(jobs)
        reloaded = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
        assert reloaded.upload.queue.jobs() == snapshot
        assert reloaded.status()['recent']['task_id'] == 'B'
        assert recorder.start(selection=a) == first  # Lost start response cannot record twice.
        with pytest.raises(UploadError, match='RECORDING_REQUEST_CONFLICT'):
            recorder.start(selection={**a, 'task_id': 'B'})
    finally: recorder.close()


def test_offline_active_capture_continues_new_platform_start_fails_and_local_can_bind_later(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    original = service.request
    try:
        recorder.start(selection=choice(service, 'A'))
        monkeypatch.setattr(service, 'request', lambda *a, **kw: (_ for _ in ()).throw(UploadError('NETWORK_UNCERTAIN', retry=True)))
        assert recorder.record({'topic': '/test', 'payload': {'offline': True}})
        recorder.stop(); finish(recorder)
        assert len(service.upload.queue.jobs()) == 1
        with pytest.raises(UploadError, match='NETWORK_UNCERTAIN'):
            recorder.start(selection=choice(service, 'B'))
        local = capture(recorder, {'mode': 'local'})
        assert len(service.upload.queue.jobs()) == 1
        record = next(j for j in recorder.records() if j['path'] == local)
        monkeypatch.setattr(service, 'request', original)
        recorder.upload_recording(record['recording_id'], choice(service, 'B')); finish(recorder)
        assert [j['frozen']['task_id'] for j in service.upload.queue.jobs()] == ['A', 'B']
        with pytest.raises(UploadError, match='TASK_BINDING_MISMATCH'):
            recorder.upload_recording(record['recording_id'], choice(service, 'A'))
    finally: recorder.close()


def test_concurrent_starts_are_serialized(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    try:
        def start(task):
            try: return recorder.start(selection=choice(service, task))
            except UploadError as error: return error.code
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(start, ('A','B')))
        assert results.count('RECORDING_ACTIVE') == 1
        assert len(recorder.records()) == 1
        recorder.stop(); finish(recorder)
    finally: recorder.close()


@pytest.mark.parametrize('with_platform', [True, False])
def test_controller_x_starts_y_stops_without_browser_setup(tmp_path, monkeypatch, with_platform):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from middleware.core.app import MiddlewareRuntime
    from middleware.core.config import validate_config

    cfg, service = setup(tmp_path, monkeypatch)
    raw = TopicRecorder(cfg['ros']['recording'], tmp_path)
    recorder = TaskRecorder(raw, service) if with_platform else raw
    runtime = MiddlewareRuntime(validate_config(cfg), tmp_path)
    runtime.recorder = recorder
    runtime.platform = service if with_platform else None
    runtime.upload_error = None if with_platform else 'optional uploader unavailable'
    runtime.recording_ros = SimpleNamespace(prepare_recording=Mock())
    runtime.emit = Mock()
    runtime.session_profile_metadata = {'profile_id': LOCAL['profile_id']}
    try:
        with ThreadPoolExecutor(max_workers=1) as commands:
            runtime._record_commands = commands
            def packet(held, pressed=0):
                runtime._on_pose(SimpleNamespace(left_input=SimpleNamespace(held_mask=held, pressed_mask=pressed)))
                commands.submit(lambda: None).result(timeout=5)
            packet(1, 1)
            assert recorder.is_recording()
            path = Path(recorder.status()['path'])
            packet(1)  # Holding X must not create another recording.
            assert recorder.status()['path'] == str(path)
            for i in range(3): recorder.record({'topic': '/test', 'payload': {'frame': i}})
            packet(0)
            packet(2, 2)
            assert not recorder.is_recording()
            assert path.is_file()
            assert not any(call.args[0]['kind'] == 'recording_error' for call in runtime.emit.call_args_list)
            if with_platform:
                finish(recorder)
                assert len(recorder.records()) == 1
                assert recorder.records()[0]['binding'] is None
                assert service.upload.queue.jobs() == []
            else:
                runtime.recording_ros.prepare_recording.assert_called_once()
    finally:
        recorder.close() if with_platform else recorder.shutdown()


def test_shared_selection_survives_restart_and_local_mode_does_not_reuse_recent_task(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    service.set_auto(False)
    try:
        recorder.set_selection(choice(service, 'A', 'must-not-be-reused'))
        recorder.start()
        assert recorder.status()['task_recording']['binding']['collection_task_id'] == 'A'
        with pytest.raises(UploadError, match='RECORDING_ACTIVE'):
            recorder.set_selection(choice(service, 'B'))
        for i in range(3): recorder.record({'topic': '/test', 'payload': {'frame': i}})
        recorder.stop(); finish(recorder)
    finally: recorder.close()
    restored = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
    monkeypatch.setattr(restored, 'request', service.request)
    restored.config = copy.deepcopy(service.config)
    restored.capture_health = service.capture_health
    assert restored.recording_selection() == choice(service, 'A')
    recorder = TaskRecorder(TopicRecorder(cfg['ros']['recording'], tmp_path), restored)
    try:
        capture(recorder, None)
        assert len(recorder.records()) == 2
        assert len({job['request_id'] for job in recorder.records()}) == 2
        assert all(job['binding']['collection_task_id'] == 'A' for job in recorder.records())
        recorder.set_selection({'mode': 'local'})
        capture(recorder, None)
        assert recorder.records()[-1]['binding'] is None
        assert restored.status()['recent']['task_id'] == 'A'
        reloaded = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
        assert reloaded.recording_selection() == {'mode': 'local'}
        assert reloaded.upload.queue.jobs() == []
    finally: recorder.close()


def test_failed_selection_keeps_saved_task_and_new_connection_clears_it(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    try:
        recorder.set_selection(choice(service, 'A'))
        request = service.request
        monkeypatch.setattr(service, 'request', lambda *a, **kw: (_ for _ in ()).throw(UploadError('NETWORK_UNCERTAIN')))
        with pytest.raises(UploadError, match='NETWORK_UNCERTAIN'):
            recorder.set_selection(choice(service, 'B'))
        assert service.recording_selection() == choice(service, 'A')
        with pytest.raises(UploadError, match='NETWORK_UNCERTAIN'):
            recorder.start()  # Invalid bound task must not silently become a local recording.
        assert not recorder.is_recording()
        monkeypatch.setattr(service, 'request', request)
        service.connect('http://other-platform.invalid/#robot-connect='+'z'*48)
        assert service.recording_selection() == {'mode': 'local'}
    finally: recorder.close()


def test_legacy_configuration_and_queue_unchanged_when_automatic_upload_was_off(tmp_path, monkeypatch):
    old = dict(enabled=True, auto_enqueue=False, platform_base_url='http://platform.invalid',
               credential_ref='env:LEGACY_ROBOT_CREDENTIAL', robot_id='robot',
               local_robot_id=LOCAL['local_robot_id'], task_id='A')
    cfg, service = setup(tmp_path, monkeypatch, legacy=old)
    assert service.status()['auto_upload'] is False
    assert service.status()['recent']['task_id'] == 'A'
    recorder = TaskRecorder(TopicRecorder(cfg['ros']['recording'], tmp_path), service)
    try:
        capture(recorder, choice(service, 'A'))
        assert service.upload.queue.jobs() == []
        service.connect('http://platform.invalid/#robot-connect='+'e'*48)
        assert service.status()['auto_upload'] is False
        assert old['credential_ref'] in (tmp_path/'queue/bindings.json').read_text()
    finally: recorder.close()


def test_interrupted_recording_is_preserved_without_auto_upload(tmp_path, monkeypatch):
    cfg, service = setup(tmp_path, monkeypatch)
    state = service.directory/'task-recordings'
    atomic_json(state/'old.json', {'recording_id': 'old', 'state': 'recording', 'binding': None, 'path': '/preserved.mcap'})
    recorder = TaskRecorder(TopicRecorder(cfg['ros']['recording'], tmp_path), service)
    assert recorder.records()[0]['state'] == 'interrupted'
    assert service.upload.queue.jobs() == []
    recorder.close()


def test_reconnect_repairs_managed_credential_without_retargeting_existing_queue(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    try:
        capture(recorder, choice(service, 'A'))
        original = copy.deepcopy(service.upload.queue.jobs())
        secret = Path(original[0]['frozen']['credential_ref'].removeprefix('file:'))
        request = service.request
        def renew(*args, **kwargs):
            value = request(*args, **kwargs)
            if args[2] == 'connections:redeem': value['token'] = 'hcri_'+'d'*32+'.'+'e'*64
            return value
        monkeypatch.setattr(service, 'request', renew)
        service.connect('http://platform.invalid/#robot-connect='+'f'*48)
        assert secret.read_text() == 'hcri_'+'d'*32+'.'+'e'*64
        assert service.upload.queue.jobs() == original
        assert len(list((service.directory/'credentials').glob('*.token'))) == 1
    finally: recorder.close()


def test_restart_completes_history_binding_after_durable_intent_before_descriptor(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    capture(recorder, {'mode': 'local'})
    job = recorder.records()[0]
    descriptor = service.upload.state_dir/(job['recording_id']+'.json')
    assert read_json(descriptor)['binding_id'] is None
    job.update(binding=service.freeze(choice(service,'B'),remember=False),
               upload_requested=True,state='upload_pending')
    recorder.save(job)
    recorder.close()
    restored = TaskRecorder(TopicRecorder(cfg['ros']['recording'],tmp_path),service)
    try:
        finish(restored)
        queued = service.upload.queue.jobs()
        assert len(queued) == 1 and queued[0]['manifest']['collection_task_id'] == 'B'
        assert read_json(descriptor)['binding_id'] == job['binding']['binding_id']
        assert queued[0]['manifest']['format_metadata']['ownership_snapshot'] == job['binding']['task']['ownership']
    finally: restored.close()


def test_existing_queue_without_connection_registry_migrates_without_rewriting_jobs(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    capture(recorder, choice(service, 'A'))
    recorder.close()
    before = service.upload.queue.jobs()
    bindings = (service.directory/'bindings.json').read_bytes()
    service.path.unlink()  # Simulate the pre-connection-registry queue format.
    cfg['uploads'].update(enabled=True, auto_enqueue=False)
    migrated = PlatformConnection(cfg,tmp_path,tmp_path/'data',identity_provider=lambda:dict(LOCAL))
    assert migrated.upload.queue.jobs() == before
    assert (service.directory/'bindings.json').read_bytes() == bindings
    assert migrated.status()['auto_upload'] is False
    assert migrated.status()['recent']['task_id'] == 'A'


def test_historical_lerobot_uses_source_capture_and_its_own_format(tmp_path, monkeypatch):
    cfg, service = setup(tmp_path, monkeypatch)
    service.connect('http://platform.invalid/#robot-connect='+'c'*48)
    raw = TopicRecorder(cfg['ros']['recording'],tmp_path)
    raw.start('old.mcap', {'profile_id': LOCAL['profile_id']})
    raw.record({'topic':'/test','payload':{'old':True},'stamp_ns':time.time_ns()})
    time.sleep(.002)
    raw.record({'topic':'/test','payload':{'old':True},'stamp_ns':time.time_ns()})
    raw.stop()
    dataset = tmp_path/'data/lerobot/old'
    (dataset/'meta').mkdir(parents=True)
    (dataset/'data').mkdir()
    atomic_json(dataset/'meta/info.json', {'codebase_version':'v3.0','total_frames':1,
        'total_episodes':1,'fps':10,'robot_type':LOCAL['profile_id'],
        'features':{'observation.state':{'dtype':'float32','shape':[1]}}})
    (dataset/'data/part.parquet').write_bytes(b'PAR1synthetic-closed-filePAR1')
    calls=[]; original=service.request
    def request(*args, **kwargs):
        if args[2].startswith('tasks/'): calls.append(args[3]['source_format'])
        return original(*args, **kwargs)
    monkeypatch.setattr(service,'request',request)
    recorder=TaskRecorder(raw,service)
    try:
        recorder.upload_recording('lerobot:old',choice(service,'B')); finish(recorder)
        assert calls == ['LEROBOT_V3']
        queued=service.upload.queue.jobs()
        assert len(queued)==1 and queued[0]['manifest']['source_format']=='LEROBOT_V3'
        assert queued[0]['manifest']['collection_task_id']=='B'
        with pytest.raises(ValueError,match='上传队列'): recorder.delete_recording('lerobot:old')
    finally: recorder.close()


def test_upload_format_persists_and_is_frozen_per_recording(tmp_path, monkeypatch):
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    try:
        recorder.set_upload_format('mcap')
        recorder.start(selection=choice(service, 'A'))
        with pytest.raises(UploadError, match='RECORDING_ACTIVE'):
            recorder.set_upload_format('lerobot')
        recorder.record({'topic': '/test', 'payload': {'frame': 1}, 'stamp_ns': time.time_ns()})
        recorder.stop(); finish(recorder)
        recorder.set_upload_format('lerobot')
        assert recorder.records()[0]['upload_format'] == 'mcap'
        assert service.upload.queue.jobs()[0]['manifest']['source_format'] == 'MCAP'
        restored = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
        assert restored.status()['upload_format'] == 'lerobot'
        with pytest.raises(UploadError, match='UPLOAD_FORMAT_UNAVAILABLE'):
            recorder.start(selection=choice(service, 'A'))
        assert not recorder.is_recording()
    finally:
        recorder.close()


def configure_lerobot_upload(recorder, service, monkeypatch):
    recorder.recorder.save_format = 'both'
    recorder.set_upload_format('lerobot')
    topics = ['/test', '/observation/joint_states', '/action/joint_states']
    service.config['ros']['subscriptions'] = [
        {'topic': topic, 'type': 'sensor_msgs/msg/JointState', 'outputs': ['record']}
        for topic in topics
    ]
    recorder.capture_health = lambda: {topic: {'has_data': True, 'hz': 15} for topic in topics}
    monkeypatch.setattr('middleware.core.lerobot_export.ensure_export_available', lambda: 'synthetic-converter')


def test_both_local_uploads_only_selected_lerobot(tmp_path, monkeypatch):
    from middleware.core.export_progress import progress_path
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    configure_lerobot_upload(recorder, service, monkeypatch)
    def exported():
        import pyarrow as pa
        import pyarrow.parquet as pq
        dataset = recorder.directory/'lerobot'/recorder.recorder.path.stem
        snapshot = json.loads(read_recording_metadata(recorder.recorder.path)['capture_snapshot'])
        features = {key: {field: value for field, value in feature.items()
                         if field in ('dtype', 'shape', 'names', 'units')}
                    for key, feature in snapshot['output']['features'].items() if feature['required']}
        (dataset/'meta').mkdir(parents=True)
        (dataset/'data').mkdir()
        atomic_json(dataset/'meta/info.json', {'codebase_version':'v3.0','total_frames':1,
            'total_episodes':1,'fps':15,'robot_type':LOCAL['profile_id'], 'features':features})
        pq.write_table(pa.table({'observation.state': [[1.0]], 'action': [[2.0]], 'timestamp': [0.0]}),
                       dataset/'data/part.parquet')
        atomic_json(progress_path(dataset), {'state':'completed'})
    monkeypatch.setattr(recorder.recorder, '_start_lerobot_export', exported)
    try:
        path = capture(recorder, choice(service, 'A'))
        assert Path(path).is_file()
        jobs = service.upload.queue.jobs()
        assert len(jobs) == 1
        assert jobs[0]['manifest']['source_format'] == 'LEROBOT_V3'
        assert jobs[0]['manifest']['declared_episode_count'] == 1
        assert not any(a['path'].endswith('.mcap') for a in jobs[0]['manifest']['assets'])
        assert recorder.records()[0]['format'] == 'both'
        assert recorder.records()[0]['upload_format'] == 'lerobot'
    finally:
        recorder.close()


def test_lerobot_export_failure_never_uploads_mcap_fallback(tmp_path, monkeypatch):
    from middleware.core.export_progress import progress_path
    cfg, service, recorder = connected(tmp_path, monkeypatch)
    configure_lerobot_upload(recorder, service, monkeypatch)
    def failed_export():
        dataset = recorder.directory/'lerobot'/recorder.recorder.path.stem
        atomic_json(progress_path(dataset), {'state':'error'})
    monkeypatch.setattr(recorder.recorder, '_start_lerobot_export', failed_export)
    try:
        path = capture(recorder, choice(service, 'A'))
        assert Path(path).is_file()
        assert service.upload.queue.jobs() == []
        assert recorder.records()[0]['code'] == 'EXPORT_FAILED'
    finally:
        recorder.close()
