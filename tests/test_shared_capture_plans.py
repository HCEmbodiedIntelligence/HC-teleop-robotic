"""Executable recording -> sealing -> real HTTP upload contract, using synthetic data."""
import copy
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from capture_contract_support import snapshot_document, task_document
from shared_capture_http import CaptureServer
from test_platform_task_recording import LOCAL, choice, finish
from humanoid_upload.artifacts import inspect_ready
from humanoid_upload.capture_plan import digest, reference, validate_lerobot_info, validate_snapshot
from humanoid_upload.common import UploadError, atomic_json, read_json
from middleware.core.capture_requirements import check_device
from middleware.core.export_progress import ExportProgress, progress_path
from middleware.core.lerobot_export import preserved_episode
from middleware.core.platform_connection import PlatformConnection
from middleware.core.task_recording import TaskRecorder
from middleware.core.topic_recorder import TopicRecorder, read_recording_metadata


def test_joint_only_mcap_can_be_packaged_without_dummy_video(tmp_path):
    from mcap.reader import make_reader
    from mcap.writer import Writer
    from middleware.core.video_bundle import prepare

    source = tmp_path / 'joints.mcap'
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'jsonschema', b'{}')
        channel = writer.register_channel('/joints', 'json', schema)
        for index in range(3):
            stamp = 1_000_000_000 + index * 100_000_000
            writer.add_message(channel, stamp, json.dumps({'name': ['axis'], 'position': [index]}).encode(), stamp)
        writer.finish()
    original = source.read_bytes()
    result = prepare(source, tmp_path / 'bundle', {
        'recording_id': 'joint-only', 'started_at': '2026-10-21T01:00:00+00:00',
        'ended_at': '2026-10-21T01:00:01+00:00',
    })
    assert json.loads((result / 'session.json').read_text())['videos'] == []
    with (result / 'raw.mcap').open('rb') as stream:
        rows = list(make_reader(stream, validate_crcs=True).iter_messages())
    assert len(rows) == 3 and json.loads(rows[-1][2].data)['position'] == [2]
    assert source.read_bytes() == original


@pytest.fixture
def server():
    server = CaptureServer(LOCAL)
    yield server
    server.close()


def environment(tmp_path, monkeypatch, server, fmt='mcap'):
    topics = ['/test', '/observation/joint_states', '/action/joint_states']
    cfg = {'ros': {'recording': {'format': fmt, 'directory': str(tmp_path/'data'), 'lerobot_fps': 15},
                   'subscriptions': [{'topic': t, 'type': 'sensor_msgs/msg/JointState', 'outputs': ['record']} for t in topics]},
           'uploads': {'queue_directory': str(tmp_path/'queue'), 'processing_endpoint': True, 'timeout_seconds': 2}}
    platform = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
    monkeypatch.setattr(platform.upload, 'start', lambda: None)
    platform.capture_health = lambda: {t: {'has_data': True, 'hz': 15, 'state': 'ok'} for t in topics}
    platform.connect(server.base+'/#robot-connect='+'x'*48)
    return cfg, platform


def recorder_for(cfg, platform, tmp_path, monkeypatch):
    raw = TopicRecorder(cfg['ros']['recording'], tmp_path)
    if raw.save_format in ('lerobot', 'both'):
        monkeypatch.setattr('middleware.core.lerobot_export.ensure_export_available', lambda: 'synthetic-converter')
        def convert():
            # A format-valid synthetic Parquet dataset replaces only the external
            # ROS/official writer process; recorder, seal, queue, HTTP are real.
            import pyarrow as pa
            import pyarrow.parquet as pq
            target = raw.directory/'lerobot'/raw.path.stem
            from middleware.core.capture_formats import lerobot_snapshot
            snapshot = lerobot_snapshot(read_recording_metadata(raw.path)) or json.loads(read_recording_metadata(raw.path)['capture_snapshot'])
            features = {k: {f: v for f, v in feature.items() if f in ('dtype', 'shape', 'names', 'units')}
                        for k, feature in snapshot['output']['features'].items() if feature['required']}
            atomic_json(target/'meta/info.json', {'codebase_version': 'v3.0', 'fps': 15, 'robot_type': LOCAL['profile_id'],
                'total_frames': 167, 'total_episodes': 1, 'features': features})
            (target/'data/chunk-000').mkdir(parents=True)
            pq.write_table(pa.table({'observation.state': [[float(i)] for i in range(167)],
                                    'action': [[float(i+1)] for i in range(167)],
                                    'timestamp': [i/15 for i in range(167)]}), target/'data/chunk-000/file-000.parquet')
            ExportProgress(progress_path(target)).update('complete', 100, state='completed', force=True)
        monkeypatch.setattr(raw, '_start_lerobot_export', convert)
    return TaskRecorder(raw, platform)


def record(recorder, selection, name=''):
    path = recorder.start(name, {'profile_id': LOCAL['profile_id']}, selection)
    for index in range(3): recorder.record({'topic': '/test', 'payload': {'synthetic': index}, 'stamp_ns': time.time_ns()})
    recorder.stop()
    finish(recorder)
    assert recorder.records()[-1]['state'] == 'queued', recorder.records()
    return path


@pytest.mark.parametrize(('fmt', 'upload_format'), [
    ('mcap', 'mcap'), ('lerobot', 'lerobot'), ('both', 'mcap'), ('both', 'lerobot')])
def test_two_task_versions_seal_queue_http_retry_restart(fmt, upload_format, tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server, fmt)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    try:
        recorder.set_upload_format(upload_format)
        platform.tasks()
        recorder.set_selection(choice(platform, 'A'))
        platform.preflight(choice(platform, 'B'))
        assert server.calls['prepare-recording'] == 0
        assert not platform.upload.bindings  # Form selection has no durable upload binding side effect.
        first = record(recorder, choice(platform, 'A', 'start-A'))
        before = copy.deepcopy(platform.upload.queue.jobs()[0])
        second = record(recorder, choice(platform, 'B', 'start-B'))
        assert first != second
        assert len(server.snapshots) == 2
        jobs = recorder.records()
        assert [j['capture_snapshot']['scheme']['version'] for j in jobs] == [1, 2]
        for job, queued in zip(jobs, platform.upload.queue.jobs()):
            snapshot = job['capture_snapshot']
            assert json.loads(read_recording_metadata(Path(job['path']))['capture_snapshot']) == snapshot
            assert queued['capture_snapshot'] == queued['descriptor']['capture_snapshot'] == snapshot
            assert queued['manifest']['capture_plan'] == reference(snapshot)
            assert 'effective_adapter' not in queued['manifest']
            assert job['upload_format'] == upload_format
            assert snapshot['source_format'] == ('LEROBOT_V3' if upload_format == 'lerobot' else 'MCAP')
            if upload_format == 'lerobot':
                root = Path(queued['root'])
                context = read_json(root/'capture-context.json')
                assert context['capture_snapshot'] == snapshot
                assert context['feature_presence']['action.base.velocity'] is False
                assert not list(root.rglob('*.mp4'))
                import pyarrow.parquet as pq
                table = pq.read_table(root/'data/chunk-000/file-000.parquet')
                assert len(table) == 167 and 'action.base.velocity' not in table.column_names
                assert table['timestamp'][-1].as_py() == 166/15
        server.default_scheme = 'new-default'
        server.latest_version = 100
        platform.tasks()
        assert platform.upload.queue.jobs()[0] == before
        platform.upload.step(scan=False)
        platform.upload.step(scan=False)
        uploaded = platform.upload.queue.jobs()
        assert all(job['server_state'] == 'COMMITTED' for job in uploaded)
        if upload_format == 'lerobot':
            assert all(job['state'] == 'ready' and job['data_status'] == 'READABLE' for job in uploaded)
            assert all(job['annotation_status'] == 'NOT_CONFIGURED' and job['training_status'] == 'NOT_READY' for job in uploaded)
            assert all('annotation_task_id' not in ep for job in uploaded for ep in job['episodes'])
            server.receipt_annotation = 'FAILED'
        puts = sum(api.puts for api in server.uploads.values())
        restored = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
        assert restored.upload.queue.jobs() == uploaded
        for job in uploaded: restored.upload.queue.retry(job['id'])
        restored.upload.step(scan=False)
        restored.upload.step(scan=False)
        assert len(server.uploads) == 2 and sum(api.puts for api in server.uploads.values()) == puts
        for old, new in zip(uploaded, restored.upload.queue.jobs()):
            assert new['manifest'] == old['manifest'] and new['capture_snapshot'] == old['capture_snapshot']
            if upload_format == 'lerobot':
                assert new['state'] == 'ready' and new['data_status'] == 'READABLE'
                assert new['annotation_status'] == 'FAILED'
        assert recorder.start(selection=choice(platform, 'A', 'start-A')) == first
        assert server.calls['prepare-recording'] == 2
    finally:
        recorder.close()


@pytest.mark.parametrize('restart', [False, True])
def test_prepare_response_loss_reuses_durable_identity(restart, tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    selection = choice(platform, 'A', 'lost-start')
    def durable(command):
        saved = read_json(platform.directory/'task-recordings'/(command['recording_id']+'.json'))
        assert saved['preparation']['command'] == command
    server.before_prepare = durable
    server.drop_prepare = True
    try:
        with pytest.raises(UploadError, match='NETWORK_UNCERTAIN'): recorder.start(selection=selection)
        pending = recorder.records()[0]
        assert pending['state'] == 'prepare_failed' and not recorder.is_recording()
        assert not Path(pending['path']).exists()
        assert len(server.snapshots) == 1
        recorder.set_upload_format('lerobot')
        if restart:
            recorder.close()
            restored = PlatformConnection(cfg, tmp_path, tmp_path/'data', identity_provider=lambda: dict(LOCAL))
            restored.capture_health = platform.capture_health
            monkeypatch.setattr(restored.upload, 'start', lambda: None)
            platform = restored
            recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
            selection = {**selection, 'request_id': 'browser-refreshed'}
        path = record(recorder, selection)
        assert path == pending['path']
        assert len(server.snapshots) == len(recorder.records()) == 1
        assert server.commands[0] == server.commands[1]
        assert server.commands[1]['source_format'] == 'MCAP'
        assert server.calls['preflight'] == 1
    finally: recorder.close()


def test_revision_race_never_falls_back_and_explicit_reselection_recovers(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    try:
        def change_revision(command): server.revisions['A'] = 2
        server.before_prepare = change_revision
        with pytest.raises(UploadError, match='CAPTURE_REVISION_CONFLICT') as failure:
            recorder.start(selection=choice(platform, 'A', 'stale'))
        assert not failure.value.retry
        assert '重新选择' in failure.value.action
        assert not recorder.is_recording() and not server.snapshots
        original_id = recorder.records()[0]['recording_id']
        with pytest.raises(UploadError, match='CAPTURE_REVISION_CONFLICT'):
            recorder.start(selection=choice(platform, 'A', 'stale'))
        assert recorder.records()[0]['recording_id'] == original_id
        updated = {**choice(platform, 'A', 'fresh'), 'expected_plan_revision': 2}
        recorder.set_selection(updated)
        record(recorder, updated)
        assert recorder.records()[-1]['capture_snapshot']['plan_revision'] == 2
    finally: recorder.close()


@pytest.mark.parametrize('field,value', [('recording_id','other'),('robot_id','other'),('dataset_id','other'),
                                         ('scheme',{'scheme_id':'other','version':2,'content_sha256':'f'*64})])
def test_snapshot_scope_and_digest_tampering_rejected(field, value):
    task = task_document()
    command = dict(recording_id='r', expected_plan_revision=1, source_format='MCAP')
    snapshot = snapshot_document(task, command)
    changed = copy.deepcopy(snapshot); changed[field] = value
    with pytest.raises(UploadError, match='CAPTURE_SNAPSHOT_INVALID'): validate_snapshot(changed)
    changed['snapshot_sha256'] = digest({k:v for k,v in changed.items() if k != 'snapshot_sha256'})
    with pytest.raises(UploadError, match='CAPTURE_SNAPSHOT_MISMATCH'): validate_snapshot(changed, **{field: snapshot[field]})


def test_device_data_not_implied_by_configuration(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    try:
        recorder.capture_health = lambda: {}
        with pytest.raises(UploadError, match='CAPTURE_REQUIRED_DATA_MISSING'): recorder.start(selection=choice(platform, 'A'))
        assert not recorder.is_recording()
        identity = recorder.records()[0]['recording_id']
        recorder.capture_health = platform.capture_health
        record(recorder, choice(platform, 'A'))
        assert server.calls['prepare-recording'] == 1 and recorder.records()[0]['recording_id'] == identity
    finally: recorder.close()


def test_fps_and_required_camera_gates_preserve_device_configuration():
    task = task_document(); snapshot = snapshot_document(task, dict(recording_id='r', expected_plan_revision=1, source_format='MCAP'))
    snapshot['effective_adapter']['configuration']['cameras'] = [dict(camera_id='head', topic='/head', message_type='sensor_msgs/msg/Image', required=True)]
    snapshot['snapshot_sha256'] = digest({k:v for k,v in snapshot.items() if k != 'snapshot_sha256'})
    cfg = {'ros': {'subscriptions': [{'topic':'/test', 'type':'sensor_msgs/msg/JointState', 'outputs':['record']}]}}
    before = copy.deepcopy(cfg)
    with pytest.raises(UploadError, match='CAPTURE_SOURCE_NOT_RECORDING'): check_device(snapshot, cfg, SimpleNamespace(), {'/test':{'has_data':True,'hz':15}})
    assert cfg == before
    snapshot['effective_adapter']['configuration']['cameras'][0]['required'] = False
    snapshot['snapshot_sha256'] = digest({k:v for k,v in snapshot.items() if k != 'snapshot_sha256'})
    check_device(snapshot, cfg, SimpleNamespace(), {'/test':{'has_data':True,'hz':15}})
    with pytest.raises(UploadError, match='CAPTURE_FREQUENCY_MISMATCH'): check_device(snapshot, cfg, SimpleNamespace(), {'/test':{'has_data':True,'hz':30}})


def test_original_167_frames_at_15hz_and_optional_absence():
    stamps = [10**18 + round(i*1e9/15) for i in range(167)]
    streams = {key: [(stamp, [float(i)]) for i, stamp in enumerate(stamps)] for key in ('observation.state','action')}
    audit = {}
    episodes = preserved_episode(streams, 15, audit)
    assert len(episodes) == 1 and len(episodes[0]) == 167
    assert [row[0] for row in episodes[0]] == stamps
    assert audit['candidate_frames'] == audit['retained_frames'] == 167
    assert 'action.base.velocity' not in episodes[0][0][1]
    streams['observation.images.head'] = [(stamps[1], 'real-image.png')]
    with pytest.raises(UploadError, match='CAPTURE_SOURCE_GAPS_UNSUPPORTED'): preserved_episode(streams, 15, {})


def test_writer_owned_columns_need_no_device_topic_but_must_exist_in_output():
    from humanoid_upload.capture_plan import LEROBOT_GENERATED_FEATURES

    task = task_document(source_format='LEROBOT_V3')
    features = task['capture_plan']['output']['features']
    for key, definition in LEROBOT_GENERATED_FEATURES.items():
        features[key] = {**definition, 'required': True, 'role': 'numeric', 'units': []}
    snapshot = snapshot_document(task, dict(recording_id='r', expected_plan_revision=1,
                                           source_format='LEROBOT_V3'))
    topics = ('/observation/joint_states', '/action/joint_states')
    cfg = {'ros': {'subscriptions': [dict(topic=t, type='sensor_msgs/msg/JointState',
                                          outputs=['record']) for t in topics]}}
    health = {t: {'has_data': True, 'hz': 15, 'state': 'ok'} for t in topics}
    check_device(snapshot, cfg, SimpleNamespace(lerobot_fps=15), health)
    info = dict(fps=15, codebase_version='v3.0', features={key: copy.deepcopy(value)
                for key, value in features.items() if value['required']})
    assert validate_lerobot_info(snapshot, info)['frame_index'] is True
    del info['features']['frame_index']
    with pytest.raises(UploadError, match='CAPTURE_REQUIRED_DATA_MISSING'):
        validate_lerobot_info(snapshot, info)


@pytest.mark.parametrize('changed', ['names', 'units', 'shape', 'dtype', 'fps'])
def test_lerobot_actual_structure_not_just_dimension(changed):
    task = task_document(source_format='LEROBOT_V3')
    snapshot = snapshot_document(task, dict(recording_id='r', expected_plan_revision=1, source_format='LEROBOT_V3'))
    info = dict(fps=15, codebase_version='v3.0', features={key: copy.deepcopy(value) for key,value in task['capture_plan']['output']['features'].items() if value['required']})
    assert validate_lerobot_info(snapshot, info)['action.base.velocity'] is False
    if changed == 'fps': info['fps'] = 30
    else: info['features']['action'][changed] = {'names':['other'], 'units':['deg'], 'shape':[2], 'dtype':'float64'}[changed]
    with pytest.raises(UploadError): validate_lerobot_info(snapshot, info)


def test_prepared_snapshot_survives_restart_without_second_prepare(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    recorder.capture_health = lambda: {}
    with pytest.raises(UploadError): recorder.start(selection=choice(platform, 'A'))
    saved = recorder.records()[0]
    recorder.close()
    restored = recorder_for(cfg, platform, tmp_path, monkeypatch)
    try:
        record(restored, choice(platform, 'A'))
        assert restored.records()[0]['capture_snapshot'] == saved['capture_snapshot']
        assert server.calls['prepare-recording'] == 1
    finally: restored.close()


def test_seal_validator_rejects_snapshot_replacement_and_unknown_fields(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    try:
        record(recorder, choice(platform, 'A'))
        job = platform.upload.queue.jobs()[0]
        descriptor = copy.deepcopy(job['descriptor'])
        descriptor['unexpected'] = True
        with pytest.raises(UploadError, match='ARTIFACT_SCHEMA_INVALID'): inspect_ready(descriptor, platform.upload.cfg)
        descriptor.pop('unexpected')
        descriptor['capture_snapshot']['scheme']['version'] = 42
        with pytest.raises(UploadError, match='CAPTURE_SNAPSHOT_INVALID'): inspect_ready(descriptor, platform.upload.cfg)
        descriptor['capture_snapshot']['snapshot_sha256'] = digest({k:v for k,v in descriptor['capture_snapshot'].items() if k != 'snapshot_sha256'})
        with pytest.raises(UploadError, match='CAPTURE_SNAPSHOT_MISMATCH'): inspect_ready(descriptor, platform.upload.cfg)
        before = copy.deepcopy(platform.upload.queue.jobs())
        platform.upload.queue.retry(job['id'])
        assert platform.upload.queue.jobs()[0]['manifest'] == before[0]['manifest']
    finally: recorder.close()


def test_plan_missing_blocks_new_capture_but_explicit_local_and_old_seal_remain(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    original = platform.preflight
    def legacy(*args, **kwargs):
        checked = original(*args, **kwargs)
        checked['task'].pop('capture_plan')
        return checked
    monkeypatch.setattr(platform, 'preflight', legacy)
    try:
        with pytest.raises(UploadError, match='CAPTURE_PLAN_REQUIRED'): recorder.start(selection=choice(platform, 'A'))
        assert not recorder.is_recording() and not server.snapshots
        path = recorder.start(selection={'mode':'local'})
        recorder.record({'topic':'/test', 'payload': {'synthetic': True}})
        recorder.stop(); finish(recorder)
        assert recorder.records()[-1]['binding'] is None
        assert 'capture_snapshot' not in read_recording_metadata(Path(path))
        job = recorder.records()[-1]
        descriptor = read_json(platform.upload.state_dir/(job['recording_id']+'.json'))
        inspect_ready(descriptor, platform.upload.cfg)
        assert 'capture_snapshot' not in descriptor
        recorder.upload_recording(job['recording_id'], choice(platform, 'A')); finish(recorder)
        queued = platform.upload.queue.jobs()[0]
        assert 'capture_plan' not in queued['manifest'] and 'capture_snapshot' not in queued['descriptor']
    finally: recorder.close()


@pytest.mark.parametrize('code,expected', [('CAPTURE_PLAN_REQUIRED','awaiting_configuration'), ('CAPTURE_STRUCTURE_MISMATCH','structure_mismatch')])
def test_processing_problem_is_configuration_or_structure_not_file_damage(code, expected):
    from humanoid_upload.worker import Worker
    result = dict(schema_version='openarm-processing-result/v1', upload_id='u', raw_source_id='raw',
        processing_status='FAILED', quality_status='PENDING', terminal=True, poll_after_seconds=0,
        episodes=[dict(source_episode_id='s', source_episode_index=0, episode_id=None, dataset_id='d',
                       status='FAILED', quality_status='PENDING', error_code=code, retryable=False, next_action='repair_export')])
    job = dict(manifest={'source_format':'LEROBOT_V3'}, upload_id='u', raw_source_id='raw')
    queue = SimpleNamespace(save=lambda value: None)
    Worker(queue, {'processing_endpoint':True}).processing(job, SimpleNamespace(api=lambda *args: result), '/u')
    assert job['state'] == expected and '原始' in job['next_action']


@pytest.mark.parametrize('status,label', [('PENDING_CONFIGURATION', 'NOT_CONFIGURED'),
                                         ('CREATED', 'READY'), ('APPROVED', 'COMPLETED'), ('FAILED', 'FAILED')])
def test_actual_platform_episode_receipts_keep_data_and_annotation_independent(status, label):
    from humanoid_upload.worker import Worker
    result = dict(schema_version='openarm-processing-result/v1', upload_id='u', raw_source_id='raw',
        processing_status='READY', quality_status='PASS', terminal=True, poll_after_seconds=0,
        episodes=[dict(source_episode_id='s', source_episode_index=0, episode_id='episode', dataset_id='d',
                       status='READY', quality_status='PASS', error_code=None, retryable=False,
                       next_action='open_episode', annotation_status=status, data_readable=True)])
    job = dict(manifest={'source_format':'LEROBOT_V3'}, upload_id='u', raw_source_id='raw')
    queue = SimpleNamespace(save=lambda value: None)
    Worker(queue, {'processing_endpoint':True}).processing(job, SimpleNamespace(api=lambda *args: result), '/u')
    assert job['state'] == 'ready' and job['data_status'] == 'READABLE'
    assert job['annotation_status'] == label and job['training_status'] == 'UNKNOWN'
    result['episodes'][0].pop('data_readable')
    result['episodes'][0].pop('annotation_status')
    Worker(queue, {'processing_endpoint':True}).processing(job, SimpleNamespace(api=lambda *args: result), '/u')
    assert job['state'] == 'ready' and job['data_status'] == job['annotation_status'] == 'UNKNOWN'


@pytest.mark.parametrize('source_format', ['MCAP', 'LEROBOT_V3'])
def test_shared_export_preparation_uses_original_samples_without_placeholder_assets(source_format, tmp_path, monkeypatch):
    import sys
    from mcap.writer import Writer
    from middleware.core.lerobot_export import prepare_mcap
    from middleware.core.topic_recorder import RECORDING_METADATA_NAME
    # Only the ROS serializer is substituted; MCAP traversal and export preparation are real.
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', SimpleNamespace(
        deserialize_message=lambda data, _: SimpleNamespace(**json.loads(data))))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', SimpleNamespace(get_message=lambda _: object))
    task = task_document(source_format=source_format)
    snapshot = snapshot_document(task, dict(recording_id='r', expected_plan_revision=1, source_format=source_format))
    source = tmp_path/'source.mcap'
    with source.open('wb') as stream:
        writer = Writer(stream); writer.start()
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'string[] name\nfloat64[] position')
        state = writer.register_channel('/observation/joint_states', 'cdr', schema)
        action = writer.register_channel('/action/joint_states', 'cdr', schema)
        writer.add_metadata(RECORDING_METADATA_NAME, {'recording_id':'r', 'capture_snapshot':json.dumps(snapshot)})
        for i in range(167):
            stamp = 10**18 + round(i*1e9/15)
            payload = json.dumps({'name':['joint'], 'position':[i]}).encode()
            writer.add_message(action, stamp, payload, stamp)
            writer.add_message(state, stamp, payload, stamp)
        writer.finish()
    staging = tmp_path/'stage'; staging.mkdir()
    fps = 15 if source_format == 'LEROBOT_V3' else 30
    path = prepare_mcap(source, staging, fps, task['name'], LOCAL['profile_id'])
    manifest = read_json(path)
    assert manifest['fps'] == fps
    if source_format == 'LEROBOT_V3':
        assert manifest['capture_snapshot'] == snapshot
        assert len(manifest['episodes'][0]) == 167
    else:
        assert 'capture_snapshot' not in manifest
        assert len(manifest['episodes'][0]) > 167
    assert json.loads(read_recording_metadata(source)['capture_snapshot']) == snapshot
    assert manifest['source_keys'] == ['action','observation.state']
    assert manifest['dimensions'] == {} and not list(staging.rglob('*.png'))
    assert all(set(row['values']) == {'action','observation.state'} for row in manifest['episodes'][0])
    assert manifest['episodes'][0][-1]['values']['action'] == [166.0]


@pytest.mark.parametrize('source_format', ['MCAP', 'LEROBOT_V3'])
def test_export_process_uses_captured_frequency_and_task_after_defaults_change(source_format, tmp_path, monkeypatch):
    from unittest.mock import Mock
    task = task_document(source_format=source_format)
    snapshot = snapshot_document(task, dict(recording_id='r', expected_plan_revision=1, source_format=source_format))
    raw = TopicRecorder({'directory': str(tmp_path), 'format':'mcap'}, tmp_path)
    raw.path = tmp_path/'capture.mcap'; raw.path.write_bytes(b'synthetic source')
    raw._active_metadata = {'capture_snapshot':json.dumps(snapshot), 'platform_task':json.dumps({'task':task})}
    raw.lerobot_fps, raw.lerobot_task = 30, 'changed default'
    launch = Mock(return_value=Mock())
    monkeypatch.setattr('middleware.core.topic_recorder.subprocess.Popen', launch)
    try:
        raw._start_lerobot_export()
        command = launch.call_args.args[0]
        assert command[command.index('--fps')+1] == ('15' if source_format == 'LEROBOT_V3' else '30')
        assert command[command.index('--task')+1] == (task['name'] if source_format == 'LEROBOT_V3' else 'changed default')
        assert json.loads(raw._active_metadata['capture_snapshot']) == snapshot
    finally:
        if raw._export_log: raw._export_log.close()
        raw._export_process = None
        raw.shutdown()


def test_retry_before_preflight_keeps_frozen_upload_format(tmp_path, monkeypatch, server):
    cfg, platform = environment(tmp_path, monkeypatch, server)
    recorder = recorder_for(cfg, platform, tmp_path, monkeypatch)
    original = platform.request
    try:
        def unavailable(*args, **kwargs):
            raise UploadError('NETWORK_UNCERTAIN', retry=True)
        monkeypatch.setattr(platform, 'request', unavailable)
        with pytest.raises(UploadError, match='NETWORK_UNCERTAIN'):
            recorder.start(selection=choice(platform, 'A'))
        original_id = recorder.records()[0]['recording_id']
        assert recorder.records()[0]['upload_format'] == 'mcap'
        recorder.set_upload_format('lerobot')
        monkeypatch.setattr(platform, 'request', original)
        record(recorder, choice(platform, 'A'))
        assert len(recorder.records()) == 1
        assert recorder.records()[0]['recording_id'] == original_id
        assert recorder.records()[0]['capture_snapshot']['source_format'] == 'MCAP'
        assert platform.upload.queue.jobs()[0]['manifest']['source_format'] == 'MCAP'
    finally:
        recorder.close()
