"""Behavior checks against E's frozen inputs, plus the real recorder/exporter."""
import copy
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import time

import av
import numpy as np
import pyarrow.parquet as pq
import pytest
from sensor_msgs.msg import JointState

from middleware.core.capture import completion, exporter
from middleware.core.capture.alignment import Aligner
from middleware.core.capture.buffers import TimeCache
from middleware.core.capture.catalog import connection, edits, read_json, save_edits
from middleware.core.capture.clocks import Clocks
from middleware.core.capture.config import validate
from middleware.core.capture.models import normalize
from middleware.core.capture.named import action_at
from middleware.core.capture.offline import realign
from middleware.core.capture.profile import openarm_profile
from middleware.core.capture.ros_bridge import joint_document, validity_document
from middleware.core.capture.synthetic import generate, header, synthetic_config

G0 = Path(os.environ.get('OPENARM_G0', '/home/hc_op/workspace/openarm-data-integration-plan/g0'))
CASES = json.loads((G0 / 'examples/action-cases.json').read_text()) if (G0 / 'examples/action-cases.json').exists() else []


@pytest.mark.parametrize('case', CASES, ids=lambda c: c['name'])
def test_frozen_action_cases(case):
    docs = []
    for message in case['messages']:
        doc = {'joint_names': message['names'], 'payload': {'position': message['positions']},
               'capture_time_ns': message['source_ms'] * 10**6, 'receive_time_ns': message['received_ms'] * 10**6,
               'source_seq': message['sequence'], 'clock_epoch': message['epoch'], 'source_epoch': message['epoch'],
               'validity_known': True, 'hold_leases': {}}
        for lease in case.get('hold_leases', []):
            doc['hold_leases'].setdefault(lease['axis'], []).append({
                'command_sequence': lease['command_sequence'], 'source_epoch': lease['epoch'],
                'issued_ns': lease['issued_ms'] * 10**6, 'valid_until_ns': lease['valid_until_ms'] * 10**6})
        docs.append(doc)
    for event in case.get('invalidations', []):
        docs.append({'joint_names': event['axes'], 'capture_time_ns': event['time_ms'] * 10**6,
                     'receive_time_ns': event['time_ms'] * 10**6, 'source_seq': 999,
                     'clock_epoch': case['epoch'], 'source_epoch': case['epoch'], 'validity_event': event['reason']})
    axes = [{'name': n, 'max_age_ms': case['max_age_ms'][n], 'mode': case.get('mode', 'periodic')} for n in case['axes']]
    actual = action_at(docs, axes, case['target_ms'] * 10**6, case['epoch'], case['observed_ms'] * 10**6)
    assert {k: actual[k] for k in ('values', 'valid')} == case['expected']


def aligned_input():
    cfg = synthetic_config()
    # Unit association does not require camera messages.
    cfg['sources'] = {n: s for n, s in cfg['sources'].items() if s['kind'] in {'state', 'action'}}
    cache = TimeCache(cfg['buffers'])
    return cfg, cache, Aligner(cfg, cache)


def offer(cfg, cache, ident, names, stamp, values, seq, *, evidence=True, epoch=0, control='control-0', receive=None):
    source = cfg['sources'][ident]
    d = joint_document(ident, source, JointState(header=header(stamp), name=names, position=list(map(float, values))),
                       seq, receive or stamp, seq, epoch,
                       evidence={'validity_known': evidence, 'source_epoch': control})
    sample = normalize(d, source, Clocks(), 'synthetic-test', seq)
    cache.add(sample, stamp)
    return sample


def test_interleaved_arms_grippers_state_interpolation_and_causal_commands():
    cfg, cache, aligner = aligned_input()
    axes = openarm_profile()['axes']
    base = 1_000_000_000
    for ident, source in cfg['sources'].items():
        names = [a['name'] for a in source['named_axes']]
        for seq, stamp in enumerate((base, base + 20_000_000), 1):
            for i, name in enumerate(reversed(names)):
                value = next(i for i, a in enumerate(axes) if a['name'] == name) + seq
                offer(cfg, cache, ident, [name], stamp, [value], seq * 100 + i)
    row = aligner.row(0, base + 10_000_000, base + 20_000_000, deadline=10**10)
    assert row['valid'], row['invalid_reasons']
    public_order = [*range(7,14),15,*range(7),14]
    assert exporter.vector(row, 'state', cfg)[0] == [i + 1.5 for i in public_order]
    assert exporter.vector(row, 'action', cfg)[0] == [i + 1 for i in public_order]
    assert exporter.vector(row, 'action', cfg)[1] == [*[f'Joint{i}_R' for i in range(1,8)],'R_ban',*[f'Joint{i}_L' for i in range(1,8)],'L_ban']


def test_bare_jointstate_unknown_and_bad_units_never_exportable():
    cfg, cache, aligner = aligned_input()
    source = cfg['sources']['arm_0_action']
    names = [a['name'] for a in source['named_axes']]
    sample = offer(cfg, cache, 'arm_0_action', names, 10**9, [1] * 14, 1, evidence=False)
    assert not any(aligner.row(0, 10**9, 10**9, deadline=10**10)['action']['arm_0_action']['components']['valid'])
    bad = sample.data()
    bad['joint_units'][0] = 'm'
    bad['validity_known'] = True
    normalized = normalize(bad, source, Clocks(), 'synthetic-test', 2)
    assert not normalized.data()['valid']
    with pytest.raises(ValueError, match='unique names'):
        joint_document('arm_0_action', source, JointState(header=header(10**9), name=[names[0]] * 2, position=[1., 2.]), 2, 10**9, 2)


@pytest.mark.parametrize('reason', ['pause','input_loss','clutch_release','rebind','clock_epoch_change','source_restart'])
def test_termination_and_delayed_previous_control_epoch(reason):
    cfg, cache, aligner = aligned_input()
    ident = 'arm_0_action'
    names = [a['name'] for a in cfg['sources'][ident]['named_axes']]
    base = 10**9
    offer(cfg, cache, ident, names, base, [1] * 14, 1)
    d = validity_document(ident, cfg['sources'][ident], names[:7], reason, base + 10**6, 2, base + 10**6, 2, 0, 'control-1')
    cache.add(normalize(d, cfg['sources'][ident], Clocks(), 's', 2), base + 10**6)
    # Old producer packet delayed across a rebind, even with a later source timestamp.
    offer(cfg, cache, ident, names[:7], base + 2 * 10**6, [9] * 7, 3)
    row = aligner.row(0, base + 3 * 10**6, base, deadline=10**10)
    valid = row['action'][ident]['components']['valid']
    assert valid == [False] * 7 + [True] * 7
    offer(cfg, cache, ident, names[:7], base + 4 * 10**6, [2] * 7, 1, control='control-1')
    row = aligner.row(0, base + 5 * 10**6, base, deadline=10**10)
    assert all(row['action'][ident]['components']['valid'])
    assert not any(aligner.row(0, base + 5 * 10**6, base, deadline=10**10, epoch=1)['action'][ident]['components']['valid'])


def test_ttl_boundaries_and_sequence_tie_break():
    axes = [{'name': 'x', 'max_age_ms': 100}]
    doc = {'joint_names': ['x'], 'payload': {'position': [1]}, 'capture_time_ns': 10**9,
           'receive_time_ns': 10**9, 'clock_epoch': 0, 'source_epoch': 's', 'validity_known': True, 'source_seq': 1}
    newer = {**doc, 'payload': {'position': [2]}, 'source_seq': 2}
    assert action_at([newer, doc], axes, 1_100_000_000, 0)['values'] == [2]
    assert action_at([doc], axes, 1_100_000_001, 0)['valid'] == [False]
    assert action_at([doc], axes, 10**9, 0, 999_999_999)['valid'] == [False]


@pytest.fixture(scope='module')
def produced(tmp_path_factory):
    root = tmp_path_factory.mktemp('openarm') / 'actual'
    seen = []
    def done(event):
        assert Path(event['root']).is_dir()
        assert Path(event['marker_path']).is_file()
        assert not Path(event['root']).name.startswith('.export-')
        seen.append(event)
    result = generate(root, on_complete=done)
    assert len(seen) == 1
    return result


def test_real_export_matches_public_contract_and_values(produced):
    root = Path(produced['path'])
    completion.validate_tables(root,read_json(root/'capture-context.json'))
    assert read_json(root/'meta/info.json')['total_frames'] == 12
    context = read_json(root / 'capture-context.json')
    assert [e['outcome'] for e in context['episodes']] == ['success', 'failure']
    assert context['synthetic'] is True and context['physical_sync'] == 'unknown'
    mappings = [json.loads(line) for line in (root / 'source_mapping.jsonl').read_text().splitlines()]
    all_rows = pq.read_table(root / 'data').to_pylist()
    for m, row in zip(mappings, all_rows):
        import struct
        cloud, data = exporter.PointCloudReader(root).get('synthetic_cloud',m['episode_index'],m['frame_index'])
        assert struct.unpack('<f',data)[0] == m['source_frame']
        assert cloud['source_depth_seq'] == m['cameras']['camera_left']['source_frame']
        tick = (m['target_ns'] - produced['input_base_ns']) / 10**7
        for public_i,i in enumerate([*range(7,14),15,*range(7),14]):
            expected = (i + 1) * .1 + tick * .001 if i < 14 else .01 + .005 * (i % 2) + tick * .00001
            assert row['observation.state'][public_i] == pytest.approx(expected, rel=1e-6, abs=1e-7)
            action_tick = (m['action_source_ns'][i] - produced['input_base_ns']) / 10**7
            expected = (i + 1) * .1 + action_tick * .001 + .05 if i < 14 else .011 + .005 * (i % 2) + action_tick * .00001
            assert row['action'][public_i] == pytest.approx(expected, rel=1e-6, abs=1e-7)
        for ident, camera in m['cameras'].items():
            metadata, depth = exporter.DepthReader(root).get(ident, m['episode_index'], m['frame_index'])
            assert np.all(depth == 1000 + m['source_frame'])
            assert metadata['rgb']['source_seq'] == camera['source_frame']
    for cindex, camera in enumerate(context['profile']['cameras']):
        for ep, source in enumerate(context['episodes']):
            camera_id = "head" if camera["id"] == "camera_hand" else camera["id"]
            path = root / f"videos/observation.images.{camera_id}/chunk-000/file-{ep:03d}.mp4"
            with av.open(str(path)) as video:
                frames = list(video.decode(video=0))
            assert len(frames) == 6
            for j, frame in enumerate(frames):
                expected = ((source['start_frame'] + j) * 7 + cindex * 40) % 220
                assert abs(float(frame.to_ndarray(format='rgb24')[100:120,100:120].mean()) - expected) <= 5


def test_manual_invalid_episode_rejected_without_event(produced, tmp_path):
    session = Path(produced['session'])
    old = edits(session)['data']
    changed = copy.deepcopy(old)
    changed['invalid_intervals'] = [{'start_frame': 2, 'end_frame': 3, 'reason': 'synthetic rejection'}]
    save_edits(session, changed, edits(session)['etag'])
    seen = []
    try:
        with pytest.raises(ValueError, match='无效'):
            exporter.export(session, tmp_path / 'rejected', on_complete=seen.append)
        assert not seen and not (tmp_path / 'rejected').exists()
        assert list(tmp_path.glob('.export-*/FAILED.json'))
        assert not list(tmp_path.glob('.export-*/export-complete.json'))
    finally:
        save_edits(session, old, edits(session)['etag'])


@pytest.mark.parametrize('failure', [RuntimeError, KeyboardInterrupt])
def test_encoder_failure_or_interruption_never_completes(produced, tmp_path, monkeypatch, failure):
    seen = []
    def fail(*args):
        raise failure('synthetic interrupted export')
    monkeypatch.setattr(exporter.TrainingVideo, 'write', fail)
    with pytest.raises(failure):
        exporter.export(produced['session'], tmp_path / 'interrupted', on_complete=seen.append)
    assert not seen and not (tmp_path / 'interrupted').exists()
    assert list(tmp_path.glob('.export-*/FAILED.json'))
    assert not list(tmp_path.glob('.export-*/export-complete.json'))


def test_missing_camera_data_preserves_raw_and_rejects(produced, tmp_path):
    session = Path(produced['session'])
    raw_hash = hashlib.sha256((session / 'raw.mcap').read_bytes()).hexdigest()
    with sqlite3.connect(session / 'index.sqlite3') as db:
        original = db.execute('SELECT body FROM rows WHERE k=0').fetchone()[0]
        row = json.loads(original)
        row['images']['camera_left']['source_seq'] = 99999
        db.execute('UPDATE rows SET body=? WHERE k=0', (json.dumps(row),))
    try:
        with pytest.raises(ValueError, match='rgb_binding_missing'):
            exporter.export(session, tmp_path / 'missing')
    finally:
        with sqlite3.connect(session / 'index.sqlite3') as db:
            db.execute('UPDATE rows SET body=? WHERE k=0', (original,))
    assert hashlib.sha256((session / 'raw.mcap').read_bytes()).hexdigest() == raw_hash
    assert not (tmp_path / 'missing').exists()


def test_offline_realign_keeps_both_arms_and_grippers(produced):
    session = Path(produced['session'])
    result = realign(session)
    with connection(session) as db, sqlite3.connect(session / 'alignments' / result['version'] / 'index.sqlite3') as offline:
        original = json.loads(db.execute('SELECT body FROM rows WHERE k=3').fetchone()[0])
        actual = json.loads(offline.execute('SELECT body FROM rows WHERE k=3').fetchone()[0])
    assert exporter.vector(original, 'action', read_json(session/'session.json')['config']) == exporter.vector(actual, 'action', read_json(session/'session.json')['config'])


def test_crc64_xz_and_bundled_schema_are_frozen(tmp_path):
    (tmp_path / 'check').write_bytes(b'123456789')
    assert completion.inventory(tmp_path)[0]['crc64'] == str(0x995DC9BBDF1939FA)
    from middleware.core.capture.profile import CONTRACTS
    for path in CONTRACTS.glob('*.json'):
        expected = G0 / ('profiles' if path.name == 'openarmx-v1.json' else 'schemas') / path.name
        if not expected.exists():
            pytest.skip('External frozen G0 fixture is unavailable; migrated contracts remain installed')
        assert path.read_bytes() == expected.read_bytes()


def test_profile_units_dimensions_and_rate_must_agree():
    cfg = synthetic_config()
    cfg['sources']['gripper_0_action']['named_axes'][0]['unit'] = 'rad'
    with pytest.raises(ValueError, match='named action'):
        validate(cfg)
    cfg = synthetic_config()
    cfg['dataset']['fps'] = 20
    with pytest.raises(ValueError, match='30 Hz'):
        validate(cfg)


def test_context_freezes_at_recording_start_and_unbound_needs_explicit_binding(tmp_path):
    from middleware.core.capture.session import Session
    cfg = synthetic_config()
    metadata = {'capture_context': {'local_robot_id': 'local', 'synthetic': True,
                                   'model_sha256': 'a' * 64, 'configuration_sha256': 'b' * 64}}
    reports = {ident: {'registered_monotonic_ns': time.monotonic_ns()} for ident in cfg['sources']}
    session = Session(tmp_path, cfg, Clocks(), reports, metadata)
    metadata['capture_context']['local_robot_id'] = 'changed-global'
    cfg['capture']['profile']['axes'][0]['name'] = 'changed-global'
    assert session.metadata['capture_context']['local_robot_id'] == 'local'
    assert session.metadata['capture_context']['profile']['axes'][0]['name'] == 'openarmx_left_joint1'
    snapshot = {'session_id': session.id, 'metadata': session.metadata, 'started_unix_ns': 10**9}
    manifest = {'state': 'completed', 'ended_unix_ns': 2 * 10**9, 'committed': [{'path': 'raw.mcap'}]}
    annotations = {'episodes': [{'source_episode_id': 'e', 'outcome': 'unknown', 'start_frame': 0, 'end_frame': 1, 'task': 'offline'}], 'invalid_intervals': []}
    from middleware.core.capture.storage import atomic_json
    atomic_json(session.path / 'session.json', snapshot)
    atomic_json(session.path / 'manifest.json', manifest)
    with pytest.raises(ValueError, match='explicit frozen'):
        completion.context_for_export(session.path, snapshot, manifest, annotations)
    completion.bind_offline_session(session.path, robot_id='robot', collection_task_id='task')
    context = completion.context_for_export(session.path, snapshot, manifest, annotations)
    assert context['collection_task_id'] == 'task'
    with pytest.raises(FileExistsError):
        completion.bind_offline_session(session.path, robot_id='robot', collection_task_id='changed')


@pytest.mark.parametrize('stage', ['before_marker', 'after_marker', 'after_rename'])
def test_sealing_or_rename_durability_failure_has_no_upload_marker(produced, tmp_path, monkeypatch, stage):
    seen = []
    if stage == 'before_marker':
        def fail(*args):
            raise OSError('synthetic disk failure')
        monkeypatch.setattr(completion, 'validate_tables', fail)
    elif stage == 'after_marker':
        real = completion.seal
        def fail(*args):
            real(*args)
            raise OSError('synthetic seal interruption')
        monkeypatch.setattr(completion, 'seal', fail)
    else:
        real = completion.sync_directory
        def fail(path):
            if Path(path) == tmp_path:
                raise OSError('synthetic parent fsync failure')
            return real(path)
        monkeypatch.setattr(completion, 'sync_directory', fail)
    with pytest.raises(OSError):
        exporter.export(produced['session'], tmp_path / 'failed', on_complete=seen.append)
    assert not seen and not (tmp_path / 'failed').exists()
    assert list(tmp_path.glob('.export-*/FAILED.json'))
    assert not list(tmp_path.glob('.export-*/export-complete.json'))


def test_subscriber_failure_retains_recoverable_completed_marker(produced, tmp_path):
    def unavailable(event):
        raise RuntimeError('D unavailable')
    result = exporter.export(produced['session'], tmp_path / 'sealed', on_complete=unavailable)
    assert result['notification_error'] == 'D unavailable'
    assert (tmp_path / 'sealed/export-complete.json').is_file()
    assert not list((tmp_path / 'sealed').rglob('FAILED.json'))
    assert read_json(tmp_path / 'sealed/capture-context.json')['export_id'] != read_json(Path(produced['path']) / 'capture-context.json')['export_id']


def test_new_contract_rejects_unfinished_failed_and_interrupted_session(produced, tmp_path):
    from middleware.core.capture.storage import atomic_json
    path = Path(produced['session']) / 'manifest.json'
    original = read_json(path)
    try:
        for state in ('recording', 'failed', 'interrupted'):
            atomic_json(path, {**original, 'state': state})
            with pytest.raises(ValueError):
                exporter.export(produced['session'], tmp_path / state)
            assert not (tmp_path / state).exists()
    finally:
        atomic_json(path, original)


def test_generic_profile_uses_two_dimensions_one_camera(tmp_path):
    profile = openarm_profile()
    profile.update(robot_type='generic-test', profile_version='generic-test-v1',
                   axes=[{'name': 'hinge', 'unit': 'rad', 'group': 'arm'}, {'name': 'finger', 'unit': 'm', 'group': 'gripper'}],
                   cameras=[{'id': 'single', 'width': 320, 'height': 240, 'calibration_ref': 'synthetic:test'}])
    result = generate(tmp_path / 'generic', profile)
    from middleware.core.capture.inspect_capture import inspect
    report = inspect(result['path'], result['session'])
    assert report['dimensions'] == {'observation.state': [2], 'action': [2]}
    assert report['cameras']['single']['frames'] == 12
    root=Path(result['path'])
    completion.validate_tables(root,read_json(root/'capture-context.json'))
    assert read_json(root/'meta/info.json')['features']['observation.images.single_depth']['shape'] == [240,320,1]


def test_named_queue_loss_stops_session_instead_of_losing_termination(tmp_path):
    from middleware.core.capture.session import Session
    cfg = synthetic_config()
    cfg['buffers']['ingress_items'] = 1
    metadata = {'capture_context': {'local_robot_id': 'local', 'synthetic': True,
                                   'model_sha256': 'a' * 64, 'configuration_sha256': 'b' * 64}}
    reports = {ident: {'registered_monotonic_ns': time.monotonic_ns()} for ident in cfg['sources']}
    session = Session(tmp_path, cfg, Clocks(), reports, metadata)
    # Deliberately hold the consumer before start to fill the real bounded queue.
    session.state = 'warming_up'
    source = cfg['sources']['arm_0_action']
    names = [a['name'] for a in source['named_axes']]
    stamp = time.time_ns()
    doc = joint_document('arm_0_action', source, JointState(header=header(stamp), name=names, position=[1.] * len(names)),
                         1, stamp, time.monotonic_ns(), evidence={'validity_known': True, 'source_epoch': 's'})
    assert session.submit(doc)['accepted']
    event = validity_document('arm_0_action', source, names, 'pause', stamp + 1, 2, stamp + 1,
                              time.monotonic_ns(), 0, 's-next')
    assert not session.submit(event)['accepted']
    assert session.stop_event.is_set()
    assert session.errors[-1]['reason'] == 'named_input_ingress_overflow:arm_0_action'


def test_fresh_session_never_inherits_previous_action():
    cfg, cache, first = aligned_input()
    ident = 'gripper_0_action'
    names = [a['name'] for a in cfg['sources'][ident]['named_axes']]
    offer(cfg, cache, ident, names, 10**9, [.01, .04], 1)
    assert first.row(0, 1_150_000_000, 10**9, deadline=10**10)['action'][ident]['components']['valid'] == [True, True]
    assert first.row(0, 1_150_000_001, 10**9, deadline=10**10)['action'][ident]['components']['valid'] == [False, False]
    second = Aligner(cfg, TimeCache(cfg['buffers']))
    assert second.row(0, 10**9, 10**9, deadline=10**10)['action'][ident]['components']['values'] == [None, None]


def test_session_future_and_duplicate_packets_do_not_poison_current_commands(tmp_path):
    from middleware.core.capture.session import Session
    cfg = synthetic_config()
    metadata = {'capture_context': {'local_robot_id': 'local', 'synthetic': True,
                                   'model_sha256': 'a' * 64, 'configuration_sha256': 'b' * 64}}
    reports = {ident: {'registered_monotonic_ns': time.monotonic_ns()} for ident in cfg['sources']}
    session = Session(tmp_path, cfg, Clocks(), reports, metadata)
    ident = 'arm_0_action'
    source = cfg['sources'][ident]
    names = [a['name'] for a in source['named_axes']]
    base = time.time_ns()
    for seq, stamp, value in [(1, base, 1.), (2, base + 2 * 10**9, 99.), (3, base + 10**6, 2.), (3, base + 10**6, 2.)]:
        d = joint_document(ident, source, JointState(header=header(stamp), name=names, position=[value] * len(names)),
                           seq, base + 2 * 10**6, time.monotonic_ns(), evidence={'validity_known': True, 'source_epoch': 's'})
        session.process(normalize(d, source, Clocks(), session.id, time.monotonic_ns()))
    row = session.aligner.row(0, base + 2 * 10**6, base, deadline=2**63-1)
    assert row['action'][ident]['components']['values'] == [2.] * len(names)
    assert session.counters['duplicate_samples'] == 1
    assert not session.counters['clock_jumps:' + ident]


def test_named_actions_require_source_timestamp_quality():
    cfg, cache, aligner = aligned_input()
    ident = 'arm_0_action'
    source = cfg['sources'][ident]
    names = [a['name'] for a in source['named_axes']]
    original = offer(cfg, cache, ident, names, 10**9, [1.] * len(names), 1)
    doc = original.data()
    doc.update(source_seq=2, timestamp_quality='host_receive')
    cache.add(normalize(doc, source, Clocks(), 's', 2), 10**9)
    assert not any(aligner.row(0, 10**9, 10**9, deadline=10**10)['action'][ident]['components']['valid'])
