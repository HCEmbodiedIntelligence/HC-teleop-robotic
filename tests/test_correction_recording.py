"""Ordered short windows, bounded lateness, real drain and local raw download."""
import json
import time
import zipfile

import pytest

from middleware.core.correction_recording import WindowRouter, SessionRouter
from middleware.core.control_ownership import ControlToken
from tests.correction_recording_support import local_service as service


def test_full_session_retains_program_manual_and_paused_data_with_multiple_boundaries():
    saved = []
    bounds = [1_000, 0]
    router = SessionRouter(lambda event: saved.append(event) or True, lambda _: bounds,
                           {'session_id': 'one'}, late_ns=10)
    router.observe('/action/joint_states', 'joint', b'program-before', 1_020, 1_020)
    router.open(dict(key='manual-1', start_ns=1_050))
    router.observe('/action/joint_states', 'joint', b'human-one', 1_060, 1_060)
    router.close(dict(key='manual-1', end_ns=1_100, reason='release'))
    router.observe('/action/joint_states', 'joint', b'program-after', 1_120, 1_120)
    router.open(dict(key='manual-2', start_ns=1_200))
    router.observe('/action/joint_states', 'joint', b'human-two', 1_220, 1_220)
    router.close(dict(key='manual-2', end_ns=1_250, reason='hold'))
    router.observe('/observation/joint_states', 'joint', b'paused-feedback', 1_300, 1_300)
    router.observe('/custom/headerless', 'custom', b'headerless', 0, 1_301)
    bounds[1] = 1_400
    router.ending = True
    router.observe('/action/joint_states', 'joint', b'outside', 1_400, 1_401)
    assert not router.finish_due(1_409)
    done = router.finish_due(1_410)
    assert len(done) == 1 and len(done[0]['boundary']['interventions']) == 2
    assert [item['_raw'] for item in saved if '_raw' in item] == [
        b'program-before', b'human-one', b'program-after', b'human-two', b'paused-feedback', b'headerless']
    events = [item['payload'] for item in saved if 'payload' in item]
    assert [event['kind'] for event in events] == [
        'session_open', 'manual_open', 'manual_close', 'manual_open', 'manual_close', 'session_close']
    assert all(event['capture_mode'] == 'full_session' for event in events)
    assert not router.finish_due(1_500)


def router_fixture():
    saved = [[], []]
    bounds = [[1_000_000_000, 0], [0, 0]]
    router = WindowRouter([lambda event, index=i: saved[index].append(event) or True for i in range(2)],
        lambda index: bounds[index], max_age_ns=100_000_000, late_ns=50_000_000, cache_bytes=100,
        long_gap_ns=50_000_000)
    token = ControlToken('controller', 'session', 1, 'vr', 'one')
    return router, saved, bounds, token


def opening(token, slot, start):
    return dict(key=token.key(), slot=slot, start_ns=start)


def action(token, seq, stamp):
    return dict(key=token.key(), command_seq=seq, stamp_ns=stamp, source='vr', intervention=True,
                names=['arm'], positions=[seq])


def test_initial_observation_source_time_and_short_window_no_auto_leak():
    router, saved, bounds, token = router_fixture()
    router.observe('camera', 'image', b'old', 800_000_000, 800_000_000)
    router.observe('camera', 'image', b'fresh', 990_000_000, 995_000_000)
    router.open(opening(token, 0, bounds[0][0]))
    initial = saved[0][1]
    assert initial['_raw'] == b'fresh'
    assert initial['_provenance']['role'] == 'initial_observation'
    assert initial['_provenance']['source_stamp_ns'] == 990_000_000
    bounds[0][1] = 1_100_000_000
    router.close(dict(key=token.key(), end_ns=bounds[0][1], last_command_seq=3, reason='release'))
    # Accepted after the close arrived, by original immutable identity/time.
    assert router.action(action(token, 3, 1_090_000_000), lambda *_: True)
    assert router.action(action(token, 1, 1_010_000_000), lambda *_: True)
    assert router.action(action(token, 2, 1_050_000_000), lambda *_: True)
    assert not router.action(action(token, 2, 1_050_000_000), lambda *_: True)
    assert not router.action(action(token, 4, 1_100_000_000), lambda *_: True)
    assert not router.action(dict(action(token, 4, 1_090_000_000), source='motion'), lambda *_: True)
    router.observe('camera', 'image', b'late', 1_080_000_000, 1_120_000_000)
    router.observe('camera', 'image', b'auto', 1_101_000_000, 1_120_000_000)
    done = router.finish_due(1_150_000_000)
    assert done[0]['quality']['missing_actions'] == 0
    assert done[0]['quality']['action_received'] == 3
    assert b'auto' not in [event.get('_raw') for event in saved[0]]
    assert [event['_raw'] for event in saved[0] if '_raw' in event] == [b'fresh', b'late']


def test_open_and_close_are_not_lost_when_next_grip_arrives_before_receiver():
    router, saved, bounds, one = router_fixture()
    two = ControlToken('controller', 'session', 3, 'vr', 'two')
    bounds[:] = [[1_000_000_000, 1_100_000_000], [1_110_000_000, 1_210_000_000]]
    router.observe('state', 'joint', b'one', 1_050_000_000, 1_070_000_000)
    router.observe('state', 'joint', b'two', 1_150_000_000, 1_170_000_000)
    for slot, token in enumerate((one, two)):
        router.open(opening(token, slot, bounds[slot][0]))
        router.close(dict(key=token.key(), end_ns=bounds[slot][1], last_command_seq=1, reason='release'))
        assert router.action(action(token, 1, bounds[slot][0]+10_000_000), lambda *_: True)
    assert len(router.finish_due(1_300_000_000)) == 2
    assert b'two' not in [event.get('_raw') for event in saved[0]]
    assert len([event for event in saved[1] if event.get('_provenance', {}).get('role') == 'initial_observation']) == 1


def test_missing_sequence_is_quality_not_failed_file_and_cache_has_byte_limit():
    router, saved, bounds, token = router_fixture()
    for n in range(50):
        router.observe('camera', 'image', bytes(10), 950_000_000+n, 950_000_000+n)
    assert router.bytes <= 100
    router.open(opening(token, 0, bounds[0][0]))
    assert router.action(action(token, 1, 1_001_000_000), lambda *_: True)
    assert router.action(action(token, 3, 1_010_000_000), lambda *_: True)
    bounds[0][1] = 1_100_000_000
    router.close(dict(key=token.key(), end_ns=bounds[0][1], last_command_seq=3, reason='release'))
    quality = router.finish_due(1_160_000_000)[0]['quality']
    assert quality['missing_actions'] == 1 and quality['grade'] == 'warning'
    assert quality['outcome'] == 'unknown'


def test_required_source_missing_and_silent_tail_are_quality_gaps():
    router, saved, bounds, token = router_fixture()
    router.required_topics = ('camera', 'state')
    router.observe('camera', 'image', b'initial', 990_000_000, 995_000_000)
    router.open(opening(token, 0, bounds[0][0]))
    assert router.action(action(token, 1, 1_001_000_000), lambda *_: True)
    bounds[0][1] = 1_100_000_000
    router.close(dict(key=token.key(), end_ns=bounds[0][1], last_command_seq=1, reason='release'))
    quality = router.finish_due(1_160_000_000)[0]['quality']
    assert quality['grade'] == 'warning' and quality['missing_topics'] == ['state']
    assert {gap['reason'] for gap in quality['invalid_ranges']} == {'missing_source', 'long_tail_gap'}
    assert quality['counts']['camera'] == 1


def test_existing_export_backlog_rejects_prepare_before_starting_processes(tmp_path):
    recorder = service(tmp_path)
    recorder.config['max_pending_jobs'] = 1
    recorder.pending.add('pending')
    try:
        with pytest.raises(ValueError, match='pending recordings'):
            recorder.prepare_corrections()
        assert recorder.corrections is None
        assert not list((tmp_path/'data').glob('*.mcap'))
    finally:
        recorder.pending.clear()
        recorder.shutdown()


def test_prepared_slot_is_hidden_and_cannot_be_uploaded_before_intervention(tmp_path):
    from middleware.core.topic_recorder import TopicRecorder
    from humanoid_upload.common import UploadError
    recorder = service(tmp_path)
    slot = TopicRecorder(recorder.config, tmp_path)
    slot.task_managed = True
    try:
        recorder.start('prepared.mcap', _prepared_writer=slot)
        identity = next(iter(recorder.jobs))
        assert not recorder.records()
        assert not recorder.list_recordings()
        with pytest.raises(UploadError) as error:
            recorder.upload_recording(identity)
        assert error.value.code == 'RECORDING_NOT_READY'
        assert recorder.jobs[identity]['state'] == 'armed'
    finally:
        slot.shutdown()
        recorder.shutdown()
