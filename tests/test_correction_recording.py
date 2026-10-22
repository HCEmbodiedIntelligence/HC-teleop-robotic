"""Ordered short windows, bounded lateness, real drain and local raw download."""
import json
import time
import zipfile

import pytest

from middleware.core.correction_recording import WindowRouter
from middleware.core.control_ownership import ControlToken
from tests.correction_recording_support import local_service as service


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
