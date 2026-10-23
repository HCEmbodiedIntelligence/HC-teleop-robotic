"""Exact recording time boundaries and partial writer failure reporting."""
import pytest

from middleware.core.correction_recording import WindowRouter


def fixture(*, drop=None):
    saved = []
    bounds = [1000, 1100]

    def sink(event):
        if drop and drop(event):
            return False
        saved.append(event)
        return True

    router = WindowRouter([sink], lambda _: bounds, max_age_ns=100, late_ns=50,
                          long_gap_ns=1000, cache_bytes=16)
    router.open(dict(key='one', slot=0, start_ns=1000))
    return router, saved


def evidence(**changes):
    return dict(dict(key='one', source='vr', intervention=True, command_seq=1, stamp_ns=1050,
                     names=['arm'], positions=[0.1]), **changes)


def finish(router):
    router.close(dict(key='one', end_ns=1100, last_command_seq=1, reason='release'))
    assert not router.finish_due(1149)
    result = router.finish_due(1150)[0]
    assert not router.finish_due(1200)
    return result['quality']


@pytest.mark.parametrize('stamp,received,accepted', [
    (1000,1000,True), (1099,1150,True), (1099,1151,False), (1100,1100,False),
    (999,1000,False), (1051,1050,False), (0,1050,False), (1000,1150,True),
    (999,1150,False), (1000,1151,False),
])
def test_observation_half_open_window_and_exact_lateness(stamp, received, accepted):
    router, saved = fixture()
    router.observe('camera', 'image', b'frame', stamp, received)
    assert any(e.get('_raw') == b'frame' for e in saved) is accepted


@pytest.mark.parametrize('changes', [
    {'command_seq':True}, {'command_seq':0}, {'command_seq':-1}, {'command_seq':1.0},
    {'stamp_ns':999}, {'stamp_ns':1100}, {'source':'motion'}, {'intervention':False}, {'key':'old'},
])
def test_invalid_or_foreign_action_never_reaches_writer(changes):
    router, saved = fixture()
    called = []
    assert not router.action(evidence(**changes), lambda *args: called.append(args) or True)
    assert not called and not any(e.get('topic') == '/events/control/action_evidence' for e in saved)


@pytest.mark.parametrize('dropped', ['manual_open', 'manual_close', 'evidence', 'action', 'observation'])
def test_partial_recording_failure_cannot_receive_ok_quality(dropped):
    router, _ = fixture(drop=lambda e: e.get('payload', {}).get('kind') == dropped
        or (dropped == 'evidence' and e.get('topic') == '/events/control/action_evidence')
        or (dropped == 'observation' and '_raw' in e))
    router.observe('camera', 'image', b'frame', 1050, 1050)
    router.action(evidence(), lambda *_: dropped != 'action')
    quality = finish(router)
    assert quality['grade'] == 'warning', quality
    if dropped == 'manual_close':
        assert quality['end_event_in_mcap'] is False


def test_oversized_observation_does_not_expand_cache_but_can_be_recorded_live():
    router, saved = fixture()
    router.observe('camera', 'image', b'x' * 1024, 1050, 1050)
    assert router.bytes <= 16
    assert any(e.get('_raw') == b'x' * 1024 for e in saved)


def test_extremely_fragmented_action_sequences_remain_bounded_and_flag_loss():
    router, _ = fixture()
    for seq in range(1, 2048, 2):
        assert router.action(evidence(command_seq=seq), lambda *_: True)
    assert not router.action(evidence(command_seq=2049), lambda *_: True)
    assert len(router.windows['one']['action_ranges']) == 1024
    router.close(dict(key='one', end_ns=1100, last_command_seq=2049, reason='release'))
    quality = router.finish_due(1150)[0]['quality']
    assert quality['grade'] == 'warning' and quality['missing_actions'] == 1025
