"""Recorder-side targets use the next DATASET frame, not the next device packet."""
from copy import deepcopy
import json

import pytest

from middleware.core.capture.alignment import target_time
from middleware.core.capture.models import Sample, freeze_json
from middleware.core.capture.next_state_action import NextStateActions

START = 1_000_000_000


def config():
    axes = [{'name': 'joint', 'unit': 'rad'}]
    return {'dataset': {'fps': 30}, 'buffers': {'cache_items_per_source': 128,
        'cache_bytes': 1024*1024, 'retention_ms': 1000},
        'alignment': {'primary_camera': 'camera', 'state_reference': 'grid', 'strict': True,
            'joint_max_bracket_gap_ms': 30, 'delay_ms': 80, 'max_catchup_rows': 8, 'causal': False},
        'sources': {'state': {'kind': 'state', 'named_axes': axes},
                    'action': {'kind': 'action', 'named_axes': axes}}}


def sample(stamp, value, *, ident='state', valid=True, epoch='one', continuity='one'):
    data = {'source_id': ident, 'source_seq': stamp-START, 'capture_time_ns': stamp,
        'receive_time_ns': stamp+1000, 'clock_epoch': 0, 'clock_id': 'ros',
        'source_epoch': epoch, 'continuity_id': continuity, 'timestamp_quality': 'ros_stamped',
        'joint_names': ['joint'], 'payload': {'position': [value]}, 'valid': valid}
    return Sample(ident, data['source_seq'], stamp, stamp+1000, stamp+1000, 0, freeze_json(data))


def feed(generator):
    for tick in range(12):
        generator.observe(sample(START+tick*10_000_000, float(tick)))


def primary_camera(generator, index, offset):
    stamp = START+offset
    data = {'source_id': 'camera', 'source_seq': index, 'rgb': {
        'capture_time_ns': stamp, 'exposure_midpoint_ns': stamp,
        'timestamp_semantics': 'exposure_midpoint', 'timestamp_evidence': 'fixture',
        'source_seq': index, 'clock_id': 'camera-clock', 'clock_epoch': 0}}
    generator.observe(Sample('camera', index, stamp, stamp, stamp, 0, freeze_json(data)))


def test_primary_actions_follow_actual_next_image_and_never_the_nominal_grid():
    cfg = config()
    cfg['alignment'].update(timeline_mode='primary_camera', state_reference='primary_exposure_midpoint')
    g = NextStateActions(cfg)
    feed(g)
    for index, offset in enumerate((6_000_000, 40_000_000, 73_000_000)):
        primary_camera(g, index, offset)
    result = list(g.advance(START, START+100_000_000, end=START+100_000_000, final=True))
    assert [r['capture_time_ns'] for r in result] == [START+n for n in (6_000_000,40_000_000,73_000_000)]
    assert result[0]['payload']['position'] == [4.]
    assert result[1]['payload']['position'] == pytest.approx([7.3])
    assert result[0]['primary_frame']['rgb_source_seq'] == 0
    assert result[0]['next_primary_frame']['rgb_source_seq'] == 1
    assert result[-1]['valid'] is False


def test_primary_action_uses_next_recorded_camera_frame_across_a_drop():
    cfg = config()
    cfg['alignment'].update(timeline_mode='primary_camera', state_reference='primary_exposure_midpoint')
    g = NextStateActions(cfg)
    feed(g)
    primary_camera(g, 0, 6_000_000)
    primary_camera(g, 2, 73_000_000)
    result = list(g.advance(START, START+100_000_000, end=START+100_000_000, final=True))
    assert len(result) == 2
    assert result[0]['valid'] and not result[1]['valid']
    assert result[0]['payload']['position'] == pytest.approx([7.3])
    assert result[0]['action_dt_ns'] == 67_000_000
    assert result[0]['next_primary_frame']['rgb_source_seq'] == 2


def test_next_dataset_frame_not_previous_or_same_frame_and_last_is_invalid():
    g = NextStateActions(config())
    feed(g)
    end = target_time(START, 3, 30)
    result = list(g.advance(START, end, end=end, final=True))
    assert len(result) == 3
    assert result[0]['payload']['position'] == pytest.approx([3.3333333])
    assert result[1]['payload']['position'] == pytest.approx([6.6666667])
    assert result[0]['action_stage'] == 'derived'
    assert result[-1]['valid'] is False and result[-1]['payload']['position'] == [None]
    assert result[-1]['invalid_reasons'] == ['joint:next_frame_missing']
    assert list(g.advance(START, end, end=end, final=True)) == []


@pytest.mark.parametrize('invalid', [False, True])
def test_even_invalid_real_action_permanently_suppresses_candidates(invalid):
    g = NextStateActions(config())
    feed(g)
    g.observe(sample(START, 99., ident='action', valid=not invalid))
    assert list(g.advance(START, START+100_000_000, end=START+100_000_000, final=True)) == []
    assert g.status()['real_action_sources'] == ['action']


def test_late_real_source_disqualifies_previously_written_candidates():
    g = NextStateActions(config())
    feed(g)
    assert list(g.advance(START, START+120_000_000))
    g.observe(sample(START+60_000_000, 99., ident='action'))
    assert list(g.advance(START, START+100_000_000, end=START+100_000_000, final=True)) == []
    assert g.status()['selected_sources'] == []
    assert g.status()['candidate_counts']['action'] > 0


def test_late_camera_waits_for_next_frame_instead_of_reusing_old_frame():
    cfg = config()
    cfg['alignment']['state_reference'] = 'primary_exposure_midpoint'
    g = NextStateActions(cfg)
    feed(g)
    def camera(offset):
        stamp = START+offset
        d = {'rgb': {'capture_time_ns': stamp, 'exposure_midpoint_ns': stamp,
                    'timestamp_semantics': 'exposure_midpoint', 'timestamp_evidence': 'fixture'}}
        g.observe(Sample('camera', offset, stamp, stamp, stamp, 0, freeze_json(d)))
    camera(1_000_000)
    camera(34_000_000)
    first = list(g.advance(START, START+155_000_000))
    assert len(first) == 1 and first[0]['payload']['position'] == pytest.approx([3.4])
    camera(67_000_000)
    second = list(g.advance(START, START+160_000_000))
    assert len(second) == 1 and second[0]['payload']['position'] == pytest.approx([6.7])
    assert second[0]['next_state_time_ns'] == START+67_000_000


@pytest.mark.parametrize('change', ['epoch', 'continuity', 'invalid', 'gap'])
def test_does_not_bridge_state_faults(change):
    g = NextStateActions(config())
    for tick in range(12):
        if change == 'gap' and 1 <= tick <= 5:
            continue
        kwargs = {'epoch': 'two'} if change == 'epoch' and tick >= 2 else {}
        if change == 'continuity' and tick >= 2: kwargs['continuity'] = 'two'
        if change == 'invalid' and tick >= 2: kwargs['valid'] = False
        g.observe(sample(START+tick*10_000_000, float(tick), **kwargs))
    first = list(g.advance(START, START+100_000_000, end=START+100_000_000, final=True))[0]
    assert first['valid'] is False


def test_real_capture_writes_candidates_before_stop_and_sealing_preserves_them(tmp_path, monkeypatch):
    from pathlib import Path
    from mcap.reader import make_reader
    from middleware.core.capture.session import Session
    from middleware.core.capture.storage import Storage
    from middleware.core.capture.synthetic import generate
    from middleware.core.capture import synthetic

    submit = Session.submit
    required = Storage.required
    synthetic_config = synthetic.synthetic_config
    def enabled_config(*args, **kwargs):
        cfg = synthetic_config(*args, **kwargs)
        cfg['alignment']['derive_action_from_next_state'] = True
        return cfg
    during_capture = []
    def state_only(self, metadata, *args, **kwargs):
        if self.cfg['sources'][metadata['source_id']]['kind'] == 'action':
            return {'accepted': True}
        return submit(self, metadata, *args, **kwargs)
    def observe_write(self, kind, data, *args, **kwargs):
        if kind == 'action_target':
            # A running session manifest proves this was not added by packaging.
            during_capture.append(json.loads((self.path/'manifest.json').read_text())['state'])
        return required(self, kind, data, *args, **kwargs)
    monkeypatch.setattr(Session, 'submit', state_only)
    monkeypatch.setattr(synthetic, 'synthetic_config', enabled_config)
    monkeypatch.setattr(Storage, 'required', observe_write)
    result = generate(tmp_path/'capture', raw_only=True)
    source, bundle = Path(result['session']), Path(result['path'])
    def targets(path):
        with (path/'raw.mcap').open('rb') as f:
            return [m.data for _, c, m in make_reader(f).iter_messages(log_time_order=False)
                    if '/action_target/' in c.topic]
    captured = targets(source)
    assert captured and targets(bundle) == captured
    assert set(during_capture) == {'recording'}
    assert any(json.loads(data)['valid'] for data in captured)
    manifest = json.loads((source/'manifest.json').read_text())
    assert manifest['action_fallback']['real_action_sources'] == []
    assert manifest['action_fallback']['selected_sources']
    assert not any(ident.endswith('action') for ident in manifest['source_counts'])


def test_disabled_by_default_and_strict_boolean_policy():
    from middleware.core.capture.config import validate
    from middleware.core.capture.synthetic import synthetic_config
    cfg = synthetic_config()
    assert not validate(cfg)['alignment'].get('derive_action_from_next_state',False)
    cfg['alignment']['derive_action_from_next_state'] = 'true'
    with pytest.raises(ValueError,match='derive_action_from_next_state'):
        validate(cfg)


def test_next_observation_never_crosses_clock_epoch_or_long_interruption():
    from middleware.core.capture.primary_timeline import next_observation
    before = dict(source_id='camera', clock_id='host', clock_epoch=0,
                  rgb_source_seq=49, capture_time_ns=1_000_000_000)
    after = dict(before, rgb_source_seq=51, capture_time_ns=1_066_666_666)
    assert next_observation(before, after, {})
    assert not next_observation(before, dict(after, clock_epoch=1), {})
    assert not next_observation(before, dict(after, capture_time_ns=2_100_000_000), {})
