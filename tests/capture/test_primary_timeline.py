"""Camera phase, synchronization requirements and missing frames are distinct."""
from copy import deepcopy
import json
import pytest

from middleware.core.capture.alignment import Aligner
from middleware.core.capture.models import Sample
from middleware.core.capture.config import validate
from test_exposure_association import example, BASE, GRID


def cameras(skew=6_000_000):
    cfg, cache = example()
    cfg['alignment'].update(timeline_mode='primary_camera', camera_timing_reference='per_camera_rgb',
                            camera_pair_max_skew_ms=2, require_camera_sync=False)
    cfg['sources']['wrist']['rgbd_max_midpoint_skew_ms'] = 2
    cfg['sources']['head'] = {**cfg['sources']['wrist'], 'depth_enabled': False}
    # A secondary camera comes first in config; it must not choose the state clock.
    cfg['sources'] = {'head': cfg['sources'].pop('head'), **cfg['sources']}
    original = cache.values['wrist'][0][2]
    body = original.data()
    body['source_id'] = 'head'
    body['capture_time_ns'] += skew
    body['rgb'] = {**body['rgb'], 'capture_time_ns': body['capture_time_ns'],
                   'exposure_midpoint_ns': body['capture_time_ns']}
    cache.add(Sample('head', 11, body['capture_time_ns'], original.receive_time_ns,
                     original.arrival_time_ns, 7, json.dumps(body).encode()), BASE)
    return validate(cfg), cache


def test_actual_primary_exposure_ignores_record_button_phase_and_interpolates_state():
    cfg, cache = cameras()
    stamp = GRID + 500_000
    row = Aligner(cfg, cache).row(0, BASE, stamp+80_000_000, target_ns=stamp)
    assert row['valid'], row['invalid_reasons']
    assert row['state']['joints']['components']['values'] == pytest.approx([1.05])
    assert row['images']['head']['rgb']['time_error_ns'] == 6_000_000
    assert row['timing_warnings'] == ['camera_pair_skew_exceeded']
    assert not any('grid_error' in reason for reason in row['invalid_reasons'])


def test_strict_sync_is_optional_and_rejects_the_same_six_ms_difference():
    cfg, cache = cameras()
    cfg['alignment']['require_camera_sync'] = True
    row = Aligner(cfg, cache).row(0, BASE, GRID+80_000_000, target_ns=GRID+500_000)
    assert row['invalid_reasons'] == ['camera_pair_skew_exceeded']


def test_missing_secondary_is_not_replaced_by_an_image_beyond_half_a_frame():
    cfg, cache = cameras(18_000_000)
    row = Aligner(cfg, cache).row(0, BASE, GRID+80_000_000, target_ns=GRID+500_000)
    assert 'head:camera_missing' in row['invalid_reasons']
    assert 'head' not in row['images']


@pytest.mark.parametrize('key,value', [('require_camera_sync','false'),
    ('camera_match_max_skew_ms',20), ('timeline_mode','unknown')])
def test_invalid_policy_values_are_rejected(key,value):
    cfg, _ = cameras()
    cfg['alignment'][key] = value
    with pytest.raises(ValueError):
        validate(cfg)
