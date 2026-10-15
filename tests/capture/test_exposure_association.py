"""The grid selects images, but state must describe each measured exposure."""
import copy
import json
import pytest

from middleware.core.capture.alignment import Aligner
from middleware.core.capture.buffers import TimeCache
from middleware.core.capture.config import validate
from middleware.core.capture.models import Sample
from middleware.core.capture.holobrain_fields import source_timestamp

BASE = 1_000_000_000
GRID = BASE + 10_000_000


def example(*, reference="primary_exposure_midpoint", depth_offset=-500_000, missing=False):
    cfg = validate({
        "alignment": {"primary_camera": "wrist", "state_reference": reference},
        "sources": {
            # Deliberately before the image source: input order cannot select the state clock.
            "joints": {"kind": "state", "named_axes": [
                {"name": "axis", "unit": "rad", "max_age_ms": 100}]},
            "commands": {"kind": "action", "named_axes": [
                {"name": "axis", "unit": "rad", "max_age_ms": 100}]},
            "wrist": {"kind": "rgbd", "require_exposure_metadata": True},
        },
    })
    cache = TimeCache(cfg["buffers"])
    def add(ident, seq, stamp, **data):
        d = {"source_id": ident, "source_seq": seq, "capture_time_ns": stamp,
             "receive_time_ns": BASE + 30_000_000, "clock_epoch": 0, "clock_id": "ros",
             "clock_model_id": "ros_header_direct", "timestamp_quality": "ros_stamped",
             "timestamp_semantics": "read", "valid": True, **data}
        cache.add(Sample(ident, seq, stamp, d["receive_time_ns"], 0, 0, json.dumps(d).encode()), BASE)
    for i, (stamp, value) in enumerate([(BASE, 0.), (BASE+20_000_000, 2.)], 1):
        add("joints", i, stamp, joint_names=["axis"], payload={"position": [value]},
            source_epoch="driver-1")
    # The command between grid time and exposure time must be active at the exposure.
    for i, (stamp, value) in enumerate([(BASE, 1.), (GRID+250_000, 3.)], 1):
        add("commands", i, stamp, joint_names=["axis"], payload={"position": [value]},
            source_epoch="control-1", validity_known=True)
    def image(stamp):
        return {"capture_time_ns": stamp, "header_timestamp_ns": stamp,
                "exposure_midpoint_ns": stamp, "source_seq": 11, "clock_id": "camera-clock",
                "clock_epoch": 7, "timestamp_semantics": "exposure_midpoint",
                "timestamp_evidence": "synthetic sensor timestamp",
                "actual_exposure_us": 3900., "exposure_evidence": "synthetic actual exposure",
                "clock_uncertainty_ns": None, "grid_uncertainty_ns": None}
    rgb, depth = image(GRID+500_000), image(GRID+depth_offset)
    if missing:
        rgb.pop("timestamp_evidence")
        rgb.pop("actual_exposure_us")
    add("wrist", 11, rgb["capture_time_ns"], rgb=rgb, depth=depth, pair_seq=11,
        calibration_id="test-only", camera_config_id="test-only", clock_epoch=7)
    return cfg, cache


def row(cfg, cache):
    return Aligner(cfg, cache).row(0, GRID, GRID+80_000_000)


def test_state_and_command_use_primary_exposure_and_depth_has_its_own_brackets():
    cfg, cache = example()
    r = row(cfg, cache)
    assert r["valid"], r["invalid_reasons"]
    assert not r["strict_qualified"]  # No measured clock-error bounds.
    assert r["association_time_ns"] == GRID+500_000
    assert r["target_time_ns"] == GRID
    assert r["state"]["joints"]["components"]["values"] == pytest.approx([1.05])
    assert r["action"]["commands"]["components"]["values"] == [3.]
    views = r["images"]["wrist"]["state_at_exposure"]
    assert views["rgb"]["state"]["joints"]["components"]["values"] == pytest.approx([1.05])
    c = views["depth"]["state"]["joints"]["components"]
    assert c["values"] == pytest.approx([.95])
    assert c["brackets_ns"] == [[BASE, BASE+20_000_000]]
    assert c["interpolation"][0]["alpha"] == pytest.approx(.475)
    assert c["interpolation"][0]["left_seq"] == 1
    assert c["interpolation"][0]["right_seq"] == 2
    # A head camera must not override the configured reference.
    r["images"]["camera_hand"] = copy.deepcopy(r["images"]["wrist"])
    r["images"]["camera_hand"]["rgb"]["header_timestamp_ns"] = BASE
    assert source_timestamp(r, cfg) == GRID+500_000


def test_old_grid_configuration_keeps_old_numeric_values():
    cfg, cache = example(reference="grid")
    r = row(cfg, cache)
    assert r["state"]["joints"]["components"]["values"] == [1.]
    assert r["action"]["commands"]["components"]["values"] == [1.]
    assert "association_time_ns" not in r


def test_missing_evidence_cannot_fall_back_to_grid_or_driver_header():
    cfg, cache = example(missing=True)
    r = row(cfg, cache)
    assert not r["valid"]
    assert r["association_time_ns"] is None
    assert r["state"]["joints"]["components"]["values"] == [None]
    assert any("exposure_midpoint_evidence_missing" in e for e in r["invalid_reasons"])
    with pytest.raises(ValueError):
        source_timestamp(r, cfg)


def test_missing_joint_bracket_never_extrapolates():
    cfg, cache = example()
    cache.values["joints"].pop()
    r = row(cfg, cache)
    assert not r["valid"]
    assert r["state"]["joints"]["components"]["values"] == [None]
    assert not r["images"]["wrist"]["state_at_exposure"]["depth"]["valid"]


def test_real_rgb_depth_skew_and_repeat_still_fail_existing_one_ms_limits():
    cfg, cache = example(depth_offset=-1_499_000)
    r = row(cfg, cache)
    assert r["images"]["wrist"]["rgbd_midpoint_skew_ns"] == 1_999_000
    assert not r["valid"]
    assert "wrist:rgbd_midpoint_skew_exceeded" in r["invalid_reasons"]
    cfg, cache = example()
    aligner = Aligner(cfg, cache)
    assert aligner.row(0, GRID, GRID+80_000_000)["valid"]
    assert "wrist:rgb:repeated_source_frame" in aligner.row(1, GRID, GRID+120_000_000)["invalid_reasons"]



@pytest.mark.parametrize('skew,valid', [(1_949_000, True), (2_000_000, True), (2_000_001, False)])
def test_per_camera_rgbd_limit_is_inclusive_without_duplicate_grid_failure(skew, valid):
    from middleware.core.capture.camera_rules import depth_reference
    cfg, cache = example(depth_offset=500_000-skew)
    cfg['alignment']['camera_timing_reference'] = 'per_camera_rgb'
    cfg['sources']['wrist']['rgbd_max_midpoint_skew_ms'] = 2
    r = row(validate(cfg), cache)
    assert r['valid'] is valid, r['invalid_reasons']
    assert r['images']['wrist']['rgbd_limit_ns'] == 2_000_000
    assert r['camera_pair_time_basis'] == 'rgb_midpoints'
    assert r['camera_pair_max_bound_ns'] == 0  # No second camera.
    assert not any('grid_error_exceeded' in error for error in r['invalid_reasons'])
    assert depth_reference(r, 'wrist', cfg) == (GRID+500_000, 2_000_000)
    assert r['images']['wrist']['state_at_exposure']['depth']['time_ns'] == GRID+500_000-skew


def test_other_camera_and_cross_rgb_limits_are_not_relaxed():
    cfg, cache = example(depth_offset=-1_000_001)
    cfg['alignment']['camera_timing_reference'] = 'per_camera_rgb'
    r = row(cfg, cache)
    assert 'wrist:rgbd_midpoint_skew_exceeded' in r['invalid_reasons']
    cfg, cache = example()
    cfg['alignment']['camera_timing_reference'] = 'per_camera_rgb'
    cfg['sources']['wrist']['rgbd_max_midpoint_skew_ms'] = 2
    cfg['sources']['front'] = {**cfg['sources']['wrist'], 'camera_max_error_ms': 3,
                               'rgbd_max_midpoint_skew_ms': 1}
    original = cache.values['wrist'][0][2]
    d = original.data()
    d['source_id'] = 'front'
    for name in ('rgb', 'depth'):
        for key in ('capture_time_ns', 'exposure_midpoint_ns', 'header_timestamp_ns'):
            d[name][key] += 1_100_000
    d['capture_time_ns'] += 1_100_000
    cache.add(Sample('front', d['source_seq'], d['capture_time_ns'], d['receive_time_ns'],
                     0, 0, json.dumps(d).encode()), BASE)
    r = row(validate(cfg), cache)
    assert 'camera_pair_skew_exceeded' in r['invalid_reasons']
    assert r['camera_pair_max_bound_ns'] == 1_100_000
    assert not any('grid_error_exceeded' in error for error in r['invalid_reasons'])


def test_per_camera_repeat_rule_overrides_global_rule():
    cfg, cache = example()
    cfg['alignment']['camera_timing_reference'] = 'per_camera_rgb'
    cfg['sources']['wrist']['allow_camera_repeat'] = True
    aligner = Aligner(cfg, cache)
    assert aligner.row(0, GRID, GRID+80_000_000)['valid']
    repeated = aligner.row(0, GRID, GRID+80_000_000)
    assert repeated['valid']
    assert repeated['images']['wrist']['rgb']['repeated']
    cfg['alignment']['allow_camera_repeat'] = True
    cfg['sources']['wrist']['allow_camera_repeat'] = False
    aligner = Aligner(cfg, cache)
    assert aligner.row(0, GRID, GRID+80_000_000)['valid']
    assert 'wrist:rgb:repeated_source_frame' in aligner.row(0, GRID, GRID+80_000_000)['invalid_reasons']


@pytest.mark.parametrize('key,value', [('rgbd_max_midpoint_skew_ms', float('nan')),
    ('rgbd_max_midpoint_skew_ms', -1), ('camera_max_error_ms', True),
    ('allow_camera_repeat', 'false')])
def test_invalid_per_camera_rules_are_rejected(key, value):
    cfg, _ = example()
    cfg['sources']['wrist'][key] = value
    with pytest.raises(ValueError):
        validate(cfg)
