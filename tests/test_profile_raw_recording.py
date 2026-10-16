from pathlib import Path
import time

import pytest
import yaml

from middleware.core.config import ConfigError, validate_config
from middleware.core.profile_capture import capture_config
from middleware.core.recording_service import RecordingService
from middleware.core.capture.adapters import preflight

PROFILE = Path(__file__).resolve().parents[1] / 'adapters/robots/openarmx'


def test_current_public_profile_generates_lossless_depth_and_measured_axes():
    cfg = capture_config(PROFILE)
    axes = cfg['capture']['profile']['axes']
    assert len(axes) == 16
    assert {a['name'] for a in axes} == {
        *[f'{side}_joint_{n}' for side in ('left', 'right') for n in range(1, 8)],
        'left_gripper_joint_1', 'right_gripper_joint_1'}
    assert {a['unit'] for a in axes if 'gripper' in a['name']} == {'m'}
    assert cfg['sources']['robot_state']['topic'] == '/observation/joint_states'
    assert cfg['sources']['robot_action']['topic'] == '/action/joint_states'
    assert cfg['storage']['retain_original_ros'] is False
    assert cfg['storage']['depth_video_encoding'] is False
    assert cfg['alignment']['derive_action_from_next_state'] is False


def test_test_mode_relaxes_only_missing_action_precheck():
    cfg = capture_config(PROFILE)
    reports = {'robot_state': {'registered_monotonic_ns': time.monotonic_ns()}}
    missing = lambda: {x['source_id'] for x in preflight(cfg, reports, None)['issues']}
    assert 'robot_action' in missing()
    cfg['alignment']['derive_action_from_next_state'] = True
    assert 'robot_action' not in missing()
    assert missing()  # Camera readiness still blocks capture.
    reports.clear()
    assert 'robot_state' in missing()  # Never relax actual state readiness.


def test_dashboard_flag_is_strict_and_cannot_silently_apply_to_hc():
    assert validate_config({})['ros']['recording']['derive_action_from_next_state'] is False
    for value in ('true', 1):
        with pytest.raises(ConfigError):
            validate_config({'ros': {'recording': {'derive_action_from_next_state': value}}})
    with pytest.raises(ConfigError, match='raw'):
        validate_config({'ros': {'recording': {'derive_action_from_next_state': True}}})


def test_toggle_off_recreates_idle_raw_hub_and_does_not_inherit_test_file(tmp_path):
    cfg = capture_config(PROFILE)
    cfg['alignment']['derive_action_from_next_state'] = True
    path = tmp_path / 'capture.yaml'
    path.write_text(yaml.safe_dump(cfg))
    recording = dict(workflow='platform_raw', capture_config_file=str(path),
        directory=str(tmp_path / 'data'), job_state_dir=str(tmp_path / 'jobs'),
        artifact_state_dir=str(tmp_path / 'ready'), derive_action_from_next_state=True)
    service = RecordingService(recording, tmp_path, {'enabled': False})
    try:
        old = service.raw_hub()
        assert old.cfg['alignment']['derive_action_from_next_state'] is True
        service.config['derive_action_from_next_state'] = False
        new = service.raw_hub()
        assert new is not old
        assert new.cfg['alignment']['derive_action_from_next_state'] is False
        assert yaml.safe_load(path.read_text())['alignment']['derive_action_from_next_state'] is True
    finally:
        service.shutdown()
