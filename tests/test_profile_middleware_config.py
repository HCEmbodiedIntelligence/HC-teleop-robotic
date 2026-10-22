"""Robot packages own portable settings; host configuration stays local."""
import copy
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

from middleware.core.config import (
    ConfigError, ConfigStore, ROBOT_MIDDLEWARE_DEFAULTS,
    validate_robot_middleware,
)
from middleware.core.ros_recording_executor import recording_subscriptions


def write_yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value))


@pytest.fixture
def deployment(tmp_path):
    config_path = tmp_path / 'middleware' / 'config.yaml'
    robot_path = tmp_path / 'robots' / 'a' / 'middleware.yaml'
    host = {
        'robot_profiles': {'root': '../robots', 'active': 'a'},
        'server': {'host': '127.0.0.1', 'port': 7987},
        'ros': {'domain_id': 168, 'recording': {'directory': '/tmp/local-recordings'}},
        'vr': {'pose_port': 5305},
        'camera': {'enabled': True, 'streams': [{'id': 'old', 'topic': '/old'}]},
    }
    write_yaml(config_path, host)
    write_yaml(robot_path, {'camera': {'enabled': True, 'streams': [{'id': 'head', 'topic': '/head/compressed'}]}})
    return ConfigStore(config_path), robot_path


def test_load_package_overrides_robot_fields_and_preserves_machine_settings(deployment):
    store, path = deployment
    config = store.load()
    assert config['camera']['streams'][0]['id'] == 'head'
    assert config['server'] == {'host': '127.0.0.1', 'port': 7987}
    assert config['ros']['domain_id'] == 168
    assert config['ros']['recording']['directory'] == '/tmp/local-recordings'
    assert config['vr']['pose_port'] == 5305
    assert config['vr']['pose_timeout_ms'] == 600
    assert config['safety']['stop_on_startup'] is True
    assert '/old' not in {s['topic'] for s in config['ros']['subscriptions']}
    assert path.exists()


def test_gui_changes_save_to_package_and_survive_reload(deployment):
    store, path = deployment
    config = store.load()
    config['camera']['streams'][0]['fps'] = 24
    config['vr']['pose_timeout_ms'] = 700
    config['safety']['stop_on_startup'] = False
    store.save(config)
    package = yaml.safe_load(path.read_text())
    assert package['camera']['streams'][0]['fps'] == 24
    assert package['vr'] == {'pose_timeout_ms': 700}
    host = yaml.safe_load(store.path.read_text())
    assert host['camera'] == {}
    assert 'subscriptions' not in host['ros']
    assert 'pose_timeout_ms' not in host['vr']
    assert host['ros']['domain_id'] == 168
    reloaded = ConfigStore(store.path).load()
    assert reloaded == store.value
    assert reloaded['safety']['stop_on_startup'] is False


def test_switching_profiles_cannot_copy_previous_camera_or_subscriptions(deployment):
    store, path = deployment
    config = store.load()
    target = path.parent.parent / 'b' / 'middleware.yaml'
    write_yaml(target, {})
    original = target.read_bytes()
    config['robot_profiles']['active'] = 'b'
    result = store.save(config)
    assert target.read_bytes() == original
    assert result['camera']['enabled'] is False
    assert result['camera']['streams'] == []
    assert {s['topic'] for s in result['ros']['subscriptions']} == {
        '/observation/joint_states', '/action/joint_states'}
    assert ConfigStore(store.path).load() == result


def test_reapply_same_profile_reads_package_without_writing(deployment):
    store, path = deployment
    config = store.load()
    write_yaml(path, {'vr': {'pose_timeout_ms': 900}})
    host_before = store.path.read_bytes()
    result = store.apply_profile(config)
    assert result['vr']['pose_timeout_ms'] == 900
    assert result['camera']['streams'] == []
    assert store.path.read_bytes() == host_before


@pytest.mark.parametrize('value', [
    {'server': {'port': 5000}}, {'robot_profiles': {'active': 'a'}},
    {'ros': {'domain_id': 22}}, {'ros': {'recording': {'directory': '/tmp'}}},
    {'vr': {'pose_port': 5500}}, {'camera': {'directory': '/tmp'}},
    {'safety': {'unknown': True}}, {'ros': {'command_mux': {'path': '/tmp'}}},
])
def test_package_rejects_machine_fields(value):
    with pytest.raises(ConfigError):
        validate_robot_middleware(value)


def test_package_validation_fills_missing_fields_without_mutating_defaults():
    original = copy.deepcopy(ROBOT_MIDDLEWARE_DEFAULTS)
    result = validate_robot_middleware({'camera': {'enabled': True}})
    assert result['camera']['streams'] == []
    assert result['vr'] == {'pose_timeout_ms': 600}
    assert result['ros']['command_mux']['output_topic'] == '/action/joint_states'
    assert ROBOT_MIDDLEWARE_DEFAULTS == original


def test_x1_records_camera_calibration_and_extrinsics_for_legacy_conversion():
    profile_path = Path(__file__).resolve().parents[1] / 'adapters/robots/x1/middleware.yaml'
    profile = yaml.safe_load(profile_path.read_text())
    subscriptions = {
        item['topic']: item for item in recording_subscriptions(profile['ros'])
    }

    for topic in (
        '/io_teleop/camera_head/calibration',
        '/io_teleop/camera_head/extrinsics',
    ):
        item = subscriptions[topic]
        assert item['type'] == 'std_msgs/msg/String'
        assert item['qos_reliability'] == 'reliable'
        assert item['max_hz'] == 0.0


def test_legacy_profile_without_middleware_file_retains_host_settings(deployment):
    store, path = deployment
    path.unlink()
    config = store.load()
    assert config['camera']['streams'][0]['id'] == 'old'
    config['vr']['pose_timeout_ms'] = 800
    store.save(config)
    assert ConfigStore(store.path).load() == config
    assert yaml.safe_load(store.path.read_text())['vr']['pose_timeout_ms'] == 800


def test_failed_second_replace_restores_first_file(deployment):
    store, path = deployment
    config = store.load()
    before = {p: p.read_bytes() for p in (path, store.path)}
    config['vr']['pose_timeout_ms'] = 850
    import os
    replace = os.replace
    count = 0

    def fail_host_once(source, target):
        nonlocal count
        count += 1
        if count == 2:
            raise OSError('simulated host write failure')
        return replace(source, target)

    with patch('middleware.core.config.os.replace', side_effect=fail_host_once):
        with pytest.raises(OSError, match='simulated'):
            store.save(config)
    assert {p: p.read_bytes() for p in before} == before
    assert store.value['vr']['pose_timeout_ms'] == 600
    assert not list(path.parent.glob('.middleware.yaml.*'))
