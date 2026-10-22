"""Cross-branch contracts: explicit control mode, portable profiles and raw depth."""
import hashlib
import io
import json
from pathlib import Path
import zipfile

import pytest
import yaml

from adapters.core.runtime_config import unified_profile
from middleware.core.app import MiddlewareRuntime
from middleware.core.config import ConfigStore, validate_config
from middleware.core.robot_profiles import RobotProfileManager

ROOT = Path(__file__).resolve().parents[1]


def test_startup_profile_override_loads_its_camera_and_recording_settings(tmp_path, monkeypatch):
    profiles = tmp_path/'profiles'
    selected = profiles/'openarmx'
    selected.mkdir(parents=True)
    settings = (ROOT/'adapters/robots/openarmx/middleware.yaml').read_bytes()
    (selected/'middleware.yaml').write_bytes(settings)
    host = tmp_path/'config.yaml'
    original = yaml.safe_dump({'robot_profiles': {'root': '/unused', 'active': 'x1'}})
    host.write_text(original)
    monkeypatch.setenv('HC_ROBOT_NAME', 'openarmx')
    monkeypatch.setenv('HC_ROBOT_CONFIG_ROOT', str(profiles))
    config = ConfigStore(host).load()
    assert config['robot_profiles'] == {'root': str(profiles), 'active': 'openarmx'}
    expected = yaml.safe_load(settings)
    assert config['camera'] == expected['camera']
    assert config['ros']['subscriptions'] == validate_config({'ros': expected['ros']})['ros']['subscriptions']
    assert host.read_text() == original


@pytest.mark.parametrize('robot', ['x1', 'openarmx'])
def test_legacy_default_and_explicit_motion_use_matching_recording_names(robot, monkeypatch):
    monkeypatch.delenv('HC_CONTROL_BACKEND', raising=False)
    monkeypatch.delenv('HC_ROBOT_NAME', raising=False)
    monkeypatch.delenv('HC_ROBOT_CONFIG_ROOT', raising=False)
    config = validate_config({'robot_profiles': {'root': str(ROOT/'adapters/robots'), 'active': robot}})
    profile = ROOT/'adapters/robots'/robot
    runtime = MiddlewareRuntime(config, ROOT/'middleware')
    assert not unified_profile(profile)
    legacy = runtime._load_session_profile_metadata()
    assert 'robot_api_schema' not in legacy
    monkeypatch.setenv('HC_CONTROL_BACKEND', 'motion')
    assert unified_profile(profile)
    public = runtime._load_session_profile_metadata()
    assert public['robot_api_schema'] == 'hc-robot-api/v1'
    assert 'left_joint_1' in json.loads(public['free_joints'])
    assert public['free_joints'] != legacy['free_joints']


def test_unified_profile_round_trip_keeps_main_middleware_settings(tmp_path):
    source = RobotProfileManager(ROOT/'adapters/robots')
    payload = source.export_archive('x1')
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        assert 'robot_api.yaml' in archive.namelist()
        assert 'middleware.yaml' in archive.namelist()
        assert not any(name.endswith('.zip') for name in archive.namelist())
    target = RobotProfileManager(tmp_path/'profiles')
    profile = target.import_archive('x1_copy', 'X1 copy', payload)
    assert profile['unified_api']['available']
    assert (tmp_path/'profiles/x1_copy/middleware.yaml').read_bytes() == (ROOT/'adapters/robots/x1/middleware.yaml').read_bytes()
    store = ConfigStore(tmp_path/'config.yaml')
    config = store.apply_profile({'robot_profiles': {'root': str(target.root), 'active': 'x1_copy'}})
    assert config['vr']['pose_timeout_ms'] == 600
    assert config['safety']['stop_on_startup'] is True


def test_video_bundle_preserves_depth_bytes_stamps_and_source(tmp_path):
    pytest.importorskip('av')
    from mcap.writer import Writer
    from mcap.reader import make_reader
    from middleware.core.video_bundle import prepare, VERSION
    source = tmp_path/'source.mcap'
    original_depth = []
    with source.open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        schema = writer.register_schema('sensor_msgs/msg/Image', 'jsonschema', b'{}')
        rgb = writer.register_channel('/camera/color', 'json', schema)
        depths = [writer.register_channel('/camera/depth/'+kind, 'json', schema) for kind in ('16UC1','32FC1')]
        for index in range(3):
            stamp = 1_000_000_000 + index*100_000_000
            image = {'width': 4, 'height': 4, 'encoding': 'rgb8', 'step': 12, 'data': [index*60, 30, 100]*16}
            writer.add_message(rgb, stamp, json.dumps(image).encode(), stamp-100, index)
            for channel, encoding, size in zip(depths, ('16UC1','32FC1'), (2,4)):
                payload = json.dumps({'width':4, 'height':4, 'encoding':encoding, 'step':4*size,
                                      'data':list(range(16*size))}).encode()
                writer.add_message(channel, stamp, payload, stamp-100, index)
                original_depth.append((f'/camera/depth/{encoding}', stamp, stamp-100, index, payload))
        writer.finish()
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    target = prepare(source, tmp_path/'bundle', {'recording_id':'depth-check', 'started_at':'2026-10-22T00:00:00Z',
                                               'ended_at':'2026-10-22T00:00:01Z', 'video_fps':10})
    with (target/'raw.mcap').open('rb') as stream:
        messages = list(make_reader(stream, validate_crcs=True).iter_messages(log_time_order=False))
    depth = [(c.topic,m.log_time,m.publish_time,m.sequence,m.data) for _,c,m in messages if '/depth/' in c.topic]
    assert depth == original_depth
    assert all(c.message_encoding == VERSION for _,c,_ in messages if c.topic == '/camera/color')
    assert list((target/'videos').glob('*.mp4'))
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
