from types import SimpleNamespace as Obj
from unittest.mock import patch

from middleware.core.ros_bridge import RosBridge
from adapters.core.runtime_config import backend
from adapters.nodes.motion_teleop_bridge import MotionCache, profile_settings
from pathlib import Path
import pytest


def message(stamp=100.1, names=('j',)):
    return Obj(name=list(names), position=[.5] * len(names), header=Obj(stamp=Obj(sec=int(stamp), nanosec=round((stamp % 1) * 1e9))))


def make_bridge():
    bridge = RosBridge({'enabled': True, 'command_mux': {'source': 'motion'}}, lambda *a: None)
    bridge._ros_now = lambda: 100.1
    bridge.receive_hardware_ready(True, 1.)
    return bridge


def test_move_without_vr_grip_and_source_exclusion():
    bridge = make_bridge()
    sent = []
    with patch('middleware.core.ros_bridge.time.monotonic', return_value=1.1):
        assert bridge.forward_live_command('motion', message(), sent.append)
        assert not bridge.forward_live_command('vr', message(), sent.append)
        assert not bridge.forward_live_command('motion', message(), sent.append)
    assert len(sent) == 1


def test_ready_timeout_and_stop_require_new_enable_and_new_target():
    bridge = make_bridge()
    with patch('middleware.core.ros_bridge.time.monotonic', return_value=2.):
        assert not bridge.forward_live_command('motion', message(), lambda m: None)
        bridge.receive_hardware_ready(True)
        assert not bridge.forward_live_command('motion', message(), lambda m: None)
        bridge.set_command_output_enabled(True)
        assert not bridge.forward_live_command('motion', message(), lambda m: None)
        bridge._ros_now = lambda: 100.2
        assert bridge.forward_live_command('motion', message(100.2), lambda m: None)
        bridge.emergency_stop('/control/emergency_stop', 'test')
        assert not bridge.forward_live_command('motion', message(100.21), lambda m: None)


def test_source_switch_rejects_previous_and_queued_target():
    bridge = make_bridge()
    with patch('middleware.core.ros_bridge.time.monotonic', return_value=1.1):
        bridge.set_control_source('vr')
        assert not bridge.forward_live_command('motion', message(), lambda m: None)
        bridge.set_command_output_enabled(True)
        assert not bridge.forward_live_command('vr', message(), lambda m: None)


@pytest.mark.parametrize('stamp', [0, 99, 101])
def test_invalid_headers(stamp):
    bridge = make_bridge()
    with patch('middleware.core.ros_bridge.time.monotonic', return_value=1.1):
        assert not bridge.forward_live_command('motion', message(stamp), lambda m: None)
        assert not bridge.forward_live_command('motion', message(names=('j', 'j')), lambda m: None)


def test_profile_backend_override_and_single_arm_cache():
    assert backend({'control': {'backend': 'v23'}}) == 'v23'
    assert backend({'control': {'backend': 'v23'}}, 'motion') == 'motion'
    with pytest.raises(ValueError):
        backend({}, 'unknown')
    cache = MotionCache({'arm': ('axis_1', 'axis_2')})
    cache.feedback_received(['axis_1', 'axis_2'], [0., 0.], 100., 1., 100.)
    cache.hardware, cache.hardware_time = True, 1.
    cache.update_status({'enabled': True, 'arms': {'arm': {'active': True}}}, 1., 100.)
    assert cache.grant('arm', 1.01, 100.01)
    cache.accept_result(['axis_1', 'axis_2'], [.1, .2], 100.02, 1.02, 100.02)
    assert cache.output(1.03, 100.03) == {'axis_1': .1, 'axis_2': .2}


def test_one_arm_feedback_does_not_require_other_arm():
    cache = MotionCache({'left': ('l',), 'right': ('r',)})
    cache.feedback_received(['l'], [0.], 100., 1., 100.)
    cache.hardware, cache.hardware_time = True, 1.
    cache.update_status({'enabled': True, 'arms': {'left': {'active': True}, 'right': {'active': True}}}, 1., 100.)
    assert cache.grant('left', 1.01, 100.01)
    assert not cache.grant('right', 1.01, 100.01)


@pytest.mark.parametrize('name', ['x1', 'openarmx'])
def test_profile_channels_resolve(name):
    root = Path(__file__).resolve().parents[1]
    names, frame, endpoints, control = profile_settings(root / 'adapters/robots' / name / 'arm_teleop.yaml')
    assert len(names) == 2
    assert frame and len(endpoints) == 4
