from types import SimpleNamespace as Obj
from unittest.mock import patch

from middleware.core.control.ros_bridge import RosBridge
from adapters.core.runtime_config import backend
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
    with patch('middleware.core.control.ros_bridge.time.monotonic', return_value=1.1):
        assert bridge.forward_live_command('motion', message(), sent.append)
        assert not bridge.forward_live_command('vr', message(), sent.append)
        assert not bridge.forward_live_command('motion', message(), sent.append)
    assert len(sent) == 1


def test_ready_timeout_and_stop_require_new_enable_and_new_target():
    bridge = make_bridge()
    with patch('middleware.core.control.ros_bridge.time.monotonic', return_value=2.):
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
    with patch('middleware.core.control.ros_bridge.time.monotonic', return_value=1.1):
        bridge.set_control_source('vr')
        assert not bridge.forward_live_command('motion', message(), lambda m: None)
        bridge.set_command_output_enabled(True)
        assert not bridge.forward_live_command('vr', message(), lambda m: None)


@pytest.mark.parametrize('stamp', [0, 99, 101])
def test_invalid_headers(stamp):
    bridge = make_bridge()
    with patch('middleware.core.control.ros_bridge.time.monotonic', return_value=1.1):
        assert not bridge.forward_live_command('motion', message(stamp), lambda m: None)
        assert not bridge.forward_live_command('motion', message(names=('j', 'j')), lambda m: None)


def test_profile_backend_override():
    assert backend({'control': {'backend': 'v23'}}) == 'v23'
    assert backend({'control': {'backend': 'v23'}}, 'motion') == 'motion'
    with pytest.raises(ValueError):
        backend({}, 'unknown')
