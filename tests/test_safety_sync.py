import pytest
from types import SimpleNamespace
from unittest.mock import Mock
from middleware.core.app import MiddlewareRuntime


def snapshot(enabled, state='running', ownership=None):
    runtime = SimpleNamespace(
        ros=Mock(), vr=Mock(), _safety_reason='no VR pose data for 602 ms')
    ros_status = {'state': state, 'command_mux': {'output_enabled': enabled}}
    if ownership is not None:
        ros_status['ownership'] = ownership
    runtime.ros.status.return_value = ros_status
    MiddlewareRuntime._send_safety_snapshot(runtime)
    runtime.ros.set_command_output_enabled.assert_not_called()
    runtime.ros.publish.assert_not_called()
    return runtime


def test_repeats_authoritative_stop_for_lost_events():
    runtime = snapshot(False)
    event = runtime.vr.send_event.call_args.args[0]
    assert event['kind'] == 'safety_stop'
    assert event['payload']['reason'] == 'no VR pose data for 602 ms'


def test_normal_state_repairs_headset_latch_without_enabling_robot():
    runtime = snapshot(True)
    assert runtime.vr.send_event.call_args.args[0]['kind'] == 'safety_resume'


def test_unknown_or_stopped_bridge_does_not_clear_headset():
    for enabled, state in [(None, 'running'), (True, 'error'), (True, 'stopped')]:
        snapshot(enabled, state).vr.send_event.assert_not_called()


@pytest.mark.parametrize('ownership_state', ['TAKEOVER', 'RETURNING'])
def test_dagger_handover_does_not_emit_false_safety_stop(ownership_state):
    runtime = snapshot(False, ownership={'state': ownership_state, 'reason': 'manual request'})
    runtime.vr.send_event.assert_not_called()


@pytest.mark.parametrize('ownership_state,reason', [
    ('HOLD', 'VR controller B button pressed'),
    ('FAULT', 'teleop input timeout'),
    ('ESTOP', 'dashboard emergency stop'),
])
def test_real_ownership_stop_states_still_emit_safety_stop(ownership_state, reason):
    runtime = snapshot(False, ownership={'state': ownership_state, 'reason': reason})
    event = runtime.vr.send_event.call_args.args[0]
    assert event['kind'] == 'safety_stop'
    assert event['payload']['reason'] == reason
