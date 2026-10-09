from types import SimpleNamespace
from unittest.mock import Mock
from middleware.core.app import MiddlewareRuntime


def snapshot(enabled, state='running'):
    runtime = SimpleNamespace(
        ros=Mock(), vr=Mock(), _safety_reason='no VR pose data for 602 ms')
    runtime.ros.status.return_value = {'state': state, 'command_mux': {'output_enabled': enabled}}
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
