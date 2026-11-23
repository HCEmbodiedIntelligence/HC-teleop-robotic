"""Status publication cannot retain authority while a ROS clock is blocked."""
import json
import threading
import time
from unittest.mock import Mock

from middleware.core.control.control_ownership import ControlOwnership
from test_control_status_isolation import input_frame, setup_callbacks
from test_legacy_control import Rig


def test_snapshot_default_reads_real_clock_and_explicit_wall_is_observational():
    clock = Mock(side_effect=[123, 789])
    control = ControlOwnership(ros_clock=clock)
    before = (control.token, control.state, control.phase, control.event_seq)
    assert control.snapshot()['published_stamp_ns'] == 123
    assert control.snapshot(wall_ns=456)['published_stamp_ns'] == 456
    assert control.snapshot()['published_stamp_ns'] == 789
    assert clock.call_count == 2
    assert (control.token, control.state, control.phase, control.event_seq) == before


def test_status_timer_blocked_clock_does_not_hold_authority_or_pause_input():
    rig = Rig()
    callbacks, timers, messages, _ = setup_callbacks(rig)
    entered, release, input_done = (threading.Event() for _ in range(3))
    held, errors = [], []
    original_clock = rig.control.ros_clock

    def slow_clock():
        if threading.current_thread() is status_worker:
            held.append(rig.bridge._lock._is_owned())
            entered.set()
            assert release.wait(1.5), 'status clock test release timed out'
        return original_clock()

    rig.control.ros_clock = slow_clock

    def status():
        try:
            timers['ownership_status']()
        except BaseException as error:
            errors.append(error)

    def receive_input():
        try:
            callbacks['/events/teleop/clutch'](input_frame(2))
            callbacks['/events/teleop/clutch'](input_frame(3, pause_requested=True))
        except BaseException as error:
            errors.append(error)
        finally:
            input_done.set()

    status_worker = threading.Thread(target=status)
    input_worker = threading.Thread(target=receive_input)
    status_worker.start()
    try:
        assert entered.wait(.5)
        input_worker.start()
        assert input_done.wait(.2), 'blocked status clock stalls real semantic-input callback'
        assert held == [False]
        assert rig.control.input_diagnostics['last_sequence'] == 3
        assert rig.control.state == 'HOLD'
        assert not rig.bridge._command_output_enabled
        assert status_worker.is_alive(), 'observational clock must remain blocked'
    finally:
        release.set()
        status_worker.join(2)
        if input_worker.ident is not None:
            input_worker.join(2)
    assert not errors
    assert not status_worker.is_alive() and not input_worker.is_alive()
    assert rig.control.input_timeout_ns == 300_000_000
    states = [json.loads(data) for topic, data in messages
              if topic == '/status/control/ownership']
    assert states[-1]['state'] == 'HOLD'
    assert isinstance(states[-1]['published_stamp_ns'], int)
