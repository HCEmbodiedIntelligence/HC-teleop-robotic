"""Diagnostic aggregation must not run under the actuator send lock."""
from types import SimpleNamespace
import threading
import time

from middleware.core.control.ros_bridge import RosBridge


def test_sampled_command_diagnostics_leave_the_final_gate_and_targets_unchanged():
    bridge = RosBridge({}, lambda *args: None)
    bridge.receive_hardware_ready(True)
    bridge._command_output_enabled = True
    sent = []

    class UnlockedDiagnostics:
        def __enter__(self):
            assert not bridge._lock._is_owned(), 'diagnostic aggregation holds actuator lock'

        def __exit__(self, *_):
            pass

    bridge._callback_timing_lock = UnlockedDiagnostics()
    for i in range(17):
        sec, nanosec = divmod(time.time_ns(), 10**9)
        message = SimpleNamespace(name=['arm'], position=[i / 100],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))
        assert bridge.forward_live_command('vr', message, sent.append)
        assert sent[-1] is message
    assert len(sent) == 17
    assert bridge._command_stage_timings['vr:total']['samples'] == 3
    assert bridge._command_stage_timings['vr:commit_publish']['samples'] == 3
    bridge.set_command_output_enabled(False)
    before = len(sent)
    assert not bridge.forward_live_command('vr', message, sent.append)
    assert len(sent) == before


def test_publish_wait_is_separate_and_owner_sampling_never_opens_the_gate():
    bridge = RosBridge({}, lambda *args: None)
    bridge.receive_hardware_ready(True)
    bridge._command_output_enabled = True
    bridge._slow_command_stages.append(dict(stamp_ns=time.time_ns()))
    entered, release = threading.Event(), threading.Event()
    sec, nanosec = divmod(time.time_ns(), 10**9)
    message = SimpleNamespace(name=['arm'], position=[.1],
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))
    def publish(_):
        entered.set()
        assert release.wait(2)
    results = []
    thread = threading.Thread(name='blocked_fake_publish', target=lambda: results.append(
        bridge.forward_live_command('vr', message, publish)))
    thread.start()
    try:
        assert entered.wait(1)
        # The original final authority fence is still held during publication.
        assert not bridge._lock.acquire(blocking=False)
        sample = bridge._sample_control_contention()
        assert sample['owner_thread'] == 'blocked_fake_publish'
        assert sample['trigger'] == 'slow_command'
        assert bridge._sample_control_contention() is None  # At most 1 Hz.
        assert not release.wait(.02)
    finally:
        release.set()
        thread.join(2)
    assert results == [True]
    timings = bridge._command_stage_timings
    assert timings['vr:dds_publish']['last_ms'] >= 20
    assert timings['vr:dds_publish']['last_ms'] > timings['vr:send_lock_wait']['last_ms']
    assert timings['vr:hands:dds_publish']['samples'] == 1
