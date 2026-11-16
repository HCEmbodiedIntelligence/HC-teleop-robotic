"""Diagnostic aggregation must not run under the actuator send lock."""
from types import SimpleNamespace
import time

from middleware.core.ros_bridge import RosBridge


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
