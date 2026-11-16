"""High-rate sensor events must not consume log capacity or rescan saved files."""
import threading
import time

from middleware.core.diagnostic_logs import DiagnosticLogs


def test_ignored_telemetry_leaves_capacity_for_actual_fault(tmp_path):
    service = DiagnosticLogs(tmp_path, queue_size=1)
    # Freeze the worker, as with a slow disk: ignored traffic cannot fill it.
    for _ in range(5000):
        assert service.emit({'kind': 'ros_message', 'payload': {'data': 'not a log'}})
    assert service._queue.empty()
    assert service.log('error', 'original control fault')
    assert service._dropped == 0
    service.start()
    service.close()
    entries = service.snapshot(priority=False)['entries']
    assert len(entries) == 1
    assert entries[0]['message'] == 'original control fault'


def test_actual_log_burst_does_not_scan_saved_files_per_event(tmp_path, monkeypatch):
    service = DiagnosticLogs(tmp_path, queue_size=256, poll_interval=.25)
    calls = []
    consumed = threading.Event()
    original = service._append

    def append(entry, **kwargs):
        original(entry, **kwargs)
        if entry and entry.get('message') == 'log 99':
            consumed.set()

    monkeypatch.setattr(service, '_append', append)
    monkeypatch.setattr(service, '_tail', lambda: calls.append(time.monotonic()))
    for i in range(100):
        assert service.log('info', f'log {i}')
    service.start()
    try:
        assert consumed.wait(3)
        assert len(service.snapshot(limit=200, priority=False)['entries']) == 100
    finally:
        service.close()
    # This is a cadence invariant, independent of the test machine's speed.
    assert all(b-a >= service.poll_interval*.95 for a, b in zip(calls, calls[1:]))
