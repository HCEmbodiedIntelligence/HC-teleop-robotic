"""Publication capture barriers without ROS, network, hardware or busy loops."""
import threading
import time
from types import SimpleNamespace

import pytest

from middleware.core.publication_capture import PublicationCapture


def message(value):
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=123, nanosec=value), frame_id='original'),
        name=['arm'], position=[value], velocity=[], effort=[])


def serializer(snapshot):
    return repr((snapshot.sec, snapshot.nanosec, snapshot.frame_id,
                 snapshot.name, snapshot.position)).encode()


class Recorder:
    def __init__(self):
        self.events = []
        self.file = 1

    def record(self, event):
        self.events.append((self.file, event))
        return True


def test_all_64_publications_have_original_fields_and_stamps():
    recorder = Recorder()
    capture = PublicationCapture(recorder, serializer=serializer, pre_roll_s=0)
    capture.begin_generation()
    try:
        for index in range(64):
            ticket = capture.reserve()
            msg = message(index)
            capture.complete(ticket, msg, publication_ns=1000+index)
            msg.position[0] = -1  # The queued snapshot has its own immutable data.
        capture.end_generation()
        assert len(recorder.events) == 64
        for index, (_, event) in enumerate(recorder.events):
            assert event['stamp_ns'] == 1000+index
            assert event['topic'] == '/action/joint_states'
            assert str((index,)).encode() in event['_raw']
        assert capture.status()['enqueued'] == 64
    finally:
        capture.close()


def test_stalled_writer_does_not_block_reserve_complete_or_revoke():
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    recorder = Recorder()
    original = recorder.record
    def blocked(event):
        entered.set()
        assert release.wait(3)
        return original(event)
    recorder.record = blocked
    capture = PublicationCapture(recorder, serializer=serializer, pre_roll_s=0)
    capture.begin_generation()
    capture.complete(capture.reserve(), message(1))
    assert entered.wait(1)
    # These execute while the worker is still blocked: no recorder lock can
    # enter the command thread, including the rejected/revoked send path.
    capture.complete(capture.reserve(), message(2))
    capture.complete(capture.reserve(), None)
    def stop():
        capture.end_generation()
        stopped.set()
    thread = threading.Thread(target=stop)
    thread.start()
    assert not stopped.wait(.02)
    release.set()
    assert stopped.wait(1)
    thread.join(1)
    assert len(recorder.events) == 2
    capture.close()


def test_stop_fences_reserved_send_and_prevents_old_generation_leakage():
    recorder = Recorder()
    capture = PublicationCapture(recorder, serializer=serializer, pre_roll_s=0)
    capture.begin_generation()
    ticket = capture.reserve()
    drained = threading.Event()
    def stop():
        capture.end_generation()
        drained.set()
    thread = threading.Thread(target=stop)
    thread.start()
    assert not drained.wait(.02)
    capture.complete(ticket, message(10))
    assert drained.wait(1)
    thread.join(1)
    recorder.file = 2
    capture.begin_generation()
    capture.complete(capture.reserve(), message(20))
    capture.end_generation()
    assert [file for file, _ in recorder.events] == [1, 2]
    capture.close()


def test_start_accounts_for_send_reserved_while_idle():
    recorder = Recorder()
    capture = PublicationCapture(recorder, serializer=serializer, pre_roll_s=0)
    ticket = capture.reserve()
    capture.begin_generation()
    capture.complete(ticket, message(1))
    capture.end_generation()
    assert len(recorder.events) == 1
    capture.close()


def test_queue_overflow_is_visible_without_overwriting():
    entered, release = threading.Event(), threading.Event()
    recorder = Recorder()
    original = recorder.record
    def blocked(event):
        entered.set()
        assert release.wait(3)
        return original(event)
    recorder.record = blocked
    capture = PublicationCapture(recorder, serializer=serializer, max_queue=1, pre_roll_s=0)
    capture.begin_generation()
    capture.complete(capture.reserve(), message(1))
    assert entered.wait(1)
    capture.complete(capture.reserve(), message(2))
    capture.complete(capture.reserve(), message(3))
    assert capture.status()['dropped'] == 1
    assert 'outbox full' in capture.status()['error']
    release.set()
    capture.end_generation()
    assert len(recorder.events) == 2
    capture.close()


def test_idle_history_uses_genuine_publications_without_serializing():
    recorder = Recorder()
    calls = []
    def serial(snapshot):
        calls.append(snapshot)
        return serializer(snapshot)
    capture = PublicationCapture(recorder, serializer=serial)
    capture.complete(capture.reserve(), message(1), publication_ns=11)
    capture.complete(capture.reserve(), None)
    assert calls == []
    capture.begin_generation()
    capture.end_generation()
    assert len(calls) == 1
    assert recorder.events[0][1]['stamp_ns'] == 11
    capture.close()


def test_delayed_out_of_order_completion_cannot_extend_preroll_age():
    recorder = Recorder()
    capture = PublicationCapture(recorder, serializer=serializer)
    capture.complete(capture.reserve(), message(1), publication_ns=1,
                     publication_monotonic=time.monotonic())
    capture.complete(capture.reserve(), message(2), publication_ns=2,
                     publication_monotonic=time.monotonic()-1)
    capture.begin_generation()
    capture.end_generation()
    assert [event['stamp_ns'] for _, event in recorder.events] == [1]
    capture.close()


def test_selection_off_and_recording_rejection_are_explicit():
    recorder = Recorder()
    capture = PublicationCapture(recorder, serializer=serializer, selected=lambda: False)
    capture.begin_generation()
    capture.complete(capture.reserve(), message(1))
    capture.end_generation()
    assert recorder.events == []
    capture.close()
    recorder.record = lambda event: False
    capture = PublicationCapture(recorder, serializer=serializer, pre_roll_s=0)
    capture.begin_generation()
    capture.complete(capture.reserve(), message(2))
    capture.end_generation()
    assert capture.status()['failed'] == 1
    assert 'rejected by recorder ingress' in capture.status()['error']
    capture.close()
