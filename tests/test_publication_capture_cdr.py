"""Already-sent immutable CDR must not be copied or serialized a second time."""
import threading
from types import SimpleNamespace
from unittest.mock import patch

from middleware.core.recording.publication_capture import PublicationCapture, serialize_snapshot, SerializedSnapshot


def recorder():
    events = []
    return SimpleNamespace(record=lambda event: events.append(event) or True), events


def test_serialized_snapshot_returns_the_exact_same_immutable_bytes():
    payload = b'\x00\x01\x00\x00unchanged-original-cdr'
    assert serialize_snapshot(SerializedSnapshot(payload, 123)) is payload


def test_successful_cdr_bypasses_message_access_and_snapshot_copy():
    target, events = recorder()
    capture = PublicationCapture(target, pre_roll_s=0)
    payload = b'\x00\x01\x00\x00same-successful-publication'
    capture.begin_generation()
    try:
        with patch('middleware.core.recording.publication_capture.JointSnapshot.copy',
                   side_effect=AssertionError('raw capture read message fields')):
            capture.complete(capture.reserve(), object(), serialized_cdr=payload, publication_ns=123456789)
            capture.end_generation()
        assert len(events) == 1
        assert events[0]['_raw'] is payload
        assert events[0]['stamp_ns'] == 123456789
        assert capture.status()['captured'] == capture.status()['enqueued'] == 1
        assert capture.status()['failed'] == capture.status()['dropped'] == 0
    finally:
        capture.close()


def test_mutable_cdr_is_explicitly_failed_and_rejected_send_is_not_captured():
    target, events = recorder()
    capture = PublicationCapture(target, pre_roll_s=0)
    capture.begin_generation()
    try:
        capture.complete(capture.reserve())  # Publisher failed or authority rejected.
        capture.complete(capture.reserve(), serialized_cdr=bytearray(b'not-immutable'))
        capture.end_generation()
        assert events == []
        assert capture.status()['failed'] == 1
        assert 'immutable bytes' in capture.status()['error']
        assert capture.status()['captured'] == 0
    finally:
        capture.close()


def test_idle_cdr_history_and_promoted_ticket_use_real_publication_lower_bound():
    target, events = recorder()
    capture = PublicationCapture(target)
    old = capture.reserve()
    inside = capture.reserve()
    recent = b'original-inside-window'
    capture.complete(capture.reserve(), serialized_cdr=recent, publication_ns=100)
    assert events == []
    capture.begin_generation(window_start_ns=100)
    try:
        capture.complete(old, serialized_cdr=b'old-published-before-window', publication_ns=99)
        capture.complete(inside, serialized_cdr=b'late-completed-inside-window', publication_ns=101)
        capture.end_generation()
        assert [e['stamp_ns'] for e in events] == [100, 101]
        assert events[0]['_raw'] is recent
        assert capture.status()['outside_preroll'] == 1
        assert capture.status()['failed'] == 0
    finally:
        capture.close()


def test_stop_drains_reserved_successful_raw_send_and_does_not_admit_new_sends():
    target, events = recorder()
    capture = PublicationCapture(target, pre_roll_s=0)
    capture.begin_generation()
    ticket = capture.reserve()
    fence = capture.close_admission()
    drain = threading.Thread(target=capture.end_generation)
    drain.start()
    try:
        drain.join(.02)
        assert drain.is_alive()
        payload = b'actual-reserved-before-stop-payload'
        capture.complete(ticket, serialized_cdr=payload, publication_ns=fence[0]+1)
        capture.complete(capture.reserve(), serialized_cdr=b'outside-new-admission', publication_ns=fence[0]+2)
        drain.join(2)
        assert not drain.is_alive()
        assert len(events) == 1 and events[0]['_raw'] is payload
        assert capture.status()['pending_ingress'] == capture.status()['inflight'] == 0
    finally:
        capture.close()


def test_raw_recorder_rejection_is_counted_without_claiming_enqueued_evidence():
    capture = PublicationCapture(SimpleNamespace(record=lambda event:False), pre_roll_s=0)
    capture.begin_generation()
    try:
        capture.complete(capture.reserve(), serialized_cdr=b'actual-successful-send')
        capture.end_generation()
        assert capture.status()['captured'] == 1
        assert capture.status()['enqueued'] == 0
        assert capture.status()['failed'] == 1
        assert 'rejected by recorder ingress' in capture.status()['error']
    finally:
        capture.close()
