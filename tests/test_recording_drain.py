"""A real full writer queue must retain every accepted message on normal stop."""
import os
import signal
import threading
import time

from mcap.reader import make_reader

from middleware.core.topic_recorder import TopicRecorder


def test_full_queue_slow_writer_stop_keeps_all_accepted_messages(tmp_path):
    recorder = TopicRecorder({'directory': str(tmp_path), 'queue_capacity': 2}, tmp_path)
    recorder.start('full.mcap')
    process = recorder._process
    os.kill(process.pid, signal.SIGSTOP)
    result = []
    stopper = None
    try:
        # Wait for the actual child to stop, not an arbitrary disk-delay sleep.
        deadline = time.monotonic()+2
        while 'T' not in open(f'/proc/{process.pid}/stat').read().split()[2]:
            assert time.monotonic() < deadline
            time.sleep(.001)
        assert recorder.record({'topic': '/evidence', 'payload': {'seq': 1}})
        assert recorder.record({'topic': '/evidence', 'payload': {'seq': 2}})
        stopper = threading.Thread(target=lambda: result.append(recorder.stop()))
        stopper.start()
        assert recorder._finish_event.wait(2)
        assert stopper.is_alive()
        assert recorder.status()['accepted'] == 2
        assert recorder.status()['dropped'] == 0
    finally:
        os.kill(process.pid, signal.SIGCONT)
        if stopper:
            stopper.join(5)
        else:
            recorder.stop()
    assert not stopper.is_alive()
    assert result[0]['messages'] == 2 and result[0]['dropped'] == 0
    with recorder.path.open('rb') as stream:
        messages = list(make_reader(stream).iter_messages())
    assert [m.data for _, _, m in messages] == [b'{"seq":1}', b'{"seq":2}']


def test_ingress_distinguishes_message_limit_from_byte_limit(tmp_path):
    from middleware.core.topic_recorder import enqueue_record
    for name, config, expected in [
        ('count', dict(queue_capacity=1), 'queue_capacity_rejections'),
        ('bytes', dict(queue_bytes=1500), 'byte_budget_rejections')]:
        recorder = TopicRecorder(dict(directory=str(tmp_path), **config), tmp_path)
        recorder._accepting_event.set()  # Test ingress alone; no writer or ROS process.
        event = dict(topic='/evidence', payload={'sequence': 1})
        assert enqueue_record(recorder.ipc_resources(), event)
        assert not enqueue_record(recorder.ipc_resources(), event)
        status = recorder.status()
        assert status['ingress_diagnostics'][expected] == 1
        assert status['accepted'] == status['dropped'] == 1
        recorder._accepting_event.clear()
        recorder.shutdown()


def test_bulk_raw_events_use_separate_queue_and_drain_cleanly(tmp_path):
    from middleware.core.topic_recorder import enqueue_record
    recorder = TopicRecorder({'directory': str(tmp_path), 'queue_capacity': 16}, tmp_path)
    recorder.start('bulk.mcap')
    try:
        bulk_event = {
            'kind': 'ros_message',
            'topic': '/observation/cameras/head/color/compressed',
            'msg_type': 'sensor_msgs/msg/CompressedImage',
            '_raw': b'x' * 24000,
            'stamp_ns': time.time_ns(),
        }
        small_event = {'topic': '/evidence', 'payload': {'seq': 1}}
        assert enqueue_record(recorder.ipc_resources(), bulk_event)
        assert enqueue_record(recorder.ipc_resources(), small_event)
        result = recorder.stop()
        assert result['messages'] == 2 and result['dropped'] == 0
    finally:
        recorder.shutdown()


def test_enqueue_record_unlocks_before_queue_put_and_rolls_back_on_full(tmp_path):
    import multiprocessing as mp
    from middleware.core.topic_recorder import enqueue_record

    lock = threading.RLock()
    accepting = threading.Event()
    accepting.set()
    dropped = mp.Value('Q', 0)
    accepted = mp.Value('Q', 0)
    queued_bytes = mp.Value('Q', 0)
    stats = mp.Array('Q', 5, lock=False)
    lock_states = []

    class ProbeQueue:
        def __init__(self, *, full=False):
            self.full = full
            self.items = []

        def put_nowait(self, item):
            # Putting large IPC blobs must never hold the shared ingress lock.
            acquired = lock.acquire(blocking=False)
            if acquired:
                lock.release()
            lock_states.append(acquired)
            if self.full:
                import queue
                raise queue.Full
            self.items.append(item)

    ok_q = ProbeQueue(full=False)
    resources = (ok_q, accepting, dropped, lock, accepted, queued_bytes, 1024 * 1024, stats, None)
    assert enqueue_record(resources, {'topic': '/evidence', 'payload': {'seq': 1}})
    assert lock_states == [True]
    assert accepted.value == 1 and queued_bytes.value > 0

    full_q = ProbeQueue(full=True)
    full_resources = (full_q, accepting, dropped, lock, accepted, queued_bytes, 1024 * 1024, stats, None)
    before_bytes = queued_bytes.value
    assert not enqueue_record(full_resources, {'topic': '/evidence', 'payload': {'seq': 2}})
    assert accepted.value == 1
    assert queued_bytes.value == before_bytes
    assert dropped.value == 1 and stats[0] == 1
