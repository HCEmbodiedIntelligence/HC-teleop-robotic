"""A full writer queue must retain every accepted message on normal stop."""
import os
import signal
import threading
import time

from mcap.reader import make_reader

from middleware.core.topic_recorder import TopicRecorder


def test_full_queue_slow_writer_stop_keeps_all_accepted_messages(tmp_path):
    recorder = TopicRecorder(
        {"directory": str(tmp_path), "queue_capacity": 2}, tmp_path
    )
    recorder.start("full.mcap")
    process = recorder._process
    os.kill(process.pid, signal.SIGSTOP)
    result = []
    stopper = None
    try:
        deadline = time.monotonic() + 2
        while "T" not in open(f"/proc/{process.pid}/stat").read().split()[2]:
            assert time.monotonic() < deadline
            time.sleep(0.001)
        assert recorder.record({"topic": "/evidence", "payload": {"seq": 1}})
        assert recorder.record({"topic": "/evidence", "payload": {"seq": 2}})
        stopper = threading.Thread(target=lambda: result.append(recorder.stop()))
        stopper.start()
        assert recorder._finish_event.wait(2)
        assert stopper.is_alive()
        assert recorder.status()["accepted"] == 2
        assert recorder.status()["dropped"] == 0
    finally:
        os.kill(process.pid, signal.SIGCONT)
        if stopper:
            stopper.join(5)
        else:
            recorder.stop()
    assert not stopper.is_alive()
    assert result[0]["messages"] == 2 and result[0]["dropped"] == 0
    with recorder.path.open("rb") as stream:
        messages = list(make_reader(stream).iter_messages())
    assert [message.data for _, _, message in messages] == [
        b'{"seq":1}',
        b'{"seq":2}',
    ]
