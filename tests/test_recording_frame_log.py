import json
import struct
import threading
import time

from middleware.core.recording_frame_log import RecordingFrameLog


def raw(ns):
    return b'\x00\x01\x00\x00'+struct.pack('<iI', *divmod(ns, 10**9))+b'pixels'*100


def rows(log):
    log.close()
    assert not log.status()['error']
    return [json.loads(line) for line in log.path.read_text().splitlines()]


def test_intervals_use_callback_clocks_and_separate_physical_sources(tmp_path):
    log = RecordingFrameLog('dds_small', directory=tmp_path)
    for source, header, mono in [('/public', 10**9, 10**9), ('/private', 2*10**9, 2*10**9),
                                 ('/public', 1_010_000_000, 1_060_000_000),
                                 ('/public', 1_010_000_000, 1_065_000_000)]:
        log.offer('/logical', source, 'sensor_msgs/msg/JointState', raw(header), header+70_000_000,
                  mono, active=True)
    a,b,c,d = rows(log)
    assert a['receive_interval_ms'] is b['receive_interval_ms'] is None
    assert c['receive_interval_ms'] == 60 and c['source_interval_ms'] == 10
    assert c['interval_gt_40ms'] and not c['source_gap_gt_40ms']
    assert c['source_age_ms'] == 70
    assert d['source_interval_ms'] == 0
    assert c['ingress_accepted'] is None  # observation is not proof of MCAP admission


def test_action_uses_actual_publication_time_and_generation_not_worker_time(tmp_path):
    log = RecordingFrameLog('published_action', directory=tmp_path)
    for generation, wall, mono in [(1, 10**9, 10**9), (1, 1_045_000_000, 1_000_000_001),
                                    (2, 1_000_000_000, 3*10**9)]:
        log.offer('/action/joint_states', '/action/joint_states', 'sensor_msgs/msg/JointState',
                  raw(wall-10_000_000), wall, mono, active=True, generation=generation, accepted=True)
    a,b,c = rows(log)
    assert b['publication_interval_ms'] == 45 and b['interval_gt_40ms']
    assert b['publication_ns'] == 1_045_000_000
    assert b['receive_interval_ms'] is None
    assert c['publication_interval_ms'] is None


def test_slow_sink_and_queue_overflow_do_not_wait_on_producer(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = RecordingFrameLog._run
    def slow(self):
        entered.set()
        release.wait(2)
        original(self)
    monkeypatch.setattr(RecordingFrameLog, '_run', slow)
    log = RecordingFrameLog('dds_images', directory=tmp_path, capacity=2)
    assert entered.wait(1)
    started = time.monotonic()
    for index in range(100):
        log.offer('/color', '/color', 'sensor_msgs/msg/CompressedImage', raw(10**9), index, index, active=True)
    assert time.monotonic()-started < .1
    assert log.status()['dropped'] == 98
    assert len(log.queue.queue[0][4]) == 12  # never retain pixels
    release.set()
    assert len(rows(log)) == 2


def test_rotation_is_bounded_and_io_error_is_visible(tmp_path):
    log = RecordingFrameLog('dds_small', directory=tmp_path, max_bytes=100)
    for index in range(10):
        log.offer('/x', '/x', 'sensor_msgs/msg/JointState', raw(10**9), index, index, active=False)
    log.close()
    assert len(list(tmp_path.glob('*.jsonl*'))) <= 3
    invalid = tmp_path/'file'
    invalid.write_text('not a directory')
    log = RecordingFrameLog('dds_small', directory=invalid)
    log.close()
    assert log.status()['error']
