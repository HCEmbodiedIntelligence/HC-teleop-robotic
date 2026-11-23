"""Source-clock recording windows preserve evidence across transport delay."""
import multiprocessing as mp
import struct

from middleware.core.recording.capture_window import (
    CaptureAudit, CaptureWindow, SensorPreroll, buffered_sensor, header_stamp_ns,
)


def event(stamp, *, received=None, topic='/observation/cameras/head/color/compressed'):
    sec, nano = divmod(stamp, 1_000_000_000)
    return dict(topic=topic, msg_type='sensor_msgs/msg/CompressedImage',
        stamp_ns=stamp if received is None else received,
        _raw=b'\0\1\0\0'+struct.pack('<iI', sec, nano)+b'unchanged-payload')


def test_source_window_accepts_late_real_frame_but_not_after_stop_exposure():
    window = CaptureWindow(mp.get_context('spawn'))
    window.reset(True, action_at_publication=True)
    window.open(10_000_000_000)
    assert window.allows(event(9_750_000_000, received=10_030_000_000)) == (True, 'header')
    assert window.allows(event(9_749_999_999, received=10_000_000_000)) == (False, 'header')
    window.request_stop(11_000_000_000)
    assert window.allows(event(10_999_999_999, received=11_180_000_000)) == (True, 'header')
    assert window.allows(event(11_000_000_001, received=11_030_000_000)) == (False, 'header')
    assert window.snapshot()['action_at_publication'] == 'true'


def test_delayed_preroll_keeps_original_payload_and_receive_stamp_with_hard_bounds():
    sample = event(9_755_000_000, received=10_005_000_000)
    buffer = SensorPreroll([sample['topic']])
    buffer.remember(99.8, sample)
    assert buffer.snapshot(100.0) == [(99.8, sample)]
    assert buffer.snapshot(100.0)[0][1] is sample
    buffer.remember(100.0, event(9_800_000_000))
    assert len(buffer.snapshot(100.54)) == 2
    assert len(buffer.snapshot(100.56)) == 1
    bounded = SensorPreroll([sample['topic']], byte_budget=len(sample['_raw'])-1)
    bounded.remember(100., sample)
    assert bounded.snapshot(100.) == []
    assert bounded.evicted_bytes == len(sample['_raw'])


def test_rgbd_matches_headers_and_preserves_orphan_duplicate_and_reordered_evidence():
    audit = CaptureAudit()
    color = '/observation/cameras/head/color/compressed'
    depth = color.replace('/color/', '/depth/')
    for stamp in (1_000_000_000, 1_033_333_333, 1_100_000_000, 1_066_666_666, 1_100_000_000):
        audit.observe(event(stamp, topic=color))
    for stamp in (1_000_000_000, 1_066_666_666, 1_100_000_000, 1_133_333_333):
        audit.observe(event(stamp, topic=depth))
    result = audit.snapshot()
    assert result['streams'][color]['frames'] == 5
    assert result['streams'][color]['duplicates'] == 1
    assert result['streams'][color]['out_of_order'] == 1
    assert result['streams'][color]['gaps_gt_40ms'] == 0
    pair = result['rgbd']['/observation/cameras/head']
    assert pair['paired_exact'] == 3
    assert pair['color_unmatched'] == 2 and pair['depth_unmatched'] == 1
    assert result['streams'][depth]['gaps_gt_40ms'] == 1


def test_unknown_or_invalid_cdr_is_not_treated_as_a_capture_clock():
    assert header_stamp_ns(b'\0\1\0\0'+struct.pack('<iI', 1, 1_000_000_000), 'sensor_msgs/msg/Image') is None
    assert header_stamp_ns(b'\0\7\0\0'+struct.pack('<iI', 1, 1), 'sensor_msgs/msg/Image') is None
    assert header_stamp_ns(event(1_000_000_000)['_raw'], 'std_msgs/msg/String') is None
    window = CaptureWindow(mp.get_context('spawn'))
    window.reset(True)
    window.open(10_000_000_000)
    assert window.allows(event(1_000_000_000, received=10_100_000_000)) == (True, 'invalid_header')


def test_warm_stream_selection_is_bounded_to_cameras_and_small_evidence_streams():
    assert buffered_sensor(dict(topic='/control/joint_cmd', type='sensor_msgs/msg/JointState'))
    assert buffered_sensor(dict(topic='/control/solver/joint_targets', type='sensor_msgs/msg/JointState'))
    assert buffered_sensor(dict(topic='/control/vr/joint_states', type='sensor_msgs/msg/JointState'))
    assert buffered_sensor(dict(topic='/observation/cameras/head/depth/compressed', type='sensor_msgs/msg/CompressedImage'))
    assert not buffered_sensor(dict(topic='/action/joint_states', type='sensor_msgs/msg/JointState'))
    assert not buffered_sensor(dict(topic='/observation/ee_poses', type='geometry_msgs/msg/PoseArray'))


def test_audit_storage_is_bounded_and_reports_when_evidence_is_truncated():
    audit = CaptureAudit(max_samples=2)
    for stamp in (1_000_000_000, 1_033_333_333, 1_066_666_666):
        audit.observe(event(stamp))
    assert audit.snapshot()['observed_camera_frames'] == 2
    assert audit.snapshot()['truncated'] is True


def test_ingress_rechecks_a_pre_stop_decision_under_the_shared_fence(tmp_path):
    from middleware.core.recording.topic_recorder import TopicRecorder, enqueue_record
    recorder = TopicRecorder(dict(directory=str(tmp_path)), tmp_path)
    try:
        recorder._capture_window.reset(True)
        recorder._capture_window.open(10_000_000_000)
        recorder._accepting_event.set()
        # A source clock may lead receive time by a permitted small offset.
        future = event(10_010_000_000, received=10_000_000_000)
        assert recorder._capture_window.allows(future)[0]
        with recorder._ingress_lock:
            recorder._capture_window.request_stop(10_005_000_000)
        assert not enqueue_record(recorder.ipc_resources(), future,
                                  capture_window=recorder._capture_window)
        assert recorder.status()['accepted'] == 0
    finally:
        recorder._accepting_event.clear()
        recorder.shutdown()


def test_truncated_mcap_metadata_is_best_effort_and_does_not_raise(tmp_path):
    from middleware.core.recording.capture_window import read_capture_metadata
    source = tmp_path/'broken.mcap'
    for raw in (b'', b'not mcap', b'\x89MCAP0\r\n'):
        source.write_bytes(raw)
        assert read_capture_metadata(source) == {}
