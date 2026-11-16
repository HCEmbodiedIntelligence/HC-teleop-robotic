"""Exercise recorder workers and real CDR without a robot or ROS publisher."""
from pathlib import Path

from mcap.reader import make_reader
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage

from middleware.core.capture_window import read_capture_metadata
from middleware.core.ros_recording_executor import RosRecordingExecutor
from middleware.core.topic_recorder import TopicRecorder


def frame(stamp, data):
    message = CompressedImage(format='png', data=data)
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return serialize_message(message)


def test_closed_mcap_persists_real_window_and_late_depth_without_post_stop_frame(tmp_path):
    color = '/observation/cameras/head/color/compressed'
    depth = color.replace('/color/', '/depth/')
    topics = [dict(topic=t, type='sensor_msgs/msg/CompressedImage') for t in (color, depth)]
    recorder = TopicRecorder(dict(directory=str(tmp_path)), tmp_path, topics=topics)
    # Exercise the raw CDR ingress helper and the actual writer subprocess; do
    # not start any ROS nodes or change a robot's command output.
    executor = RosRecordingExecutor({}, recorder)
    recorder.prepare_capture_window = None
    try:
        path = Path(recorder.start('window', {'profile_id': 'x1'}))
        start = recorder.status()['capture_window']['admission_start_ns']
        source = start-100_000_000
        color_raw, depth_raw = frame(source, b'original-color'), frame(source, b'original-depth')
        executor._handle_message(color_raw, topic=color, msg_type='sensor_msgs/msg/CompressedImage', max_hz=0.)
        intent = recorder.request_capture_stop()
        assert intent['action_at_publication'] == 'false'
        # Depth arrives after stop intent but has the same original exposure.
        executor._handle_message(depth_raw, topic=depth, msg_type='sensor_msgs/msg/CompressedImage', max_hz=0.)
        executor._handle_message(frame(intent['stop_requested_ns']+1, b'after-stop'),
            topic=color, msg_type='sensor_msgs/msg/CompressedImage', max_hz=0.)
        result = recorder.stop()
        metadata = read_capture_metadata(path)
        window = metadata['capture_window']
        assert window['admission_start_ns'] == start
        assert window['stop_requested_ns'] == intent['stop_requested_ns']
        assert window['raw_ingress_closed_ns'] >= window['stop_requested_ns']
        with path.open('rb') as stream:
            actual = [(channel.topic, message.data) for _, channel, message in make_reader(stream).iter_messages()]
        assert actual == [(color, color_raw), (depth, depth_raw)]
        assert result['accepted'] == result['messages'] == 2
        assert result['dropped'] == 0 and result['error'] is None
        assert metadata['capture_audit']['rgbd']['/observation/cameras/head']['paired_exact'] == 1
        assert result['capture_audit'] == metadata['capture_audit']
    finally:
        recorder.shutdown()


def test_snapshot_and_other_profiles_keep_original_boundary_behavior(tmp_path):
    recorder = TopicRecorder(dict(directory=str(tmp_path)), tmp_path)
    try:
        for name, metadata in [('snapshot', {'profile_id':'x1', 'capture_snapshot':'{}'}),
                               ('other', {'profile_id':'other'})]:
            path = Path(recorder.start(name, metadata))
            assert recorder.status()['capture_window'] == {}
            recorder.stop()
            assert read_capture_metadata(path) == {}
    finally:
        recorder.shutdown()


def test_truncated_writer_preserves_original_error_and_task_can_leave_active_state(tmp_path, monkeypatch):
    from test_platform_task_recording import connected
    _, _, task = connected(tmp_path, monkeypatch)
    calls = []
    close = task.recorder._artifacts.close
    monkeypatch.setattr(task.recorder._artifacts, 'close',
        lambda *args: (calls.append(args), close(*args))[-1])
    try:
        task.start('broken', {'profile_id':'x1'}, {'mode':'local'})
        process = task.recorder._process
        process.terminate()
        process.join(timeout=3)
        result = task.stop()
        assert task.active is None
        job = next(iter(task.jobs.values()))
        assert job['state'] == 'interrupted'
        assert 'MCAP writer exited' in task.recorder.status()['error']
        assert calls and 'MCAP writer exited' in calls[0][1]
    finally:
        task.close()
