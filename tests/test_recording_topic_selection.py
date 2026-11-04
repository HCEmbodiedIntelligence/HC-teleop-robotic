from pathlib import Path

import pytest
from mcap.reader import make_reader
from rclpy.serialization import serialize_message
from std_msgs.msg import String

from middleware.core.ros_recording_executor import recording_subscriptions
from middleware.core.topic_recorder import TopicRecorder, _inspect_mcap_file, enqueue_record


@pytest.mark.parametrize('profile', ['x1', 'other'])
@pytest.mark.parametrize('reusable', [False, True])
def test_selected_topics_include_empty_channels_and_preserve_real_samples(tmp_path, profile, reusable):
    subscriptions = [
        dict(topic='/custom/sensor', type='std_msgs/msg/String', outputs=['record']),
        dict(topic='/custom/no_publisher', type='std_msgs/msg/String', outputs=['record']),
        dict(topic='/custom/unselected', type='std_msgs/msg/String', outputs=['websocket']),
        dict(topic='/custom/disabled', type='std_msgs/msg/String', outputs=['record'], enabled=False),
    ]
    selected = recording_subscriptions(dict(legacy_control_profile=profile, subscriptions=subscriptions))
    recorder = TopicRecorder(dict(directory=str(tmp_path), reuse_writer_process=reusable),
                             tmp_path, topics=selected)
    raw = serialize_message(String(data='real sample'))
    event = dict(topic='/custom/sensor', msg_type='std_msgs/msg/String', _raw=raw, stamp_ns=123456789)
    try:
        # A completely silent recording still declares every selected channel.
        empty = Path(recorder.start('empty', {'profile_id': profile}))
        recorder.stop()
        info = _inspect_mcap_file(empty)
        assert info['message_count'] == 0
        assert info['topic_count'] == 2
        assert {c['topic']: c['count'] for c in info['channels']} == {
            '/custom/sensor': 0, '/custom/no_publisher': 0}
        with empty.open('rb') as stream:
            reader = make_reader(stream)
            assert list(reader.iter_messages()) == []
            summary = reader.get_summary()
            assert summary.statistics.message_start_time == summary.statistics.message_end_time == 0
            assert all(s.data for s in summary.schemas.values())

        path = Path(recorder.start('sample', {'profile_id': profile}))
        assert not recorder.record(dict(topic='/events/control/intervention', payload={'automatic': True}))
        assert not recorder.record({**event, 'topic': '/custom/unselected'})
        # The raw ROS worker enqueues directly; enforce the same selection there.
        assert enqueue_record(recorder.ipc_resources(), {**event, 'topic': '/custom/disabled'})
        assert recorder.record(event)
        status = recorder.stop()
        assert status['error'] is None
        with path.open('rb') as stream:
            samples = [(c.topic, m.data, m.log_time, m.publish_time)
                       for _, c, m in make_reader(stream).iter_messages()]
        assert samples == [('/custom/sensor', raw, 123456789, 123456789)]
        info = _inspect_mcap_file(path)
        assert info['message_count'] == 1
        assert info['topic_count'] == 2
        assert {c['topic']: c['count'] for c in info['channels']} == {
            '/custom/sensor': 1, '/custom/no_publisher': 0}
    finally:
        recorder.shutdown()


def test_empty_page_selection_does_not_record_any_topic(tmp_path):
    recorder = TopicRecorder(dict(directory=str(tmp_path)), tmp_path, topics=[])
    try:
        path = Path(recorder.start('none'))
        assert not recorder.record(dict(topic='/anything', msg_type='std_msgs/msg/String',
                                        _raw=serialize_message(String(data='not selected'))))
        recorder.stop()
        info = _inspect_mcap_file(path)
        assert info['message_count'] == info['topic_count'] == 0
    finally:
        recorder.shutdown()
