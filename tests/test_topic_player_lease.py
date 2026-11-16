"""A stopped MCAP producer cannot reuse a later replay's event or authority."""
import threading
from unittest.mock import MagicMock

import pytest

from mcap.writer import Writer
from middleware.core.topic_player import TopicPlayer


def make_recording(path):
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        schema = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'string data\n')
        channel = writer.register_channel('/action/joint_states', 'cdr', schema)
        for index in range(10):
            stamp = 1_000_000_000+index*1_000_000
            writer.add_message(channel, stamp, b'opaque replay bytes', stamp)
        writer.finish()


def test_stop_reset_refuses_new_producer_until_old_provider_returns(tmp_path):
    make_recording(tmp_path/'first.mcap')
    make_recording(tmp_path/'second.mcap')
    entered, release = threading.Event(), threading.Event()
    ros = MagicMock()
    sent = []
    ros.publish_raw = lambda *args: sent.append(args)
    def provider():
        if threading.current_thread().name == 'TopicPlayer-first.mcap':
            entered.set()
            assert release.wait(2.), 'test did not release the paused provider'
        return ros
    player = TopicPlayer(tmp_path, provider)
    old_thread = None
    try:
        player.play('first.mcap', speed=10., mode='raw')
        assert entered.wait(1.)
        old_thread = player._thread
        old_stop = player._stop_event
        player.stop()
        assert old_stop.is_set()
        player.acknowledge_reset()
        with pytest.raises(RuntimeError, match='already active'):
            player.play('second.mcap', speed=10., mode='raw')
        assert old_stop.is_set()
        release.set()
        old_thread.join(1.)
        assert not old_thread.is_alive()
        assert not sent  # no publish_raw after the blocked provider returned
        assert player.status()['state'] == 'idle'
        player.play('second.mcap', speed=10., mode='raw')
        assert player._stop_event is not old_stop
        assert old_stop.is_set()
        player._thread.join(1.)
        assert player.status()['state'] == 'completed'
        assert len(sent) == 10
    finally:
        release.set()
        player.stop()
        if old_thread:
            old_thread.join(1.)
        if player._thread:
            player._thread.join(1.)


def test_cancelled_reader_error_cannot_replace_stop_status_or_release_again(tmp_path, monkeypatch):
    make_recording(tmp_path/'first.mcap')
    entered, release = threading.Event(), threading.Event()
    ros = MagicMock()
    player = TopicPlayer(tmp_path, lambda: ros)
    from middleware.core import topic_player
    original = topic_player.make_reader
    def reader(stream):
        if threading.current_thread().name == 'TopicPlayer-first.mcap':
            entered.set()
            assert release.wait(2.)
            raise OSError('late error from cancelled reader')
        return original(stream)
    monkeypatch.setattr(topic_player, 'make_reader', reader)
    try:
        player.play('first.mcap', mode='raw')
        assert entered.wait(1.)
        worker = player._thread
        player.stop()
        released = ros.end_replay.call_count
        release.set()
        worker.join(1.)
        assert not worker.is_alive()
        assert player.status()['state'] == 'stopped'
        assert player.status()['requires_reset']
        assert ros.end_replay.call_count == released == 1
        ros.publish_raw.assert_not_called()
    finally:
        release.set()
        player.stop()
        if player._thread:
            player._thread.join(1.)
