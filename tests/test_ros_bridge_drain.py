"""The command drain must wake on enqueue instead of polling on a fixed sleep."""
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('std_msgs.msg')
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message

from middleware.core.ros_bridge import RosBridge


def wire():
    sent = []
    node = Mock()
    node.create_publisher.side_effect = lambda _type, topic, _qos: SimpleNamespace(
        publish=lambda msg: sent.append((topic, msg.data)))
    return node, sent


def test_waiting_drain_publishes_frame_enqueued_during_the_wait():
    bridge = RosBridge({'enabled': True}, lambda *a: None)
    node, sent = wire()
    timer = threading.Timer(.05, bridge.publish, ('/vrdata', 'std_msgs/msg/String', {'data': 'frame'}))
    timer.start()
    started = time.monotonic()
    bridge._drain_commands(node, {}, get_message, set_message_fields, wait=2.)
    timer.join()
    assert sent == [('/vrdata', 'frame')]
    assert time.monotonic()-started < 1.


def test_waiting_drain_returns_after_timeout_when_idle():
    bridge = RosBridge({'enabled': True}, lambda *a: None)
    node, sent = wire()
    started = time.monotonic()
    bridge._drain_commands(node, {}, get_message, set_message_fields, wait=.02)
    assert sent == []
    assert time.monotonic()-started < .5
