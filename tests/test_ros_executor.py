"""Tests for CachedWaitSetExecutor correctness and batch draining."""
import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.context import Context
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from middleware.core.control.ros_executor import CachedWaitSetExecutor, create_single_threaded_executor


def test_cached_wait_set_drains_callbacks_guards_timers_and_zero_polls():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = rclpy.create_node('cached_ws_test_node', context=context,
                             enable_rosout=False, start_parameter_services=False)
    executor = CachedWaitSetExecutor(context=context)
    executor.add_node(node)
    received = []
    guard_hits = []
    timer_hits = []

    pub = node.create_publisher(String, '/test/cached_ws/msg', QoSProfile(depth=10))
    sub = node.create_subscription(String, '/test/cached_ws/msg',
                                   lambda msg: received.append(msg.data), QoSProfile(depth=10))
    guard = node.create_guard_condition(lambda: guard_hits.append(True))
    timer = node.create_timer(0.01, lambda: timer_hits.append(True))
    try:
        pub.publish(String(data='a'))
        pub.publish(String(data='b'))
        guard.trigger()

        # First wait loads ready items; follow-up 0.0 poll must drain remaining items.
        executor.spin_once(timeout_sec=0.05)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.0)
        deadline = time.monotonic() + 1.0
        while (len(received) < 2 or not guard_hits or not timer_hits) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

        assert received == ['a', 'b']
        assert guard_hits
        assert timer_hits
        assert executor._cached_wait_set is not None

        # Destroying an entity must invalidate/resize WaitSet cleanly without raising.
        node.destroy_timer(timer)
        pub.publish(String(data='c'))
        deadline = time.monotonic() + 1.0
        while 'c' not in received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert received == ['a', 'b', 'c']
    finally:
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)
        node.destroy_guard_condition(guard)
        executor.shutdown(timeout_sec=1.0)
        assert executor._cached_wait_set is None
        node.destroy_node()
        context.try_shutdown()


def test_create_single_threaded_executor_preserves_test_mocks():
    calls = []

    class FakeExecutor:
        def __init__(self, context=None):
            calls.append(context)

    mocked = create_single_threaded_executor(FakeExecutor, context='ctx')
    assert isinstance(mocked, FakeExecutor)
    assert calls == ['ctx']


def test_cached_wait_set_handles_waitables_via_qos_observer():
    from middleware.core.control.qos_event_observer import QosEventObserver

    context = Context()
    rclpy.init(args=[], context=context, domain_id=231)
    node = rclpy.create_node('cached_ws_waitable_node', context=context,
                             enable_rosout=False, start_parameter_services=False)
    executor = CachedWaitSetExecutor(context=context)
    executor.add_node(node)
    received = []
    pub = node.create_publisher(String, '/test/cached_ws/waitable', QoSProfile(depth=10))
    sub = node.create_subscription(String, '/test/cached_ws/waitable',
                                   lambda msg: received.append(msg.data), QoSProfile(depth=10))
    observer = QosEventObserver([node], enabled=True)
    try:
        # First spin runs with QoSEventHandler waitables attached.
        pub.publish(String(data='with-waitables'))
        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert received == ['with-waitables']

        # Moving default QoS handlers off shrinks WaitSet cleanly and keeps delivering.
        assert observer.start()
        pub.publish(String(data='zero-waitables'))
        deadline = time.monotonic() + 1.0
        while 'zero-waitables' not in received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert received == ['with-waitables', 'zero-waitables']
    finally:
        observer.close()
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        context.try_shutdown()
