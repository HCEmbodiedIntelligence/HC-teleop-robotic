"""Native QoS warnings survive migration and restoration on localhost only."""
import functools
import os
import threading
import time

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from middleware.core.control.qos_event_observer import QosEventObserver


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '231' or
                    os.environ.get('ROS_LOCALHOST_ONLY') != '1',
                    reason='requires explicitly reserved localhost domain 231')
def test_original_native_incompatible_warning_runs_off_control_then_restores(capfd):
    context = Context()
    rclpy.init(context=context, domain_id=231)
    owner = rclpy.create_node('qos_observer_owner_test', context=context)
    probe = rclpy.create_node('qos_observer_probe_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(owner)
    received, callbacks = [], []
    subscription = owner.create_subscription(String, '/test/qos_observer/incompatible', received.append,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    handlers = [h for h in subscription.event_handlers if QosEventObserver._is_default_incompatible(h)]
    if not handlers:
        executor.shutdown(); owner.destroy_node(); probe.destroy_node(); context.try_shutdown()
        pytest.skip('RMW does not support native incompatible QoS events')
    handler = handlers[0]
    original_callback, original_group = handler.callback, handler.callback_group
    @functools.wraps(original_callback)
    def tracked(event):
        original_callback(event)
        callbacks.append(threading.get_ident())
    handler.callback = tracked
    original_handle, original_qos = subscription.handle, subscription.qos_profile
    observer = QosEventObserver([owner], enabled=True)
    try:
        assert observer.start(), observer.status()
        assert handler not in tuple(owner.waitables)
        assert subscription.handle is original_handle
        assert subscription.qos_profile is original_qos
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        first = probe.create_publisher(String, '/test/qos_observer/incompatible', qos)
        deadline = time.monotonic()+5
        while not callbacks and time.monotonic()<deadline:
            first.publish(String(data='must remain incompatible'))
            time.sleep(.01)
        assert callbacks, observer.status()
        assert callbacks[0] == observer._thread.ident
        assert callbacks[0] != threading.get_ident()
        assert not received
        warning = capfd.readouterr().err
        assert 'incompatible QoS' in warning and '/test/qos_observer/incompatible' in warning
        observer.close()
        assert handler in tuple(owner.waitables)
        assert handler.callback is tracked and handler.callback_group is original_group
        assert not observer.status()['thread_alive']
        before = len(callbacks)
        second = probe.create_publisher(String, '/test/qos_observer/incompatible', qos)
        deadline = time.monotonic()+5
        while len(callbacks)==before and time.monotonic()<deadline:
            second.publish(String(data='still incompatible after restoration'))
            executor.spin_once(timeout_sec=.01)
        assert len(callbacks)>before
        assert callbacks[-1] == threading.get_ident()
        assert not received
    finally:
        observer.close()
        executor.shutdown()
        owner.destroy_subscription(subscription)
        owner.destroy_node()
        probe.destroy_node()
        context.try_shutdown()
