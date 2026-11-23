"""Keep default QoS warnings off the control executor's entity scan.

Only existing default incompatible-QoS handlers move. Their native event,
callback, endpoint, and message QoS remain unchanged. Start before the owning
nodes spin; close before destroying any of their endpoints. Late-created
endpoints retain the normal owner-executor behaviour.
"""
from __future__ import annotations

import logging
import os
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import (
    ExternalShutdownException, ShutdownException, SingleThreadedExecutor, TimeoutException,
)
from rclpy.node import Node
from rclpy.qos_event import QoSEventHandler, QoSPublisherEventType, QoSSubscriptionEventType
from .ros_executor import create_single_threaded_executor


class QosEventObserver:
    def __init__(self, nodes, *, enabled=None):
        self._nodes = tuple(nodes)
        self._enabled = enabled
        self._observer = self._executor = self._thread = None
        self._entries = []
        self._closing = False
        self._started = False
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._state = dict(enabled=False, moved=0, moved_total=0, restored=0,
                           error='', running=False, thread_alive=False, poll_interval_ms=100)

    def _update(self, **fields):
        self._state = {**self._state, **fields}

    @staticmethod
    def _is_default_incompatible(handler):
        if not isinstance(handler, QoSEventHandler):
            return False
        if handler.event_type not in (
                QoSPublisherEventType.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS,
                QoSSubscriptionEventType.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS):
            return False
        callback = handler.callback
        return (getattr(callback, '__module__', '') == 'rclpy.qos_event' and
                getattr(callback, '__qualname__', '') in {
                    'PublisherEventCallbacks.create_event_handlers.<locals>._default_incompatible_qos_callback',
                    'SubscriptionEventCallbacks.create_event_handlers.<locals>._default_incompatible_qos_callback',
                })

    def status(self):
        # Status never waits for a native event callback or its logger.
        result = dict(self._state)
        result['thread_alive'] = self._thread is not None and self._thread.is_alive()
        return result

    def start(self):
        if self._started:
            return self._state['running']
        self._started = True
        try:
            enabled = self._enabled
            if enabled is None:
                value = os.environ.get('HC_QOS_EVENT_OBSERVER', '1').strip().lower()
                if value not in {'0', '1', 'false', 'true', 'off', 'on', 'no', 'yes'}:
                    raise ValueError('HC_QOS_EVENT_OBSERVER must be boolean')
                enabled = value in {'1', 'true', 'on', 'yes'}
            if enabled is not True:
                return False
            self._update(enabled=True)
            if not self._nodes:
                return False
            context = self._nodes[0].context
            if any(node.context is not context for node in self._nodes):
                raise ValueError('QoS observer nodes must share one ROS context')
            self._observer = Node(self._nodes[0].get_name()+'_qos_observer', context=context,
                                  enable_rosout=False, start_parameter_services=False)
            self._group = ReentrantCallbackGroup()
            self._executor = create_single_threaded_executor(SingleThreadedExecutor, context=context)
            self._executor.add_node(self._observer)
            for owner in self._nodes:
                for endpoint in (*owner.publishers, *owner.subscriptions):
                    for handler in endpoint.event_handlers:
                        if not self._is_default_incompatible(handler):
                            continue
                        if handler not in tuple(owner.waitables):
                            continue
                        original_group = handler.callback_group
                        owner.remove_waitable(handler)
                        self._entries.append((owner, handler, original_group))
                        handler.callback_group = self._group
                        self._group.add_entity(handler)
                        self._observer.add_waitable(handler)
            self._update(moved=len(self._entries), moved_total=len(self._entries))
            self._thread = threading.Thread(target=self._run,
                name='qos-event-observer', daemon=True)
            self._update(running=True)
            self._thread.start()
            return True
        except Exception as exc:
            self._update(error=str(exc), running=False)
            self._restore()
            self._dispose_observer()
            logging.getLogger(__name__).warning('QoS observer unavailable; original handlers retained: %s', exc)
            return False

    def _run(self):
        if hasattr(os, 'sched_getaffinity') and hasattr(os, 'sched_setaffinity'):
            try:
                cur = os.sched_getaffinity(0)
                if len(cur) == 1:
                    from .control_process import _parse_cpu_list
                    with open('/sys/devices/system/cpu/online', encoding='utf-8') as f:
                        online = _parse_cpu_list(f.read())
                    with open('/sys/devices/system/cpu/isolated', encoding='utf-8') as f:
                        isolated = _parse_cpu_list(f.read())
                    if cur <= isolated:
                        non_iso = online - isolated
                        if non_iso:
                            os.sched_setaffinity(0, non_iso)
            except OSError:
                pass
        try:
            # Some RMWs wake event-only wait sets on ordinary data activity.
            # Unbounded spin then just moves CPU consumption to this thread.
            # These are diagnostic warnings, never data or safety callbacks:
            # drain ready warnings at 10 Hz, using one cached ready iterator.
            while not self._stop.is_set():
                for _ in range(max(1, len(self._entries))):
                    if self._stop.is_set():
                        break
                    try:
                        handler, _, _ = self._executor.wait_for_ready_callbacks(timeout_sec=0.)
                    except TimeoutException:
                        break
                    handler()
                    failure = handler.exception()
                    if failure is not None:
                        raise failure
                self._stop.wait(.1)
        except (ExternalShutdownException, ShutdownException):
            pass
        except Exception as exc:
            self._update(error=str(exc))
            logging.getLogger(__name__).warning('QoS observer stopped; restoring original handlers: %s', exc)
        finally:
            self._update(running=False)
            if not self._closing:
                # No diagnostic callback can still be executing on this thread.
                # An observer failure falls back to the original control-node
                # handlers instead of silently removing QoS warnings.
                self._restore()

    def _restore(self):
        with self._lock:
            remaining = []
            for owner, handler, original_group in reversed(self._entries):
                try:
                    if self._observer is not None and handler in tuple(self._observer.waitables):
                        self._observer.remove_waitable(handler)
                    # A diagnostic executor may have selected a task just before
                    # shutdown/failure. It is no longer executing here; leave
                    # the original executor able to take that native event.
                    handler._executor_event = False
                    handler.callback_group = original_group
                    if handler not in tuple(owner.waitables):
                        owner.add_waitable(handler)
                except Exception as exc:
                    remaining.append((owner, handler, original_group))
                    self._update(error='QoS handler restore failed: '+str(exc))
            restored = len(self._entries)-len(remaining)
            self._entries = list(reversed(remaining))
            self._update(moved=len(self._entries), restored=self._state['restored']+restored)

    def _dispose_observer(self):
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.)
        if self._observer is not None:
            self._observer.destroy_node()
            self._observer = None

    def close(self):
        self._closing = True
        self._stop.set()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2.)
        if self._thread is not None:
            self._thread.join(timeout=2.)
            if self._thread.is_alive():
                self._update(error='QoS observer is still executing; endpoint destruction deferred')
                raise RuntimeError(self._state['error'])
        self._restore()
        if self._entries:
            raise RuntimeError(self._state['error'])
        self._dispose_observer()
        self._update(running=False)
