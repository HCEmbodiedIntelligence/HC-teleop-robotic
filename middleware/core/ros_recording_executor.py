from __future__ import annotations

import threading
import time
import traceback
from typing import Any, Callable

from .topic_recorder import TopicRecorder


def recording_subscriptions(config: dict[str, Any]) -> list[dict[str, Any]]:
    """Return enabled ROS subscriptions explicitly selected for MCAP recording."""
    return [
        dict(item)
        for item in config.get("subscriptions", [])
        if item.get("enabled", True) and "record" in item.get("outputs", [])
    ]


class RateGate:
    """Rate limit without halving a source that jitters around the configured cap."""

    def __init__(self) -> None:
        self._next_emit: dict[str, float] = {}
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self._next_emit.clear()

    def allow(self, topic: str, max_hz: float, now: float) -> bool:
        if max_hz <= 0:
            return True
        period = 1.0 / max_hz
        with self._lock:
            deadline = self._next_emit.get(topic)
            # ROS timers and camera clocks routinely jitter by 1-2 ms. A small
            # tolerance prevents a nominal 30 Hz input from alternating pass/drop.
            tolerance = min(0.002, period * 0.1)
            if deadline is not None and now + tolerance < deadline:
                return False
            self._next_emit[topic] = now + period
            return True


class RosRecordingExecutor:
    """Dedicated ROS context/executor for serializing configured MCAP topics.

    The dashboard/VR bridge deliberately does not perform recording work. This
    executor has its own ROS context and worker pool so large image messages
    cannot starve control telemetry or WebRTC callbacks.
    """

    def __init__(self, config: dict[str, Any], recorder: TopicRecorder):
        self.config = config
        self.recorder = recorder
        self.domain_id = int(config.get("domain_id", 13))
        self.subscriptions = recording_subscriptions(config)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._executor: Any = None
        self._lock = threading.Lock()
        self._gate = RateGate()
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", True) else "starting",
            "domain_id": self.domain_id,
            "node_name": f"{config.get('node_name', 'hc_teleop_middleware')}_recorder",
            "subscriptions": [item["topic"] for item in self.subscriptions],
            "received": 0,
            "recorded": 0,
            "throttled": 0,
            "rejected": 0,
            "serialization_errors": 0,
            "error": None,
        }

    def start(self) -> None:
        if not self.config.get("enabled", True):
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="ros-recording-executor",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        executor = self._executor
        if executor is not None:
            try:
                executor.wake()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._status)

    def prepare_recording(self) -> None:
        """Reset per-session counters and limiter state before opening an MCAP."""
        self._gate.reset()
        with self._lock:
            self._status.update(
                received=0,
                recorded=0,
                throttled=0,
                rejected=0,
                serialization_errors=0,
            )

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            self._status.update(changes)

    def _handle_message(
        self,
        message: Any,
        *,
        topic: str,
        msg_type: str,
        max_hz: float,
        serialize_message: Callable[[Any], Any],
    ) -> None:
        if not self.recorder.is_recording():
            return
        now = time.monotonic()
        with self._lock:
            self._status["received"] += 1
        if not self._gate.allow(topic, max_hz, now):
            with self._lock:
                self._status["throttled"] += 1
            return
        try:
            raw_data = bytes(serialize_message(message))
        except Exception:
            with self._lock:
                self._status["serialization_errors"] += 1
            return
        accepted = self.recorder.record(
            {
                "kind": "ros_message",
                "topic": topic,
                "msg_type": msg_type,
                "_raw": raw_data,
                "stamp_ns": time.time_ns(),
            }
        )
        with self._lock:
            if accepted:
                self._status["recorded"] += 1
            else:
                self._status["rejected"] += 1

    def _run(self) -> None:
        context = None
        executor = None
        node = None
        try:
            import rclpy
            from rclpy.callback_groups import ReentrantCallbackGroup
            from rclpy.context import Context
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.serialization import serialize_message
            from rosidl_runtime_py.utilities import get_message

            context = Context()
            rclpy.init(args=[], context=context, domain_id=self.domain_id)
            node_name = str(self._status["node_name"])
            node = rclpy.create_node(node_name, context=context)
            executor = MultiThreadedExecutor(num_threads=4, context=context)
            callback_group = ReentrantCallbackGroup()
            ros_subscriptions = []
            for item in self.subscriptions:
                topic = str(item["topic"])
                msg_type = str(item["type"])
                max_hz = float(item.get("max_hz", 0.0))
                message_type = get_message(msg_type)

                def callback(
                    message: Any,
                    *,
                    topic: str = topic,
                    msg_type: str = msg_type,
                    max_hz: float = max_hz,
                ) -> None:
                    self._handle_message(
                        message,
                        topic=topic,
                        msg_type=msg_type,
                        max_hz=max_hz,
                        serialize_message=serialize_message,
                    )

                ros_subscriptions.append(
                    node.create_subscription(
                        message_type,
                        topic,
                        callback,
                        qos_profile_sensor_data,
                        callback_group=callback_group,
                    )
                )
            self._executor = executor
            executor.add_node(node)
            self._set_status(state="running", error=None)
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            self._set_status(
                state="error",
                error=f"{type(exc).__name__}: {exc}",
                traceback=traceback.format_exc(limit=8),
            )
        finally:
            self._executor = None
            if executor is not None:
                try:
                    if node is not None:
                        executor.remove_node(node)
                    executor.shutdown(timeout_sec=2.0)
                except Exception:
                    pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    context.try_shutdown()
                except Exception:
                    pass
            if self.status()["state"] != "error":
                self._set_status(state="stopped")
