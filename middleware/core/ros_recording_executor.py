from __future__ import annotations

import multiprocessing as mp
import queue
import threading
import time
import traceback
from collections import deque
from typing import Any

from .topic_recorder import TopicRecorder, _counter_add, _lower_recording_priority


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
            tolerance = min(0.002, period * 0.1)
            if deadline is not None and now + tolerance < deadline:
                return False
            self._next_emit[topic] = now + period
            return True


def _put_latest(target: Any, value: dict[str, Any]) -> None:
    try:
        target.put_nowait(value)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(value)
    except queue.Full:
        pass


def _health_snapshot(
    subscriptions: list[dict[str, Any]],
    timestamps: dict[str, deque[float]],
    message_counts: dict[str, int],
    now: float,
) -> dict[str, dict[str, Any]]:
    from .ros_bridge import DEFAULT_TOPIC_STANDARDS

    result: dict[str, dict[str, Any]] = {}
    for item in subscriptions:
        topic = str(item["topic"])
        samples = timestamps[topic]
        last_stamp = samples[-1] if samples else 0.0
        default = DEFAULT_TOPIC_STANDARDS.get(
            topic, {"target_hz": 10.0, "min_hz": 1.0}
        )
        target_hz = float(item.get("target_hz", 0.0) or default["target_hz"])
        min_hz = float(item.get("min_hz", 0.0) or default["min_hz"])
        if len(samples) >= 2 and now - last_stamp <= 1.8:
            elapsed = samples[-1] - samples[0]
            hz = (len(samples) - 1) / elapsed if elapsed > 0.0001 else 0.0
        else:
            hz = 0.0
        has_data = bool(samples) and now - last_stamp <= 1.8
        if not has_data:
            state = "no_data"
            message = "未检测到消息发布 (0 Hz)"
        elif min_hz > 0 and hz < min_hz:
            state = "low_rate"
            message = f"频率偏低 ({hz:.1f} Hz < 标准 {min_hz:.1f} Hz)"
        else:
            state = "ok"
            message = f"正常 ({hz:.1f} Hz)"
        result[topic] = {
            "topic": topic,
            "type": str(item["type"]),
            "messages": message_counts[topic],
            "hz": round(hz, 1),
            "target_hz": target_hz,
            "min_hz": min_hz,
            "record_enabled": True,
            "has_data": has_data,
            "state": state,
            "message": message,
            "last_received_age": round(now - last_stamp, 2) if last_stamp else None,
        }
    return result


def _raw_recording_process(
    config: dict[str, Any],
    subscriptions: list[dict[str, Any]],
    event_queue: Any,
    accepting_event: Any,
    dropped_counter: Any,
    stop_event: Any,
    reset_event: Any,
    status_queue: Any,
) -> None:
    """Receive serialized ROS messages without deserializing them."""
    _lower_recording_priority()
    context = None
    executor = None
    node = None
    counters = {
        "received": 0,
        "recorded": 0,
        "throttled": 0,
        "rejected": 0,
        "serialization_errors": 0,
    }
    timestamps = {str(item["topic"]): deque(maxlen=120) for item in subscriptions}
    message_counts = {str(item["topic"]): 0 for item in subscriptions}
    gate = RateGate()

    def publish_status(state: str, error: str | None = None) -> None:
        now = time.monotonic()
        _put_latest(
            status_queue,
            {
                "state": state,
                "active": accepting_event.is_set(),
                "active_subscriptions": len(subscriptions),
                "subscriptions": [str(item["topic"]) for item in subscriptions],
                "topic_health": _health_snapshot(
                    subscriptions, timestamps, message_counts, now
                ),
                "pid": __import__("os").getpid(),
                "priority": "low",
                "error": error,
                **counters,
            },
        )

    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message

        domain_id = int(config.get("domain_id", 13))
        context = Context()
        rclpy.init(args=[], context=context, domain_id=domain_id)
        node_name = f"{config.get('node_name', 'hc_teleop_middleware')}_recorder"
        node = rclpy.create_node(node_name, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        ros_subscriptions = []

        for item in subscriptions:
            topic = str(item["topic"])
            msg_type = str(item["type"])
            max_hz = float(item.get("max_hz", 0.0))
            message_type = get_message(msg_type)

            def callback(
                raw_data: bytes,
                *,
                topic: str = topic,
                msg_type: str = msg_type,
                max_hz: float = max_hz,
            ) -> None:
                now = time.monotonic()
                counters["received"] += 1
                message_counts[topic] += 1
                timestamps[topic].append(now)
                if not accepting_event.is_set():
                    return
                if not gate.allow(topic, max_hz, now):
                    counters["throttled"] += 1
                    return
                if not isinstance(raw_data, bytes):
                    counters["serialization_errors"] += 1
                    return
                try:
                    event_queue.put_nowait(
                        {
                            "kind": "ros_message",
                            "topic": topic,
                            "msg_type": msg_type,
                            "_raw": raw_data,
                            "stamp_ns": time.time_ns(),
                        }
                    )
                    counters["recorded"] += 1
                except queue.Full:
                    counters["rejected"] += 1
                    _counter_add(dropped_counter)

            ros_subscriptions.append(
                node.create_subscription(
                    message_type,
                    topic,
                    callback,
                    qos_profile_sensor_data,
                    raw=True,
                )
            )

        publish_status("running")
        last_status = time.monotonic()
        while not stop_event.is_set():
            if reset_event.is_set():
                reset_event.clear()
                gate.reset()
                for key in counters:
                    counters[key] = 0
            executor.spin_once(timeout_sec=0.02)
            now = time.monotonic()
            if now - last_status >= 0.5:
                publish_status("running")
                last_status = now
    except Exception as exc:
        publish_status(
            "error",
            f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=8)}",
        )
    finally:
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
        publish_status("stopped")


class RosRecordingExecutor:
    """Low-priority process containing all raw ROS recording subscriptions."""

    def __init__(self, config: dict[str, Any], recorder: TopicRecorder):
        self.config = config
        self.recorder = recorder
        self.domain_id = int(config.get("domain_id", 13))
        self.subscriptions = recording_subscriptions(config)
        recording_config = config.get("recording", {})
        recording_enabled = (
            bool(recording_config.get("enabled", True))
            if isinstance(recording_config, dict)
            else True
        )
        self.enabled = bool(config.get("enabled", True)) and recording_enabled
        self._mp = mp.get_context("spawn")
        # Multiprocessing Events/Queues start Python's resource-tracker child.
        # Allocate them only when recording subscriptions are actually enabled.
        self._stop_event: Any = None
        self._reset_event: Any = None
        self._status_queue: Any = None
        self._process: Any = None
        self._gate = RateGate()
        self._lock = threading.Lock()
        self._status: dict[str, Any] = {
            "state": "disabled" if not self.enabled else "starting",
            "active": False,
            "domain_id": self.domain_id,
            "node_name": f"{config.get('node_name', 'hc_teleop_middleware')}_recorder",
            "subscriptions": [item["topic"] for item in self.subscriptions],
            "received": 0,
            "recorded": 0,
            "throttled": 0,
            "rejected": 0,
            "serialization_errors": 0,
            "topic_health": {},
            "priority": "low",
            "error": None,
        }

    def _initialize_ipc(self) -> None:
        if self._stop_event is not None:
            return
        self._stop_event = self._mp.Event()
        self._reset_event = self._mp.Event()
        self._status_queue = self._mp.Queue(maxsize=4)

    def _close_ipc(self) -> None:
        status_queue = self._status_queue
        self._stop_event = None
        self._reset_event = None
        self._status_queue = None
        if status_queue is not None:
            try:
                status_queue.close()
                status_queue.join_thread()
            except (OSError, ValueError):
                pass

    def start(self) -> None:
        if not self.enabled:
            with self._lock:
                self._status.update(state="disabled", active=False, pid=None)
            return
        self._refresh_status()
        if self._process is not None:
            return
        self._initialize_ipc()
        event_queue, accepting_event, dropped_counter = self.recorder.ipc_resources()
        self._stop_event.clear()
        self._process = self._mp.Process(
            target=_raw_recording_process,
            args=(
                self.config,
                self.subscriptions,
                event_queue,
                accepting_event,
                dropped_counter,
                self._stop_event,
                self._reset_event,
                self._status_queue,
            ),
            name="ros-cdr-recorder",
            daemon=True,
        )
        self._process.start()

    def stop(self) -> None:
        process = self._process
        if process is None:
            self._close_ipc()
            with self._lock:
                self._status.update(
                    state="disabled" if not self.enabled else "stopped",
                    active=False,
                    pid=None,
                )
            return
        self._stop_event.set()
        process.join(timeout=8.0)
        if process.is_alive():
            process.terminate()
            process.join(timeout=2.0)
            if process.is_alive():
                process.kill()
                process.join(timeout=2.0)
        else:
            # Explicitly collect an already-exited child.  Merely observing
            # is_alive()/exitcode can otherwise leave a zombie until the parent
            # middleware process exits or creates another multiprocessing child.
            process.join(timeout=0.0)
        if process.is_alive():
            with self._lock:
                self._status.update(
                    state="error",
                    active=False,
                    error="recording process could not be stopped",
                )
            return
        self._process = None
        self._close_ipc()
        with self._lock:
            self._status.update(
                state="disabled" if not self.enabled else "stopped",
                active=False,
                pid=None,
            )

    def _refresh_status(self) -> None:
        latest = None
        status_queue = self._status_queue
        if status_queue is not None:
            while True:
                try:
                    latest = status_queue.get_nowait()
                except queue.Empty:
                    break
        if latest is not None:
            with self._lock:
                self._status.update(latest)
        process = self._process
        if process is not None and not process.is_alive():
            exitcode = process.exitcode
            process.join(timeout=0.0)
            self._process = None
            self._close_ipc()
            with self._lock:
                if self._status.get("state") not in {"error", "stopped"}:
                    self._status.update(
                        state="error",
                        active=False,
                        pid=None,
                        error=f"recording process exited with code {exitcode}",
                    )
                else:
                    self._status.update(active=False, pid=None)

    def status(self) -> dict[str, Any]:
        self._refresh_status()
        with self._lock:
            result = dict(self._status)
            result["topic_health"] = dict(self._status.get("topic_health", {}))
            return result

    def get_topic_health(self) -> dict[str, Any]:
        return dict(self.status().get("topic_health", {}))

    def prepare_recording(self) -> None:
        self._gate.reset()
        if self._reset_event is not None:
            self._reset_event.set()
        with self._lock:
            self._status.update(
                received=0,
                recorded=0,
                throttled=0,
                rejected=0,
                serialization_errors=0,
            )

    def _handle_message(
        self,
        raw_data: bytes,
        *,
        topic: str,
        msg_type: str,
        max_hz: float,
    ) -> None:
        """Testable in-process equivalent of the raw CDR callback."""
        if not self.recorder.is_recording():
            return
        now = time.monotonic()
        with self._lock:
            self._status["received"] += 1
        if not self._gate.allow(topic, max_hz, now):
            with self._lock:
                self._status["throttled"] += 1
            return
        if not isinstance(raw_data, bytes):
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
            self._status["recorded" if accepted else "rejected"] += 1
