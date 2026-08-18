from __future__ import annotations

import queue
import threading
import time
import traceback
from collections import deque
from typing import Any, Callable

from .protocol import envelope


EventCallback = Callable[[dict[str, Any], list[str]], None]

DEFAULT_TOPIC_STANDARDS: dict[str, dict[str, float]] = {
    "/hc_teleop/joint_states": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/joint_cmd": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/joint_cmd_arm": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/sol_q": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/controller_target_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/hc_teleop/target_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/hc_teleop/actual_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/hc_teleop/target_base_move": {"target_hz": 60.0, "min_hz": 20.0},
    "/io_teleop/joint_states": {"target_hz": 100.0, "min_hz": 50.0},
    "/io_teleop/joint_cmd": {"target_hz": 100.0, "min_hz": 50.0},
    "/io_teleop/joint_cmd_arm": {"target_hz": 100.0, "min_hz": 50.0},
    "/io_teleop/sol_q": {"target_hz": 100.0, "min_hz": 50.0},
    "/io_teleop/controller_target_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/io_teleop/target_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/io_teleop/actual_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/io_teleop/target_base_move": {"target_hz": 60.0, "min_hz": 20.0},
    "/io_teleop/joint_cmd_finger_left": {"target_hz": 100.0, "min_hz": 50.0},
    "/io_teleop/joint_cmd_finger_right": {"target_hz": 100.0, "min_hz": 50.0},
    "/io_teleop/hand_joint_states": {"target_hz": 100.0, "min_hz": 30.0},
    "/io_teleop/camera_head/color": {"target_hz": 30.0, "min_hz": 10.0},
    "/io_teleop/camera_head/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/io_teleop/camera_d405_left/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/io_teleop/camera_d405_right/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/teleop/arm/status": {"target_hz": 10.0, "min_hz": 2.0},
    "/vrdata": {"target_hz": 60.0, "min_hz": 30.0},
    "/tf": {"target_hz": 20.0, "min_hz": 5.0},
}


class TopicHealthTracker:
    """Track message rate and quality for a subscribed ROS topic."""

    def __init__(
        self,
        topic: str,
        msg_type: str,
        target_hz: float = 0.0,
        min_hz: float = 0.0,
        record_enabled: bool = False,
    ):
        self.topic = topic
        self.msg_type = msg_type
        default_std = DEFAULT_TOPIC_STANDARDS.get(topic, {"target_hz": 10.0, "min_hz": 1.0})
        self.target_hz = float(target_hz or default_std["target_hz"])
        self.min_hz = float(min_hz or default_std["min_hz"])
        self.record_enabled = record_enabled
        self.messages = 0
        self.last_stamp = 0.0
        self._timestamps: deque[float] = deque(maxlen=40)
        self._lock = threading.Lock()

    def record_message(self, now: float) -> None:
        with self._lock:
            self.messages += 1
            self.last_stamp = now
            self._timestamps.append(now)

    def compute_hz(self, now: float) -> float:
        with self._lock:
            if not self._timestamps or (now - self.last_stamp > 1.8):
                return 0.0
            if len(self._timestamps) < 2:
                return 0.0
            dt = self._timestamps[-1] - self._timestamps[0]
            if dt <= 0.0001:
                return 0.0
            return (len(self._timestamps) - 1) / dt

    def status(self, now: float) -> dict[str, Any]:
        with self._lock:
            messages = self.messages
            last_stamp = self.last_stamp
            timestamps_len = len(self._timestamps)
            dt = (self._timestamps[-1] - self._timestamps[0]) if timestamps_len >= 2 else 0.0

        if not timestamps_len or (now - last_stamp > 1.8):
            hz = 0.0
        elif timestamps_len < 2 or dt <= 0.0001:
            hz = 0.0
        else:
            hz = round((timestamps_len - 1) / dt, 1)

        has_data = messages > 0 and (now - last_stamp <= 1.8)
        if not has_data:
            state = "no_data"
            message = "未检测到消息发布 (0 Hz)"
        elif self.min_hz > 0 and hz < self.min_hz:
            state = "low_rate"
            message = f"频率偏低 ({hz:.1f} Hz < 标准 {self.min_hz:.1f} Hz)"
        else:
            state = "ok"
            message = f"正常 ({hz:.1f} Hz)"

        return {
            "topic": self.topic,
            "type": self.msg_type,
            "messages": messages,
            "hz": hz,
            "target_hz": self.target_hz,
            "min_hz": self.min_hz,
            "record_enabled": self.record_enabled,
            "has_data": has_data,
            "state": state,
            "message": message,
            "last_received_age": round(now - last_stamp, 2) if last_stamp > 0 else None,
        }


class RosBridge:
    """Run rclpy in its own thread and expose thread-safe bridge operations."""

    def __init__(self, config: dict[str, Any], on_event: EventCallback):
        self.config = config
        self.on_event = on_event
        self._stop = threading.Event()
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue(maxsize=1000)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._trackers: dict[str, TopicHealthTracker] = {}
        domain_id = int(config.get("domain_id", 0))
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", True) else "starting",
            "domain_id": domain_id,
            "error": None,
            "subscriptions": [],
            "discovered_topics": [],
            "messages": 0,
            "dropped_commands": 0,
            "topic_health": {},
        }

    def start(self) -> None:
        if not self.config.get("enabled", True):
            return
        self._thread = threading.Thread(target=self._run, name="ros-bridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            result = dict(self._status)
            result["topic_health"] = {
                topic: tracker.status(now) for topic, tracker in self._trackers.items()
            }
            return result

    def get_topic_health(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            return {
                topic: tracker.status(now) for topic, tracker in self._trackers.items()
            }

    def publish(self, topic: str, msg_type: str, data: dict[str, Any]) -> bool:
        return self._enqueue("publish", (topic, msg_type, data))

    def publish_raw(self, topic: str, msg_type: str, raw_data: bytes) -> bool:
        return self._enqueue("publish_raw", (topic, msg_type, raw_data))

    def emergency_stop(self, topic: str, reason: str) -> bool:
        return self._enqueue("stop", (topic, reason))

    def _enqueue(self, command: str, data: Any) -> bool:
        try:
            self._commands.put_nowait((command, data))
            return True
        except queue.Full:
            with self._lock:
                self._status["dropped_commands"] += 1
            return False

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            self._status.update(changes)

    def _run(self) -> None:
        rclpy = None
        node = None
        executor = None
        try:
            import os
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.utilities import get_message

            domain_id = int(self.config.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", 13))))
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)

            rclpy.init(args=[], domain_id=domain_id)
            node = rclpy.create_node(self.config.get("node_name", "hc_teleop_middleware"))
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            publishers: dict[tuple[str, str], Any] = {}
            subscriptions = []
            subscription_names = []
            trackers: dict[str, TopicHealthTracker] = {}
            last_emit: dict[str, float] = {}

            from rclpy.serialization import serialize_message
            for item in self.config.get("subscriptions", []):
                if not item.get("enabled", True):
                    continue
                topic = item["topic"]
                msg_type_name = item["type"]
                outputs = list(item.get("outputs", ["websocket"]))
                max_hz = float(item.get("max_hz", 0))
                message_type = get_message(msg_type_name)
                tracker = TopicHealthTracker(
                    topic,
                    msg_type_name,
                    target_hz=float(item.get("target_hz", 0)),
                    min_hz=float(item.get("min_hz", 0)),
                    record_enabled="record" in outputs,
                )
                trackers[topic] = tracker

                def callback(
                    message: Any,
                    *,
                    topic: str = topic,
                    msg_type_name: str = msg_type_name,
                    outputs: list[str] = outputs,
                    max_hz: float = max_hz,
                    tracker: TopicHealthTracker = tracker,
                ) -> None:
                    now = time.monotonic()
                    tracker.record_message(now)
                    if max_hz and now - last_emit.get(topic, 0.0) < 1.0 / max_hz:
                        return
                    last_emit[topic] = now
                    raw_bytes = None
                    if "record" in outputs:
                        try:
                            raw_bytes = bytes(serialize_message(message))
                        except Exception:
                            pass
                    event = envelope(
                        "ros_message",
                        "ros2",
                        message_to_ordereddict(message),
                        topic=topic,
                        msg_type=msg_type_name,
                        _raw=raw_bytes,
                        stamp_ns=time.time_ns(),
                    )
                    self.on_event(event, outputs)
                    with self._lock:
                        self._status["messages"] += 1

                subscriptions.append(
                    node.create_subscription(
                        message_type, topic, callback, qos_profile_sensor_data
                    )
                )
                subscription_names.append(topic)

            with self._lock:
                self._trackers = trackers
            self._set_status(state="running", subscriptions=subscription_names, error=None)
            last_discovery = 0.0
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.02)
                self._drain_commands(node, publishers, get_message, set_message_fields)
                now = time.monotonic()
                if now - last_discovery >= 2.0:
                    discovered = [
                        {"topic": name, "types": types}
                        for name, types in node.get_topic_names_and_types()
                    ]
                    self._set_status(discovered_topics=discovered)
                    last_discovery = now
        except Exception as exc:
            self._set_status(
                state="error",
                error=f"{type(exc).__name__}: {exc}",
                traceback=traceback.format_exc(limit=8),
            )
        finally:
            if executor is not None and node is not None:
                executor.remove_node(node)
            if node is not None:
                node.destroy_node()
            if rclpy is not None:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            if self.status()["state"] != "error":
                self._set_status(state="stopped")

    def _drain_commands(
        self,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        get_message: Callable[[str], Any],
        set_message_fields: Callable[[Any, dict[str, Any]], None],
    ) -> None:
        for _ in range(100):
            try:
                command, args = self._commands.get_nowait()
            except queue.Empty:
                return
            try:
                if command == "publish":
                    topic, msg_type_name, data = args
                    message_type = get_message(msg_type_name)
                    publisher = self._publisher(
                        node, publishers, topic, msg_type_name, message_type
                    )
                    message = message_type()
                    set_message_fields(message, data)
                    publisher.publish(message)
                elif command == "publish_raw":
                    topic, msg_type_name, raw_bytes = args
                    message_type = get_message(msg_type_name)
                    publisher = self._publisher(
                        node, publishers, topic, msg_type_name, message_type
                    )
                    from rclpy.serialization import deserialize_message

                    message = deserialize_message(raw_bytes, message_type)
                    publisher.publish(message)
                elif command == "stop":
                    topic, reason = args
                    from std_msgs.msg import Bool

                    publisher = self._publisher(
                        node, publishers, topic, "std_msgs/msg/Bool", Bool
                    )
                    message = Bool(data=True)
                    publisher.publish(message)
                    node.get_logger().warning(f"Emergency stop: {reason}")
            except Exception as exc:
                self._set_status(error=f"publish failed: {type(exc).__name__}: {exc}")

    @staticmethod
    def _publisher(
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
    ) -> Any:
        key = (topic, msg_type_name)
        if key not in publishers:
            publishers[key] = node.create_publisher(message_type, topic, 10)
        return publishers[key]
