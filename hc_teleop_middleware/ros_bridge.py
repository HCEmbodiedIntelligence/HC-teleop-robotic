from __future__ import annotations

import queue
import threading
import time
import traceback
from typing import Any, Callable

from .protocol import envelope


EventCallback = Callable[[dict[str, Any], list[str]], None]


class RosBridge:
    """Run rclpy in its own thread and expose thread-safe bridge operations."""

    def __init__(self, config: dict[str, Any], on_event: EventCallback):
        self.config = config
        self.on_event = on_event
        self._stop = threading.Event()
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue(maxsize=1000)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        domain_id = int(config.get("domain_id", 0))
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", True) else "starting",
            "domain_id": domain_id,
            "error": None,
            "subscriptions": [],
            "discovered_topics": [],
            "messages": 0,
            "dropped_commands": 0,
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
        with self._lock:
            return dict(self._status)

    def publish(self, topic: str, msg_type: str, data: dict[str, Any]) -> bool:
        return self._enqueue("publish", (topic, msg_type, data))

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

            domain_id = int(self.config.get("domain_id", 0))
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)

            rclpy.init(args=[])
            node = rclpy.create_node(self.config.get("node_name", "hc_teleop_middleware"))
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            publishers: dict[tuple[str, str], Any] = {}
            subscriptions = []
            subscription_names = []
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

                def callback(
                    message: Any,
                    *,
                    topic: str = topic,
                    msg_type_name: str = msg_type_name,
                    outputs: list[str] = outputs,
                    max_hz: float = max_hz,
                ) -> None:
                    now = time.monotonic()
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
