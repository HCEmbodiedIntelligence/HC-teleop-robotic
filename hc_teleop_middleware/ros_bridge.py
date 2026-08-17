from __future__ import annotations

import json
import queue
import threading
import time
import traceback
from typing import Any, Callable

from .protocol import ControllerInput, Pose, envelope


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
        self._status: dict[str, Any] = {
            "state": "disabled" if not config.get("enabled", True) else "starting",
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

    def publish_pose(self, topic: str, pose: Pose, sequence: int) -> bool:
        return self._enqueue("pose", (topic, pose, sequence))

    def publish_controller_input(
        self,
        topic: str,
        event_topic: str,
        side: str,
        controller_input: ControllerInput,
        sequence: int,
        vr_timestamp: float,
    ) -> bool:
        return self._enqueue(
            "controller_input",
            (topic, event_topic, side, controller_input, sequence, vr_timestamp),
        )

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
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.utilities import get_message

            rclpy.init(args=[])
            node = rclpy.create_node(self.config.get("node_name", "hc_teleop_middleware"))
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            publishers: dict[tuple[str, str], Any] = {}
            subscriptions = []
            subscription_names = []
            last_emit: dict[str, float] = {}

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
                    event = envelope(
                        "ros_message",
                        "ros2",
                        message_to_ordereddict(message),
                        topic=topic,
                        msg_type=msg_type_name,
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
                elif command == "pose":
                    topic, pose, sequence = args
                    from geometry_msgs.msg import PoseStamped

                    publisher = self._publisher(
                        node, publishers, topic, "geometry_msgs/msg/PoseStamped", PoseStamped
                    )
                    message = PoseStamped()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = "vr"
                    message.pose.position.x, message.pose.position.y, message.pose.position.z = (
                        pose.position
                    )
                    (
                        message.pose.orientation.x,
                        message.pose.orientation.y,
                        message.pose.orientation.z,
                        message.pose.orientation.w,
                    ) = pose.quaternion
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
                elif command == "controller_input":
                    (
                        topic,
                        event_topic,
                        side,
                        controller_input,
                        sequence,
                        vr_timestamp,
                    ) = args
                    from sensor_msgs.msg import Joy

                    publisher = self._publisher(
                        node, publishers, topic, "sensor_msgs/msg/Joy", Joy
                    )
                    message = Joy()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.header.frame_id = f"vr_{side}_controller"
                    message.axes = controller_input.joy_axes()
                    message.buttons = controller_input.joy_buttons()
                    publisher.publish(message)

                    if controller_input.pressed_mask or controller_input.released_mask:
                        from std_msgs.msg import String

                        event_publisher = self._publisher(
                            node,
                            publishers,
                            event_topic,
                            "std_msgs/msg/String",
                            String,
                        )
                        event_publisher.publish(
                            String(
                                data=json.dumps(
                                    {
                                        "side": side,
                                        "sequence": sequence,
                                        "vr_timestamp": vr_timestamp,
                                        "pressed": controller_input.decode_buttons(
                                            controller_input.pressed_mask
                                        ),
                                        "released": controller_input.decode_buttons(
                                            controller_input.released_mask
                                        ),
                                        "pressed_mask": controller_input.pressed_mask,
                                        "released_mask": controller_input.released_mask,
                                    },
                                    ensure_ascii=False,
                                    separators=(",", ":"),
                                )
                            )
                        )
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
