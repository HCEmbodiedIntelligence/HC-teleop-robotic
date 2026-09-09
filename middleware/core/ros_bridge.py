from __future__ import annotations

import queue
import math
import threading
import time
import traceback
from collections import deque
from typing import Any, Callable

from .protocol import envelope


EventCallback = Callable[[dict[str, Any], list[str]], None]

DEFAULT_TOPIC_STANDARDS: dict[str, dict[str, float]] = {
    "/hc_teleop/hardware_ready": {"target_hz": 100.0, "min_hz": 10.0},
    "/hc_teleop/joint_states": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/joint_cmd": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/joint_cmd_arm": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/sol_q": {"target_hz": 100.0, "min_hz": 50.0},
    "/hc_teleop/controller_target_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/hc_teleop/target_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/hc_teleop/actual_ee_poses": {"target_hz": 60.0, "min_hz": 30.0},
    "/hc_teleop/target_base_move": {"target_hz": 60.0, "min_hz": 20.0},
    "/hc_teleop/joint_cmd_finger_left": {"target_hz": 10.0, "min_hz": 1.0},
    "/hc_teleop/joint_cmd_finger_right": {"target_hz": 10.0, "min_hz": 1.0},
    "/hc_teleop/hand_joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/hc_teleop/camera_head/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/hc_teleop/camera_head/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/hc_teleop/camera_overhead/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/hc_teleop/camera_d405_left/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/hc_teleop/camera_d405_right/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/hc_teleop/camera_d405_left/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/hc_teleop/camera_d405_right/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
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

    def __init__(
        self,
        config: dict[str, Any],
        on_event: EventCallback,
        on_frame: Callable[[str, Any], None] | None = None,
        camera_topic: str = "",
        camera_topics: list[str] | None = None,
    ):
        self.config = config
        self.on_event = on_event
        self.on_frame = on_frame
        self.camera_topic = str(camera_topic)
        self.camera_topics = {
            str(topic) for topic in (camera_topics or []) if str(topic)
        }
        if self.camera_topic:
            self.camera_topics.add(self.camera_topic)
        self._stop = threading.Event()
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue(maxsize=1000)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._trackers: dict[str, TopicHealthTracker] = {}
        mux_config = config.get("command_mux", {})
        self._mux_enabled = bool(mux_config.get("enabled", True))
        self._command_source = str(mux_config.get("source", "vr"))
        self._command_output_enabled = True
        self._hardware_ready = False
        self._replay_active = False
        self._mux_received = {"vr": 0, "exoskeleton": 0}
        self._mux_last_received = {"vr": 0.0, "exoskeleton": 0.0}
        self._mux_forwarded = 0
        self._mux_rejected = 0
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
            mux_config = self.config.get("command_mux", {})
            result["command_mux"] = {
                "enabled": self._mux_enabled,
                "output_enabled": self._command_output_enabled,
                "hardware_ready": self._hardware_ready,
                "replay_active": self._replay_active,
                "source": self._command_source,
                "vr_topic": mux_config.get("vr_topic", "/hc_teleop/joint_cmd_vr"),
                "exoskeleton_topic": mux_config.get(
                    "exoskeleton_topic", "/hc_teleop/joint_cmd_exoskeleton"
                ),
                "output_topic": mux_config.get(
                    "output_topic", "/hc_teleop/joint_cmd"
                ),
                "control_source_topic": mux_config.get(
                    "control_source_topic", "/hc_teleop/control_source"
                ),
                "received": dict(self._mux_received),
                "last_received_age": {
                    source: round(now - stamp, 2) if stamp else None
                    for source, stamp in self._mux_last_received.items()
                },
                "forwarded": self._mux_forwarded,
                "rejected": self._mux_rejected,
            }
            return result

    def get_topic_health(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            return {
                topic: tracker.status(now) for topic, tracker in self._trackers.items()
            }

    def publish(self, topic: str, msg_type: str, data: dict[str, Any]) -> bool:
        if self._is_final_joint_command(topic):
            # Final actuator commands must come through the mux, or through the
            # explicitly armed replay path below.  The generic HTTP publisher
            # must never bypass source selection and hardware readiness.
            return False
        return self._enqueue("publish", (topic, msg_type, data))

    def publish_raw(self, topic: str, msg_type: str, raw_data: bytes) -> bool:
        if self._is_final_joint_command(topic):
            with self._lock:
                allowed = (
                    self._replay_active
                    and self._command_output_enabled
                    and self._hardware_ready
                )
            if not allowed:
                return False
            return self._enqueue("replay_raw", (topic, msg_type, raw_data))
        return self._enqueue("publish_raw", (topic, msg_type, raw_data))

    def begin_replay(self) -> bool:
        """Exclusively arm replay while retaining the normal safety gates."""
        with self._lock:
            if not self.config.get("enabled", True):
                self._replay_active = True
                return True
            if not self._command_output_enabled or not self._hardware_ready:
                return False
            self._replay_active = True
            return True

    def end_replay(self, require_reset: bool = True) -> None:
        with self._lock:
            self._replay_active = False
            if require_reset:
                # A deliberate dashboard/VR resume is required before any live
                # source can regain ownership of the actuator topic.
                self._command_output_enabled = False

    def _is_final_joint_command(self, topic: str) -> bool:
        mux_config = self.config.get("command_mux", {})
        return str(topic) in {
            str(mux_config.get("output_topic", "/hc_teleop/joint_cmd")),
            "/io_teleop/joint_cmd",
        }

    def emergency_stop(self, topic: str, reason: str) -> bool:
        with self._lock:
            self._command_output_enabled = False
        return self._enqueue("stop", (topic, reason))

    def set_control_source(self, source: str) -> bool:
        source = str(source).strip().lower()
        if source not in {"vr", "exoskeleton"}:
            raise ValueError("control source must be vr or exoskeleton")
        with self._lock:
            self._command_source = source
        return self._enqueue("control_source", source)

    def set_command_output_enabled(self, enabled: bool) -> bool:
        with self._lock:
            self._command_output_enabled = bool(enabled)
        return self._enqueue("command_output", bool(enabled))

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
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.utilities import get_message

            domain_id = int(self.config.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", 14))))
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)

            if not rclpy.ok():
                rclpy.init(args=[], domain_id=domain_id)
            node = rclpy.create_node(self.config.get("node_name", "hc_teleop_middleware"))
            # Camera decoding is performed by CameraService's latest-frame worker,
            # so callbacks here are deliberately short. A multi-threaded rclpy
            # executor adds substantial wait-set and GIL overhead at these rates.
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            publishers: dict[tuple[str, str], Any] = {}
            subscriptions = []
            subscription_names = []
            trackers: dict[str, TopicHealthTracker] = {}
            last_emit: dict[str, float] = {}

            mux_config = self.config.get("command_mux", {})
            if self._mux_enabled:
                from sensor_msgs.msg import JointState
                from std_msgs.msg import Bool, String

                hardware_ready_topic = str(
                    mux_config.get(
                        "hardware_ready_topic", "/hc_teleop/hardware_ready"
                    )
                )
                def hardware_ready_cb(msg: Any) -> None:
                    ready = bool(getattr(msg, "data", False))
                    with self._lock:
                        self._hardware_ready = ready

                subscriptions.append(
                    node.create_subscription(
                        Bool, hardware_ready_topic, hardware_ready_cb, qos_profile_sensor_data
                    )
                )
                subscription_names.append(hardware_ready_topic)

                control_source_topic = str(
                    mux_config.get(
                        "control_source_topic", "/hc_teleop/control_source"
                    )
                )
                control_source_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
                control_source_publisher = node.create_publisher(
                    String, control_source_topic, control_source_qos
                )
                publishers[(control_source_topic, "std_msgs/msg/String")] = (
                    control_source_publisher
                )
                control_source_publisher.publish(String(data=self._command_source))

                output_topic = str(
                    mux_config.get("output_topic", "/hc_teleop/joint_cmd")
                )
                output_publisher = self._publisher(
                    node,
                    publishers,
                    output_topic,
                    "sensor_msgs/msg/JointState",
                    JointState,
                )

                for source, topic in (
                    ("vr", str(mux_config.get("vr_topic", "/hc_teleop/joint_cmd_vr"))),
                    (
                        "exoskeleton",
                        str(
                            mux_config.get(
                                "exoskeleton_topic",
                                "/hc_teleop/joint_cmd_exoskeleton",
                            )
                        ),
                    ),
                ):
                    def mux_callback(
                        message: Any,
                        *,
                        source: str = source,
                    ) -> None:
                        now = time.monotonic()
                        valid = (
                            len(message.name) == len(message.position)
                            and bool(message.name)
                            and all(math.isfinite(float(value)) for value in message.position)
                        )
                        with self._lock:
                            self._mux_received[source] += 1
                            self._mux_last_received[source] = now
                            selected = self._command_source == source
                            output_enabled = self._command_output_enabled
                            hw_ready = self._hardware_ready
                            replay_active = self._replay_active
                            if not valid:
                                self._mux_rejected += 1
                        if (
                            not valid
                            or not selected
                            or not output_enabled
                            or not hw_ready
                            or replay_active
                        ):
                            return
                        output_publisher.publish(message)
                        with self._lock:
                            self._mux_forwarded += 1

                    subscriptions.append(
                        node.create_subscription(
                            JointState, topic, mux_callback, qos_profile_sensor_data
                        )
                    )
                    subscription_names.append(topic)

            for item in self.config.get("subscriptions", []):
                if not item.get("enabled", True):
                    continue
                topic = item["topic"]
                msg_type_name = item["type"]
                outputs = list(item.get("outputs", ["websocket"]))
                event_outputs = [output for output in outputs if output != "record"]
                is_webrtc_color = bool(
                    self.on_frame is not None
                    and topic in self.camera_topics
                )
                # Pure recording topics are owned exclusively by the isolated
                # raw-CDR process. The main bridge retains only UI/control topics
                # and the exact Color topic consumed by WebRTC.
                if not event_outputs and not is_webrtc_color:
                    continue
                max_hz = float(item.get("max_hz", 0))
                event_max_hz = float(item.get("event_max_hz", max_hz))
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
                    event_outputs: list[str] = event_outputs,
                    is_webrtc_color: bool = is_webrtc_color,
                    max_hz: float = event_max_hz,
                    tracker: TopicHealthTracker = tracker,
                ) -> None:
                    now = time.monotonic()
                    tracker.record_message(now)
                    if self.on_frame is not None and is_webrtc_color:
                        try:
                            self.on_frame(topic, message)
                        except Exception:
                            pass
                    # ROS recording is handled by RosRecordingExecutor. Keeping
                    # it out of this callback prevents image serialization from
                    # starving dashboard health and WebRTC work.
                    if not event_outputs:
                        with self._lock:
                            self._status["messages"] += 1
                        return
                    if max_hz and now - last_emit.get(topic, 0.0) < 1.0 / max_hz:
                        return
                    last_emit[topic] = now
                    payload = {}
                    if "CompressedImage" not in msg_type_name and "Image" not in msg_type_name:
                        try:
                            payload = message_to_ordereddict(message)
                        except Exception:
                            payload = {}
                    elif len(outputs) > 1 or "websocket" in outputs:
                        try:
                            payload = message_to_ordereddict(message)
                        except Exception:
                            payload = {}
                    event = envelope(
                        "ros_message",
                        "ros2",
                        payload,
                        topic=topic,
                        msg_type=msg_type_name,
                        stamp_ns=time.time_ns(),
                    )
                    self.on_event(event, event_outputs)
                    with self._lock:
                        self._status["messages"] += 1

                subscriptions.append(
                    node.create_subscription(
                        message_type, topic, callback, qos_profile_sensor_data
                    )
                )
                subscription_names.append(topic)

            # Keep the standard VR frame health visible even before recording
            # is selected. This subscriber only counts messages; raw-CDR recording
            # remains owned by RosRecordingExecutor.
            if "/vrdata" not in trackers:
                from std_msgs.msg import String

                vr_tracker = TopicHealthTracker("/vrdata", "std_msgs/msg/String")
                trackers["/vrdata"] = vr_tracker
                subscriptions.append(node.create_subscription(
                    String, "/vrdata",
                    lambda message: vr_tracker.record_message(time.monotonic()),
                    qos_profile_sensor_data,
                ))
                subscription_names.append("/vrdata")

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
                elif command in {"publish_raw", "replay_raw"}:
                    topic, msg_type_name, raw_bytes = args
                    if command == "replay_raw":
                        with self._lock:
                            allowed = (
                                self._replay_active
                                and self._command_output_enabled
                                and self._hardware_ready
                            )
                        if not allowed:
                            continue
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
                    with self._lock:
                        self._command_output_enabled = False
                    node.get_logger().warning(f"Emergency stop: {reason}")
                elif command == "control_source":
                    with self._lock:
                        self._command_source = str(args)
                    mux_config = self.config.get("command_mux", {})
                    topic = str(
                        mux_config.get(
                            "control_source_topic", "/hc_teleop/control_source"
                        )
                    )
                    from std_msgs.msg import String

                    publisher = self._publisher(
                        node, publishers, topic, "std_msgs/msg/String", String
                    )
                    publisher.publish(String(data=str(args)))
                    node.get_logger().info(f"Joint command source: {args}")
                elif command == "command_output":
                    with self._lock:
                        self._command_output_enabled = bool(args)
                    node.get_logger().info(
                        "Joint command output %s"
                        % ("enabled" if args else "disabled")
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
