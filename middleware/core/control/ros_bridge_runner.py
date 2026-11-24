"""RosBridge 后台 ROS 2 运行循环（节点/执行器创建、订阅接线、自旋与安全回收）。"""
from __future__ import annotations

import math
import os
import threading
import time
import traceback
from typing import Any

from ..protocol import envelope
from .topic_health import TopicHealthTracker


class RosBridgeRunner:
    """将原 619 行 RosBridge._run() 拆解为清晰的分阶段生命周期执行器。"""

    def __init__(self, bridge: Any) -> None:
        self.bridge = bridge
        self.node: Any = None
        self.executor: Any = None
        self.camera_node: Any = None
        self.camera_executor: Any = None
        self.camera_executor_thread: threading.Thread | None = None
        self.telemetry_node: Any = None
        self.telemetry_executor: Any = None
        self.telemetry_executor_thread: threading.Thread | None = None
        self.control_node: Any = None
        self.control_executor: Any = None
        self.control_executor_thread: threading.Thread | None = None
        self.command_node: Any = None
        self.command_executor: Any = None
        self.command_executor_thread: threading.Thread | None = None
        self.hand_node: Any = None
        self.hand_executor: Any = None
        self.hand_executor_thread: threading.Thread | None = None
        self.executor_thread: threading.Thread | None = None
        self.context: Any = None
        self.robot_api_boundary: Any = None
        self.dataset_process: Any = None
        self.wake_guard: Any = None
        self.publishers: dict[tuple[str, str], Any] = {}
        self.subscriptions: list[Any] = []
        self.subscription_names: list[str] = []
        self.trackers: dict[str, TopicHealthTracker] = {}
        self.last_emit: dict[str, float] = {}

    def _init_nodes_and_executors(self, domain_id: int) -> None:
        import rclpy
        from rclpy.callback_groups import (
            MutuallyExclusiveCallbackGroup,
            ReentrantCallbackGroup,
        )
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from .ros_executor import create_single_threaded_executor

        bridge = self.bridge
        setup_affinity = getattr(bridge, "_actor_setup_affinity", None)
        if bridge._control_actor_mode and callable(setup_affinity):
            try:
                setup_affinity(False)
            except OSError:
                pass
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=domain_id)
        node_options = (
            {"enable_rosout": False, "start_parameter_services": False}
            if bridge._control_actor_mode
            else {}
        )
        base_name = bridge.config.get("node_name", "hc_teleop_middleware")
        self.node = rclpy.create_node(base_name, context=self.context, **node_options)

        def control_node_for(suffix: str) -> Any:
            if bridge._control_actor_mode:
                return self.node
            return rclpy.create_node(base_name + suffix, context=self.context)

        self.executor = create_single_threaded_executor(
            SingleThreadedExecutor, context=self.context
        )
        bridge._ownership_callback_group = MutuallyExclusiveCallbackGroup()
        bridge._command_callback_group = MutuallyExclusiveCallbackGroup()
        bridge._hand_command_callback_group = MutuallyExclusiveCallbackGroup()
        bridge._telemetry_callback_group = ReentrantCallbackGroup()
        self.executor.add_node(self.node)

        self.telemetry_node = control_node_for("_telemetry")
        self.telemetry_executor = (
            self.executor
            if bridge._control_actor_mode
            else create_single_threaded_executor(
                SingleThreadedExecutor, context=self.context
            )
        )
        if self.telemetry_node is not self.node:
            self.telemetry_executor.add_node(self.telemetry_node)

        if bridge._mux_enabled:
            for attr_node, attr_exec, suffix in (
                ("control_node", "control_executor", "_control"),
                ("command_node", "command_executor", "_commands"),
                ("hand_node", "hand_executor", "_hands"),
            ):
                n = control_node_for(suffix)
                ex = (
                    self.executor
                    if bridge._control_actor_mode
                    else create_single_threaded_executor(
                        SingleThreadedExecutor, context=self.context
                    )
                )
                if n is not self.node:
                    ex.add_node(n)
                setattr(self, attr_node, n)
                setattr(self, attr_exec, ex)

        if bridge.camera_topics and not bridge._control_actor_mode:
            self.camera_node = rclpy.create_node(
                base_name + "_camera", context=self.context
            )
            self.camera_executor = create_single_threaded_executor(
                SingleThreadedExecutor, context=self.context
            )
            self.camera_executor.add_node(self.camera_node)

    def _setup_command_mux(self) -> None:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, Header, String
        from .control_clock import ControlClock

        bridge = self.bridge
        mux_config = bridge.config.get("command_mux", {})
        bridge._replay_home_publisher = bridge._publisher(
            self.node, self.publishers, "/teleop/arm/home", "std_msgs/msg/Bool", Bool
        )

        hardware_ready_topic = str(
            mux_config.get("hardware_ready_topic", "/status/hardware/ready")
        )

        def hardware_ready_cb(msg: Any) -> None:
            bridge.receive_hardware_ready(bool(getattr(msg, "data", False)))

        self.subscriptions.append(
            self.control_node.create_subscription(
                Bool,
                hardware_ready_topic,
                bridge._timed_callback("hardware_ready", hardware_ready_cb),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
                callback_group=bridge._ownership_callback_group,
            )
        )
        self.subscription_names.append(hardware_ready_topic)

        control_source_topic = str(
            mux_config.get("control_source_topic", "/control/source")
        )
        control_source_pub = self.node.create_publisher(
            String,
            control_source_topic,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.publishers[(control_source_topic, "std_msgs/msg/String")] = control_source_pub
        control_source_pub.publish(String(data=bridge._command_source))

        output_topic = str(mux_config.get("output_topic", "/action/joint_states"))
        output_publisher = bridge._publisher(
            self.command_node,
            self.publishers,
            output_topic,
            "sensor_msgs/msg/JointState",
            JointState,
        )

        bridge._control_clock = ControlClock(self.node.get_clock())
        bridge._ros_now = lambda: bridge._control_clock.now_ns() * 1e-9
        permit_pub = self.node.create_publisher(
            Header,
            str(mux_config.get("motion_permit_topic", "/control/motion/permit")),
            1,
        )

        def publish_motion_permit() -> None:
            with bridge._lock:
                if bridge.ownership.active:
                    bridge._ownership_permit(
                        bridge.ownership.token,
                        bridge.ownership.phase,
                        bridge.ownership.cutoff_ns,
                        True,
                    )
                    return
                if (
                    bridge._ready_locked(time.monotonic())
                    and bridge._command_output_enabled
                    and not bridge._actor_output_blocked()
                    and not bridge._replay_active
                    and bridge._command_source in {"vr", "motion"}
                ):
                    header = Header()
                    header.stamp = self.node.get_clock().now().to_msg()
                    header.frame_id = f"{bridge._command_source}:{bridge._output_epoch}"
                    permit_pub.publish(header)

        self.control_node.create_timer(
            0.05,
            bridge._timed_callback("ownership_permit", publish_motion_permit),
            callback_group=bridge._ownership_callback_group,
        )
        if bridge._legacy_enabled:
            from .legacy_control import LegacyTopicControl

            self.robot_api_boundary = LegacyTopicControl(
                self.command_node,
                bridge,
                output_publisher,
                feedback_node=self.control_node,
                hand_node=self.hand_node,
            )
            bridge._boundary = self.robot_api_boundary
        bridge._setup_ownership_ros(
            self.control_node,
            self.publishers,
            permit_pub,
            output_publisher,
            self.subscriptions,
            command_node=self.command_node,
            status_node=self.telemetry_node,
        )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=4,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        for source, topic in (
            ("motion", str(mux_config.get("motion_topic", "/control/motion/joint_targets"))),
            ("vr", str(mux_config.get("vr_topic", "/control/vr/joint_states"))),
            (
                "exoskeleton",
                str(
                    mux_config.get(
                        "exoskeleton_topic", "/control/exoskeleton/joint_states"
                    )
                ),
            ),
        ):
            def mux_callback(message: Any, *, source: str = source) -> None:
                if bridge._legacy_enabled and source == "motion":
                    return
                bridge.forward_live_command(source, message, output_publisher.publish)

            self.subscriptions.append(
                self.command_node.create_subscription(
                    JointState,
                    topic,
                    bridge._timed_callback("mux:" + source, mux_callback),
                    command_qos,
                    callback_group=bridge._command_callback_group,
                )
            )
            self.subscription_names.append(topic)

    def _setup_telemetry_subscriptions(self, get_message: Any, message_to_ordereddict: Any) -> None:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from .ros_qos import subscription_qos

        bridge = self.bridge
        if bridge._control_actor_mode:
            return

        for item in bridge.config.get("subscriptions", []):
            if not item.get("enabled", True) and item["topic"] not in bridge.camera_topics:
                continue
            topic = item["topic"]
            msg_type_name = item["type"]
            outputs = list(item.get("outputs", ["websocket"]))
            event_outputs = [output for output in outputs if output != "record"]
            is_webrtc_color = bool(
                bridge.on_frame is not None and topic in bridge.camera_topics
            )
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
            self.trackers[topic] = tracker

            def callback(
                message: Any,
                *,
                topic: str = topic,
                msg_type_name: str = msg_type_name,
                event_outputs: list[str] = event_outputs,
                is_webrtc_color: bool = is_webrtc_color,
                max_hz: float = event_max_hz,
                tracker: TopicHealthTracker = tracker,
            ) -> None:
                now = time.monotonic()
                tracker.record_message(now)
                if bridge.on_frame is not None and is_webrtc_color:
                    try:
                        bridge.on_frame(topic, message)
                    except Exception:
                        pass
                if not event_outputs:
                    bridge._count_telemetry_message()
                    return
                if max_hz and now - self.last_emit.get(topic, 0.0) < 1.0 / max_hz:
                    return
                self.last_emit[topic] = now
                payload: dict[str, Any] = {}
                if "CompressedImage" not in msg_type_name and "Image" not in msg_type_name:
                    try:
                        payload = message_to_ordereddict(message)
                        if msg_type_name == "sensor_msgs/msg/JointState":
                            for field in ("position", "velocity", "effort"):
                                payload[field] = [
                                    v if math.isfinite(v) else None
                                    for v in payload[field]
                                ]
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
                bridge.on_event(event, event_outputs)
                bridge._count_telemetry_message()

            target_node = (
                self.camera_node
                if is_webrtc_color and self.camera_node is not None
                else self.telemetry_node
            )
            self.subscriptions.append(
                target_node.create_subscription(
                    message_type,
                    topic,
                    bridge._timed_callback("telemetry:" + topic, callback),
                    subscription_qos(item),
                    callback_group=(
                        bridge._telemetry_callback_group if not is_webrtc_color else None
                    ),
                )
            )
            self.subscription_names.append(topic)

        if "/vrdata" not in self.trackers:
            from std_msgs.msg import String

            vr_tracker = TopicHealthTracker("/vrdata", "std_msgs/msg/String")
            self.trackers["/vrdata"] = vr_tracker
            self.subscriptions.append(
                self.telemetry_node.create_subscription(
                    String,
                    "/vrdata",
                    lambda _message: vr_tracker.record_message(time.monotonic()),
                    qos_profile_sensor_data,
                    callback_group=bridge._telemetry_callback_group,
                )
            )
            self.subscription_names.append("/vrdata")

        if bridge.config.get("rosout_diagnostics", True):
            try:
                from rcl_interfaces.msg import Log as RosoutLog
                from ..diagnostic_logs import RosoutLogFilter

                rosout_filter = RosoutLogFilter()

                def rosout_cb(message: Any) -> None:
                    stamp = getattr(message, "stamp", None)
                    for ev in rosout_filter.process(
                        name=getattr(message, "name", ""),
                        level=getattr(message, "level", 0),
                        msg=getattr(message, "msg", ""),
                        stamp_sec=getattr(stamp, "sec", None),
                        stamp_nanosec=getattr(stamp, "nanosec", 0),
                        file=getattr(message, "file", ""),
                        function=getattr(message, "function", ""),
                        line=getattr(message, "line", 0),
                    ):
                        bridge.on_event(ev, [])

                self.subscriptions.append(
                    self.telemetry_node.create_subscription(
                        RosoutLog,
                        "/rosout",
                        rosout_cb,
                        QoSProfile(
                            depth=50,
                            history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE,
                        ),
                        callback_group=bridge._telemetry_callback_group,
                    )
                )
            except Exception:
                pass

    def _spin_actor_loop(self, get_message: Any, set_message_fields: Any) -> None:
        from .qos_event_observer import QosEventObserver

        bridge = self.bridge
        bridge._qos_event_observer = QosEventObserver([self.node])
        bridge._qos_event_observer.start()
        bridge._actor_thread_ident = threading.get_ident()
        bridge._actor_flush_commands = lambda: bridge._drain_commands(
            self.node, self.publishers, get_message, set_message_fields, wait=0.0
        )
        if hasattr(self.node, "create_guard_condition"):
            self.wake_guard = self.node.create_guard_condition(lambda: None)
            bridge._actor_wake = self.wake_guard.trigger
        setup_affinity = getattr(bridge, "_actor_setup_affinity", None)
        if callable(setup_affinity):
            try:
                setup_affinity(True)
            except OSError:
                pass
        ready_hook = getattr(bridge, "_actor_ready", None)
        if callable(ready_hook):
            ready_hook()
        while not bridge._stop.is_set():
            bridge._actor_checkpoint(allow_snapshot=False)
            if bridge._stop.is_set():
                break
            bridge._actor_flush_commands()
            bridge._tick_handover_holds()
            bridge._tick_replay_alignment(self.node, self.publishers, get_message)
            bridge._spin_actor_callbacks(self.executor)
            if bridge._stop.is_set():
                break
            bridge._actor_flush_commands()
            bridge._drain_actor_targets()
            bridge._tick_handover_holds()
            bridge._tick_replay_alignment(self.node, self.publishers, get_message)
            if not bridge._actor_targets:
                bridge._actor_checkpoint(allow_snapshot=True)

    def _spin_normal_loop(self, get_message: Any, set_message_fields: Any) -> None:
        from .qos_event_observer import QosEventObserver

        bridge = self.bridge
        unique_nodes = []
        seen_node_ids = set()
        for candidate in (
            self.node,
            self.telemetry_node,
            self.control_node,
            self.command_node,
            self.hand_node,
            self.camera_node,
        ):
            if candidate is not None and id(candidate) not in seen_node_ids:
                seen_node_ids.add(id(candidate))
                unique_nodes.append(candidate)
        if unique_nodes:
            bridge._qos_event_observer = QosEventObserver(unique_nodes)
            bridge._qos_event_observer.start()

        self.telemetry_executor_thread = threading.Thread(
            target=self.telemetry_executor.spin,
            name="ros-telemetry-executor",
            daemon=True,
        )
        self.telemetry_executor_thread.start()
        for ex, thread_attr, name in (
            (self.control_executor, "control_executor_thread", "ros-control-executor"),
            (self.command_executor, "command_executor_thread", "ros-command-executor"),
            (self.hand_executor, "hand_executor_thread", "ros-hand-command-executor"),
            (self.camera_executor, "camera_executor_thread", "ros-camera-executor"),
        ):
            if ex is not None:
                t = threading.Thread(target=ex.spin, name=name, daemon=True)
                setattr(self, thread_attr, t)
                t.start()
        self.executor_thread = threading.Thread(
            target=self.executor.spin, name="ros-bridge-executor", daemon=True
        )
        self.executor_thread.start()

        last_discovery = 0.0
        while not bridge._stop.is_set():
            bridge._drain_commands(
                self.node, self.publishers, get_message, set_message_fields, wait=0.01
            )
            bridge._tick_replay_alignment(self.node, self.publishers, get_message)
            now = time.monotonic()
            if now - last_discovery >= 2.0:
                if self.dataset_process is not None:
                    self.dataset_process.check()
                discovered = [
                    {"topic": name, "types": types}
                    for name, types in self.node.get_topic_names_and_types()
                ]
                bridge._set_status(discovered_topics=discovered)
                last_discovery = now

    def _teardown(self) -> None:
        bridge = self.bridge
        if bridge._control_actor_mode:
            with bridge._lock:
                bridge._command_output_enabled = False
                bridge._revoke_locked()
                if self.node is not None:
                    try:
                        bridge._publish_hardware_enable_locked(
                            self.node, self.publishers, False
                        )
                    except Exception:
                        pass
            bridge._actor_wake = None
            bridge._actor_flush_commands = None
            if bridge._qos_event_observer is not None:
                bridge._qos_event_observer.close()
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=5.0)
                seen_nodes = set()
                for actor_node in (
                    self.telemetry_node,
                    self.control_node,
                    self.command_node,
                    self.hand_node,
                    self.camera_node,
                    self.node,
                ):
                    if actor_node is not None and id(actor_node) not in seen_nodes:
                        self.executor.remove_node(actor_node)
                        seen_nodes.add(id(actor_node))
            if self.wake_guard is not None and hasattr(
                self.node, "destroy_guard_condition"
            ):
                try:
                    self.node.destroy_guard_condition(self.wake_guard)
                except Exception:
                    pass
            self.telemetry_node = None if self.telemetry_node is self.node else self.telemetry_node
            self.control_node = None if self.control_node is self.node else self.control_node
            self.command_node = None if self.command_node is self.node else self.command_node
            self.hand_node = None if self.hand_node is self.node else self.hand_node
            self.executor = self.telemetry_executor = self.control_executor = None
            self.command_executor = self.hand_executor = self.camera_executor = None

        bridge._permit_sender = None
        if not bridge._control_actor_mode and bridge._qos_event_observer is not None:
            bridge._qos_event_observer.close()
        if self.telemetry_executor is not None:
            self.telemetry_executor.shutdown(timeout_sec=5.0)
        if self.telemetry_executor_thread is not None:
            self.telemetry_executor_thread.join(timeout=5.0)
        if self.telemetry_executor is not None and self.telemetry_node is not None:
            self.telemetry_executor.remove_node(self.telemetry_node)
        if self.telemetry_node is not None:
            self.telemetry_node.destroy_node()
        for ex, thread in (
            (self.control_executor, self.control_executor_thread),
            (self.command_executor, self.command_executor_thread),
            (self.hand_executor, self.hand_executor_thread),
            (self.camera_executor, self.camera_executor_thread),
        ):
            if ex is not None:
                ex.shutdown(timeout_sec=5.0)
            if thread is not None:
                thread.join(timeout=5.0)
        if self.camera_executor is not None and self.camera_node is not None:
            self.camera_executor.remove_node(self.camera_node)
        if self.camera_node is not None:
            self.camera_node.destroy_node()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=5.0)
        if self.executor_thread is not None:
            self.executor_thread.join(timeout=5.0)
        if self.dataset_process is not None:
            self.dataset_process.close()
            bridge._set_status(dataset_process=dict(state="stopped"))
        if self.robot_api_boundary is not None:
            self.robot_api_boundary.close()
        if bridge._control_clock is not None:
            bridge._control_clock.close()
        for ex, n in (
            (self.command_executor, self.command_node),
            (self.hand_executor, self.hand_node),
            (self.control_executor, self.control_node),
            (self.executor, self.node),
        ):
            if ex is not None and n is not None:
                ex.remove_node(n)
            if n is not None:
                n.destroy_node()
        if self.context is not None:
            try:
                self.context.try_shutdown()
            except Exception:
                pass
        if bridge.status()["state"] != "error":
            bridge._set_status(state="stopped")

    def run(self) -> None:
        bridge = self.bridge
        try:
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.utilities import get_message

            domain_id = int(
                bridge.config.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", 14)))
            )
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
            self._init_nodes_and_executors(domain_id)

            if bridge.config.get("robot_api_profile"):
                from ..robot.robot_api_boundary import RobotApiBoundary

                self.robot_api_boundary = RobotApiBoundary(
                    self.node, bridge._public_api, subscribe_commands=False
                )
                bridge._boundary = self.robot_api_boundary

            dataset_config = bridge.config.get("dataset_topics", {})
            if dataset_config.get("enabled", False) and not bridge._control_actor_mode:
                from ..recording.dataset_worker import DatasetTopicProcess

                self.dataset_process = DatasetTopicProcess(
                    dataset_config,
                    domain_id,
                    bridge.config.get("node_name", "hc_teleop_middleware") + "_dataset",
                )
                self.dataset_process.start()
                bridge._set_status(
                    dataset_process=dict(
                        pid=self.dataset_process.process.pid, state="ready"
                    )
                )

            if bridge._mux_enabled:
                self._setup_command_mux()
            self._setup_telemetry_subscriptions(get_message, message_to_ordereddict)

            with bridge._lock:
                bridge._trackers = self.trackers
            bridge._set_status(
                state="running", subscriptions=self.subscription_names, error=None
            )
            if bridge._control_actor_mode:
                self._spin_actor_loop(get_message, set_message_fields)
            else:
                self._spin_normal_loop(get_message, set_message_fields)
        except Exception as exc:
            bridge._set_status(
                state="error",
                error=f"{type(exc).__name__}: {exc}",
                traceback=traceback.format_exc(limit=8),
            )
        finally:
            self._teardown()

