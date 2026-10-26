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
    "/status/hardware/ready": {"target_hz": 100.0, "min_hz": 10.0},
    "/observation/joint_states": {"target_hz": 100.0, "min_hz": 20.0},
    "/action/joint_states": {"target_hz": 100.0, "min_hz": 20.0},
    "/control/solver/joint_targets": {"target_hz": 100.0, "min_hz": 20.0},
    "/status/solver/joint_states": {"target_hz": 100.0, "min_hz": 20.0},
    "/control/solver/ee_targets": {"target_hz": 60.0, "min_hz": 20.0},
    "/action/ee_poses": {"target_hz": 60.0, "min_hz": 20.0},
    "/observation/ee_poses": {"target_hz": 60.0, "min_hz": 20.0},
    "/action/base/command": {"target_hz": 60.0, "min_hz": 10.0},
    "/action/left_hand/joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/action/right_hand/joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/hands/joint_states": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/hands/triggers": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/hands/vibration_feedback": {"target_hz": 10.0, "min_hz": 1.0},
    "/observation/ee_wrenches": {"target_hz": 100.0, "min_hz": 20.0},
    "/observation/cameras/head/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/head/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/head/calibration": {"target_hz": 2.0, "min_hz": 0.2},
    "/observation/cameras/head/extrinsics": {"target_hz": 2.0, "min_hz": 0.2},
    "/observation/head/pose": {"target_hz": 60.0, "min_hz": 20.0},
    "/action/head/pose": {"target_hz": 60.0, "min_hz": 20.0},
    "/observation/cameras/overhead/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_left/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_right/color/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_left/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/observation/cameras/d405_right/depth/compressed": {"target_hz": 30.0, "min_hz": 10.0},
    "/status/teleop": {"target_hz": 10.0, "min_hz": 2.0},
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
            timestamps = list(self._timestamps)
            timestamps_len = len(timestamps)
            dt = (timestamps[-1] - timestamps[0]) if timestamps_len >= 2 else 0.0

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

        intervals = [later - earlier for earlier, later in zip(timestamps, timestamps[1:])]
        max_gap_ms = round(max(intervals) * 1000.0, 1) if intervals else None
        if intervals:
            ordered = sorted(intervals)
            p95_gap_ms = round(ordered[int((len(ordered) - 1) * 0.95)] * 1000.0, 1)
        else:
            p95_gap_ms = None

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
            "max_gap_ms": max_gap_ms,
            "p95_gap_ms": p95_gap_ms,
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
        self._public_api = None
        if config.get("robot_api_profile"):
            from .robot_api import RobotApiProfile
            self._public_api = RobotApiProfile(config["robot_api_profile"])
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
        self._lock = threading.RLock()
        self._boundary = None
        self._permit_sender = None
        self._action_sink = lambda evidence: None
        from .control_ownership import ControlOwnership
        self.ownership = ControlOwnership(
            lock=self._lock, ros_clock=lambda: int(self._ros_now() * 1e9),
            revoke=self._ownership_revoke, permit=self._ownership_permit)
        self._trackers: dict[str, TopicHealthTracker] = {}
        mux_config = config.get("command_mux", {})
        self._mux_enabled = bool(mux_config.get("enabled", True))
        self._command_source = str(mux_config.get("source", "vr"))
        # A resume only arms the selected source. VR actuator output remains
        # closed until a fresh physical Grip press has been observed.
        self._command_output_enabled = False
        self._vr_grip_gate_open = False
        self._vr_grip_release_seen = False
        self._vr_left_gripped = False
        self._vr_right_gripped = False
        self._hardware_ready = False
        self._hardware_ready_last = -math.inf
        self._ready_timeout = float(mux_config.get('hardware_ready_timeout_sec', 0.5))
        if not math.isfinite(self._ready_timeout) or self._ready_timeout <= 0:
            raise ValueError('hardware_ready_timeout_sec must be positive and finite')
        self._ros_now = time.time
        self._source_cutoff = -math.inf
        self._source_stamps = {}
        self._output_epoch = 0
        self._hardware_enable_sent_epoch = None
        self._replay_active = False
        self._mux_received = {"vr": 0, "exoskeleton": 0, "motion": 0}
        self._mux_last_received = {"vr": 0.0, "exoskeleton": 0.0, "motion": 0.0}
        self._mux_received_times = {
            "vr": deque(maxlen=300), "exoskeleton": deque(maxlen=300), "motion": deque(maxlen=300)
        }
        self._mux_forwarded_times: deque[float] = deque(maxlen=300)
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
            def recent_rate(samples: deque[float]) -> float:
                recent = [stamp for stamp in samples if now - stamp <= 2.0]
                if len(recent) < 2 or now - recent[-1] > 1.0:
                    return 0.0
                return round((len(recent) - 1) / (recent[-1] - recent[0]), 1)

            result = dict(self._status)
            result['ownership'] = self.ownership.snapshot()
            result["topic_health"] = {
                topic: tracker.status(now) for topic, tracker in self._trackers.items()
            }
            mux_config = self.config.get("command_mux", {})
            ready = self._ready_locked(now)
            result["command_mux"] = {
                "enabled": self._mux_enabled,
                "output_enabled": self._command_output_enabled,
                "vr_grip_gate_open": self._vr_grip_gate_open,
                "vr_grip_release_seen": self._vr_grip_release_seen,
                "vr_grips": {
                    "left": self._vr_left_gripped,
                    "right": self._vr_right_gripped,
                },
                "hardware_ready": ready,
                "hardware_ready_age": (now - self._hardware_ready_last
                                       if math.isfinite(self._hardware_ready_last) else None),
                "replay_active": self._replay_active,
                "source": self._command_source,
                "vr_topic": mux_config.get("vr_topic", "/control/vr/joint_states"),
                "exoskeleton_topic": mux_config.get(
                    "exoskeleton_topic", "/control/exoskeleton/joint_states"
                ),
                "output_topic": mux_config.get(
                    "output_topic", "/action/joint_states"
                ),
                "control_source_topic": mux_config.get(
                    "control_source_topic", "/control/source"
                ),
                "received": dict(self._mux_received),
                "received_hz": {
                    source: recent_rate(samples)
                    for source, samples in self._mux_received_times.items()
                },
                "last_received_age": {
                    source: round(now - stamp, 2) if stamp else None
                    for source, stamp in self._mux_last_received.items()
                },
                "forwarded": self._mux_forwarded,
                "forwarded_hz": recent_rate(self._mux_forwarded_times),
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
        if self.ownership.active and self._ownership_control_topic(topic):
            return False
        if self._is_final_joint_command(topic):
            # Final actuator commands must come through the mux, or through the
            # explicitly armed replay path below.  The generic HTTP publisher
            # must never bypass source selection and hardware readiness.
            return False
        return self._enqueue("publish", (topic, msg_type, data))

    def publish_raw(self, topic: str, msg_type: str, raw_data: bytes) -> bool:
        if self.ownership.active and self._ownership_control_topic(topic):
            return False
        if self._is_final_joint_command(topic):
            with self._lock:
                allowed = (
                    self._replay_active
                    and self._command_output_enabled
                    and self._ready_locked(time.monotonic())
                )
                epoch = self._output_epoch
            if not allowed:
                return False
            return self._enqueue("replay_raw", (topic, msg_type, raw_data, epoch))
        return self._enqueue("publish_raw", (topic, msg_type, raw_data))

    def begin_replay(self) -> bool:
        """Exclusively arm replay while retaining the normal safety gates."""
        with self._lock:
            if self.ownership.active:
                return False
            if not self.config.get("enabled", True):
                self._replay_active = True
                return True
            if not self._command_output_enabled or not self._ready_locked(time.monotonic()):
                return False
            self._replay_active = True
            return True

    def end_replay(self, require_reset: bool = True) -> None:
        with self._lock:
            self._revoke_locked()
            self._replay_active = False
            if require_reset:
                # A deliberate dashboard/VR resume is required before any live
                # source can regain ownership of the actuator topic.
                self._command_output_enabled = False

    def _is_final_joint_command(self, topic: str) -> bool:
        mux_config = self.config.get("command_mux", {})
        return str(topic) in {
            str(mux_config.get("output_topic", "/action/joint_states")),
            "/io_teleop/joint_cmd",
        }

    def emergency_stop(self, topic: str, reason: str) -> bool:
        with self._lock:
            self.ownership.fault(reason, estop=True)
            self._revoke_locked()
            self._command_output_enabled = False
        return self._enqueue("stop", (topic, reason))

    def set_control_source(self, source: str) -> bool:
        source = str(source).strip().lower()
        if source not in {"vr", "exoskeleton", "motion"}:
            raise ValueError("control source must be vr, exoskeleton or motion")
        with self._lock:
            if self.ownership.active:
                raise ValueError('intervention session owns source selection')
            if self._command_source != source:
                self._revoke_locked()
                self._command_output_enabled = False
                self._reset_vr_grip_gate_locked()
            self._command_source = source
        return self._enqueue("control_source", source)

    def set_command_output_enabled(self, enabled: bool) -> bool:
        with self._lock:
            if self.ownership.active:
                if enabled:
                    raise ValueError('intervention resume must return to HOLD')
                self.ownership.hold()
            self._revoke_locked()
            self._command_output_enabled = bool(enabled)
            self._reset_vr_grip_gate_locked()
            epoch = self._output_epoch
        return self._enqueue("command_output", (bool(enabled), epoch))

    def _reset_vr_grip_gate_locked(self) -> None:
        self._vr_grip_gate_open = False
        self._vr_grip_release_seen = False
        self._vr_left_gripped = False
        self._vr_right_gripped = False

    def observe_vr_grips(
        self, left: float, right: float, *, valid: bool = True
    ) -> bool:
        """Open VR output only after release followed by a fresh Grip press."""
        mux_config = self.config.get("command_mux", {})
        press = float(mux_config.get("vr_grip_threshold", 0.55))
        release = float(mux_config.get("vr_grip_release_threshold", 0.50))
        with self._lock:
            previous_open = self._vr_grip_gate_open
            previous_grips = (self._vr_left_gripped, self._vr_right_gripped)
            try:
                left_value, right_value = float(left), float(right)
                sample_valid = (
                    bool(valid)
                    and math.isfinite(left_value)
                    and math.isfinite(right_value)
                    and 0.0 <= left_value <= 1.0
                    and 0.0 <= right_value <= 1.0
                    and 0.0 <= release < press <= 1.0
                )
            except (TypeError, ValueError):
                sample_valid = False
                left_value = right_value = 0.0
            if not sample_valid:
                self._reset_vr_grip_gate_locked()
            else:
                self._vr_left_gripped = left_value >= (
                    release if self._vr_left_gripped else press
                )
                self._vr_right_gripped = right_value >= (
                    release if self._vr_right_gripped else press
                )
                any_gripped = self._vr_left_gripped or self._vr_right_gripped
                if not any_gripped:
                    self._vr_grip_release_seen = True
                    self._vr_grip_gate_open = False
                else:
                    self._vr_grip_gate_open = bool(
                        self._command_output_enabled
                        and self._command_source == "vr"
                        and self._vr_grip_release_seen
                    )
            if (previous_open != self._vr_grip_gate_open or
                    previous_grips != (self._vr_left_gripped, self._vr_right_gripped)):
                # Fence targets prepared before either clutch transition,
                # including switching directly from one hand to the other.
                self._revoke_locked()
                if self._vr_grip_gate_open:
                    # VR resume is only an arm operation. The first fresh Grip
                    # is the event that enables hardware execution.
                    self._enqueue(
                        "command_output", (True, self._output_epoch)
                    )
            return self._vr_grip_gate_open

    def _source_gate_open_locked(self, source: str) -> bool:
        # Intervention commands carry their own immutable ownership token and
        # semantic clutch lease; the normal live-VR gate applies outside it.
        return self.ownership.active or source != "vr" or self._vr_grip_gate_open

    def _revoke_locked(self) -> None:
        self._source_cutoff = self._ros_now()
        self._output_epoch += 1

    @staticmethod
    def _ownership_control_topic(topic):
        return str(topic).startswith(('/control/', '/internal/', '/teleop/', '/action/'))

    def _ownership_revoke(self):
        self._command_output_enabled = False
        self._revoke_locked()

    def _ownership_permit(self, token, phase, cutoff_ns, active):
        if self._permit_sender is not None:
            self._permit_sender(token, phase, cutoff_ns, active)

    def control_ready(self):
        with self._lock:
            return bool(self._ready_locked(time.monotonic()) and self._boundary and
                        self._boundary.handover_ready())

    def recover_intervention(self):
        """An explicit reset may rearm hardware, but can only return to HOLD."""
        with self._lock:
            control = self.ownership
            if control.state not in {'FAULT', 'ESTOP'}:
                control.recover(control_ready=self.control_ready())
                return
            if not control.input_valid or control.gripped or not self._boundary or not self._boundary.handover_ready():
                raise ValueError('恢复前需要松开 Grip、有效跟踪及新鲜静止反馈')
            if not getattr(self, '_recovery', None):
                self._recovery = (control.token, time.monotonic())
                self._enqueue('ownership_recover', self._recovery)

    def _ready_locked(self, now: float) -> bool:
        if self._hardware_ready and now - self._hardware_ready_last > self._ready_timeout:
            self._hardware_ready = False
            self._command_output_enabled = False
            self._revoke_locked()
            self.ownership.fault('hardware ready timeout')
        return self._hardware_ready

    def receive_hardware_ready(self, ready: bool, now: float | None = None) -> None:
        with self._lock:
            current = time.monotonic() if now is None else now
            self._ready_locked(current)
            if not ready and self._hardware_ready:
                self._command_output_enabled = False
                self._revoke_locked()
                self.ownership.fault('hardware ready lost')
            self._hardware_ready = bool(ready)
            self._hardware_ready_last = current

    def forward_live_command(self, source: str, message: Any, publish: Callable, token=None) -> bool:
        """Keep authority across slow conversion and recheck at actuator send."""
        now = time.monotonic()
        try:
            names, positions = message.name, message.position
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            valid = (bool(names) and len(names) == len(positions)
                     and len(set(names)) == len(names) and all(names)
                     and all(math.isfinite(float(v)) for v in positions)
                     and math.isfinite(stamp) and stamp > 0
                     and -.05 <= self._ros_now() - stamp <= .3)
            if valid and self._public_api is not None:
                self._public_api.convert_message(message, to_model=True)
        except (AttributeError, TypeError, ValueError):
            valid, stamp = False, 0.0
        with self._lock:
            self._mux_received[source] += 1
            self._mux_last_received[source] = now
            self._mux_received_times[source].append(now)
            ready = self._ready_locked(now)
            if (not valid or stamp <= max(self._source_cutoff, self._source_stamps.get(source, 0))
                    or not ready or not self._command_output_enabled
                    or not self._source_gate_open_locked(source)
                    or self._replay_active or self._command_source != source
                    or (self.ownership.active and not self.ownership.allows(token))):
                self._mux_rejected += 1
                return False
            epoch = self._output_epoch
        prepared = self._boundary.prepare(message) if self._boundary else message
        with self._lock:
            if (prepared is None or epoch != self._output_epoch or not self._command_output_enabled
                    or not self._source_gate_open_locked(source)
                    or not self._ready_locked(time.monotonic()) or self._command_source != source
                    or self._replay_active or stamp <= self._source_stamps.get(source, 0)
                    or (self.ownership.active and not self.ownership.allows(token))):
                self._mux_rejected += 1
                return False
            if self._boundary and not (self._boundary.commit(prepared, intervention=True)
                    if self.ownership.active else self._boundary.commit(prepared)):
                self._mux_rejected += 1
                return False
            self._source_stamps[source] = stamp
            publish(message)
            if self.ownership.active:
                evidence = self.ownership.sent(token, message, int(self._ros_now()*1e9))
                if evidence is not None:
                    # Keep both times: stamp_ns is final publication, while
                    # this exact integer stamp links evidence to the original
                    # action in a continuous recording without guessing by age.
                    evidence['command_stamp_ns'] = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
                    self._action_sink(evidence)
            self._mux_forwarded += 1
            self._mux_forwarded_times.append(time.monotonic())
            return True

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
        camera_node = None
        camera_executor = None
        camera_executor_thread = None
        context = None
        robot_api_boundary = None
        try:
            import os
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from .ros_qos import subscription_qos
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.set_message import set_message_fields
            from rosidl_runtime_py.utilities import get_message

            domain_id = int(self.config.get("domain_id", int(os.environ.get("ROS_DOMAIN_ID", 14))))
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)

            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain_id)
            node = rclpy.create_node(self.config.get("node_name", "hc_teleop_middleware"), context=context)
            # Ownership callbacks have their own group: slow collision/model
            # work must not prevent Grip from revoking the final output lock.
            executor = MultiThreadedExecutor(num_threads=3, context=context)
            self._ownership_callback_group = MutuallyExclusiveCallbackGroup()
            # The node's default callback group is mutually exclusive.  Keep
            # actuator commands out of that group so status/recording traffic
            # cannot reduce a 100 Hz control stream to a sporadic output.
            self._command_callback_group = MutuallyExclusiveCallbackGroup()
            from rclpy.callback_groups import ReentrantCallbackGroup
            self._telemetry_callback_group = ReentrantCallbackGroup()
            executor.add_node(node)
            if self.camera_topics:
                # BEST_EFFORT camera frames must not wait behind 100 Hz control,
                # status and command callbacks.  A separate node/wait-set keeps
                # DDS draining at the publisher rate while control remains on
                # its existing executor and frequency.
                camera_node = rclpy.create_node(
                    self.config.get("node_name", "hc_teleop_middleware") + "_camera",
                    context=context,
                )
                camera_executor = SingleThreadedExecutor(context=context)
                camera_executor.add_node(camera_node)
            publishers: dict[tuple[str, str], Any] = {}
            subscriptions = []
            subscription_names = []
            trackers: dict[str, TopicHealthTracker] = {}
            last_emit: dict[str, float] = {}

            if self.config.get("robot_api_profile"):
                from .robot_api_boundary import RobotApiBoundary
                robot_api_boundary = RobotApiBoundary(node, self._public_api, subscribe_commands=False)
                self._boundary = robot_api_boundary

            dataset_config = self.config.get("dataset_topics", {})
            if dataset_config.get("enabled", False):
                from .dataset_topics import DatasetTopics
                dataset_topics = DatasetTopics(node, dataset_config)


            mux_config = self.config.get("command_mux", {})
            if self._mux_enabled:
                from sensor_msgs.msg import JointState
                from std_msgs.msg import Bool, String

                hardware_ready_topic = str(
                    mux_config.get(
                        "hardware_ready_topic", "/status/hardware/ready"
                    )
                )
                def hardware_ready_cb(msg: Any) -> None:
                    ready = bool(getattr(msg, "data", False))
                    self.receive_hardware_ready(ready)

                subscriptions.append(
                    node.create_subscription(
                        Bool, hardware_ready_topic, hardware_ready_cb,
                        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
                        callback_group=self._command_callback_group,
                    )
                )
                subscription_names.append(hardware_ready_topic)

                control_source_topic = str(
                    mux_config.get(
                        "control_source_topic", "/control/source"
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
                    mux_config.get("output_topic", "/action/joint_states")
                )
                output_publisher = self._publisher(
                    node,
                    publishers,
                    output_topic,
                    "sensor_msgs/msg/JointState",
                    JointState,
                )

                self._ros_now = lambda: node.get_clock().now().nanoseconds * 1e-9
                from std_msgs.msg import Header
                permit_pub = node.create_publisher(Header,
                    str(mux_config.get('motion_permit_topic', '/control/motion/permit')), 1)
                def publish_motion_permit():
                    with self._lock:
                        if self.ownership.active:
                            self._ownership_permit(self.ownership.token, self.ownership.phase,
                                                   self.ownership.cutoff_ns, True)
                            return
                        if (self._ready_locked(time.monotonic()) and self._command_output_enabled
                                and self._source_gate_open_locked(self._command_source)
                                and not self._replay_active and self._command_source in {'vr', 'motion'}):
                            header = Header()
                            header.stamp = node.get_clock().now().to_msg()
                            header.frame_id = f'{self._command_source}:{self._output_epoch}'
                            permit_pub.publish(header)
                node.create_timer(.05, publish_motion_permit, callback_group=self._ownership_callback_group)
                self._setup_ownership_ros(node, publishers, permit_pub, output_publisher, subscriptions)
                for source, topic in (
                    ("motion", str(mux_config.get("motion_topic", "/control/motion/joint_targets"))),
                    ("vr", str(mux_config.get("vr_topic", "/control/vr/joint_states"))),
                    (
                        "exoskeleton",
                        str(
                            mux_config.get(
                                "exoskeleton_topic",
                                "/control/exoskeleton/joint_states",
                            )
                        ),
                    ),
                ):
                    def mux_callback(
                        message: Any,
                        *,
                        source: str = source,
                    ) -> None:
                        self.forward_live_command(source, message, output_publisher.publish)

                    subscriptions.append(
                        node.create_subscription(
                            JointState,
                            topic,
                            mux_callback,
                            qos_profile_sensor_data,
                            callback_group=self._command_callback_group,
                        )
                    )
                    subscription_names.append(topic)

            for item in self.config.get("subscriptions", []):
                # Recording enablement is independent of a headset projection.
                if not item.get("enabled", True) and item["topic"] not in self.camera_topics:
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
                    else:
                        # Image bytes use on_frame/WebRTC and the raw-CDR recorder.
                        # Even when websocket is selected, never expand pixels into
                        # JSON here: this executor also forwards joint commands.
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
                    (camera_node if is_webrtc_color and camera_node is not None else node).create_subscription(
                        message_type, topic, callback, subscription_qos(item),
                        callback_group=self._telemetry_callback_group if not is_webrtc_color else None,
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
                    callback_group=self._telemetry_callback_group,
                ))
                subscription_names.append("/vrdata")

            with self._lock:
                self._trackers = trackers
            self._set_status(state="running", subscriptions=subscription_names, error=None)
            if camera_executor is not None:
                camera_executor_thread = threading.Thread(
                    target=camera_executor.spin,
                    name="ros-camera-executor",
                    daemon=True,
                )
                camera_executor_thread.start()
            executor_thread = threading.Thread(target=executor.spin, name="ros-bridge-executor", daemon=True)
            executor_thread.start()
            last_discovery = 0.0
            while not self._stop.is_set():
                time.sleep(0.01)
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
            self._permit_sender = None
            if camera_executor is not None:
                camera_executor.shutdown(timeout_sec=5.0)
            if camera_executor_thread is not None:
                camera_executor_thread.join(timeout=5.0)
            if camera_executor is not None and camera_node is not None:
                camera_executor.remove_node(camera_node)
            if camera_node is not None:
                camera_node.destroy_node()
            if executor is not None:
                executor.shutdown(timeout_sec=5.)
                # Humble's base shutdown waits active callbacks but does not
                # join the ThreadPoolExecutor's already submitted callbacks.
                executor._executor.shutdown(wait=True)
            if 'executor_thread' in locals() and executor_thread is not None:
                executor_thread.join(timeout=5.0)
            if robot_api_boundary is not None:
                robot_api_boundary.close()
            if executor is not None and node is not None:
                executor.remove_node(node)
            if node is not None:
                node.destroy_node()
            if context is not None:
                try:
                    context.try_shutdown()
                except Exception:
                    pass
            if self.status()["state"] != "error":
                self._set_status(state="stopped")

    def _setup_ownership_ros(self, node, publishers, permit_pub, output_pub, subscriptions):
        """Reuse Header/String/JointState on the private HC control chain.

        Public Servo inputs keep their existing header contract. The identity in
        a *private* target's frame_id is removed before the public projection.
        """
        import copy
        import json
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header, String
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from .control_ownership import ControlToken

        state_pub = node.create_publisher(String, '/status/control/ownership', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        events_pub = node.create_publisher(String, '/events/control/intervention', 100)
        action_pub = node.create_publisher(String, '/internal/control/action_evidence', 100)
        previous_sink = self._action_sink
        def evidence_sink(evidence):
            previous_sink(evidence)
            action_pub.publish(String(data=json.dumps(evidence, allow_nan=False)))
        self._action_sink = evidence_sink

        def send(token, phase, cutoff_ns, active):
            # Publish the revoke immediately, independently of the general HTTP
            # command queue and the 50 ms heartbeat. Keep-last heartbeats repeat
            # the transaction until both receivers confirm it.
            peers = self.ownership.peers
            key = '#'.join((token.key(), phase if active else 'off',
                            peers.get('motion', ''), peers.get('teleop', '')))
            permit_pub.publish(Header(stamp=node.get_clock().now().to_msg(), frame_id=key))
            if phase == 'revoke' and self.ownership.state in {'OFF', 'FAULT', 'ESTOP'}:
                self._publish_hardware_enable_locked(node, publishers, False)
        self._permit_sender = send

        def ack(msg):
            try:
                value = json.loads(msg.data)
                participant, instance = value['participant'], value['instance']
                if participant not in {'motion', 'teleop'} or not isinstance(instance, str) or not instance:
                    return
                age = node.get_clock().now().nanoseconds-int(value['stamp_ns'])
                if not -50_000_000 <= age <= 300_000_000:
                    return
                self.ownership.peer(participant, instance)
                if value.get('token'):
                    self.ownership.acknowledge(ControlToken.from_key(value['token']), value['phase'],
                        participant, instance, value.get('success') is True, value.get('reason', ''))
                tick()
            except (ValueError, KeyError, TypeError):
                return

        def semantic_input(msg):
            try:
                value = json.loads(msg.data)
                self.ownership.input(device_instance=value['device_instance'], sequence=int(value['sequence']),
                    valid=value['valid'] is True, gripped=value['gripped'] is True, stamp_ns=int(value['stamp_ns']))
                tick()
            except (ValueError, KeyError, TypeError):
                return

        def owned_target(msg):
            try:
                key, frame = msg.header.frame_id.split('#', 1)
                token = ControlToken.from_key(key)
            except (ValueError, AttributeError):
                return
            target = copy.deepcopy(msg)
            target.header.frame_id = frame
            # First version opens arms and measured grippers only. Other axes
            # remain held, even if an internal producer sends a mixed command.
            if self._public_api:
                allowed = {self._public_api.joints[n]['public_name']
                           for group, names in self._public_api.groups.items()
                           if group.endswith(('_arm', '_gripper')) for n in names}
                if self._boundary:
                    allowed.update(self._public_api.joints[n]['public_name']
                                   for n in self._boundary.gripper_publishers)
                if not set(target.name) <= allowed:
                    return
            self.forward_live_command(token.source, target, output_pub.publish, token)

        event_cursor = 0
        state_last = 0.
        def tick():
            nonlocal event_cursor, state_last
            with self._lock:
                ownership = self.ownership
                ownership.watchdog()
                recovery = getattr(self, '_recovery', None)
                if recovery:
                    if ownership.token != recovery[0] or time.monotonic()-recovery[1] > 2.:
                        self._recovery = None
                        self._hardware_enable_sent_epoch = None
                        self._publish_hardware_enable_locked(node, publishers, False)
                    elif self._hardware_ready_last > recovery[1] and self.control_ready():
                        self._recovery = None
                        ownership.recover(control_ready=True)
                if ownership.state in {'AUTO', 'MANUAL'} and self._boundary and not self._boundary.feedback_fresh():
                    ownership.fault('necessary robot feedback lost')
                previous = ownership.state
                ownership.advance(control_ready=self.control_ready(),
                                  manual_ready=ownership.input_valid and ownership.gripped)
                if ownership.state in {'AUTO', 'MANUAL'} and not self._command_output_enabled:
                    self._command_source = ownership.token.source
                    self._source_cutoff = ownership.cutoff_ns * 1e-9
                    self._command_output_enabled = True
                    # Normal handover fences Motion and this final sender.
                    # Hardware enable is a separate explicit safety operation;
                    # toggling it here would latch a driver stop on every Grip.
                while event_cursor < ownership.event_seq:
                    event = next((e for e in ownership.events if e['sequence'] > event_cursor), None)
                    if event is None:
                        break
                    events_pub.publish(String(data=json.dumps(event, allow_nan=False)))
                    event_cursor = event['sequence']
                now = time.monotonic()
                if now-state_last >= .1 or previous != ownership.state:
                    state_pub.publish(String(data=json.dumps(ownership.snapshot(), allow_nan=False)))
                    state_last = now

        subscriptions.extend([
            node.create_subscription(String, '/status/control/ack', ack, 100, callback_group=self._ownership_callback_group),
            node.create_subscription(String, '/events/teleop/clutch', semantic_input, 100, callback_group=self._ownership_callback_group),
            # Motion and the gripper adapter are independent producers. Keep
            # a bounded burst so neither overwrites the other before dispatch;
            # source age and immutable authority are still checked on receipt.
            node.create_subscription(JointState, '/internal/control/joint_targets', owned_target, 10),
        ])
        node.create_timer(.005, tick, callback_group=self._ownership_callback_group)

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
                break
            try:
                if command == 'ownership_recover':
                    with self._lock:
                        if args != getattr(self, '_recovery', None) or args[0] != self.ownership.token:
                            continue
                        from std_msgs.msg import Bool
                        self._publisher(node, publishers, '/control/emergency_stop', 'std_msgs/msg/Bool', Bool).publish(Bool(data=False))
                        self._hardware_enable_sent_epoch = None
                        self._publish_hardware_enable_locked(node, publishers, True)
                elif command == "publish":
                    topic, msg_type_name, data = args
                    if self.ownership.active and self._ownership_control_topic(topic):
                        continue
                    message_type = get_message(msg_type_name)
                    publisher = self._publisher(
                        node, publishers, topic, msg_type_name, message_type
                    )
                    message = message_type()
                    set_message_fields(message, data)
                    publisher.publish(message)
                elif command in {"publish_raw", "replay_raw"}:
                    topic, msg_type_name, raw_bytes = args[:3]
                    if self.ownership.active and self._ownership_control_topic(topic):
                        continue
                    if command == "replay_raw":
                        with self._lock:
                            allowed = (
                                args[3] == self._output_epoch
                                and self._replay_active
                                and self._command_output_enabled
                                and self._ready_locked(time.monotonic())
                            )
                        if not allowed:
                            continue
                    message_type = get_message(msg_type_name)
                    publisher = self._publisher(
                        node, publishers, topic, msg_type_name, message_type
                    )
                    from rclpy.serialization import deserialize_message

                    message = deserialize_message(raw_bytes, message_type)
                    if command == "replay_raw":
                        if self._public_api is not None:
                            self._public_api.convert_message(message, to_model=True)
                            # Recording timestamps describe the recording clock;
                            # an explicitly armed replay is a new command epoch.
                            message.header.stamp = node.get_clock().now().to_msg()
                        with self._lock:
                            if (args[3] == self._output_epoch and self._replay_active
                                    and self._command_output_enabled and self._ready_locked(time.monotonic())):
                                if self._boundary:
                                    if not self._boundary.command(message):
                                        continue
                                publisher.publish(message)
                    else:
                        publisher.publish(message)
                elif command == "stop":
                    topic, reason = args
                    from std_msgs.msg import Bool

                    publisher = self._publisher(
                        node, publishers, topic, "std_msgs/msg/Bool", Bool
                    )
                    message = Bool(data=True)
                    publisher.publish(message)
                    # emergency_stop() already revoked permission synchronously.
                    # Do not overwrite a later explicit resume queued after it.
                    node.get_logger().warning(f"Emergency stop: {reason}")
                elif command == "control_source":
                    if self.ownership.active:
                        continue
                    mux_config = self.config.get("command_mux", {})
                    topic = str(
                        mux_config.get(
                            "control_source_topic", "/control/source"
                        )
                    )
                    from std_msgs.msg import String

                    publisher = self._publisher(
                        node, publishers, topic, "std_msgs/msg/String", String
                    )
                    publisher.publish(String(data=str(args)))
                    node.get_logger().info(f"Joint command source: {args}")
                elif command == "command_output":
                    if self.ownership.active:
                        continue
                    enabled, epoch = args
                    with self._lock:
                        if (epoch != self._output_epoch or
                                enabled != self._command_output_enabled):
                            continue  # A subsequent stop/source change revoked this request.
                        if (enabled and self._command_source == "vr"
                                and not self._vr_grip_gate_open):
                            continue  # A only arms VR; fresh Grip enables hardware.
                        self._publish_hardware_enable_locked(node, publishers, enabled)
                    node.get_logger().info(
                        "Joint command output %s"
                        % ("enabled" if enabled else "disabled")
                    )
            except Exception as exc:
                self._set_status(error=f"publish failed: {type(exc).__name__}: {exc}")

        # Source changes, ready loss and replay completion can revoke output
        # without a queued enable request. Propagate their stop to hardware.
        # True is an explicit resume event only; never periodically re-arm a
        # stopped/restarted adapter, or replay a queued pre-stop enable.
        with self._lock:
            self._ready_locked(time.monotonic())
            if not getattr(self, '_recovery', None) and not self._command_output_enabled and (not self.ownership.active or
                    self.ownership.state in {'FAULT', 'ESTOP'}):
                self._publish_hardware_enable_locked(node, publishers, False)

    def _publish_hardware_enable_locked(self, node, publishers, enabled):
        now = time.monotonic()
        if self._hardware_enable_sent_epoch == self._output_epoch and (
                enabled or now-getattr(self, '_hardware_enable_last_sent', float('-inf')) < .1):
            return
        from std_msgs.msg import Bool

        topic = str(self.config.get("command_mux", {}).get(
            "hardware_enable_topic", "/control/hardware/enabled"))
        publisher = self._publisher(node, publishers, topic, "std_msgs/msg/Bool", Bool)
        publisher.publish(Bool(data=enabled))
        self._hardware_enable_sent_epoch = self._output_epoch
        # Reassert a stop across DDS discovery/restarts. Enabling remains an
        # explicit one-shot action and is never repeated by this heartbeat.
        self._hardware_enable_last_sent = now

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
