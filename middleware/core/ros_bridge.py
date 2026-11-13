from __future__ import annotations

import queue
import math
import re
import sys
import threading
import time
import traceback
from collections import Counter, deque
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
        self._legacy_enabled = bool(config.get("legacy_control_profile"))
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
        self._callback_timing_lock = threading.Lock()
        self._callback_timings = {}
        self._slow_callbacks = deque(maxlen=32)
        self._control_contention = deque(maxlen=64)
        self._contention_thread = None
        self._boundary = None
        self._permit_sender = None
        self._action_sink = lambda evidence: None
        from .control_ownership import ControlOwnership
        self.ownership = ControlOwnership(
            lock=self._lock, ros_clock=lambda: int(self._ros_now() * 1e9),
            revoke=self._ownership_revoke, permit=self._ownership_permit,
            required_peers=("solver", "teleop") if self._legacy_enabled else ("motion", "teleop"),
            recording_enabled=config.get("intervention", {}).get("automatic_recording", True))
        self._trackers: dict[str, TopicHealthTracker] = {}
        mux_config = config.get("command_mux", {})
        self._mux_enabled = bool(mux_config.get("enabled", True))
        self._command_source = str(mux_config.get("source", "vr"))
        self._command_output_enabled = True
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
        self._mux_rejection_reasons = Counter()
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
        self._contention_thread = threading.Thread(
            target=self._watch_control_contention, name='ros-latency-diagnostic', daemon=True)
        self._contention_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        if self._contention_thread is not None:
            self._contention_thread.join(timeout=1)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        # ROS clock reads may release the GIL and wait in C. Never perform
        # them while holding the lock used by the final actuator send.
        ownership = self.ownership.snapshot()
        boundary = self._boundary if self._legacy_enabled else None
        boundary_wall_ns = (boundary.node.get_clock().now().nanoseconds
                            if boundary is not None else None)
        with self._lock:
            def recent_rate(samples: deque[float]) -> float:
                recent = [stamp for stamp in samples if now - stamp <= 2.0]
                if len(recent) < 2 or now - recent[-1] > 1.0:
                    return 0.0
                return round((len(recent) - 1) / (recent[-1] - recent[0]), 1)

            result = dict(self._status)
            with self._callback_timing_lock:
                result['callback_timings'] = {name:dict(value) for name,value in self._callback_timings.items()}
                result['slow_callbacks'] = list(self._slow_callbacks)
                result['control_contention_samples'] = list(self._control_contention)
            result['ownership'] = ownership
            if self._legacy_enabled and self._boundary:
                result['legacy_control'] = self._boundary.status(wall_ns=boundary_wall_ns)
            trackers = list(self._trackers.items())
            mux_config = self.config.get("command_mux", {})
            ready = self._ready_locked(now)
            result["command_mux"] = {
                "enabled": self._mux_enabled,
                "output_enabled": self._command_output_enabled,
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
                "rejection_reasons": dict(self._mux_rejection_reasons),
            }
        result["topic_health"] = {
            topic: tracker.status(now) for topic, tracker in trackers
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

    def request_manual_home(self) -> bool:
        with self._lock:
            if (not self._legacy_enabled or self.ownership.state != 'MANUAL'
                    or self.ownership.token.source != 'vr' or not self._command_output_enabled
                    or not self._ready_locked(time.monotonic())):
                return False
            return self._enqueue('manual_home', self.ownership.token)

    def publish_raw(self, topic: str, msg_type: str, raw_data: bytes) -> bool:
        if self.ownership.active and self._ownership_control_topic(topic):
            return False
        if self._is_final_joint_command(topic):
            with self._lock:
                epoch = self._output_epoch
                if not self._replay_allowed_locked(epoch):
                    return False
            return self._enqueue("replay_raw", (topic, msg_type, raw_data, epoch))
        return self._enqueue("publish_raw", (topic, msg_type, raw_data))

    def _replay_allowed_locked(self, epoch: int) -> bool:
        return (epoch == self._output_epoch and self._replay_active and not self.ownership.active
                and self._command_output_enabled and self._ready_locked(time.monotonic()))

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
        if self._legacy_enabled and self._boundary:
            hands = self._boundary.teleop['grippers']
            if str(topic) in {hands[s]['finger_topic'] for s in ('right', 'left')}:
                return True
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
            epoch = self._output_epoch
        return self._enqueue("command_output", (bool(enabled), epoch))

    def _revoke_locked(self) -> None:
        self._source_cutoff = self._ros_now()
        self._output_epoch += 1

    @staticmethod
    def _ownership_control_topic(topic):
        return (str(topic) != '/teleop/recording_state'
                and str(topic).startswith(('/control/', '/internal/', '/teleop/', '/action/', '/hc_teleop/')))

    def _ownership_revoke(self):
        self._command_output_enabled = False
        self._revoke_locked()

    def _ownership_permit(self, token, phase, cutoff_ns, active):
        if not active:
            self._command_source = 'vr'
            self._enqueue('control_source', 'vr')
            self._enqueue('publish', ('/control/teleop/enabled', 'std_msgs/msg/Bool', {'data': True}))
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
            if ((control.input_valid and control.gripped) or
                    not self._boundary or not self._boundary.handover_ready()):
                raise ValueError('恢复前需要新鲜静止反馈，并松开正在使用的 Grip')
            if not getattr(self, '_recovery', None):
                self._recovery = (control.token, time.monotonic())
                self._enqueue('ownership_recover', self._recovery)

    def _timed_callback(self, name, callback):
        """Measure callback wall time without changing its scheduling or lock."""
        def measured(*args, **kwargs):
            started = time.monotonic_ns()
            try:
                return callback(*args, **kwargs)
            finally:
                elapsed_ms = (time.monotonic_ns()-started)/1e6
                with self._callback_timing_lock:
                    value = self._callback_timings.setdefault(name, dict(count=0, max_ms=0.))
                    value.update(count=value['count']+1, last_ms=elapsed_ms,
                                 max_ms=max(value['max_ms'], elapsed_ms))
                    if elapsed_ms >= 20.:
                        self._slow_callbacks.append(dict(name=name, elapsed_ms=elapsed_ms,
                            finished_ns=time.time_ns(), thread=threading.current_thread().name))
        measured._hc_timing_name = name
        return measured

    def _sample_control_contention(self):
        """Inspect the current lock owner; never acquire or replace control locks."""
        diagnostic = self.ownership.input_diagnostics
        now = time.time_ns()
        if (diagnostic.get('last_age_ms', 0) < 100. or
                now-diagnostic.get('last_callback_ns', 0) > 2_000_000_000):
            return None
        # CPython's RLock repr exposes its owner without taking that lock.
        # This is diagnostic-only: an unavailable owner is never a control fact.
        match = re.search(r'owner=(\d+)', repr(self._lock))
        owner = int(match.group(1)) if match else 0
        frame = sys._current_frames().get(owner) if owner else None
        stack = []
        while frame is not None and len(stack) < 8:
            stack.append(dict(file=frame.f_code.co_filename, function=frame.f_code.co_name,
                              line=frame.f_lineno))
            frame = frame.f_back
        group = getattr(self, '_ownership_callback_group', None)
        entity = getattr(group, '_active_entity', None)
        callback = getattr(entity, 'callback', None)
        return dict(stamp_ns=now, input_age_ms=diagnostic.get('last_age_ms'),
            owner_thread=next((t.name for t in threading.enumerate() if t.ident==owner), None),
            owner_stack=stack,
            active_ownership_callback=getattr(callback, '_hc_timing_name',
                                               type(entity).__name__ if entity is not None else None))

    def _watch_control_contention(self):
        while not self._stop.wait(.01):
            sample = self._sample_control_contention()
            if sample is not None:
                with self._callback_timing_lock:
                    self._control_contention.append(sample)

    def receive_intervention_input(self, value, *, callback_entry_ns=None):
        """Apply one semantic frame atomically, including B pause and A return."""
        if (not isinstance(value['device_instance'], str) or not value['device_instance']
                or type(value['sequence']) is not int or value['sequence'] < 0
                or type(value['stamp_ns']) is not int or value['stamp_ns'] <= 0):
            raise ValueError('invalid semantic input identity, sequence or timestamp')
        before_lock_ns = time.monotonic_ns()
        with self._lock:
            acquired_ns = time.monotonic_ns()
            control = self.ownership
            pause = value.get('pause_requested') is True
            resume = value.get('resume_requested') is True
            accepted = control.input(device_instance=value['device_instance'], sequence=value['sequence'],
                valid=value['valid'] is True, gripped=value['gripped'] is True, stamp_ns=value['stamp_ns'],
                pause_requested=pause, resume_requested=resume)
            if callback_entry_ns is not None:
                control.input_diagnostics.update(
                    callback_entry_source_ns=value['stamp_ns'],
                    callback_entry_age_ms=(callback_entry_ns-value['stamp_ns'])/1e6,
                    callback_lock_wait_ms=(acquired_ns-before_lock_ns)/1e6,
                    callback_input_work_ms=(time.monotonic_ns()-acquired_ns)/1e6)
            if accepted and resume and not pause and control.active:
                if control.state == 'HOLD':
                    control.start_auto(recording_ready=control.capture_ready(), control_ready=self.control_ready())
                elif control.state in {'FAULT', 'ESTOP'}:
                    # Hardware recovery only returns to HOLD. A subsequent
                    # released-then-pressed A is needed to start AUTO.
                    self.recover_intervention()
            return accepted

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
        stream = source
        try:
            names, positions = message.name, message.position
            stream = (source, tuple(names)) if self._legacy_enabled else source
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            valid = (bool(names) and len(names) == len(positions)
                     and len(set(names)) == len(names) and all(names)
                     and all(math.isfinite(float(v)) for v in positions)
                     and math.isfinite(stamp) and stamp > 0
                     and -.05 <= self._ros_now() - stamp <= .3)
            if valid and self._public_api is not None:
                # Only validation is needed here; copying an entire ROS message
                # just to discard the converted result adds control latency.
                self._public_api.convert(message.name, message.position,
                                         message.velocity, message.effort, to_model=True)
        except (AttributeError, TypeError, ValueError):
            valid, stamp = False, 0.0
        with self._lock:
            self._mux_received[source] += 1
            self._mux_last_received[source] = now
            self._mux_received_times[source].append(now)
            ready = self._ready_locked(now)
            if (not valid or stamp <= max(self._source_cutoff, self._source_stamps.get(stream, 0))
                    or not ready or not self._command_output_enabled
                    or self._replay_active or self._command_source != source
                    or (self.ownership.active and not self.ownership.allows(token))):
                self._mux_rejected += 1
                reason = ('invalid_target' if not valid else
                    'stale_or_reordered' if stamp <= max(self._source_cutoff, self._source_stamps.get(stream, 0)) else
                    'hardware_not_ready' if not ready else 'output_closed' if not self._command_output_enabled else
                    'replay_active' if self._replay_active else 'source_mismatch' if self._command_source != source else 'ownership_denied')
                self._mux_rejection_reasons[source+':'+reason] += 1
                return False
            epoch = self._output_epoch
        prepared = self._boundary.prepare(message) if self._boundary else message
        capture = getattr(self, '_publication_capture', None)
        arm_target = (self._legacy_enabled and self._boundary and prepared is not None
                      and set(prepared[0].name).intersection(self._boundary.arms))
        ticket = capture.reserve() if capture is not None and arm_target else None
        published, publication_ns, publication_monotonic = None, None, None
        try:
            with self._lock:
                if (prepared is None or epoch != self._output_epoch or not self._command_output_enabled
                        or not self._ready_locked(time.monotonic()) or self._command_source != source
                        or self._replay_active or stamp <= self._source_stamps.get(stream, 0)
                        or (self.ownership.active and not self.ownership.allows(token))):
                    self._mux_rejected += 1
                    self._mux_rejection_reasons[source+(':boundary_rejected' if prepared is None else ':authority_changed_during_prepare')] += 1
                    return False
                if self._boundary and not (self._boundary.commit(prepared, intervention=True)
                        if self.ownership.active else self._boundary.commit(prepared)):
                    self._mux_rejected += 1
                    self._mux_rejection_reasons[source+':commit_rejected'] += 1
                    return False
                self._source_stamps[stream] = stamp
                if self._legacy_enabled and self._boundary:
                    message = prepared[0]
                publish(message)
                published, publication_ns = message, time.time_ns()
                publication_monotonic = time.monotonic()
                if self.ownership.active:
                    evidence = self.ownership.sent(token, message, int(self._ros_now()*1e9))
                    if evidence is not None:
                        # This exact integer stamp links evidence to the original
                        # action in a continuous recording without guessing age.
                        evidence['command_stamp_ns'] = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
                        self._action_sink(evidence)
                self._mux_forwarded += 1
                self._mux_forwarded_times.append(time.monotonic())
                return True
        finally:
            # No snapshot copy, serialization, queue IPC or capture bookkeeping
            # executes while holding the final send/authority lock.
            if ticket is not None:
                capture.complete(ticket, published, publication_ns=publication_ns,
                                 publication_monotonic=publication_monotonic)

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
        telemetry_node = None
        telemetry_executor = None
        telemetry_executor_thread = None
        control_node = None
        control_executor = None
        control_executor_thread = None
        command_node = None
        command_executor = None
        command_executor_thread = None
        hand_node = None
        hand_executor = None
        hand_executor_thread = None
        context = None
        robot_api_boundary = None
        dataset_process = None
        try:
            import os
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
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
            # These short callbacks ran on a single executor before DAgger.
            # Humble's shared thread pool adds wait-set/GIL contention here.
            # Keep authority/feedback on their own executor so a slow command
            # preparation cannot prevent Grip from revoking the final gate.
            executor = SingleThreadedExecutor(context=context)
            self._ownership_callback_group = MutuallyExclusiveCallbackGroup()
            # The node's default callback group is mutually exclusive.  Keep
            # actuator commands out of that group so status/recording traffic
            # cannot reduce a 100 Hz control stream to a sporadic output.
            self._command_callback_group = MutuallyExclusiveCallbackGroup()
            self._hand_command_callback_group = MutuallyExclusiveCallbackGroup()
            from rclpy.callback_groups import ReentrantCallbackGroup
            self._telemetry_callback_group = ReentrantCallbackGroup()
            executor.add_node(node)
            # Dashboard callbacks must not queue behind collision checking and
            # command preparation. Count actual received messages independently.
            telemetry_node = rclpy.create_node(
                self.config.get("node_name", "hc_teleop_middleware") + "_telemetry",
                context=context,
            )
            telemetry_executor = SingleThreadedExecutor(context=context)
            telemetry_executor.add_node(telemetry_node)
            if self._mux_enabled:
                control_node = rclpy.create_node(
                    self.config.get("node_name", "hc_teleop_middleware") + "_control",
                    context=context,
                )
                control_executor = SingleThreadedExecutor(context=context)
                control_executor.add_node(control_node)
                # MutuallyExclusiveCallbackGroup alone does not isolate work
                # on a SingleThreadedExecutor. Drain actuator targets on their
                # own wait-set, independently of dataset projection/metadata.
                # Authority and feedback remain on the control executor; the
                # existing final gate still rechecks them immediately at send.
                command_node = rclpy.create_node(
                    self.config.get("node_name", "hc_teleop_middleware") + "_commands",
                    context=context,
                )
                command_executor = SingleThreadedExecutor(context=context)
                command_executor.add_node(command_node)
                # Finger targets are independent overwriteable streams. Keep
                # their callbacks from consuming the arm's depth-one reader.
                hand_node = rclpy.create_node(
                    self.config.get("node_name", "hc_teleop_middleware") + "_hands",
                    context=context,
                )
                hand_executor = SingleThreadedExecutor(context=context)
                hand_executor.add_node(hand_node)
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
                from .dataset_worker import DatasetTopicProcess
                dataset_process = DatasetTopicProcess(
                    dataset_config, domain_id,
                    self.config.get("node_name", "hc_teleop_middleware") + "_dataset",
                )
                dataset_process.start()
                self._set_status(dataset_process=dict(pid=dataset_process.process.pid, state='ready'))


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
                    control_node.create_subscription(
                        Bool, hardware_ready_topic, self._timed_callback('hardware_ready', hardware_ready_cb),
                        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
                        callback_group=self._ownership_callback_group,
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
                    command_node,
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
                                and not self._replay_active and self._command_source in {'vr', 'motion'}):
                            header = Header()
                            header.stamp = node.get_clock().now().to_msg()
                            header.frame_id = f'{self._command_source}:{self._output_epoch}'
                            permit_pub.publish(header)
                control_node.create_timer(.05, self._timed_callback('ownership_permit', publish_motion_permit), callback_group=self._ownership_callback_group)
                if self._legacy_enabled:
                    from .legacy_control import LegacyTopicControl
                    robot_api_boundary = LegacyTopicControl(
                        command_node, self, output_publisher, feedback_node=control_node,
                        hand_node=hand_node)
                    self._boundary = robot_api_boundary
                self._setup_ownership_ros(control_node, publishers, permit_pub, output_publisher,
                                          subscriptions, command_node=command_node, status_node=telemetry_node)
                # Actuator targets are overwriteable setpoints, not a FIFO.
                # If the Python executor is briefly descheduled, retain only
                # the newest target instead of replaying a stale burst later.
                command_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                )
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
                        if self._legacy_enabled and source == "motion":
                            return  # Program input is exclusively the own topic adapter.
                        self.forward_live_command(source, message, output_publisher.publish)

                    subscriptions.append(
                        command_node.create_subscription(
                            JointState,
                            topic,
                            self._timed_callback('mux:'+source, mux_callback),
                            command_qos,
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
                            if msg_type_name == 'sensor_msgs/msg/JointState':
                                # JSON has no NaN. Keep gaps empty in the UI;
                                # the independent raw-CDR recorder retains NaN.
                                for field in ('position', 'velocity', 'effort'):
                                    payload[field] = [v if math.isfinite(v) else None
                                                      for v in payload[field]]
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
                    (camera_node if is_webrtc_color and camera_node is not None else telemetry_node).create_subscription(
                        message_type, topic, self._timed_callback('telemetry:'+topic, callback), subscription_qos(item),
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
                subscriptions.append(telemetry_node.create_subscription(
                    String, "/vrdata",
                    lambda message: vr_tracker.record_message(time.monotonic()),
                    qos_profile_sensor_data,
                    callback_group=self._telemetry_callback_group,
                ))
                subscription_names.append("/vrdata")

            with self._lock:
                self._trackers = trackers
            self._set_status(state="running", subscriptions=subscription_names, error=None)
            telemetry_executor_thread = threading.Thread(
                target=telemetry_executor.spin, name="ros-telemetry-executor", daemon=True)
            telemetry_executor_thread.start()
            if control_executor is not None:
                control_executor_thread = threading.Thread(
                    target=control_executor.spin, name="ros-control-executor", daemon=True,
                )
                control_executor_thread.start()
            if command_executor is not None:
                command_executor_thread = threading.Thread(
                    target=command_executor.spin, name="ros-command-executor", daemon=True,
                )
                command_executor_thread.start()
            if hand_executor is not None:
                hand_executor_thread = threading.Thread(
                    target=hand_executor.spin, name="ros-hand-command-executor", daemon=True,
                )
                hand_executor_thread.start()
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
                    if dataset_process is not None:
                        dataset_process.check()
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
            if telemetry_executor is not None:
                telemetry_executor.shutdown(timeout_sec=5.0)
            if telemetry_executor_thread is not None:
                telemetry_executor_thread.join(timeout=5.0)
            if telemetry_executor is not None and telemetry_node is not None:
                telemetry_executor.remove_node(telemetry_node)
            if telemetry_node is not None:
                telemetry_node.destroy_node()
            if control_executor is not None:
                control_executor.shutdown(timeout_sec=5.0)
            if control_executor_thread is not None:
                control_executor_thread.join(timeout=5.0)
            if command_executor is not None:
                command_executor.shutdown(timeout_sec=5.0)
            if command_executor_thread is not None:
                command_executor_thread.join(timeout=5.0)
            if hand_executor is not None:
                hand_executor.shutdown(timeout_sec=5.0)
            if hand_executor_thread is not None:
                hand_executor_thread.join(timeout=5.0)
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
            if 'executor_thread' in locals() and executor_thread is not None:
                executor_thread.join(timeout=5.0)
            if dataset_process is not None:
                dataset_process.close()
                self._set_status(dataset_process=dict(state='stopped'))
            if robot_api_boundary is not None:
                robot_api_boundary.close()
            if command_executor is not None and command_node is not None:
                command_executor.remove_node(command_node)
            if command_node is not None:
                command_node.destroy_node()
            if hand_executor is not None and hand_node is not None:
                hand_executor.remove_node(hand_node)
            if hand_node is not None:
                hand_node.destroy_node()
            if control_executor is not None and control_node is not None:
                control_executor.remove_node(control_node)
            if control_node is not None:
                control_node.destroy_node()
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

    def _setup_ownership_ros(self, node, publishers, permit_pub, output_pub, subscriptions,
                             *, command_node=None, status_node=None):
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
                            peers.get('solver' if self._legacy_enabled else 'motion', ''), peers.get('teleop', '')))
            permit_pub.publish(Header(stamp=node.get_clock().now().to_msg(), frame_id=key))
            if phase == 'revoke' and self.ownership.state in {'OFF', 'FAULT', 'ESTOP'}:
                self._publish_hardware_enable_locked(node, publishers, False)
        self._permit_sender = send

        def ack(msg):
            try:
                value = json.loads(msg.data)
                participant, instance = value['participant'], value['instance']
                if participant not in self.ownership.required_peers or not isinstance(instance, str) or not instance:
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
            entered_ns = node.get_clock().now().nanoseconds
            try:
                value = json.loads(msg.data)
                self.receive_intervention_input(value, callback_entry_ns=entered_ns)
                tick()
            except (ValueError, KeyError, TypeError) as error:
                with self._lock:
                    self._status['last_input_callback_error'] = dict(
                        stamp_ns=node.get_clock().now().nanoseconds,
                        error=f'{type(error).__name__}: {error}')
                return

        def owned_target(msg):
            try:
                key, frame = msg.header.frame_id.split('#', 1)
                token = ControlToken.from_key(key)
            except (ValueError, AttributeError):
                return
            if self._legacy_enabled:
                # The legacy router constructs independent group targets and
                # strips the private frame there; do not clone all joints first.
                self._boundary.receive_owned(msg, token, frame_id=frame)
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

        def tick():
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
                    if self._legacy_enabled:
                        self._status['last_feedback_failure'] = dict(
                            stamp_ns=node.get_clock().now().nanoseconds,
                            **self._boundary.status())
                    ownership.fault('necessary robot feedback lost')
                ownership.advance(control_ready=self.control_ready(),
                                  manual_ready=ownership.input_valid and ownership.gripped)
                if ownership.state in {'AUTO', 'MANUAL'} and not self._command_output_enabled:
                    self._command_source = ownership.token.source
                    self._source_cutoff = ownership.cutoff_ns * 1e-9
                    self._command_output_enabled = True
                    # Normal handover fences Motion and this final sender.
                    # Hardware enable is a separate explicit safety operation;
                    # toggling it here would latch a driver stop on every Grip.
        event_cursor = 0
        state_last = 0.
        state_sequence = -1
        diagnostic_last = time.monotonic()

        def rejection_counts():
            # Source switches and closed authority routinely reject commands.
            # Report invalid commands and boundary failures separately.
            counts = Counter({reason: count for reason, count in self._mux_rejection_reasons.items()
                              if reason.partition(':')[2] in {
                                  'invalid_target', 'stale_or_reordered', 'boundary_rejected', 'commit_rejected'}})
            if self._legacy_enabled and self._boundary:
                counts.update({reason: count for reason, count in self._boundary.diagnostics.items()
                               if reason.startswith('rejected:')})
            return counts

        with self._lock:
            diagnostic_previous = rejection_counts()
        def publish_status():
            """Snapshot authority briefly; DDS/JSON work never holds its lock.

            One telemetry executor serializes publication. Events retain their
            order; current state is coalesced at the existing 10 Hz cadence.
            This observer never advances, grants or revokes robot authority.
            """
            nonlocal event_cursor, state_last, state_sequence, diagnostic_last, diagnostic_previous
            with self._lock:
                ownership = self.ownership
                events = [event for event in ownership.events
                          if event['sequence'] > event_cursor]
                now = time.monotonic()
                state = (ownership.snapshot()
                         if now-state_last >= .1 or state_sequence != ownership.event_seq
                         else None)
                sequence = ownership.event_seq
                diagnostic = None
                if now-diagnostic_last >= 1.0:
                    counts = rejection_counts()
                    delta = counts-diagnostic_previous
                    if delta:
                        diagnostic = dict(state=ownership.state, epoch=ownership.epoch, counts=dict(delta))
                        if any(key.startswith('rejected:') for key in delta):
                            diagnostic.update(recent_rejection=self._boundary.recent_rejection,
                                              rejection_stamp_ns=self._boundary.last_rejection_ns)
                    diagnostic_previous, diagnostic_last = counts, now
                    if self._legacy_enabled and self._boundary:
                        self._boundary.arm_publish_gap_max_ms = 0.0
            errors = []
            for event in events:
                events_pub.publish(String(data=json.dumps(event, allow_nan=False)))
                if (event['kind'] in {'recording_failed', 'heartbeat_timeout', 'manual_input_timeout'} or
                        (event['kind'] == 'revoked' and event['state'] in {'FAULT', 'ESTOP'})):
                    errors.append(event)
                event_cursor = event['sequence']
            if errors:
                node.get_logger().error('[control-event] '+json.dumps(errors, allow_nan=False))
            if diagnostic is not None:
                node.get_logger().error('[control-diag] '+json.dumps(diagnostic, allow_nan=False))
            if state is not None:
                state_pub.publish(String(data=json.dumps(state, allow_nan=False)))
                state_last = now
                state_sequence = sequence

        status_executor_node = status_node if status_node is not None else node
        # State is intentionally coalesced to 10 Hz above. Polling that
        # observer at 100 Hz only contends with the shared final-command lock
        # and has caused >150 ms action gaps under CPU load. 20 Hz preserves
        # prompt event/status delivery while halving lock pressure.
        status_executor_node.create_timer(.05,
            self._timed_callback('ownership_status', publish_status),
            callback_group=(self._telemetry_callback_group if status_node is not None
                            else self._ownership_callback_group))

        subscriptions.extend([
            node.create_subscription(String, '/status/control/ack', self._timed_callback('ownership_ack', ack), 100, callback_group=self._ownership_callback_group),
            node.create_subscription(String, '/events/teleop/clutch', self._timed_callback('ownership_input', semantic_input), 100, callback_group=self._ownership_callback_group),
            # Motion and the gripper adapter are independent producers. Keep
            # a bounded burst so neither overwrites the other before dispatch;
            # source age and immutable authority are still checked on receipt.
            (command_node or node).create_subscription(JointState, '/internal/control/joint_targets', self._timed_callback('owned_target', owned_target), 10,
                                     callback_group=self._command_callback_group),
        ])
        node.create_timer(.005, self._timed_callback('ownership_tick', tick), callback_group=self._ownership_callback_group)

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
                if command == 'manual_home':
                    with self._lock:
                        if (args != self.ownership.token or self.ownership.state != 'MANUAL'
                                or not self._command_output_enabled or not self._ready_locked(time.monotonic())):
                            continue
                        from std_msgs.msg import Header
                        message = Header(stamp=node.get_clock().now().to_msg(), frame_id=args.key())
                        self._publisher(node, publishers, '/internal/vr/home_request',
                                        'std_msgs/msg/Header', Header).publish(message)
                elif command == 'ownership_recover':
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
                            if not self._replay_allowed_locked(args[3]):
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
                        # This is a new actuator command, even for a legacy MCAP.
                        message.header.stamp = node.get_clock().now().to_msg()
                        boundary = self._boundary
                        prepared = boundary.prepare(message) if boundary else message
                        with self._lock:
                            if prepared is None or not self._replay_allowed_locked(args[3]):
                                continue
                            if boundary:
                                if not boundary.commit(prepared):
                                    continue
                                if self._legacy_enabled:
                                    message = prepared[0]
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
