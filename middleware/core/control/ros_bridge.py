"""ROS 2 控制桥接门面（RosBridge）：持有核心状态、锁与安全闸门，将领域逻辑委托给协作者。"""
from __future__ import annotations

from collections import Counter, deque
import math
import queue
import re
import sys
import threading
import time
from typing import Any, Callable

from .command_dispatcher import CommandDispatcher, _copy_actor_message
from .control_ownership import ControlToken
from .handover_hold import (
    _HANDOVER_HOLD_SPEC,
    _HOLD_ATTR_BY_SOURCE,
    HandoverHoldController,
    _GripReturnHold,
)
from .ownership_ros_wire import setup_ownership_ros
from .replay_bridge import ReplayBridgeController
from .ros_bridge_runner import RosBridgeRunner
from .topic_health import DEFAULT_TOPIC_STANDARDS, TopicHealthTracker

EventCallback = Callable[[dict[str, Any], list[str]], None]


class RosBridge:
    """在后台线程运行 rclpy 并提供线程安全的桥接操作与控制权状态门控。"""

    COMMAND_CHANNELS = ("arms", "right_hand", "left_hand", "other")
    supports_replay_alignment = True

    def __init__(
        self,
        config: dict[str, Any],
        on_event: EventCallback,
        on_frame: Callable[[str, Any], None] | None = None,
        camera_topic: str = "",
        camera_topics: list[str] | None = None,
    ):
        self.config = config
        self._control_actor_mode = config.get("_control_actor_mode") is True
        self._actor_thread_ident = None
        self._actor_dispatching = False
        self._actor_dispatch_epoch = None
        self._actor_dispatch_gripper_revision = None
        self._actor_pumping = False
        self._actor_callback_count = 0
        self._actor_targets: dict[Any, Any] = {}
        self._actor_replay: dict[Any, Any] = {}
        self._actor_urgent: deque[Any] = deque(maxlen=8)
        self._actor_counts: Counter = Counter()
        self._actor_flush_commands = None
        self._actor_wake = None
        self._qos_event_observer = None
        self._public_api = None
        self._legacy_enabled = bool(config.get("legacy_control_profile"))
        if config.get("robot_api_profile"):
            from ..robot.robot_api import RobotApiProfile

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
        self._telemetry_count_lock = threading.Lock()
        self._telemetry_messages = 0
        self._callback_timings: dict[str, Any] = {}
        self._slow_callbacks: deque[dict[str, Any]] = deque(maxlen=32)
        self._command_timing_sequence = 0
        self._command_stage_timings: dict[str, Any] = {}
        self._slow_command_stages: deque[dict[str, Any]] = deque(maxlen=32)
        self._control_contention: deque[dict[str, Any]] = deque(maxlen=64)
        self._command_contention_last_ns = 0
        self._contention_thread = None
        self._boundary = None
        self._return_hold = None
        self._takeover_hold = None
        self._permit_sender = None
        self._action_sink = lambda evidence: None

        from .control_ownership import ControlOwnership

        self.ownership = ControlOwnership(
            lock=self._lock,
            ros_clock=lambda: int(self._ros_now() * 1e9),
            revoke=self._ownership_revoke,
            permit=self._ownership_permit,
            required_peers=("solver", "teleop")
            if self._legacy_enabled
            else ("motion", "teleop"),
            recording_enabled=config.get("intervention", {}).get(
                "automatic_recording", True
            ),
            right_gripper_override=self._legacy_enabled,
            begin_return_hold=self._begin_return_hold,
            begin_takeover_hold=self._begin_takeover_hold,
        )
        self._trackers: dict[str, TopicHealthTracker] = {}
        mux_config = config.get("command_mux", {})
        self._mux_enabled = bool(mux_config.get("enabled", True))
        self._command_source = str(mux_config.get("source", "vr"))
        self._mux_output_topic = str(
            mux_config.get("output_topic", "/action/joint_states")
        )
        self._command_output_enabled = True
        self._hardware_ready = False
        self._hardware_ready_last = -math.inf
        self._ready_timeout = float(mux_config.get("hardware_ready_timeout_sec", 0.5))
        if not math.isfinite(self._ready_timeout) or self._ready_timeout <= 0:
            raise ValueError("hardware_ready_timeout_sec must be positive and finite")
        self._ros_now = time.time
        self._control_clock = None
        self._source_cutoff = -math.inf
        self._source_stamps: dict[Any, float] = {}
        self._output_epoch = 0
        self._hardware_enable_sent_epoch = None
        self._replay_active = False
        self._replay_home_available = False
        self._replay_home_publisher = None
        self._replay_alignment = None
        self._replay_alignment_status: dict[str, Any] = {
            "state": "idle",
            "error": "",
            "max_diff_rad": 0.0,
        }
        self._mux_received = {"vr": 0, "exoskeleton": 0, "motion": 0}
        self._mux_last_received = {"vr": 0.0, "exoskeleton": 0.0, "motion": 0.0}
        self._mux_received_times = {
            "vr": deque(maxlen=300),
            "exoskeleton": deque(maxlen=300),
            "motion": deque(maxlen=300),
        }
        self._mux_forwarded_times: deque[float] = deque(maxlen=300)
        self._mux_channel_received = dict.fromkeys(self.COMMAND_CHANNELS, 0)
        self._mux_channel_forwarded = dict.fromkeys(self.COMMAND_CHANNELS, 0)
        self._mux_channel_received_times = {
            channel: deque(maxlen=300) for channel in self.COMMAND_CHANNELS
        }
        self._mux_channel_forwarded_times = {
            channel: deque(maxlen=300) for channel in self.COMMAND_CHANNELS
        }
        self._command_channel_boundary: Any = object()
        self._command_channel_cache: dict[Any, str] = {}
        self._mux_forwarded = 0
        self._mux_rejected = 0
        self._mux_rejection_reasons: Counter = Counter()
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
        self._thread = threading.Thread(
            target=self._run, name="ros-bridge", daemon=True
        )
        self._thread.start()
        if self._control_actor_mode:
            return
        self._contention_thread = threading.Thread(
            target=self._watch_control_contention,
            name="ros-latency-diagnostic",
            daemon=True,
        )
        self._contention_thread.start()

    def stop(self) -> None:
        self._stop.set()
        wake = getattr(self, "_actor_wake", None)
        if callable(wake):
            wake()
        if self._thread is not None:
            self._thread.join(timeout=5)
        if self._contention_thread is not None:
            self._contention_thread.join(timeout=1)

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._telemetry_count_lock:
            telemetry_messages = self._telemetry_messages
        ownership = self.ownership.snapshot()
        boundary = self._boundary if self._legacy_enabled else None
        boundary_wall_ns = None
        if boundary is not None:
            read_clock = getattr(boundary, "clock_now_ns", None)
            boundary_wall_ns = (
                read_clock()
                if callable(read_clock)
                else boundary.node.get_clock().now().nanoseconds
            )
        with self._lock:
            def recent_rate(samples: deque[float] | list[float]) -> float:
                recent = [stamp for stamp in samples if now - stamp <= 2.0]
                if len(recent) < 2 or now - recent[-1] > 1.0:
                    return 0.0
                return round((len(recent) - 1) / (recent[-1] - recent[0]), 1)

            result = dict(self._status)
            result["messages"] = telemetry_messages
            if self._control_actor_mode:
                result["control_actor"] = dict(
                    enabled=True,
                    thread_id=self._actor_thread_ident,
                    pending_targets=len(self._actor_targets),
                    pending_replay=len(self._actor_replay),
                    qos_events=(
                        self._qos_event_observer.status()
                        if self._qos_event_observer is not None
                        else None
                    ),
                    **self._actor_counts,
                )
            with self._callback_timing_lock:
                result["callback_timings"] = {
                    name: dict(value) for name, value in self._callback_timings.items()
                }
                result["slow_callbacks"] = list(self._slow_callbacks)
                result["command_stage_timings"] = {
                    name: dict(value)
                    for name, value in self._command_stage_timings.items()
                }
                result["slow_command_stages"] = list(self._slow_command_stages)
                result["control_contention_samples"] = list(self._control_contention)
            result["ownership"] = ownership
            if self._legacy_enabled and self._boundary:
                result["legacy_control"] = self._boundary.status(
                    wall_ns=boundary_wall_ns
                )
            trackers = list(self._trackers.items())
            mux_config = self.config.get("command_mux", {})
            ready = self._ready_locked(now)
            result["command_mux"] = {
                "enabled": self._mux_enabled,
                "output_enabled": self._command_output_enabled,
                "hardware_ready": ready,
                "hardware_ready_age": (
                    now - self._hardware_ready_last
                    if math.isfinite(self._hardware_ready_last)
                    else None
                ),
                "replay_active": self._replay_active,
                "source": self._command_source,
                "vr_topic": mux_config.get("vr_topic", "/control/vr/joint_states"),
                "exoskeleton_topic": mux_config.get(
                    "exoskeleton_topic", "/control/exoskeleton/joint_states"
                ),
                "output_topic": mux_config.get("output_topic", "/action/joint_states"),
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
                "rate_scope": {
                    "received_hz": "per_source_combined_joint_channels",
                    "forwarded_hz": "all_sources_combined_joint_channels",
                    "channels": "per_joint_channel_all_sources",
                },
                "rejected": self._mux_rejected,
                "rejection_reasons": dict(self._mux_rejection_reasons),
            }
            channel_samples = {
                channel: (
                    self._mux_channel_received[channel],
                    self._mux_channel_forwarded[channel],
                    list(self._mux_channel_received_times[channel]),
                    list(self._mux_channel_forwarded_times[channel]),
                )
                for channel in self.COMMAND_CHANNELS
            }
        result["command_mux"]["channels"] = {}
        for channel, (received, forwarded, receives, publishes) in channel_samples.items():
            recent_publishes = [stamp for stamp in publishes if now - stamp <= 2.0]
            gaps = [
                (later - earlier) * 1000
                for earlier, later in zip(recent_publishes, recent_publishes[1:])
            ]
            result["command_mux"]["channels"][channel] = dict(
                received=received,
                forwarded=forwarded,
                received_hz=recent_rate(receives),
                forwarded_hz=recent_rate(publishes),
                max_publication_gap_ms=round(max(gaps), 3) if gaps else None,
                publication_gap_window_sec=2.0,
                last_received_age=round(now - receives[-1], 3) if receives else None,
                last_forwarded_age=round(now - publishes[-1], 3) if publishes else None,
            )
        result["topic_health"] = {
            topic: tracker.status(now) for topic, tracker in trackers
        }
        return result

    def get_topic_health(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            trackers = list(self._trackers.items())
        return {topic: tracker.status(now) for topic, tracker in trackers}

    def publish(self, topic: str, msg_type: str, data: dict[str, Any]) -> bool:
        if str(topic) == "/control/joint_cmd" or "/msg/" not in str(msg_type):
            return False
        if self.ownership.active and self._ownership_control_topic(topic):
            return False
        if self._is_final_joint_command(topic):
            return False
        return self._enqueue("publish", (topic, msg_type, data))

    def request_manual_home(self) -> bool:
        with self._lock:
            if (
                not self._legacy_enabled
                or self.ownership.state != "MANUAL"
                or self.ownership.token.source != "vr"
                or not self._command_output_enabled
                or not self._ready_locked(time.monotonic())
            ):
                return False
            return self._enqueue("manual_home", self.ownership.token)

    def _raw_replay_destination_allowed(self, topic: str) -> bool:
        return ReplayBridgeController.raw_replay_destination_allowed(topic)

    def publish_raw(self, topic: str, msg_type: str, raw_data: bytes) -> bool:
        if not self._raw_replay_destination_allowed(topic):
            return False
        if str(topic) == "/control/joint_cmd" or "/msg/" not in str(msg_type):
            return False
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
        return ReplayBridgeController.replay_allowed_locked(self, epoch)

    def begin_replay(self) -> bool:
        return ReplayBridgeController.begin_replay(self)

    def end_replay(self, require_reset: bool = True) -> None:
        ReplayBridgeController.end_replay(self, require_reset=require_reset)

    def _clear_replay_alignment_locked(self, reason: str = "") -> None:
        ReplayBridgeController.clear_alignment_locked(self, reason)

    def start_replay_alignment(
        self, arm_raw: bytes, hand_raws: dict[str, bytes] | None = None
    ) -> dict[str, Any]:
        return ReplayBridgeController.start_alignment(self, arm_raw, hand_raws)

    def replay_alignment_status(self) -> dict[str, Any]:
        return ReplayBridgeController.alignment_status(self)

    def pause_replay_alignment(self) -> dict[str, Any]:
        return ReplayBridgeController.pause_alignment(self)

    def resume_replay_alignment(self) -> dict[str, Any]:
        return ReplayBridgeController.resume_alignment(self)

    def cancel_replay_alignment(self) -> dict[str, Any]:
        return ReplayBridgeController.cancel_alignment(self)

    def _replay_home_allowed_locked(self, *, output_resumed: bool) -> bool:
        return ReplayBridgeController.replay_home_allowed_locked(
            self, output_resumed=output_resumed
        )

    def request_replay_home(self, *, output_resumed: bool = False) -> bool:
        return ReplayBridgeController.request_replay_home(
            self, output_resumed=output_resumed
        )

    def _is_final_joint_command(self, topic: str) -> bool:
        mux_config = self.config.get("command_mux", {})
        if self._legacy_enabled and self._boundary:
            hands = self._boundary.teleop["grippers"]
            if str(topic) in {hands[s]["finger_topic"] for s in ("right", "left")}:
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
                raise ValueError("intervention session owns source selection")
            if self._command_source != source:
                self._revoke_locked()
                self._command_output_enabled = False
            self._command_source = source
        return self._enqueue("control_source", source)

    def set_command_output_enabled(self, enabled: bool) -> bool:
        with self._lock:
            if self.ownership.active:
                if enabled:
                    raise ValueError("intervention resume must return to HOLD")
                self.ownership.hold()
            self._revoke_locked()
            self._command_output_enabled = bool(enabled)
            epoch = self._output_epoch
        return self._enqueue("command_output", (bool(enabled), epoch))

    def _actor_clear_targets(self, reason: str = "revoked") -> None:
        with self._lock:
            self._actor_counts["revoked_targets"] += len(self._actor_targets) + len(
                self._actor_replay
            )
            self._actor_targets.clear()
            self._actor_replay.clear()

    def _revoke_locked(self, *, keep_replay: bool = False) -> None:
        self._return_hold = None
        self._takeover_hold = None
        self._replay_home_available = False
        self._clear_replay_alignment_locked(
            "replay alignment aborted by safety or revoke"
        )
        if not keep_replay:
            self._replay_active = False
        self._source_cutoff = self._ros_now()
        self._output_epoch += 1
        if self._control_actor_mode:
            self._actor_clear_targets()

    @staticmethod
    def _ownership_control_topic(topic: str) -> bool:
        return str(topic) != "/teleop/recording_state" and str(topic).startswith(
            ("/control/", "/internal/", "/teleop/", "/action/", "/hc_teleop/")
        )

    def _ownership_revoke(self) -> None:
        self._command_output_enabled = False
        self._revoke_locked()

    def _begin_handover_hold(self, mode: str) -> None:
        HandoverHoldController.begin_hold(self, mode)

    def _begin_return_hold(self) -> None:
        self._begin_handover_hold("return")

    def _begin_takeover_hold(self) -> None:
        self._begin_handover_hold("takeover")

    def _return_hold_allowed_locked(self, hold: _GripReturnHold) -> bool:
        return HandoverHoldController.hold_allowed_locked(self, hold)

    def _tick_handover_hold(self, attr: str) -> None:
        HandoverHoldController.tick_hold(self, attr)

    def _tick_return_hold(self) -> None:
        self._tick_handover_hold("_return_hold")

    def _tick_takeover_hold(self) -> None:
        self._tick_handover_hold("_takeover_hold")

    def _tick_handover_holds(self) -> None:
        self._tick_return_hold()
        self._tick_takeover_hold()

    def _ownership_permit(
        self, token: ControlToken, phase: str, cutoff_ns: int, active: bool
    ) -> None:
        if not active:
            self._command_source = "vr"
            self._enqueue("control_source", "vr")
            self._enqueue(
                "publish", ("/control/teleop/enabled", "std_msgs/msg/Bool", {"data": True})
            )
        if self._permit_sender is not None:
            self._permit_sender(token, phase, cutoff_ns, active)

    def control_ready(self) -> bool:
        with self._lock:
            return bool(
                self._ready_locked(time.monotonic())
                and self._boundary
                and self._boundary.handover_ready()
            )

    def _handover_control_ready_locked(self) -> bool:
        return HandoverHoldController.handover_control_ready_locked(self)

    def recover_intervention(self) -> None:
        with self._lock:
            control = self.ownership
            if control.state not in {"FAULT", "ESTOP"}:
                control.recover(control_ready=self.control_ready())
                return
            if (
                (control.input_valid and control.gripped)
                or not self._boundary
                or not self._boundary.handover_ready()
            ):
                raise ValueError("恢复前需要新鲜静止反馈，并松开正在使用的 Grip")
            if not getattr(self, "_recovery", None):
                self._recovery = (control.token, time.monotonic())
                self._enqueue("ownership_recover", self._recovery)

    def _timed_callback(self, name: str, callback: Callable) -> Callable:
        def measured(*args: Any, **kwargs: Any) -> Any:
            if self._control_actor_mode:
                self._actor_callback_count += 1
            started = time.monotonic_ns()
            try:
                return callback(*args, **kwargs)
            finally:
                elapsed_ms = (time.monotonic_ns() - started) / 1e6
                with self._callback_timing_lock:
                    value = self._callback_timings.setdefault(
                        name, dict(count=0, max_ms=0.0)
                    )
                    value.update(
                        count=value["count"] + 1,
                        last_ms=elapsed_ms,
                        max_ms=max(value["max_ms"], elapsed_ms),
                    )
                    if elapsed_ms >= 20.0:
                        self._slow_callbacks.append(
                            dict(
                                name=name,
                                elapsed_ms=elapsed_ms,
                                finished_ns=time.time_ns(),
                                thread=threading.current_thread().name,
                            )
                        )

        measured._hc_timing_name = name
        return measured

    def _sample_control_contention(self) -> dict[str, Any] | None:
        diagnostic = self.ownership.input_diagnostics
        now = time.time_ns()
        input_slow = (
            diagnostic.get("last_age_ms", 0) >= 100.0
            and now - diagnostic.get("last_callback_ns", 0) <= 2_000_000_000
        )
        recent_command = (
            self._slow_command_stages[-1] if self._slow_command_stages else None
        )
        command_slow = bool(
            recent_command and now - recent_command["stamp_ns"] <= 2_000_000_000
        )
        if not input_slow and not command_slow:
            return None
        if not input_slow:
            if now - self._command_contention_last_ns < 1_000_000_000:
                return None
            self._command_contention_last_ns = now
        match = re.search(r"owner=(\d+)", repr(self._lock))
        owner = int(match.group(1)) if match else 0
        if not owner and not input_slow:
            return None
        frame = sys._current_frames().get(owner) if owner else None
        stack = []
        while frame is not None and len(stack) < 8:
            stack.append(
                dict(
                    file=frame.f_code.co_filename,
                    function=frame.f_code.co_name,
                    line=frame.f_lineno,
                )
            )
            frame = frame.f_back
        group = getattr(self, "_ownership_callback_group", None)
        entity = getattr(group, "_active_entity", None)
        callback = getattr(entity, "callback", None)
        return dict(
            stamp_ns=now,
            input_age_ms=diagnostic.get("last_age_ms"),
            trigger="slow_command" if command_slow else "input_age",
            owner_thread=next(
                (t.name for t in threading.enumerate() if t.ident == owner), None
            ),
            owner_stack=stack,
            active_ownership_callback=getattr(
                callback,
                "_hc_timing_name",
                type(entity).__name__ if entity is not None else None,
            ),
        )

    def _watch_control_contention(self) -> None:
        while not self._stop.wait(0.01):
            sample = self._sample_control_contention()
            if sample is not None:
                with self._callback_timing_lock:
                    self._control_contention.append(sample)

    def receive_intervention_input(
        self, value: dict[str, Any], *, callback_entry_ns: int | None = None
    ) -> bool:
        if (
            not isinstance(value["device_instance"], str)
            or not value["device_instance"]
            or type(value["sequence"]) is not int
            or value["sequence"] < 0
            or type(value["stamp_ns"]) is not int
            or value["stamp_ns"] <= 0
        ):
            raise ValueError("invalid semantic input identity, sequence or timestamp")
        before_lock_ns = time.monotonic_ns()
        gripper_target = None
        with self._lock:
            acquired_ns = time.monotonic_ns()
            control = self.ownership
            pause = value.get("pause_requested") is True
            resume = value.get("resume_requested") is True
            trigger = value.get("right_gripper_trigger")
            accepted = control.input(
                device_instance=value["device_instance"],
                sequence=value["sequence"],
                valid=value["valid"] is True,
                gripped=value["gripped"] is True,
                stamp_ns=value["stamp_ns"],
                pause_requested=pause,
                resume_requested=resume,
                right_gripper_trigger=trigger,
            )
            if callback_entry_ns is not None:
                control.input_diagnostics.update(
                    callback_entry_source_ns=value["stamp_ns"],
                    callback_entry_age_ms=(callback_entry_ns - value["stamp_ns"]) / 1e6,
                    callback_lock_wait_ms=(acquired_ns - before_lock_ns) / 1e6,
                    callback_input_work_ms=(time.monotonic_ns() - acquired_ns) / 1e6,
                )
            if accepted and resume and not pause and control.active:
                if control.state == "HOLD" and not self._actor_output_blocked():
                    control.start_auto(
                        recording_ready=control.capture_ready(),
                        control_ready=self.control_ready(),
                    )
                elif control.state in {"FAULT", "ESTOP"} and not self._actor_output_blocked():
                    self.recover_intervention()
            if (
                accepted
                and self._legacy_enabled
                and control.right_gripper_token is not None
                and self._boundary is not None
                and value["stamp_ns"] > control.right_gripper_cutoff_ns
            ):
                gripper_target = (
                    trigger,
                    value["stamp_ns"],
                    control.right_gripper_token,
                )
        if gripper_target is not None:
            self._boundary.send_right_gripper(*gripper_target)
        return accepted

    def _ready_locked(self, now: float) -> bool:
        if self._hardware_ready and now - self._hardware_ready_last > self._ready_timeout:
            self._hardware_ready = False
            self._command_output_enabled = False
            self._revoke_locked()
            self.ownership.fault("hardware ready timeout")
        return self._hardware_ready

    def receive_hardware_ready(self, ready: bool, now: float | None = None) -> None:
        with self._lock:
            current = time.monotonic() if now is None else now
            self._ready_locked(current)
            if not ready and self._hardware_ready:
                self._command_output_enabled = False
                self._revoke_locked()
                self.ownership.fault("hardware ready lost")
            self._hardware_ready = bool(ready)
            self._hardware_ready_last = current

    def _actor_checkpoint(self, *, allow_snapshot: bool = True) -> None:
        if not self._control_actor_mode or self._actor_pumping:
            return
        self._actor_pumping = True
        try:
            pump = (
                getattr(self, "_actor_pump_checkpoint", None)
                if not allow_snapshot
                else None
            )
            if callable(pump):
                pump(allow_snapshot)
            else:
                pump = getattr(self, "_actor_pump", None)
                if callable(pump):
                    pump()
            with self._lock:
                self._ready_locked(time.monotonic())
                self.ownership.watchdog()
                if (
                    self.ownership.state in {"AUTO", "MANUAL"}
                    and self._boundary
                    and not self._boundary.feedback_fresh()
                ):
                    self.ownership.fault("necessary robot feedback lost")
        finally:
            self._actor_pumping = False

    def _actor_output_blocked(self) -> bool:
        hook = getattr(self, "_actor_authority_blocked", None)
        return self._control_actor_mode and callable(hook) and hook()

    def _spin_actor_callbacks(self, executor: Any) -> None:
        urgent = getattr(self, "_actor_urgent_check", None)
        wait_sec = 0.01 if callable(getattr(self, "_actor_wake", None)) else 0.001
        planner = self._replay_alignment
        if planner is not None:
            wait_sec = planner.wait_sec(time.monotonic(), wait_sec)
        for attr in ("_return_hold", "_takeover_hold"):
            hold = getattr(self, attr)
            if hold is not None:
                wait_sec = min(
                    wait_sec,
                    max(0.0, (hold.next_send_ns - self.ownership.clock()) * 1e-9),
                )
        for index in range(32):
            before = self._actor_callback_count
            executor.spin_once(timeout_sec=wait_sec if index == 0 else 0.0)
            if callable(urgent):
                urgent()
            if self._stop.is_set() or self._actor_callback_count == before:
                break
        self._actor_checkpoint(allow_snapshot=False)

    def _count_live_input_locked(self, source: str, channel: str, now: float) -> None:
        CommandDispatcher.count_live_input_locked(self, source, channel, now)

    def _live_target_allowed_locked(
        self, source: str, channel: str, token: Any, stamp: float
    ) -> bool:
        return CommandDispatcher.live_target_allowed_locked(
            self, source, channel, token, stamp
        )

    @staticmethod
    def _complete_publication_capture(
        capture: Any,
        ticket: Any,
        published: Any,
        wire_cdr: bytes | None,
        publication_ns: int | None,
        publication_monotonic: float | None,
    ) -> None:
        CommandDispatcher.complete_publication_capture(
            capture, ticket, published, wire_cdr, publication_ns, publication_monotonic
        )

    def _record_forwarded_metrics_locked(
        self, message: Any, publication_monotonic: float
    ) -> None:
        CommandDispatcher.record_forwarded_metrics_locked(
            self, message, publication_monotonic
        )

    def _select_replay_publisher(
        self,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
    ) -> Any:
        return ReplayBridgeController.select_replay_publisher(
            self, node, publishers, topic, msg_type_name, message_type
        )

    def _publish_replay_joint_message(
        self,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
        message: Any,
        *,
        epoch: int,
        convert_public: bool = True,
    ) -> bool:
        return ReplayBridgeController.publish_replay_joint_message(
            self,
            node,
            publishers,
            topic,
            msg_type_name,
            message_type,
            message,
            epoch=epoch,
            convert_public=convert_public,
        )

    def _tick_replay_alignment(
        self,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        get_message: Callable[[str], Any] | None = None,
    ) -> None:
        ReplayBridgeController.tick_alignment(self, node, publishers, get_message)

    def _queue_actor_target(
        self, source: str, message: Any, publish: Callable, token: Any
    ) -> bool:
        return CommandDispatcher.queue_actor_target(self, source, message, publish, token)

    def _drain_actor_targets(self) -> None:
        CommandDispatcher.drain_actor_targets(self)

    def forward_live_command(
        self, source: str, message: Any, publish: Callable, token: Any = None
    ) -> bool:
        return CommandDispatcher.forward_live_command(
            self, source, message, publish, token
        )

    @staticmethod
    def _command_checkpoint(timing: dict[str, Any] | None, stage: str) -> None:
        if timing is not None:
            now = time.monotonic_ns(), time.thread_time_ns()
            previous = timing["_previous"]
            timing[stage] = dict(
                wall_ms=(now[0] - previous[0]) / 1e6,
                cpu_ms=(now[1] - previous[1]) / 1e6,
            )
            timing["_previous"] = now

    @staticmethod
    def _command_instant(timing: dict[str, Any] | None) -> tuple[int, int] | None:
        return (time.monotonic_ns(), time.thread_time_ns()) if timing is not None else None

    @staticmethod
    def _command_interval(
        timing: dict[str, Any] | None,
        stage: str,
        before: tuple[int, int] | None,
        after: tuple[int, int] | None,
    ) -> None:
        if timing is not None and before is not None and after is not None:
            timing[stage] = dict(
                wall_ms=(after[0] - before[0]) / 1e6,
                cpu_ms=(after[1] - before[1]) / 1e6,
            )

    def _count_telemetry_message(self) -> None:
        with self._telemetry_count_lock:
            self._telemetry_messages += 1

    def _command_channel(self, names: Any) -> str:
        return CommandDispatcher.command_channel(self, names)

    def _forward_live_command(
        self,
        source: str,
        message: Any,
        publish: Callable,
        token: Any = None,
        *,
        timing: dict[str, Any] | None = None,
    ) -> bool:
        return CommandDispatcher.forward_live_command_body(
            self, source, message, publish, token, timing=timing
        )

    def _enqueue(self, command: str, data: Any) -> bool:
        item: Any = (command, data)
        if self._control_actor_mode:
            fence_hook = getattr(self, "_actor_command_fence", None)
            item = (
                command,
                data,
                fence_hook() if callable(fence_hook) else self._output_epoch,
            )
            with self._lock:
                if command == "stop":
                    self._actor_urgent.append(item)
                    wake = self._actor_wake
                    if callable(wake):
                        wake()
                    return True
                if command == "replay_raw":
                    self._actor_replay[(data[0], data[1])] = item
                    wake = self._actor_wake
                    if callable(wake):
                        wake()
                    return True
        try:
            self._commands.put_nowait(item)
            wake = self._actor_wake
            if callable(wake):
                wake()
            return True
        except queue.Full:
            with self._lock:
                self._status["dropped_commands"] += 1
            return False

    def _set_status(self, **changes: Any) -> None:
        with self._lock:
            self._status.update(changes)

    def _run(self) -> None:
        RosBridgeRunner(self).run()

    def _setup_ownership_ros(
        self,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        permit_pub: Any,
        output_pub: Any,
        subscriptions: list[Any],
        *,
        command_node: Any = None,
        status_node: Any = None,
    ) -> None:
        setup_ownership_ros(
            self,
            node,
            publishers,
            permit_pub,
            output_pub,
            subscriptions,
            command_node=command_node,
            status_node=status_node,
        )

    def _actor_negative_command(self, command: str, args: Any) -> bool:
        return CommandDispatcher.actor_negative_command(self, command, args)

    def _actor_queue_item_allowed(self, item: tuple[Any, ...]) -> bool:
        return CommandDispatcher.actor_queue_item_allowed(self, item)

    def _drain_commands(
        self,
        node: Any,
        publishers: dict[tuple[str, str], Any],
        get_message: Callable[[str], Any],
        set_message_fields: Callable[[Any, dict[str, Any]], None],
        wait: float = 0.0,
    ) -> None:
        CommandDispatcher.drain_commands(
            self, node, publishers, get_message, set_message_fields, wait=wait
        )

    def _publish_hardware_enable_locked(
        self, node: Any, publishers: dict[tuple[str, str], Any], enabled: bool
    ) -> None:
        CommandDispatcher.publish_hardware_enable_locked(self, node, publishers, enabled)

    @staticmethod
    def _publisher(
        node: Any,
        publishers: dict[tuple[str, str], Any],
        topic: str,
        msg_type_name: str,
        message_type: Any,
    ) -> Any:
        return CommandDispatcher.publisher(
            node, publishers, topic, msg_type_name, message_type
        )
