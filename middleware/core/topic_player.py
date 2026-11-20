from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Callable

from mcap.reader import make_reader

from .topic_recorder import read_recording_metadata


logger = logging.getLogger("hc_teleop_middleware.player")


class TopicPlayer:
    """MCAP dataset player for replaying ROS 2 messages to drive simulation or real robot."""

    def __init__(
        self,
        directory: Path,
        ros_bridge_provider: Callable[[], Any],
        state_callback: Callable[[str, dict[str, Any]], None] | None = None,
        active_profile_provider: Callable[[], str] | None = None,
        active_api_metadata_provider: Callable[[], dict] | None = None,
    ):
        self.directory = directory
        self._get_ros = ros_bridge_provider
        self._state_callback = state_callback
        self._get_active_profile = active_profile_provider or (lambda: "")
        self._get_active_api_metadata = active_api_metadata_provider or (lambda: {})
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._play_generation = 0
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        self._state = "idle"
        self._filename = ""
        self._speed = 1.0
        self._loop = False
        self._selected_topics: list[str] = []
        self._topic_remap: dict[str, str] = {}
        self._current_time_sec = 0.0
        self._duration_sec = 0.0
        self._current_message_index = 0
        self._total_messages = 0
        self._progress = 0.0
        self._error: str | None = None
        self._requires_reset = False
        self._home_available = False
        self._aligning = False
        self._replay_interlock_armed = False
        self._recorded_profile_id = ""

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "state": self._state,
                "is_active": self._state in {"playing", "paused"},
                "aligning": self._aligning,
                "requires_reset": self._requires_reset,
                "home_available": self._home_available,
                "filename": self._filename,
                "speed": self._speed,
                "loop": self._loop,
                "progress": round(self._progress, 1),
                "current_time_sec": round(self._current_time_sec, 2),
                "duration_sec": round(self._duration_sec, 2),
                "current_message": self._current_message_index,
                "total_messages": self._total_messages,
                "selected_topics": list(self._selected_topics),
                "topic_remap": dict(self._topic_remap),
                "error": self._error,
                "recorded_profile_id": self._recorded_profile_id,
            }

    def _notify_state(self, event_type: str) -> None:
        callback = self._state_callback
        if callback is None:
            return
        try:
            callback(event_type, self.status())
        except Exception:
            logger.exception("Replay state callback failed for %s", event_type)

    def play(
        self,
        filename: str,
        speed: float = 1.0,
        loop: bool = False,
        mode: str = "drive",
        selected_topics: list[str] | None = None,
        topic_remap: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if self._requires_reset:
                raise RuntimeError("acknowledge replay reset before starting another replay")
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("a replay is already active")
            self._stop_locked()
            filename = Path(filename).name
            target = (self.directory / filename).resolve()
            if not target.is_file() or not target.is_relative_to(self.directory):
                raise FileNotFoundError(f"MCAP file not found: {filename}")

            recording_metadata = read_recording_metadata(target)
            active_api = self._get_active_api_metadata()
            if mode == "drive" and active_api.get("robot_api_schema"):
                if recording_metadata.get("robot_api_schema") != active_api["robot_api_schema"]:
                    raise ValueError("drive replay requires public joint metadata; historical native-name recordings need explicit offline conversion")
                if recording_metadata.get("robot_api_revision") != active_api.get("robot_api_revision"):
                    raise ValueError("drive replay mapping revision differs; review and explicitly convert the recording first")
            recorded_profile = str(recording_metadata.get("profile_id", "")).strip()
            active_profile = str(self._get_active_profile() or "").strip()
            if (
                mode == "drive"
                and recorded_profile
                and active_profile
                and recorded_profile != active_profile
            ):
                raise ValueError(
                    "drive replay profile mismatch: recording="
                    f"{recorded_profile}, active={active_profile}"
                )

            self._filename = filename
            self._recorded_profile_id = recorded_profile
            self._speed = max(0.1, min(10.0, float(speed)))
            self._loop = bool(loop)
            self._requires_reset = False
            self._error = None
            self._current_time_sec = 0.0
            self._current_message_index = 0
            self._progress = 0.0

            with target.open("rb") as stream:
                reader = make_reader(stream)
                summary = reader.get_summary()
                if not summary or not summary.statistics:
                    raise ValueError(f"invalid or empty MCAP file: {filename}")
                stats = summary.statistics
                self._total_messages = stats.message_count
                start_ns = stats.message_start_time
                end_ns = stats.message_end_time
                self._duration_sec = (end_ns - start_ns) / 1e9 if end_ns > start_ns else 0.0

                avail_topics = {c.topic for c in summary.channels.values()}

            if selected_topics is not None:
                self._selected_topics = list(selected_topics)
                self._topic_remap = dict(topic_remap or {})
            elif mode == "drive":
                # In drive mode, we isolate robot actuator commands + visualization target markers,
                # ensuring the simulator or real robot executes the joints and tracks the end-effector marker.
                drive_passthrough = {
                    "/action/ee_poses",
                    "/control/solver/ee_targets",
                    "/observation/ee_poses",
                    "/action/base/command",
                    "/action/hands/joint_targets",
                    "/action/left_hand/joint_states",
                    "/action/right_hand/joint_states",
                    "/io_teleop/target_ee_poses",
                    "/io_teleop/controller_target_ee_poses",
                    "/io_teleop/actual_ee_poses",
                }
                legacy_drive_remap = {
                    "/io_teleop/target_base_move": "/action/base/command",
                    "/io_teleop/joint_cmd_finger_left": "/action/left_hand/joint_states",
                    "/io_teleop/joint_cmd_finger_right": "/action/right_hand/joint_states",
                }
                drive_passthrough.update(legacy_drive_remap)
                if "/internal/recording/published_action" in avail_topics:
                    self._selected_topics = [t for t in avail_topics if t in drive_passthrough
                                             or t == '/internal/recording/published_action']
                    self._topic_remap = dict(legacy_drive_remap)
                    self._topic_remap['/internal/recording/published_action'] = '/action/joint_states'
                elif "/io_teleop/joint_cmd" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/io_teleop/joint_cmd"
                    ]
                    self._topic_remap = dict(legacy_drive_remap)
                    self._topic_remap["/io_teleop/joint_cmd"] = "/action/joint_states"
                elif "/action/joint_states" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/action/joint_states"
                    ]
                    self._topic_remap = dict(legacy_drive_remap)
                elif "/io_teleop/joint_cmd_arm" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/io_teleop/joint_cmd_arm"
                    ]
                    self._topic_remap = dict(legacy_drive_remap)
                elif "/control/solver/joint_targets" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/control/solver/joint_targets"
                    ]
                    self._topic_remap = dict(legacy_drive_remap)
                elif "/io_teleop/joint_states" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/io_teleop/joint_states"
                    ]
                    self._topic_remap = {"/io_teleop/joint_states": "/io_teleop/joint_cmd"}
                elif "/observation/joint_states" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/observation/joint_states"
                    ]
                    self._topic_remap = {"/observation/joint_states": "/action/joint_states"}
                else:
                    self._selected_topics = []
                    self._topic_remap = {}
            else:
                self._selected_topics = []
                self._topic_remap = {}
                if (
                    self._recorded_profile_id == "x1"
                    and "/internal/recording/published_action" in avail_topics
                ):
                    self._topic_remap["/internal/recording/published_action"] = "/action/joint_states"

            # Pause live teleop controller to prevent conflict over /action/joint_states
            ros = self._get_ros()
            if ros is not None and hasattr(ros, "begin_replay"):
                armed = ros.begin_replay()
                if armed is False:
                    raise RuntimeError(
                        "replay requires hardware_ready=true and an enabled command output"
                    )
                self._replay_interlock_armed = True
            if ros is not None and hasattr(ros, "publish"):
                ros.publish("/control/teleop/enabled", "std_msgs/msg/Bool", {"data": False})

            self._home_available = False
            self._aligning = False
            self._state = "playing"
            # A worker owns these events for its entire lifetime. A new play
            # can never clear the cancellation token of an earlier producer.
            self._stop_event = threading.Event()
            self._pause_event = threading.Event()
            self._pause_event.set()
            generation = self._play_generation
            self._thread = threading.Thread(
                target=self._run,
                args=(target, self._stop_event, self._pause_event, generation),
                name=f"TopicPlayer-{filename}",
                daemon=True,
            )
            self._thread.start()
            status = self.status()
        self._notify_state("replay_started")
        return status

    def pause(self) -> dict[str, Any]:
        changed = False
        with self._lock:
            if self._state == "playing":
                self._pause_event.clear()
                self._state = "paused"
                changed = True
            status = self.status()
        if changed:
            self._notify_state("replay_paused")
        return status

    def resume(self) -> dict[str, Any]:
        changed = False
        with self._lock:
            if self._state == "paused":
                self._pause_event.set()
                self._state = "playing"
                changed = True
            status = self.status()
        if changed:
            self._notify_state("replay_resumed")
        return status

    def set_speed(self, speed: float) -> dict[str, Any]:
        with self._lock:
            self._speed = max(0.1, min(10.0, float(speed)))
            return self.status()

    def stop(self, require_reset: bool = True) -> dict[str, Any]:
        with self._lock:
            was_active = self._state in {"playing", "paused"}
            was_waiting_for_reset = self._requires_reset
            self._stop_locked()
            needs_reset = was_active or was_waiting_for_reset
            self._state = "stopped" if needs_reset and require_reset else "idle"
            self._requires_reset = bool(require_reset and needs_reset)
            self._release_replay_interlock(require_reset=self._requires_reset)
            if was_active:
                self._home_available = self._requires_reset
            status = self.status()
        if was_active or was_waiting_for_reset:
            self._notify_state("replay_stopped")
        return status

    def acknowledge_reset(self) -> dict[str, Any]:
        with self._lock:
            changed = self._requires_reset
            self._requires_reset = False
            if self._state in {"completed", "stopped", "error"}:
                self._state = "idle"
            status = self.status()
        if changed:
            self._notify_state("replay_reset")
        return status

    def return_home(self, request_home: Callable[[bool], bool]) -> dict[str, Any]:
        """Keep the home action available through an explicit output resume."""
        with self._lock:
            if self._state in {"playing", "paused"} or self._replay_interlock_armed:
                raise ValueError("请先结束或停止回放")
            if not self._home_available:
                raise ValueError("当前没有待回初始位姿的回放")
            if not request_home(not self._requires_reset):
                raise ValueError("回初始位姿请求未被控制进程接受")
            self._requires_reset = False
            self._home_available = False
            self._state = "idle"
            status = self.status()
        self._notify_state("replay_home_requested")
        return status

    def _stop_locked(self) -> None:
        self._stop_event.set()
        self._pause_event.set()
        self._aligning = False
        self._play_generation += 1
        # Keep the actual worker reference until it exits. play() refuses a
        # second producer while an old reader/provider call is still blocked.
        # Joining here would deadlock a worker waiting for this same lock.

    def _release_replay_interlock(self, *, require_reset: bool) -> None:
        if not self._replay_interlock_armed:
            return
        ros = self._get_ros()
        if ros is not None and hasattr(ros, "end_replay"):
            ros.end_replay(require_reset=require_reset)
        self._replay_interlock_armed = False

    def _replay_current(self, generation, stop_event):
        with self._lock:
            return generation == self._play_generation and not stop_event.is_set()

    def _align_to_start(
        self,
        reader: Any,
        source_topics: list[str],
        channels: dict[int, tuple[str, str]],
        ros: Any,
        generation: int,
        stop_event: threading.Event,
        pause_event: threading.Event,
    ) -> None:
        """Smoothly move arms/hands to replay frame 0 before starting replay timing."""
        arm_raw: bytes | None = None
        hand_raws: dict[str, bytes] = {}
        want_hands = {
            t for t, _ in channels.values()
            if t in {"/action/left_hand/joint_states", "/action/right_hand/joint_states"}
        }
        first_log_time: int | None = None
        for _schema, channel, message in reader.iter_messages(topics=source_topics):
            if stop_event.is_set() or not self._replay_current(generation, stop_event):
                return
            chan_info = channels.get(channel.id)
            if chan_info is None:
                continue
            if first_log_time is None:
                first_log_time = message.log_time
            t_topic = chan_info[0]
            if t_topic == "/action/joint_states" and arm_raw is None:
                arm_raw = message.data
            elif t_topic in want_hands and t_topic not in hand_raws:
                hand_raws[t_topic] = message.data
            if arm_raw is not None and (not want_hands or len(hand_raws) == len(want_hands)):
                break
            if (
                first_log_time is not None
                and arm_raw is not None
                and message.log_time - first_log_time > 500_000_000
            ):
                break
        if arm_raw is None:
            return
        result = ros.start_replay_alignment(arm_raw, hand_raws)
        if not isinstance(result, dict) or result.get("state") != "aligning":
            return
        with self._lock:
            if not self._replay_current(generation, stop_event):
                return
            self._aligning = True
        self._notify_state("replay_aligning")
        was_paused = False
        try:
            while not stop_event.is_set():
                if not self._replay_current(generation, stop_event):
                    return
                is_paused = not pause_event.is_set()
                if is_paused and not was_paused:
                    ros.pause_replay_alignment()
                    was_paused = True
                elif not is_paused and was_paused:
                    ros.resume_replay_alignment()
                    was_paused = False
                status = ros.replay_alignment_status()
                state_val = status.get("state") if isinstance(status, dict) else "aligned"
                if state_val == "aligned":
                    break
                if state_val in {"error", "aborted"}:
                    raise RuntimeError(status.get("error") or "replay alignment failed")
                stop_event.wait(0.015)
        finally:
            with self._lock:
                self._aligning = False
        if not stop_event.is_set() and self._replay_current(generation, stop_event):
            self._notify_state("replay_aligned")

    def _run(self, target_path: Path, stop_event, pause_event, generation) -> None:
        try:
            while not stop_event.is_set():
                with target_path.open("rb") as stream:
                    reader = make_reader(stream)
                    summary = reader.get_summary()
                    if not summary or not summary.statistics:
                        raise ValueError(f"invalid or empty MCAP file: {target_path.name}")

                    stats = summary.statistics
                    total_msgs = stats.message_count
                    start_ns = stats.message_start_time
                    end_ns = stats.message_end_time
                    duration_sec = (end_ns - start_ns) / 1e9 if end_ns > start_ns else 0.0

                    schemas = {s_id: s.name for s_id, s in summary.schemas.items()}
                    # New bags can contain copies of solver inputs and outputs under
                    # their historic names. Never send those back into a live solver.
                    record_only = {m.metadata['topic'] for m in reader.iter_metadata()
                                   if m.name == 'hc_teleop.record_only' and 'topic' in m.metadata}
                    record_only.update((
                        '/control/joint_cmd',
                        '/events/recording/action_provenance',
                        '/events/recording/sample_provenance',
                    ))
                    live_x1 = "x1" in {self._recorded_profile_id, str(self._get_active_profile() or "").strip()}
                    if live_x1:
                        # X1 captures store the final joint command. Solver labels,
                        # hardware smoother output, live joint feedback, status and
                        # dataset projections are never an additional command stream.
                        record_only.update((
                            '/action/ee_poses',
                            '/control/solver/ee_targets',
                            '/control/solver/joint_targets',
                            '/observation/ee_poses',
                            '/control/vr/joint_states',
                            '/observation/joint_states',
                            '/observation/left_arm/joint_states',
                            '/observation/right_arm/joint_states',
                            '/observation/waist/joint_states',
                            '/observation/left_arm/ee_pose',
                            '/observation/right_arm/ee_pose',
                            '/observation/head/pose',
                            '/observation/headset/pose',
                            '/action/left_arm/joint_states',
                            '/action/right_arm/joint_states',
                            '/action/waist/joint_states',
                            '/action/left_arm/ee_pose',
                            '/action/right_arm/ee_pose',
                            '/action/head/pose',
                            '/status/control/ownership',
                            '/status/teleop',
                            '/status/hardware/ready',
                            '/status/base',
                            '/vrdata',
                        ))
                    with self._lock:
                        remap = dict(self._topic_remap)
                        selected = set(self._selected_topics)

                    channels: dict[int, tuple[str, str]] = {}
                    source_topics_set: set[str] = set()
                    selected_total = 0
                    msg_counts = getattr(stats, "channel_message_counts", None) or {}
                    for c_id, c in summary.channels.items():
                        msg_type = schemas.get(c.schema_id, "std_msgs/msg/String")
                        if not (
                            (
                                c.topic not in record_only
                                or (
                                    c.topic == "/observation/joint_states"
                                    and remap.get(c.topic) == "/action/joint_states"
                                )
                            )
                            and "/msg/" in msg_type
                            and (not getattr(c, "message_encoding", "") or c.message_encoding == "cdr")
                        ):
                            continue
                        target_topic = remap.get(c.topic, c.topic)
                        # Check the destination after remapping: a bag must
                        # never impersonate feedback, safety or solver nodes.
                        if live_x1 and target_topic not in {
                            "/action/joint_states",
                            "/action/base/command",
                            "/action/hands/joint_targets",
                            "/action/left_hand/joint_states",
                            "/action/right_hand/joint_states",
                        }:
                            continue
                        if not selected or (c.topic in selected) or (target_topic in selected):
                            channels[c_id] = (target_topic, msg_type)
                            source_topics_set.add(c.topic)
                            selected_total += int(msg_counts.get(c_id, 0))

                    source_topics = sorted(source_topics_set)
                    if not source_topics:
                        total_msgs = 0
                    elif selected_total > 0 or msg_counts:
                        total_msgs = selected_total
                    else:
                        total_msgs = stats.message_count

                    with self._lock:
                        if not self._replay_current(generation, stop_event):
                            return
                        self._total_messages = total_msgs
                        self._duration_sec = duration_sec

                    if (
                        source_topics
                        and any(t == "/action/joint_states" for t, _ in channels.values())
                    ):
                        ros = self._get_ros()
                        if not self._replay_current(generation, stop_event):
                            return
                        if ros is not None and getattr(ros, "supports_replay_alignment", False) is True:
                            self._align_to_start(
                                reader,
                                source_topics,
                                channels,
                                ros,
                                generation,
                                stop_event,
                                pause_event,
                            )
                            if stop_event.is_set() or not self._replay_current(generation, stop_event):
                                return

                    t0_log: int | None = None
                    bag_t0_log: int | None = None
                    prev_msg_log: int | None = None
                    t0_mono = time.monotonic()
                    active_speed = self._speed
                    msg_idx = 0

                    msg_iter = reader.iter_messages(topics=source_topics) if source_topics else ()
                    for schema, channel, message in msg_iter:
                        if stop_event.is_set():
                            break

                        chan_info = channels.get(channel.id)
                        if chan_info is None:
                            continue

                        if not pause_event.is_set():
                            pause_start = time.monotonic()
                            pause_event.wait()
                            if stop_event.is_set():
                                break
                            pause_duration = time.monotonic() - pause_start
                            if t0_log is not None:
                                t0_mono += pause_duration

                        with self._lock:
                            current_speed = self._speed

                        if t0_log is None:
                            t0_log = message.log_time
                            bag_t0_log = message.log_time
                            t0_mono = time.monotonic()
                            active_speed = current_speed
                        elif current_speed != active_speed:
                            now_anchor = time.monotonic()
                            t0_log = prev_msg_log if prev_msg_log is not None else message.log_time
                            t0_mono = now_anchor
                            active_speed = current_speed

                        prev_msg_log = message.log_time
                        log_offset = (message.log_time - t0_log) / 1e9
                        target_mono = t0_mono + (log_offset / active_speed if active_speed > 0 else 0.0)

                        while not stop_event.is_set():
                            if not pause_event.is_set():
                                pause_start = time.monotonic()
                                pause_event.wait()
                                if stop_event.is_set():
                                    break
                                pause_duration = time.monotonic() - pause_start
                                t0_mono += pause_duration
                                target_mono += pause_duration
                            remaining = target_mono - time.monotonic()
                            if remaining <= 0:
                                break
                            stop_event.wait(min(remaining, 0.02))

                        if stop_event.is_set():
                            break

                        target_topic, msg_type = chan_info
                        ros = self._get_ros()
                        # The provider may have waited while stop/reset
                        # revoked this producer. Recheck its own lease,
                        # never acquire a new replay's current fence.
                        if not self._replay_current(generation, stop_event):
                            return
                        if ros is not None and hasattr(ros, "publish_raw"):
                            ros.publish_raw(target_topic, msg_type, message.data)

                        msg_idx += 1
                        cur_time = (message.log_time - bag_t0_log) / 1e9 if bag_t0_log is not None else 0.0
                        progress = (msg_idx / total_msgs * 100.0) if total_msgs > 0 else 0.0

                        if msg_idx % 10 == 0 or msg_idx == total_msgs:
                            with self._lock:
                                if not self._replay_current(generation, stop_event):
                                    return
                                self._current_message_index = msg_idx
                                self._current_time_sec = cur_time
                                self._progress = progress

                with self._lock:
                    if not self._loop or not self._replay_current(generation, stop_event):
                        break
                    self._current_message_index = 0
                    self._current_time_sec = 0.0
                    self._progress = 0.0

            with self._lock:
                if self._replay_current(generation, stop_event):
                    self._state = "completed"
                    self._progress = 100.0
                    self._requires_reset = True
                    self._release_replay_interlock(require_reset=True)
                    self._home_available = True
            if self._replay_current(generation, stop_event):
                self._notify_state("replay_completed")
        except Exception as exc:
            with self._lock:
                if not self._replay_current(generation, stop_event):
                    return
                logger.exception("Playback failed: %s", exc)
                self._state = "error"
                self._error = str(exc)
                self._requires_reset = True
                self._release_replay_interlock(require_reset=True)
                self._home_available = True
            self._notify_state("replay_error")
