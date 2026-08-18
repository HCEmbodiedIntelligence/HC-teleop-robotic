from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Callable

from mcap.reader import make_reader


logger = logging.getLogger("hc_teleop_middleware.player")


class TopicPlayer:
    """MCAP dataset player for replaying ROS 2 messages to drive simulation or real robot."""

    def __init__(self, directory: Path, ros_bridge_provider: Callable[[], Any]):
        self.directory = directory
        self._get_ros = ros_bridge_provider
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
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

    def status(self) -> dict[str, Any]:
        with self._lock:
            is_active = self._thread is not None and self._thread.is_alive()
            return {
                "state": self._state if is_active else ("completed" if self._state == "completed" else "idle"),
                "is_active": is_active,
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
            }

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
            self._stop_locked()
            filename = Path(filename).name
            target = (self.directory / filename).resolve()
            if not target.is_file() or not target.is_relative_to(self.directory):
                raise FileNotFoundError(f"MCAP file not found: {filename}")

            self._filename = filename
            self._speed = max(0.1, min(10.0, float(speed)))
            self._loop = bool(loop)
            self._state = "playing"
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
                # ensuring the simulator executes the joints and tracks the end-effector marker.
                drive_passthrough = {
                    "/hc_teleop/target_ee_poses",
                    "/hc_teleop/controller_target_ee_poses",
                    "/hc_teleop/actual_ee_poses",
                    "/hc_teleop/target_base_move",
                    "/hc_teleop/target_finger_joints",
                }
                if "/hc_teleop/joint_cmd" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/hc_teleop/joint_cmd"
                    ]
                    self._topic_remap = {}
                elif "/hc_teleop/joint_cmd_arm" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/hc_teleop/joint_cmd_arm"
                    ]
                    self._topic_remap = {}
                elif "/hc_teleop/joint_states" in avail_topics:
                    self._selected_topics = [
                        t for t in avail_topics
                        if t in drive_passthrough or t == "/hc_teleop/joint_states"
                    ]
                    self._topic_remap = {"/hc_teleop/joint_states": "/hc_teleop/joint_cmd"}
                else:
                    self._selected_topics = []
                    self._topic_remap = {}
            else:
                self._selected_topics = []
                self._topic_remap = {}

            # Pause live teleop controller to prevent conflict over /hc_teleop/joint_cmd
            ros = self._get_ros()
            if ros is not None and hasattr(ros, "publish"):
                ros.publish("/teleop/arm/enabled", "std_msgs/msg/Bool", {"data": False})

            self._stop_event.clear()
            self._pause_event.set()
            self._thread = threading.Thread(
                target=self._run,
                args=(target,),
                name=f"TopicPlayer-{filename}",
                daemon=True,
            )
            self._thread.start()
            return self.status()

    def pause(self) -> dict[str, Any]:
        with self._lock:
            if self._state == "playing":
                self._pause_event.clear()
                self._state = "paused"
            return self.status()

    def resume(self) -> dict[str, Any]:
        with self._lock:
            if self._state == "paused":
                self._pause_event.set()
                self._state = "playing"
            return self.status()

    def set_speed(self, speed: float) -> dict[str, Any]:
        with self._lock:
            self._speed = max(0.1, min(10.0, float(speed)))
            return self.status()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            self._stop_locked()
            self._state = "idle"
            return self.status()

    def _stop_locked(self) -> None:
        self._stop_event.set()
        self._pause_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread = None

    def _run(self, target_path: Path) -> None:
        try:
            while not self._stop_event.is_set():
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
                    channels = {
                        c_id: (c.topic, schemas.get(c.schema_id, "std_msgs/msg/String"))
                        for c_id, c in summary.channels.items()
                    }

                    with self._lock:
                        self._total_messages = total_msgs
                        self._duration_sec = duration_sec

                    t0_log: int | None = None
                    t0_mono = time.monotonic()
                    msg_idx = 0

                    for schema, channel, message in reader.iter_messages():
                        if self._stop_event.is_set():
                            break

                        if not self._pause_event.is_set():
                            pause_start = time.monotonic()
                            self._pause_event.wait()
                            if self._stop_event.is_set():
                                break
                            pause_duration = time.monotonic() - pause_start
                            t0_mono += pause_duration

                        if t0_log is None:
                            t0_log = message.log_time
                            t0_mono = time.monotonic()

                        log_offset = (message.log_time - t0_log) / 1e9
                        with self._lock:
                            current_speed = self._speed

                        target_mono = t0_mono + (log_offset / current_speed if current_speed > 0 else 0)
                        now = time.monotonic()
                        if target_mono > now:
                            sleep_duration = target_mono - now
                            while sleep_duration > 0 and not self._stop_event.is_set() and self._pause_event.is_set():
                                chunk = min(sleep_duration, 0.02)
                                time.sleep(chunk)
                                sleep_duration -= chunk

                        if self._stop_event.is_set():
                            break

                        chan_info = channels.get(channel.id)
                        if chan_info:
                            orig_topic, msg_type = chan_info
                            with self._lock:
                                remap = dict(self._topic_remap)
                                selected = set(self._selected_topics)

                            target_topic = remap.get(orig_topic, orig_topic)
                            if not selected or (orig_topic in selected) or (target_topic in selected):
                                ros = self._get_ros()
                                if ros is not None and hasattr(ros, "publish_raw"):
                                    ros.publish_raw(target_topic, msg_type, message.data)

                        msg_idx += 1
                        cur_time = log_offset
                        progress = (msg_idx / total_msgs * 100.0) if total_msgs > 0 else 0.0

                        if msg_idx % 10 == 0 or msg_idx == total_msgs:
                            with self._lock:
                                self._current_message_index = msg_idx
                                self._current_time_sec = cur_time
                                self._progress = progress

                with self._lock:
                    if not self._loop or self._stop_event.is_set():
                        break
                    self._current_message_index = 0
                    self._current_time_sec = 0.0
                    self._progress = 0.0

            with self._lock:
                if not self._stop_event.is_set():
                    self._state = "completed"
                    self._progress = 100.0
        except Exception as exc:
            logger.exception("Playback failed: %s", exc)
            with self._lock:
                self._state = "error"
                self._error = str(exc)
