from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import Any, Callable

from mcap.reader import make_reader

from .binary_grip import BinaryGrip
from .recording_cdr import position_label, read_joint_state
from .robot_api import SERVO_J_COMMANDED_STATE
from .topic_recorder import read_recording_metadata


logger = logging.getLogger("hc_teleop_middleware.player")


# The eight x1 profile inputs at dbdee2498e50f0efba069fea998111a02348203b
# hash to this revision. Both arm chains and the waist have identity SI scales.
_X1_MOTION_REVISION = '0098e3161b52f196aabbaa869067e770fd9e61b4775fbc120c3e0fc75b9bc16c'
_X1_MOTION_JOINTS = {
    f'{side}_joint_{i}': f'Joint{i}_{suffix}'
    for side, suffix in (('left', 'L'), ('right', 'R')) for i in range(1, 8)
}
_X1_MOTION_JOINTS.update(zip(('waist_joint_1', 'waist_joint_2', 'waist_joint_3'),
                             ('leg_1', 'leg_2', 'zhi')))
_ARM_ACTION = '/action/joint_states'
_PUBLISHED_ACTION = '/internal/recording/published_action'


class _GripReplay:
    """Replace native hand commands independently where the MCAP contains published public bits."""

    def __init__(self, groups):
        self.sides = {}
        for side in ('left', 'right'):
            group = groups.get(side + '_hand', {})
            if group.get('command_semantics') == 'binary_grasp':
                name = side + '_grip'
                self.sides[name] = (group['native_command_topic'],
                                    BinaryGrip.from_description(name, group['grip_calibration']))

    def bits(self, data):
        # Most messages on the shared public target topic contain only arms.
        if b'left_grip\0' not in data and b'right_grip\0' not in data:
            return
        fields = read_joint_state(data)
        if len(fields.name) != len(fields.position) or len(set(fields.name)) != len(fields.name):
            raise ValueError('malformed public commanded JointState')
        for name, bit in zip(fields.name, fields.position):
            if name in self.sides:
                if bit not in (0.0, 1.0):
                    raise ValueError(f'invalid recorded {name}: expected 0 or 1')
                yield name, bit, fields.header_prefix

    def native_topics_to_replace(self, reader, available, needs_hand_evidence):
        # Only scan for a side if an alternative recorded source actually competes.
        # If no arm command exists, also confirm that the file contains a hand command.
        needed = {name for name, (topic, _) in self.sides.items()
                  if topic in available or '/io_teleop/joint_cmd_finger_' + name.split('_')[0] in available}
        found = set()
        if needed or needs_hand_evidence:
            for schema, channel, message in reader.iter_messages(topics=[SERVO_J_COMMANDED_STATE]):
                if schema and schema.name == 'sensor_msgs/msg/JointState' and channel.message_encoding == 'cdr':
                    found.update(name for name, _, _ in self.bits(message.data))
                    if found and needed <= found:
                        break
        return {self.sides[name][0] for name in found}


class _X1MotionReplay:
    """Translate verified, public-name X1 movements to the v23 model at playback."""

    def __init__(self, recording, active):
        if (recording.get('robot_api_schema') != 'hc-robot-api/v1'
                or recording.get('robot_api_revision') != _X1_MOTION_REVISION):
            raise ValueError('X1 drive replay needs a verified historical joint mapping revision')
        current = json.loads(active.get('robot_api_info') or '{}')
        joints = {j['public_name']: j for j in current.get('joints', [])}
        for public, model in _X1_MOTION_JOINTS.items():
            joint = joints.get(public, {})
            if (joint.get('model_name'), joint.get('unit'), joint.get('scale'), joint.get('offset')) != (
                    model, 'rad', 1., 0.):
                raise ValueError(f'active X1 joint mapping differs for {public}')
        self.layout = None

    def command(self, data):
        from rclpy.serialization import deserialize_message, serialize_message
        from sensor_msgs.msg import JointState
        msg = deserialize_message(data, JointState)
        if (not msg.name or len(msg.position) != len(msg.name)
                or not all(math.isfinite(q) for q in msg.position)):
            raise ValueError('X1 public drive replay requires finite recorded action positions')
        if self.layout is None or msg.name != self.layout[0]:
            if len(set(msg.name)) != len(msg.name) or any(n not in _X1_MOTION_JOINTS for n in msg.name):
                raise ValueError(f'unverified or duplicate X1 actuator names: {msg.name}')
            self.layout = (msg.name, [_X1_MOTION_JOINTS[n] for n in msg.name])
        msg.name = self.layout[1]
        return serialize_message(msg)


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
        self._replay_interlock_armed = False
        self._recorded_profile_id = ""

    def status(self) -> dict[str, Any]:
        with self._lock:
            is_active = self._thread is not None and self._thread.is_alive()
            return {
                "state": self._state,
                "is_active": is_active,
                "requires_reset": self._requires_reset,
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

            public_replay = None
            if mode == 'drive' and active_profile == 'x1' and not active_api.get('robot_api_schema'):
                schema = recording_metadata.get('robot_api_schema')
                revision = recording_metadata.get('robot_api_revision')
                if schema or revision:
                    if recorded_profile != 'x1':
                        raise ValueError('public X1 drive replay requires the recorded profile_id')
                    public_replay = _X1MotionReplay(recording_metadata, active_api)

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
                x1_default = (mode == 'drive' and selected_topics is None
                              and 'x1' in {recorded_profile, active_profile})
                grips = None
                replaced_hands = set()
                hand_only = False
                wants_public = selected_topics is None or SERVO_J_COMMANDED_STATE in selected_topics
                if (mode == 'drive' and wants_public and not public_replay
                        and not active_api.get('robot_api_schema') and SERVO_J_COMMANDED_STATE in avail_topics):
                    info = json.loads(active_api.get('robot_api_info') or '{}')
                    if info.get('recording', {}).get('name_domain') == 'public_name':
                        grips = _GripReplay(info['groups'])
                if public_replay or x1_default or grips:
                    counts = stats.channel_message_counts
                    action_topics = {c.topic for c in summary.channels.values() if counts.get(c.id, 0)
                                     and c.message_encoding == 'cdr' and c.schema_id in summary.schemas
                                     and summary.schemas[c.schema_id].name == 'sensor_msgs/msg/JointState'}

                    record_only = {m.metadata.get('topic') for m in reader.iter_metadata()
                                   if m.name == 'hc_teleop.record_only'}
                if x1_default or grips:
                    source = next((t for t in (_PUBLISHED_ACTION, '/io_teleop/joint_cmd')
                                   if t in avail_topics), _ARM_ACTION)
                    has_arm = source in action_topics and source not in record_only and not (
                        x1_default and source == _ARM_ACTION and '/events/recording/action_provenance' in avail_topics)
                if grips:
                    if SERVO_J_COMMANDED_STATE in action_topics and SERVO_J_COMMANDED_STATE not in record_only:
                        replaced_hands = grips.native_topics_to_replace(reader, action_topics, not has_arm)
                        hand_only = not has_arm and bool(replaced_hands)
                    else:
                        grips = None
                if x1_default and not hand_only:
                    if not has_arm:
                        raise ValueError('X1 drive replay needs recorded final actions; '
                                         'observations and training labels can be inspected without driving')
                    if (not recording_metadata.get('robot_api_schema')
                            and not active_api.get('robot_api_schema')):
                        # Platform-selected MCAPs can omit the session metadata.
                        # Model-name CDR stays on the normal fast path after this check.
                        first = next(reader.iter_messages(topics=[source]), None)
                        if first is None:
                            raise ValueError('X1 drive replay command stream is empty')
                        if any(n in _X1_MOTION_JOINTS for n in read_joint_state(first[2].data).name):
                            raise ValueError('public X1 command names require a verified historical mapping revision')

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
                if hand_only:
                    self._selected_topics = [t for t in avail_topics if t in drive_passthrough]
                    self._topic_remap = dict(legacy_drive_remap)
                elif "/internal/recording/published_action" in avail_topics:
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

            if grips:
                self._selected_topics = [t for t in self._selected_topics
                                         if self._topic_remap.get(t, t) not in replaced_hands]
                if SERVO_J_COMMANDED_STATE not in self._selected_topics:
                    self._selected_topics.append(SERVO_J_COMMANDED_STATE)
            if public_replay:
                # Old DAgger public recordings use /action for final commands.
                # Ordinary X1 captures may also store feedback/NaN training labels there.
                source = (_PUBLISHED_ACTION if _PUBLISHED_ACTION in avail_topics else _ARM_ACTION)
                if (source not in action_topics or source in record_only
                        or (source == _ARM_ACTION and (
                            '/events/recording/action_provenance' in avail_topics
                            or recording_metadata.get('capture_mode') not in {'full_session', 'corrections_only'}))):
                    raise ValueError('X1 drive replay needs recorded final actions; this file has no verified command stream')
                if (source not in self._selected_topics or self._topic_remap.get(source, source) != _ARM_ACTION
                        or any(self._topic_remap.get(t, t) == _ARM_ACTION
                               for t in self._selected_topics if t != source)):
                    raise ValueError('public X1 drive replay requires the original final command stream')
                if any(self._topic_remap.get(t, t) in {'/action/left_hand/joint_states',
                           '/action/right_hand/joint_states', '/action/hands/joint_targets'}
                       for t in self._selected_topics):
                    raise ValueError('public X1 drive replay has no verified historical hand mapping')
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

            # A worker owns these events for its entire lifetime. A new play
            # can never clear the cancellation token of an earlier producer.
            self._stop_event = threading.Event()
            self._pause_event = threading.Event()
            self._pause_event.set()
            generation = self._play_generation
            self._state = "playing"
            self._thread = threading.Thread(
                target=self._run,
                args=(target, self._stop_event, self._pause_event, generation,
                      frozenset(self._selected_topics), self._topic_remap, public_replay, grips),
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

    def _stop_locked(self) -> None:
        self._stop_event.set()
        self._pause_event.set()
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

    def _run(self, target_path: Path, stop_event, pause_event, generation,
             selected, remap, public_replay, grips) -> None:
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
                    channels = {}
                    for channel_id, c in summary.channels.items():
                        dst = remap.get(c.topic, c.topic)
                        if c.topic in record_only and not (c.topic == "/observation/joint_states" and dst == _ARM_ACTION):
                            continue
                        if selected and c.topic not in selected and dst not in selected:
                            continue
                        if grips and c.topic == SERVO_J_COMMANDED_STATE:
                            if schemas.get(c.schema_id) == 'sensor_msgs/msg/JointState' and c.message_encoding == 'cdr':
                                channels[channel_id] = (SERVO_J_COMMANDED_STATE, schemas[c.schema_id])
                            continue
                        if live_x1 and dst not in {
                            _ARM_ACTION, "/action/base/command", "/action/hands/joint_targets",
                            "/action/left_hand/joint_states", "/action/right_hand/joint_states",
                        }:
                            continue
                        msg_type = schemas.get(c.schema_id, "std_msgs/msg/String")
                        if "/msg/" in msg_type and (not c.message_encoding or c.message_encoding == "cdr"):
                            channels[channel_id] = (dst, msg_type)

                    with self._lock:
                        if not self._replay_current(generation, stop_event):
                            return
                        self._total_messages = total_msgs
                        self._duration_sec = duration_sec

                    t0_log: int | None = None
                    t0_mono = time.monotonic()
                    msg_idx = 0

                    for schema, channel, message in reader.iter_messages():
                        if stop_event.is_set():
                            break

                        if not pause_event.is_set():
                            pause_start = time.monotonic()
                            pause_event.wait()
                            if stop_event.is_set():
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
                            while sleep_duration > 0 and not stop_event.is_set() and pause_event.is_set():
                                chunk = min(sleep_duration, 0.02)
                                time.sleep(chunk)
                                sleep_duration -= chunk

                        if stop_event.is_set():
                            break

                        chan_info = channels.get(channel.id)
                        if chan_info and grips and channel.topic == SERVO_J_COMMANDED_STATE:
                            for name, bit, header in grips.bits(message.data):
                                hand_topic, grip = grips.sides[name]
                                raw = position_label(header, grip.command_names, grip.encode(bit))
                                ros = self._get_ros()
                                if not self._replay_current(generation, stop_event):
                                    return
                                if ros is None or ros.publish_raw(hand_topic, chan_info[1], raw) is False:
                                    raise RuntimeError(f'control bridge refused replay of {name}')
                        elif chan_info:
                            target_topic, msg_type = chan_info
                            ros = self._get_ros()
                            # The provider may have waited while stop/reset
                            # revoked this producer. Recheck its own lease,
                            # never acquire a new replay's current fence.
                            if not self._replay_current(generation, stop_event):
                                return
                            if ros is not None and hasattr(ros, "publish_raw"):
                                data = message.data
                                if public_replay and target_topic == _ARM_ACTION:
                                    if msg_type != 'sensor_msgs/msg/JointState':
                                        raise ValueError('invalid X1 drive action message type')
                                    data = public_replay.command(data)
                                accepted = ros.publish_raw(target_topic, msg_type, data)
                                if public_replay and target_topic == _ARM_ACTION and accepted is False:
                                    raise RuntimeError('X1 drive replay command was refused by the control bridge')

                        msg_idx += 1
                        cur_time = log_offset
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
            self._notify_state("replay_error")
