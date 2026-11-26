"""Convert recorded ROS CDR MCAP into an official LeRobot v3 dataset.

ROS decoding stays in the middleware interpreter. The official dataset writer runs
in an isolated interpreter (LEROBOT_PYTHON, or .venv-lerobot/bin/python).
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import os
from functools import lru_cache
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
INSTALL_COMMAND = "./tools/setup_lerobot.sh"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / 'upload_client') not in sys.path:
    sys.path.insert(0, str(ROOT / 'upload_client'))
from middleware.core.export_progress import ExportProgress, progress_path
from middleware.core.recording_time import CAMERA_MAX_AGE_NS, JOINT_MAX_AGE_NS, epoch_grid
from middleware.core.x1_recording import X1_BODY_JOINTS
from middleware.core.recording_cameras import camera_feature
from middleware.core.recording_grips import HAND_COMMANDS, SIDES, RecordingGrips, hand_selection


def writer_python() -> Path:
    return Path(os.environ.get("LEROBOT_PYTHON", str(ROOT / ".venv-lerobot/bin/python")))


def ensure_export_available() -> str:
    """Cheap dependency check; deliberately does not import torch."""
    executable = writer_python()
    if not executable.is_file():
        raise RuntimeError(f"LeRobot 导出环境未安装，请运行 {INSTALL_COMMAND}")
    check_env = {key: value for key, value in os.environ.items() if key not in ('PYTHONPATH', 'PYTHONHOME')}
    check = subprocess.run([str(executable), "-c", "import importlib.util; import sys; sys.exit(0 if all(importlib.util.find_spec(x) for x in ('lerobot','av','pyarrow','torch')) else 1)"], capture_output=True, text=True, env=check_env)
    if check.returncode:
        raise RuntimeError(f"LeRobot 导出依赖不完整，请运行 {INSTALL_COMMAND}")
    return str(executable)


@lru_cache(maxsize=256)
def _joint_indices(source: tuple[str, ...], target: tuple[str, ...]) -> tuple[int, ...]:
    by_name = {name: idx for idx, name in enumerate(source)}
    if len(by_name) != len(source):
        raise ValueError("JointState names/positions malformed")
    return tuple(by_name.get(name, -1) for name in target)


def joint_vector(message: Any, names: list[str]) -> list[float]:
    if len(message.name) != len(message.position):
        raise ValueError("JointState names/positions malformed")
    indices = _joint_indices(tuple(message.name), tuple(names))
    if -1 in indices:
        raise ValueError("JointState changed its joint set")
    result = [float(message.position[i]) for i in indices]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("JointState contains non-finite positions")
    return result


def _recording_body(info):
    recording = info.get('recording')
    if not recording or 'body' not in recording:
        return None
    if recording.get('schema') != 'hc-servoj-recording/v1':
        raise ValueError('Unsupported recorded ServoJ data contract')
    body = recording['body']
    model, public, units = (body.get(k, []) for k in ('model_names', 'joint_names', 'units'))
    if (not model or len(model) != len(public) or len(model) != len(units)
            or len(set(model)) != len(model) or len(set(public)) != len(public)
            or any(not isinstance(n, str) or not n for n in (*model, *public))
            or any(u not in ('rad', 'm') for u in units)):
        raise ValueError('Recorded ServoJ body names and units do not form a one-to-one mapping')
    return recording


class _PublishedBody:
    """Translate recorded public command snapshots into the frozen model-name layout."""
    def __init__(self, info):
        body = info['recording']['body']
        joints = {j['public_name']: j for j in info['joints']}
        self.mapping = {}
        self.layouts = {}
        for name, model in zip(body['joint_names'], body['model_names']):
            j = joints[name]
            if j['model_name'] != model:
                raise ValueError(f'{name}: inconsistent captured public/model mapping')
            self.mapping[name] = model, j['scale'], j['offset']

    def select(self, message):
        names = tuple(message.name)
        if len(names) != len(message.position):
            raise ValueError('Malformed recorded commanded JointState')
        if names not in self.layouts:
            if len(set(names)) != len(names):
                raise ValueError('Duplicate recorded commanded JointState axis')
            entries = [(i, *self.mapping[n]) for i, n in enumerate(names) if n in self.mapping]
            self.layouts[names] = (tuple(m for _, m, _, _ in entries), entries,
                                   len(entries) == len(names) and all(s == 1 and b == 0 for _, _, s, b in entries))
        model, entries, direct = self.layouts[names]
        if not model:
            return None
        positions = message.position if direct else [s*message.position[i]+b for i, _, s, b in entries]
        return SimpleNamespace(name=model, position=positions)


def _check_x1_units(feature, declaration, action_sources):
    units = declaration.get('units')
    if not units:
        return
    names = declaration['names']
    if len(units) != len(names):
        raise ValueError(f'{feature}: joint names and units have different lengths')
    known = dict.fromkeys(X1_BODY_JOINTS, 'rad')
    if feature == 'action':
        for source in action_sources:
            known.update(zip(source['names'], source['units']))
    aliases = {'radian': 'rad', 'radians': 'rad', 'meter': 'm', 'meters': 'm',
               'metre': 'm', 'metres': 'm', '1': 'unitless', 'dimensionless': 'unitless'}
    for name, unit in zip(names, units):
        if name in known and aliases.get(unit, unit) != known[name]:
            raise ValueError(f'{feature}.{name}: X1 source uses {known[name]}, frozen schema declares {unit}')


def _project_joint_stream(samples, names, correction, axis_times=None):
    target = tuple(names)
    held = [0.0] * len(target)
    last_observed = [-1] * len(target) if axis_times is not None else None
    last_sent = [-1] * len(target)
    values, validity = [], []
    for stamp, message in samples:
        if len(message.name) != len(message.position):
            raise ValueError("JointState names/positions malformed")
        indices = _joint_indices(tuple(message.name), target)
        present = [False] * len(target)
        for i, index in enumerate(indices):
            if index < 0:
                continue
            position = message.position[index]
            if math.isnan(position):
                # An explicit inactive axis revokes a preceding correction command.
                last_sent[i] = -1
                continue
            if not math.isfinite(position):
                raise ValueError("JointState contains infinite positions")
            held[i] = position
            present[i] = True
            if last_observed is not None:
                last_observed[i] = stamp
            if correction:
                last_sent[i] = stamp
        if axis_times is not None:
            axis_times.append((stamp, last_observed.copy()))
        values.append((stamp, held.copy()))
        validity.append((stamp, last_sent.copy() if correction else present))
    return values, validity


def latest_sample(samples: list[tuple[int, Any]], stamp: int, max_age_ns: int | None):
    index = bisect.bisect_right(samples, stamp, key=lambda sample: sample[0]) - 1
    if index < 0 or (max_age_ns is not None and stamp - samples[index][0] > max_age_ns):
        return None
    return samples[index]


# A header stamp may trail its MCAP receive time by queueing/transport delay,
# but must not lead it by more than clock jitter.
HEADER_CLOCK_MAX_LAG_NS = 2_000_000_000
HEADER_CLOCK_MAX_LEAD_NS = 20_000_000


def header_stamp_ns(message: Any) -> int | None:
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) or None


def apply_source_clock(samples: list[tuple[int, Any]], headers: list[int | None], *, live=False):
    """Re-time one stream on its sensor header clock, or keep MCAP log_time.

    Receive time carries each subscriber's queueing jitter (tens of ms on X1),
    which misaligns color frames with joint states. The header clock is used for
    a whole stream only if every message has a stamp consistent with its receive
    time, so one stream never mixes clocks.
    """
    if live and samples and len(headers) == len(samples):
        # A running collector cannot retroactively switch an entire stream
        # to arrival time when just one input has a bad header.
        result = []
        fallback = 0
        for (received, value), header in zip(samples, headers):
            valid = header is not None and -HEADER_CLOCK_MAX_LEAD_NS <= received - header <= HEADER_CLOCK_MAX_LAG_NS
            fallback += not valid
            result.append((header if valid else received, value))
        # Python's stable sort preserves input order for equal source stamps.
        result.sort(key=lambda item: item[0])
        clock = 'log_time' if fallback == len(samples) else ('mixed' if fallback else 'header')
        return clock, result
    if not samples or len(headers) != len(samples) or any(
            header is None or not -HEADER_CLOCK_MAX_LEAD_NS <= log_time - header <= HEADER_CLOCK_MAX_LAG_NS
            for (log_time, _), header in zip(samples, headers)):
        return 'log_time', samples
    retimed = [(header, value) for (_, value), header in zip(samples, headers)]
    retimed.sort(key=lambda sample: sample[0])
    if any(b[0] <= a[0] for a, b in zip(retimed, retimed[1:])):
        # Repeated stamps are republished/merged messages, not distinct samples.
        return 'log_time', samples
    return 'header', retimed


def required_stream(key: str) -> bool:
    return key in ('observation.state', 'action') or key.startswith('observation.images.')


def sample_max_age(key: str) -> int:
    return CAMERA_MAX_AGE_NS if key.startswith('observation.images.') else JOINT_MAX_AGE_NS


def synchronized_episodes(streams: dict[str, list], fps: int, audit: dict | None = None,
                          time_range: tuple[int, int] | None = None, max_ages: dict | None = None,
                          *, use_epoch_grid: bool = False):
    """One recording, one episode: retain every grid point, with causal values.

    Missing and stale values are represented by validity/availability/age fields
    in the manifest. Never borrow future samples or delete time to close a gap.
    """
    required = {key: samples for key, samples in streams.items() if required_stream(key)}
    if not required or any(not samples for key, samples in required.items()
                           if not (use_epoch_grid and key == 'action')):
        raise ValueError('No required state/action/camera samples')
    nonempty = [samples for samples in streams.values() if samples]
    start, end = time_range or (min(samples[0][0] for samples in nonempty),
                                max(samples[-1][0] for samples in nonempty))
    if end < start:
        raise ValueError('Invalid recording time range')
    if audit is not None:
        audit.update({'required_keys': list(required), 'grid_start_log_time_ns': start,
                      'grid_end_log_time_ns': end, 'candidate_frames': 0,
                      'retained_frames': 0, 'rejected_frames': 0, 'rejected_by_stream': {},
                      'invalid_frames': 0, 'invalid_required_frames': 0, 'invalid_by_stream': {}})
    def grid_rows():
        ticks = (epoch_grid(start, end, fps) if use_epoch_grid else
                 (start + round(i * 1_000_000_000 / fps)
                  for i in range((end - start) * fps // 1_000_000_000 + 1)))
        # Samples are already sorted on their selected clock. Only move indices
        # forward; each grid row references the existing joint/image sample.
        indices = dict.fromkeys(streams, -1)
        for stamp in ticks:
            selected = {}
            for key, samples in streams.items():
                i = indices[key]
                while i + 1 < len(samples) and samples[i + 1][0] <= stamp:
                    i += 1
                indices[key] = i
                selected[key] = samples[i] if i >= 0 else None
            invalid = [key for key, sample in selected.items()
                       if sample is None or stamp - sample[0] > (max_ages or {}).get(key, sample_max_age(key))]
            if audit is not None:
                audit['candidate_frames'] += 1
                audit['retained_frames'] += 1
                audit['invalid_frames'] += bool(invalid)
                audit['invalid_required_frames'] += any(key in required for key in invalid)
                for key in invalid:
                    audit['invalid_by_stream'][key] = audit['invalid_by_stream'].get(key, 0) + 1
            yield stamp, selected

    yield grid_rows() if use_epoch_grid else list(grid_rows())


def preserved_episode(streams, fps, audit):
    """Shared plans preserve source state timestamps/count; never synthesize frames."""
    from humanoid_upload.common import UploadError
    timeline = streams['observation.state']
    stamps = [sample[0] for sample in timeline]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise UploadError('CAPTURE_STRUCTURE_MISMATCH', '状态时间戳不是严格递增')
    if len(stamps) > 1:
        actual_hz = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
        audit['source_frequency_hz'] = actual_hz
    rows = []
    for stamp, _ in timeline:
        selected = {key: latest_sample(samples, stamp, sample_max_age(key)) for key, samples in streams.items()}
        missing = [key for key, value in selected.items()
                   if value is None and (streams[key] or required_stream(key))]
        if missing:
            raise UploadError('CAPTURE_SOURCE_GAPS_UNSUPPORTED', '当前 LeRobot 导出无法无损表达这些字段的帧内缺失：' + ', '.join(missing) + '；原始 MCAP 已保留')
        rows.append((stamp, selected))
    audit.update(candidate_frames=len(rows), retained_frames=len(rows), rejected_frames=0,
                 invalid_frames=0, invalid_required_frames=0, invalid_by_stream={},
                 frequency_policy='PRESERVE', timeline='original_state_log_time')
    return [rows]


def decode_rgb(message: Any, compressed: bool):
    import cv2
    import numpy as np
    if compressed:
        if "depth" in str(message.format).lower():
            raise ValueError("Depth images are not RGB observations")
        image = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Cannot decode compressed camera image")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    encoding = str(message.encoding).lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f"Unsupported RGB encoding: {encoding}")
    rows = np.frombuffer(bytes(message.data), np.uint8).reshape(message.height, message.step)
    image = rows[:, :message.width * channels].reshape(message.height, message.width, channels)
    if encoding == 'mono8':
        return np.repeat(image, 3, axis=2)
    if encoding.startswith('bgr'):
        return image[:, :, [2, 1, 0]]
    return image[:, :, :3]


def prepare_mcap(source: Path, staging: Path, fps: int, task: str, robot_type: str, progress: ExportProgress | None = None, action_sources: list | None = None, record_hands=None, training_schema='auto') -> Path:
    import cv2
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from .topic_recorder import read_recording_metadata
    from .recording_cdr import read_event
    from .action_origins import ActionOriginIndex, append_origin_range
    from .recording_action_labels import PROVENANCE_TOPIC
    metadata = read_recording_metadata(source)
    if metadata.get('capture_snapshot'):
        from humanoid_upload.capture_plan import validate_snapshot
        validate_snapshot(json.loads(metadata['capture_snapshot']), recording_id=metadata.get('recording_id'))
    from .capture_formats import lerobot_snapshot
    snapshot = lerobot_snapshot(metadata)
    if snapshot:
        from humanoid_upload.capture_plan import validate_snapshot
        expected_id = metadata.get('recording_id')
        if metadata.get('capture_snapshots'):
            expected_id += '-lerobot'
        validate_snapshot(snapshot, source_format='LEROBOT_V3', recording_id=expected_id)

    progress = progress or ExportProgress(None)
    progress.update("读取 MCAP", 0, force=True)
    action_sources = action_sources or []
    action_map = {item["topic"]: item for item in action_sources}
    extra_keys = {topic: "action.source." + str(index) for index, topic in enumerate(action_map)}
    joint_keys = {'/observation/joint_states': 'observation.state', '/action/joint_states': 'action', **extra_keys}
    max_ages = {extra_keys[topic]: int(item["max_age_sec"]*1e9) for topic, item in action_map.items()}
    streams: dict[str, list] = {}
    names = None
    dimensions = {}
    source_topics = {}
    source_frame_ids = {}
    unstated_joint_frames = set()
    message_classes = {}
    numeric_names = {}
    numeric_topics = {
        '/observation/left_arm/ee_pose': 'observation.left_arm.ee_pose',
        '/observation/right_arm/ee_pose': 'observation.right_arm.ee_pose',
        '/observation/headset/pose': 'observation.headset.pose',
        '/observation/base/pose': 'observation.base.pose',
        '/action/left_arm/ee_pose': 'action.left_arm.ee_pose',
        '/action/right_arm/ee_pose': 'action.right_arm.ee_pose',
        '/action/base/velocity': 'action.base.velocity',
    }
    raw_cameras = {}
    correction = None
    session = None
    session_sidecar = source.with_suffix('.session.json')
    if session_sidecar.is_file():
        session = json.loads(session_sidecar.read_text())
    sidecar = source.with_suffix('.correction.json')
    if sidecar.is_file():
        correction = json.loads(sidecar.read_text())
    profile = metadata.get('profile_id') or metadata.get('robot_name') or robot_type
    ordinary = not (snapshot or session or correction or metadata.get('capture_mode'))
    robot_info = json.loads(metadata.get('robot_api_info') or '{}') if ordinary else {}
    live_recording = _recording_body(robot_info)
    from .historical_x1_projection import HistoricalLabels, historical_body
    historical = historical_body(training_schema, live_recording, ordinary and not (
        metadata.get('capture_snapshot') or metadata.get('capture_snapshots')), metadata)
    if live_recording:
        profile = robot_info.get('profile_id') or profile
    if historical:
        live_recording = historical
    ordinary_x1 = profile == 'x1' and ordinary
    requested_hands = hand_selection(record_hands)
    mapped_hands = {HAND_COMMANDS[t] for t in action_map if t in HAND_COMMANDS}
    sides = (requested_hands if record_hands is not None else
             tuple(side for side in SIDES if side in mapped_hands))
    grips = (RecordingGrips(sides, robot_info, profile)
             if sides and ordinary and (live_recording or ordinary_x1) else None)
    if requested_hands and ordinary and grips is None:
        raise ValueError('This recording has no verified hand calibration')
    camera_topics = None
    if live_recording is not None:
        numeric_topics.clear()
        if not historical:
            camera_topics = {camera['topic'] for camera in live_recording.get('cameras', ())}
    public_body = _PublishedBody(robot_info) if live_recording and not historical else None
    public_command_topic = robot_info.get('endpoints', {}).get('commanded_joint_states')
    if public_body and public_command_topic:
        joint_keys[public_command_topic] = 'public_command'
    published_topic = '/internal/recording/published_action'
    if live_recording and ordinary_x1:
        joint_keys[published_topic] = 'published'

    origin_index = ActionOriginIndex() if ordinary_x1 else None
    first_observations = {}
    header_stamps: dict[str, list[int | None]] = {}
    source_clocks: dict[str, str] = {}
    deferred_image_topics = {}
    recording_start = recording_end = None
    label_log, label_headers = [], []
    with source.open('rb') as handle:
        reader = make_reader(handle)
        # Prefer compressed over raw when both versions of a camera are recorded.
        summary = reader.get_summary()
        topics = {channel.topic for channel in summary.channels.values()} if summary else set()
        message_total = summary.statistics.message_count if summary and summary.statistics else 0
        # Modern X1 files may carry both training labels and a byte-identical copy
        # of the target actually published. Decode the command once on that path.
        intervention_topics = {'/events/control/intervention', '/events/recording/sample_provenance'}
        defer_compressed = live_recording is not None and summary and not intervention_topics.intersection(topics)
        if ('published' in joint_keys.values() and published_topic in topics
                and not intervention_topics.intersection(topics)):
            joint_keys.pop('/action/joint_states')
        for message_index, (schema, channel, message) in enumerate(reader.iter_messages(log_time_order=True), 1):
            if recording_start is None:
                recording_start = message.log_time
            recording_end = message.log_time
            progress.update('读取 MCAP', 25 * message_index / max(1, message_total) if message_total else 0,
                            message_index, message_total, '条消息')
            topic = channel.topic
            if topic == PROVENANCE_TOPIC and origin_index is not None:
                origin_index.add_event(message.log_time, message.data, channel.message_encoding, schema.name if schema else '')
                continue
            if topic in ('/events/control/intervention', '/events/recording/sample_provenance'):
                event = read_event(message.data, channel.message_encoding, schema.name if schema else '')
                if event.get('kind') == 'session_close' and event.get('capture_mode') == 'full_session':
                    session = session or event
                elif event.get('kind') == 'manual_close' and event.get('capture_mode') != 'full_session':
                    correction = correction or event
                elif event.get('role') == 'initial_observation':
                    first_observations[event['topic']] = event
            if channel.message_encoding != 'cdr' or schema is None:
                continue
            kind = schema.name
            if historical and topic == '/action/joint_states' and topic not in joint_keys:
                # The emitted wire stream is decoded separately. Only the unchanged
                # label's header is needed to validate its provenance for the audit.
                from .capture_window import header_stamp_ns as cdr_header_ns
                label_log.append((message.log_time, None))
                label_headers.append(cdr_header_ns(message.data, kind))
                continue
            joint_key = joint_keys.get(topic)
            is_camera = kind in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage') and 'depth' not in topic.lower()
            if is_camera and camera_topics is not None and topic not in camera_topics:
                continue
            numeric_key = numeric_topics.get(topic)
            hand_tracks = grips.by_topic.get(topic, ()) if grips and kind == 'sensor_msgs/msg/JointState' else ()
            if joint_key is None and not is_camera and numeric_key is None and not hand_tracks:
                continue
            if is_camera and topic + '/compressed' in topics and (camera_topics is None or topic + '/compressed' in camera_topics):
                continue
            if is_camera and defer_compressed and kind == 'sensor_msgs/msg/CompressedImage':
                from .recording_images import compressed_fields, rgb_shape
                key = camera_feature(topic)
                if key in source_topics and source_topics[key] != topic:
                    raise ValueError(f'Camera feature name collision: {topic}')
                frame_id, stamp, payload = compressed_fields(message.data)
                shape = rgb_shape(payload)
                if key not in dimensions:
                    (staging / key).mkdir(exist_ok=True)
                elif dimensions[key] != shape:
                    raise ValueError(f'Camera resolution changed: {topic}')
                if topic in source_frame_ids and source_frame_ids[topic] != frame_id:
                    raise ValueError(f'Reference frame changed during recording: {topic}')
                dimensions[key], source_frame_ids[topic] = shape, frame_id
                deferred_image_topics[key] = source_topics[key] = topic
                samples = streams.setdefault(key, [])
                samples.append((message.log_time, len(samples)))
                header_stamps.setdefault(key, []).append(stamp)
                continue
            if kind not in message_classes:
                message_classes[kind] = get_message(kind)
            decoded = deserialize_message(message.data, message_classes[kind])
            header = getattr(decoded, 'header', None)
            header_ns = header_stamp_ns(decoded)
            if header is not None:
                frame_id = header.frame_id
                if not frame_id and kind == 'sensor_msgs/msg/JointState' and decoded.name:
                    # Older X1 publishers omit the frame on some named joint layouts.
                    unstated_joint_frames.add(topic)
                else:
                    previous = source_frame_ids.setdefault(topic, frame_id)
                    if previous != frame_id:
                        raise ValueError(f'Reference frame changed during recording: {topic} ({previous!r} to {frame_id!r})')
            if hand_tracks:
                for track in hand_tracks:
                    track.observe(decoded, message.log_time, header_ns)
                if joint_key is None:
                    continue
            if joint_key == 'public_command':
                decoded = public_body.select(decoded)
                if decoded is None:  # e.g. left_grip only
                    continue
            if numeric_key:
                if kind == 'geometry_msgs/msg/PoseStamped':
                    pose = decoded.pose
                    values = [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                    numeric_names[numeric_key] = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
                elif kind == 'geometry_msgs/msg/TwistStamped':
                    twist = decoded.twist
                    values = [twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z]
                    numeric_names[numeric_key] = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']
                else:
                    raise ValueError(f'Unexpected numeric message type on {topic}: {kind}')
                if not all(math.isfinite(value) for value in values):
                    raise ValueError(f'Non-finite numeric observation: {topic}')
                streams.setdefault(numeric_key, []).append((message.log_time, values))
                header_stamps.setdefault(numeric_key, []).append(header_ns)
                source_topics[numeric_key] = topic
            elif joint_key:
                if kind != 'sensor_msgs/msg/JointState':
                    raise ValueError(f"Expected JointState on {topic}")
                if names is None and joint_key == 'observation.state':
                    names = list(decoded.name)
                    if not names:
                        raise ValueError('Feedback has no joint names')
                raw_cameras.setdefault(joint_key, []).append((message.log_time, decoded))
                header_stamps.setdefault(joint_key, []).append(header_ns)
                source_topics[joint_key] = topic
            else:
                key = camera_feature(topic)
                if key in source_topics and source_topics[key] != topic:
                    raise ValueError(f'Camera feature name collision: {topic}')
                image = decode_rgb(decoded, kind.endswith('CompressedImage'))
                shape = list(image.shape)
                if key in dimensions and dimensions[key] != shape:
                    raise ValueError(f"Camera resolution changed: {topic}")
                dimensions[key] = shape
                camera_dir = staging / key
                camera_dir.mkdir(exist_ok=True)
                filename = camera_dir / f'{len(streams.get(key, [])):09d}.png'
                if not cv2.imwrite(str(filename), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
                    raise RuntimeError(f'Cannot save staging image {filename}')
                streams.setdefault(key, []).append((message.log_time, str(filename)))
                header_stamps.setdefault(key, []).append(header_ns)
                source_topics[key] = topic
    # Re-time before aggregating partial actions: held values must follow
    # source order on the clock that is used for alignment.
    # A correction's range/actions use HC send time (log_time). Task snapshots
    # also freeze a PRESERVE contract: keep the original state log_time samples
    # and causal lookup clock instead of moving the first state before action.
    if session or correction:
        if training_schema == 'shared':
            raise ValueError('Shared training projection requires an ordinary recording')
        live_recording = None
    wire_actions = raw_cameras.pop('published', [])
    wire_headers = header_stamps.pop('published', [])
    source_topics.pop('published', None)
    public_actions = raw_cameras.pop('public_command', [])
    public_headers = header_stamps.pop('public_command', [])
    source_topics.pop('public_command', None)
    use_public = bool(live_recording and public_actions)
    use_wire = bool(live_recording and ordinary_x1 and (
        published_topic in topics or wire_actions or (not historical and PROVENANCE_TOPIC in topics)))
    if historical:
        label_samples = raw_cameras.get('action', label_log)
        label_headers = header_stamps.get('action', label_headers)
        historical_labels = HistoricalLabels(label_samples, label_headers, origin_index, apply_source_clock)
    if use_public:
        raw_cameras['action'], header_stamps['action'] = public_actions, public_headers
        source_topics['action'] = public_command_topic
    elif use_wire:
        raw_cameras['action'], header_stamps['action'] = wire_actions, wire_headers
        source_topics['action'] = published_topic
    elif historical:
        raw_cameras['action'], header_stamps['action'] = historical_labels.published(label_samples, label_headers)
        source_topics['action'] = '/action/joint_states'
    elif live_recording and public_command_topic in topics:
        # Hand-only or empty public command channel documents no body target.
        raw_cameras['action'], header_stamps['action'] = [], []
        source_topics['action'] = public_command_topic
    elif live_recording:
        raw_cameras.setdefault('action', [])
        source_topics.setdefault('action', '/action/joint_states')
    original_actions = raw_cameras.get('action', ())
    for collection in (() if correction or snapshot else (streams, raw_cameras)):
        for key in list(collection):
            source_clocks[key], collection[key] = apply_source_clock(
                collection[key], header_stamps.get(key, []), live=live_recording is not None)
    if snapshot:
        # Preserve required auxiliary numeric columns even when their device
        # source is entirely absent. The schema's presence_and_validity policy
        # encodes this as zero values with source_available/source_valid = 0.
        topic_by_numeric = {key: topic for topic, key in numeric_topics.items()}
        feature_sources = snapshot['effective_adapter']['feature_sources']
        for name, declaration in snapshot['output']['features'].items():
            key = feature_sources.get(name, name)
            topic = topic_by_numeric.get(key)
            if (topic and declaration.get('required', True)
                    and declaration.get('missing') == 'presence_and_validity'):
                streams.setdefault(key, [])
                numeric_names.setdefault(key, list(declaration.get('names') or []))
                source_topics.setdefault(key, topic)
    if not names or (live_recording is None and not raw_cameras.get('action')):
        raise ValueError('LeRobot requires recorded /observation/joint_states and /action/joint_states; missing grid samples are explicitly masked')
    # Legacy/native X1 telemetry has model names. Its public-name intervention
    # recordings and explicitly frozen capture contracts have separate layouts.
    tagged_x1 = (metadata.get('profile_id') or metadata.get('robot_name')) == 'x1'
    native_x1 = profile == 'x1' and (set(X1_BODY_JOINTS[:14]).issubset(names)
                                     or (tagged_x1 and not set(X1_BODY_JOINTS).isdisjoint(names)))
    if historical and not native_x1:
        raise ValueError('Historical shared projection requires native X1 model joint names on /observation/joint_states')
    canonical_x1 = ordinary_x1 and native_x1 and not (session or correction)
    fixed_body = live_recording is not None or canonical_x1
    if grips and not fixed_body:
        if requested_hands and ordinary and not (session or correction):
            raise ValueError('Unable to identify a native X1 hand layout in this recording')
        grips = None
    if fixed_body and mapped_hands:
        for topic in HAND_COMMANDS.keys() & action_map.keys():
            key = extra_keys.pop(topic)
            raw_cameras.pop(key, None)
            source_clocks.pop(key, None)
            source_topics.pop(key, None)
        action_sources = [source for source in action_sources if source['topic'] not in HAND_COMMANDS]
    if grips:
        grips.finish(lambda samples, headers: apply_source_clock(samples, headers, live=live_recording is not None))
    extra_names = [name for item in action_sources for name in item['names']]
    frozen = {}
    frozen_mask = False
    if snapshot:
        for feature, declaration in snapshot['output']['features'].items():
            key = feature_sources.get(feature, feature)
            if key == 'action.valid':
                frozen_mask = True
            if key in ('action', 'observation.state') and declaration.get('names'):
                if native_x1:
                    _check_x1_units(key, declaration, action_sources)
                frozen[key] = list(declaration['names'])

    if fixed_body:
        names = list(live_recording['body']['model_names'] if live_recording else X1_BODY_JOINTS)
        action_names = names
    else:
        names = frozen.get('observation.state', names)
        if 'action' in frozen:
            if not set(extra_names).issubset(frozen['action']):
                raise ValueError('configured action sources are outside the frozen action feature')
            action_names = [name for name in frozen['action'] if name not in extra_names]
        else:
            observed = dict.fromkeys(name for _, msg in raw_cameras['action']
                                     for name, value in zip(msg.name, msg.position)
                                     if not correction or not math.isnan(value))
            if not observed:
                raise ValueError('Action has no commanded joint positions')
            state_set = set(names)
            action_names = [name for name in names if name in observed]
            action_names.extend(name for name in observed if name not in state_set)
    combined_names = action_names + extra_names
    if len(set(combined_names)) != len(combined_names):
        raise ValueError('action source joint names overlap')
    output_names = frozen.get('action', combined_names)
    output_order = (_joint_indices(tuple(combined_names), tuple(output_names))
                    if combined_names != output_names else None)
    axis_source_times = {key: [] for key in ('observation.state', 'action', *extra_keys.values())} if live_recording or grips else {}
    action_axis_samples = []
    state_axis_samples = []
    extra_axis_samples = {}
    for key, samples in raw_cameras.items():
        stream_names = (names if key == 'observation.state' else action_names if key == 'action'
                        else action_map[source_topics[key]]['names'])
        if key.startswith('action.source.'):
            numeric_names[key] = stream_names
        try:
            if key == 'action':
                streams[key], action_axis_samples = _project_joint_stream(samples, stream_names, correction, axis_source_times.get(key))
            elif fixed_body and key == 'observation.state':
                streams[key], state_axis_samples = _project_joint_stream(samples, stream_names, False, axis_source_times.get(key))
            elif fixed_body and key.startswith('action.source.'):
                streams[key], extra_axis_samples[key] = _project_joint_stream(samples, stream_names, False, axis_source_times.get(key))
            else:
                streams[key] = [(stamp, joint_vector(message, stream_names)) for stamp, message in samples]
        except ValueError as error:
            raise ValueError(f"{source_topics[key]}: {error}") from error
    for topic, key in extra_keys.items():
        if key not in streams:
            streams[key] = []
            source_topics[key] = topic
            numeric_names[key] = action_map[topic]['names']
    partial_action = (fixed_body or (not correction and any(not all(mask) for _, mask in action_axis_samples))
                      or frozen_mask)
    progress.update("对齐时间轴", 25, force=True)
    episodes = []
    origin = recording_start
    if correction:
        # Extra command sources are not final HC evidence. A correction export
        # must never append a public program target or fabricate a held axis.
        if action_sources:
            raise ValueError('correction export uses only final HC actions; disable extra action_sources for this session')
        recording_start = correction.get('quality', {}).get('first_action_ns')
        recording_end = correction['end_ns']-1
        if recording_start is None:
            raise ValueError('No final human action in this correction; source remains downloadable')
    keys = sorted(streams) if fixed_body else list(streams)
    audit = {'source_counts': {key: len(samples) for key, samples in streams.items()}}
    placeholders = {}
    for key, shape in (dimensions.items() if not snapshot else []):
        import numpy as np
        filename = staging / key / 'missing.png'
        if not cv2.imwrite(str(filename), np.zeros(shape, dtype=np.uint8)):
            raise RuntimeError(f'Cannot write masked camera placeholder: {key}')
        placeholders[key] = str(filename)
    for key in streams:
        if key not in placeholders:
            placeholders[key] = [0.0] * (len(names) if key == 'observation.state' else len(action_names) if key == 'action' else len(numeric_names[key]))
    for key in streams:
        source_clocks.setdefault(key, 'log_time')
    audit['source_clocks'] = source_clocks
    joint_frame_variants = {}
    for topic in unstated_joint_frames:
        declared = source_frame_ids.setdefault(topic, '')
        if declared:
            joint_frame_variants[topic] = ['', declared]
    if joint_frame_variants:
        audit['joint_frame_id_variants'] = joint_frame_variants
    if canonical_x1:
        aligned_origins = (dict.fromkeys((stamp for stamp, _ in streams['action']), 'published_action')
                           if use_public or use_wire or historical else origin_index.align(original_actions, header_stamps.get('action', []),
                                                               source_clocks['action']))
        audit['primary_action_origin_ranges'] = []
    live_camera_keys = []
    if live_recording is not None:
        declared = [camera['feature'] for camera in live_recording.get('cameras', ())]
        live_camera_keys = [key for key in declared if key in dimensions] if declared else sorted(dimensions)
        live_source_keys = sorted(['action', 'observation.state', *live_camera_keys])
        audit['grid_clock'] = 'ros_epoch'
    aligned = (preserved_episode(streams, fps, audit) if snapshot else
               synchronized_episodes(streams, fps, audit, (recording_start, recording_end), max_ages,
                                     use_epoch_grid=live_recording is not None))
    if snapshot and audit.get('source_frequency_hz'):
        # LeRobot uses an integer nominal rate; original timestamps remain in rows.
        fps = max(1, round(audit['source_frequency_hz']))
    for episode in aligned:
        rows = []
        if canonical_x1:
            origin_ranges = []
            audit['primary_action_origin_ranges'].append(origin_ranges)
        for stamp, selected in episode:
            values = {key: sample[1] if sample is not None else placeholders[key]
                      for key, sample in selected.items()}
            if extra_keys:
                values['action'] = values['action'] + [v for key in extra_keys.values() for v in values[key]]
            if output_order is not None:
                values['action'] = [values['action'][i] for i in output_order]
            row = {'values': values}
            if live_recording is None:
                ages = [(stamp - selected[key][0]) / 1e9 if selected[key] is not None else -1.0 for key in keys]
                row.update({
                    'source_times': [(selected[key][0] - origin) / 1e9 if selected[key] is not None else -1.0 for key in keys],
                    'source_available': [float(selected[key] is not None) for key in keys],
                    'source_valid': [float(selected[key] is not None and ages[i] <= max_ages.get(key, sample_max_age(key)) / 1e9) for i, key in enumerate(keys)],
                    'source_age': ages, 'grid_time': (stamp - origin) / 1e9})
            if fixed_body:
                state_axis = latest_sample(state_axis_samples, stamp, sample_max_age('observation.state'))
                row['state_valid'] = state_axis[1] if state_axis else [False] * len(names)
            if live_recording is not None or grips:
                row['observation.timestamp_ns'] = [stamp]
                for key, field, width in (('observation.state', 'observation.state_time_ns', len(names)),
                                          ('action', 'action.source_time_ns', len(action_names))):
                    sample = latest_sample(axis_source_times[key], stamp, None)
                    row[field] = sample[1] if sample else [-1] * width
                if extra_keys:
                    times = list(row['action.source_time_ns'])
                    for topic, key in extra_keys.items():
                        sample = latest_sample(axis_source_times.get(key, ()), stamp, None)
                        times.extend(sample[1] if sample else [-1] * len(action_map[topic]['names']))
                    row['action.source_time_ns'] = times
                if live_camera_keys:
                    row['observation.camera_time_ns'] = [
                        selected[key][0] if selected[key] is not None else -1 for key in live_camera_keys]
                    row['observation.camera_valid'] = [
                        selected[key] is not None and stamp - selected[key][0] <= sample_max_age(key)
                        for key in live_camera_keys]
            if session:
                row['intervention'] = any(interval['start_ns'] <= stamp < interval.get('end_ns', session['end_ns'])
                                           for interval in session.get('interventions', []))
            if correction:
                axis_times = latest_sample(action_axis_samples, stamp, None)
                valid = [value >= 0 and 0 <= stamp-value <= sample_max_age('action')
                         for value in axis_times[1]] if axis_times else [False]*len(action_names)
                row['action_valid'] = valid
                row['source_valid'][keys.index('action')] *= float(all(valid))
                usable = all(row['source_valid'][index] for index, key in enumerate(keys) if required_stream(key))
                if not usable:
                    if rows:
                        episodes.append(rows)
                        rows = []
                    audit['rejected_frames'] += 1
                    audit['retained_frames'] -= 1
                    continue
            elif partial_action:
                axis_sample = latest_sample(action_axis_samples, stamp, sample_max_age('action'))
                row['action_valid'] = ([bool(value) for value in axis_sample[1]] if axis_sample
                                       else [False] * len(action_names))
                for topic, key in extra_keys.items():
                    if key in extra_axis_samples:
                        axis = latest_sample(extra_axis_samples[key], stamp, max_ages[key])
                        valid = axis[1] if axis else [False] * len(action_map[topic]['names'])
                    else:
                        sample = selected[key]
                        available = sample is not None and stamp - sample[0] <= max_ages[key]
                        valid = [available] * len(action_map[topic]['names'])
                    row['action_valid'].extend(valid)
                if output_order is not None:
                    row['action_valid'] = [row['action_valid'][i] for i in output_order]
            if historical:
                historical_labels.note_grid(len(rows), stamp, JOINT_MAX_AGE_NS)
            if canonical_x1:
                append_origin_range(origin_ranges, len(rows), selected['action'], aligned_origins)
            if grips:
                grips.append(row, stamp, sample_max_age('action'))
            if live_recording is not None:
                source_ns = dict(zip(live_camera_keys, row.get('observation.camera_time_ns', ())))
                source_ns['observation.state'] = max(row['observation.state_time_ns'])
                source_ns['action'] = max(row['action.source_time_ns'])
                row['observation.source_timestamp_ns'] = [source_ns[key] for key in live_source_keys]
            rows.append(row)
        if rows:
            episodes.append(rows)
    if not episodes:
        raise ValueError('No recording timeline samples; check recorded topics')
    if deferred_image_topics:
        from .recording_images import stage_selected_images
        audit['staged_compressed_frames'] = stage_selected_images(source, staging, deferred_image_topics, episodes)

    manifest = {'action_sources': action_sources, 'action_semantics': 'upstream_command_not_execution', 'action_axis_validity': bool(partial_action), 'fps': fps, 'task': task, 'robot_type': robot_type, 'joint_names': names, 'action_joint_names': output_names, 'dimensions': dimensions, 'numeric_names': numeric_names, 'source_keys': keys, 'source_topics': source_topics, 'source_frame_ids': source_frame_ids, 'sync_audit': audit, 'progress_path': str(progress.path) if progress.path else None, 'origin_log_time_ns': origin, 'episodes': episodes}
    if live_recording:
        manifest['live_camera_keys'] = live_camera_keys
        manifest['live_source_keys'] = live_source_keys
        manifest['robot_type'] = profile
        if historical:
            audit['historical_x1_projection'] = {
                'schema': 'hc-x1-historical-shared/v1',
                'body_evidence': ('published_wire_copy' if use_wire else
                                  'matched_publication_markers' if original_actions else 'no_verified_publication'),
                'published_body_samples': len(original_actions),
                'raw_label_counts': historical_labels.counts,
                'raw_label_origin_ranges': historical_labels.ranges,
            }
        else:
            manifest['robot_api_info'] = robot_info
        manifest['action_semantics'] = ('published pre-smoother target; measured state is observation.state'
                                        if use_public or (use_wire and wire_actions) or (historical and original_actions) else
                                        'no published target observed' if historical or use_wire or source_topics['action'] == public_command_topic else
                                        'native published action target; measured state is observation.state')
    if fixed_body:
        body_units = live_recording['body']['units'] if live_recording else ['rad'] * len(names)
        manifest['state_axis_validity'] = True
        manifest['joint_units'] = {
            'observation.state': body_units,
            'action': body_units + [u for source in action_sources for u in source['units']],
        }
    if grips:
        if set(grips.names) & set(output_names):
            raise ValueError('binary hand names overlap another configured action source')
        manifest['binary_hand_sources'] = grips.sources
        manifest['joint_names'] = names + grips.names
        manifest['action_joint_names'] = output_names + grips.names
        for key, units in manifest['joint_units'].items():
            manifest['joint_units'][key] = units + ['1'] * len(grips.names)
    if snapshot: manifest["capture_snapshot"] = snapshot
    if session:
        manifest['session'] = session
    if correction:
        manifest.update(correction=correction, initial_observations=first_observations,
                        action_semantics='HC final sent goal; measured feedback is observation.state; outcome unknown')
    path = staging / 'manifest.json'
    path.write_text(json.dumps(manifest), encoding='utf-8')
    progress.update("对齐完成", 30, sum(map(len, episodes)), sum(map(len, episodes)), "帧", force=True)
    return path


def encode_episode_mp4(image_dir: Path, destination: Path, fps: int, on_frame=None) -> None:
    """Encode exact frame-grid timestamps, without B-frame DTS reordering.

    LeRobot 0.4.3 concatenates episode MP4s using stream copy. Its default
    H.264 B-frame delay produces overlapping DTS at very short boundaries.
    Each independent episode here starts at PTS=DTS=0, has no reorder delay,
    and every packet lasts exactly one frame. No frames are padded or dropped.
    """
    import av
    from fractions import Fraction
    from PIL import Image

    files = sorted(image_dir.glob("frame-*.png"))
    if not files:
        raise ValueError(f"No episode images in {image_dir}")
    with Image.open(files[0]) as image:
        width, height = image.size
    destination.parent.mkdir(parents=True, exist_ok=True)
    with av.open(str(destination), "w", options={"movflags": "faststart"}) as output:
        stream = output.add_stream("libx264", rate=fps,
                                   options={"bf": "0", "preset": "fast", "crf": "23",
                                            "tune": "zerolatency"})
        stream.width, stream.height = width, height
        stream.pix_fmt = "yuv420p" if width % 2 == 0 and height % 2 == 0 else "yuv444p"
        stream.codec_context.max_b_frames = 0
        stream.codec_context.thread_count = 1
        stream.codec_context.gop_size = fps
        stream.codec_context.time_base = Fraction(1, fps)
        for index, filename in enumerate(files):
            with Image.open(filename) as image:
                if image.size != (width, height):
                    raise ValueError("Camera dimensions changed within an episode")
                frame = av.VideoFrame.from_image(image.convert("RGB"))
            frame.pts, frame.time_base = index, Fraction(1, fps)
            for packet in stream.encode(frame):
                packet.duration = 1
                output.mux(packet)
            if on_frame is not None:
                on_frame(index + 1, len(files))
        for packet in stream.encode():
            packet.duration = 1
            output.mux(packet)


def write_dataset(manifest_path: Path, destination: Path) -> dict:
    import numpy as np
    from PIL import Image
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    manifest = json.loads(manifest_path.read_text())
    progress = ExportProgress(Path(manifest['progress_path']) if manifest.get('progress_path') else None)
    total_frames = sum(map(len, manifest['episodes']))
    total_videos = len(manifest['episodes']) * len(manifest['dimensions'])
    written_frames = encoded_videos = 0
    def work_percent():
        return 30 + 35 * written_frames / max(1, total_frames) + 30 * encoded_videos / max(1, total_videos)
    joint_names = {'observation.state': manifest['joint_names'],
                   'action': manifest.get('action_joint_names', manifest['joint_names'])}
    features = {key: {'dtype': 'float32', 'shape': (len(names),), 'names': names}
                for key, names in joint_names.items()}
    for key, units in manifest.get('joint_units', {}).items():
        features[key]['units'] = units
        if manifest.get('binary_hand_sources'):
            features[key]['hc.hand_semantics'] = {
                name.removesuffix('_grip')+'_hand': 'binary_grasp'
                for name in manifest['binary_hand_sources'][key]}
    shared_training = 'live_camera_keys' in manifest
    for key, names in manifest.get('numeric_names', {}).items():
        features[key] = {'dtype': 'float32', 'shape': (len(names),), 'names': names}
    for key, shape in manifest['dimensions'].items():
        features[key] = {'dtype': 'video', 'shape': tuple(shape), 'names': ['height', 'width', 'channels']}
    if not shared_training:
        features['observation.source_time'] = {'dtype': 'float64', 'shape': (len(manifest['source_keys']),), 'names': manifest['source_keys']}
        features['observation.recording_time'] = {'dtype': 'float64', 'shape': (1,), 'names': ['seconds_since_first_log_time']}
    if manifest.get('correction') or manifest.get('session'):
        features['intervention'] = {'dtype': 'bool', 'shape': (1,), 'names': ['human_action']}
    if manifest.get('correction') or manifest.get('action_axis_validity'):
        features['action.valid'] = {'dtype': 'bool', 'shape': (len(joint_names['action']),), 'names': joint_names['action']}
    if manifest.get('state_axis_validity'):
        features['observation.state_valid'] = {
            'dtype': 'bool', 'shape': (len(joint_names['observation.state']),), 'names': joint_names['observation.state']}
    live_fields = {}
    if manifest.get('binary_hand_sources'):
        live_fields = {'observation.state_time_ns': joint_names['observation.state'], 'action.source_time_ns': joint_names['action']}
    if 'live_camera_keys' in manifest:
        live_fields = {'observation.timestamp_ns': ['ros_epoch'],
                       'observation.source_timestamp_ns': manifest.get('live_source_keys', manifest['source_keys']),
                       'observation.state_time_ns': joint_names['observation.state'],
                       'action.source_time_ns': joint_names['action']}
        if manifest['live_camera_keys']:
            camera_names = manifest['live_camera_keys']
            live_fields['observation.camera_time_ns'] = camera_names
            live_fields['observation.camera_valid'] = camera_names
    for field, axes in live_fields.items():
        dtype = 'bool' if field == 'observation.camera_valid' else 'int64'
        features[field] = {'dtype': dtype, 'shape': (len(axes),), 'names': axes}
    if not shared_training and 'sync_audit' in manifest:
        for field in ('source_available', 'source_valid', 'source_age'):
            features['observation.' + field] = {'dtype': 'float32', 'shape': (len(manifest['source_keys']),), 'names': manifest['source_keys']}
    if manifest.get('capture_snapshot'):
        from humanoid_upload.capture_plan import LEROBOT_GENERATED_FEATURES, validate_lerobot_info
        snapshot = manifest['capture_snapshot']
        for name, declaration in snapshot['output']['features'].items():
            source = snapshot['effective_adapter']['feature_sources'].get(name, name)
            if source in features and features[source]['dtype'] in ('float32', 'float64'):
                # JointState contains float64. Preserve it when the shared MCAP/LeRobot
                # contract explicitly requires that precision.
                if declaration['dtype'] in ('float32', 'float64'):
                    features[source]['dtype'] = declaration['dtype']
                if declaration.get('units'):
                    features[source]['units'] = declaration['units']
        validate_lerobot_info(manifest['capture_snapshot'], {'fps': manifest['fps'], 'codebase_version': 'v3.0',
            'features': {**LEROBOT_GENERATED_FEATURES, **json.loads(json.dumps(features))}})
    repo_id = 'local/' + re.sub('[^a-zA-Z0-9_-]', '_', destination.name)
    class TeleopLeRobotDataset(LeRobotDataset):
        def _encode_temporary_episode_video(self, video_key, episode_index):
            nonlocal encoded_videos
            progress.update('编码视频', work_percent(), encoded_videos, total_videos, '路视频', force=True)
            import shutil
            image_dir = self._get_image_file_dir(episode_index, video_key)
            temp_path = Path(tempfile.mkdtemp(dir=self.root)) / f"{video_key}_{episode_index:03d}.mp4"
            def encoded_frame(current, total):
                progress.update(f'编码视频 {encoded_videos + 1}/{total_videos}',
                                work_percent() + 30 * current / max(1, total) / max(1, total_videos),
                                current, total, '帧')
            encode_episode_mp4(image_dir, temp_path, self.fps, encoded_frame)
            shutil.rmtree(image_dir)
            encoded_videos += 1
            progress.update('合并视频', work_percent(), encoded_videos, total_videos, '路视频', force=True)
            return temp_path

    dataset = TeleopLeRobotDataset.create(repo_id=repo_id, root=destination, fps=manifest['fps'],
                                         robot_type=manifest['robot_type'], features=features,
                                         use_videos=True, vcodec="h264", video_backend="pyav")
    if shared_training:
        # 0.7.1 writes floating extrema even for integer and boolean features.
        # Cast only episode statistics before 0.4.3 serializes them to Parquet/JSON.
        save_metadata = dataset.meta.save_episode

        def save_shared_metadata(episode_index, episode_length, episode_tasks, episode_stats, episode_metadata):
            for stats in episode_stats.values():
                for key in ('min', 'max'):
                    stats[key] = np.asarray(stats[key], dtype=np.float64)
            return save_metadata(episode_index, episode_length, episode_tasks, episode_stats, episode_metadata)

        dataset.meta.save_episode = save_shared_metadata
    image_cache = {}
    for episode in manifest['episodes']:
        for row in episode:
            frame = {'task': manifest['task']}
            for key, value in row['values'].items():
                if key in manifest['dimensions']:
                    cached = image_cache.get(key)
                    if cached is None or cached[0] != value:
                        with Image.open(value) as image:
                            cached = (value, np.asarray(image.convert('RGB')))
                        image_cache[key] = cached
                    frame[key] = cached[1]
                else:
                    frame[key] = np.asarray(value, dtype=features[key]['dtype'])
            if not shared_training:
                frame['observation.source_time'] = np.asarray(row['source_times'], dtype=np.float64)
                frame['observation.recording_time'] = np.asarray([row['grid_time']], dtype=np.float64)
            if manifest.get('correction') or manifest.get('session'):
                frame['intervention'] = np.asarray([True if manifest.get('correction') else row['intervention']], dtype=bool)
            if manifest.get('correction') or manifest.get('action_axis_validity'):
                frame['action.valid'] = np.asarray(row['action_valid'], dtype=bool)
            if manifest.get('state_axis_validity'):
                frame['observation.state_valid'] = np.asarray(row['state_valid'], dtype=bool)
            if not shared_training and 'sync_audit' in manifest:
                for field in ('source_available', 'source_valid', 'source_age'):
                    frame['observation.' + field] = np.asarray(row[field], dtype=np.float32)
            for field in live_fields:
                frame[field] = np.asarray(row[field], dtype=features[field]['dtype'])
            dataset.add_frame(frame)
            written_frames += 1
            progress.update('写入样本', work_percent(), written_frames, total_frames, '帧')
        # The official parallel worker bypasses our per-episode encoder override.
        dataset.save_episode(parallel_encoding=False)
    progress.update("整理元数据", 95, force=True)
    dataset.finalize()
    loaded = LeRobotDataset(repo_id, root=destination, video_backend='pyav')
    if len(loaded) != sum(map(len, manifest['episodes'])):
        raise RuntimeError('Official LeRobot loader frame count mismatch')
    progress.update('校验数据集', 96, force=True)
    # Decode video too, not just metadata/parquet.
    offset = 0
    for episode in manifest['episodes']:
        loaded[offset]
        loaded[offset + len(episode) - 1]
        offset += len(episode)
        progress.update('校验数据集', 96 + 3 * offset / max(1, total_frames), offset, total_frames, '帧')
    synchronization = {key: manifest[key] for key in ('source_topics', 'origin_log_time_ns', 'source_keys', 'joint_names')}
    if manifest.get('binary_hand_sources'):
        synchronization['binary_hand_sources'] = manifest['binary_hand_sources']
        synchronization['binary_grasp'] = dict(unit='1', open=0, closed=1,
            state='observed aperture or measured posture', action='published target')
    if 'robot_api_info' in manifest:
        synchronization['robot_api_info'] = manifest['robot_api_info']
    synchronization['action_joint_names'] = joint_names['action']
    synchronization['action_sources'] = manifest.get('action_sources', [])
    synchronization['action_semantics'] = manifest.get('action_semantics', 'upstream_command_not_execution')
    if manifest.get('state_axis_validity'):
        synchronization['state_axis_validity'] = (
            'observation.state_valid follows observation.state names; false for absent, NaN or stale axes; '
            'numeric placeholders hold the last measured position or zero before its first sample')
    synchronization['action_axis_validity'] = bool(manifest.get('action_axis_validity'))
    synchronization['joint_alignment'] = 'observation.state uses feedback joints; action uses commanded joints; ordered by joint name, no feedback substituted for absent commands'
    synchronization['task'] = manifest['task']
    synchronization['source_frame_ids'] = manifest.get('source_frame_ids', {})
    synchronization['sync_audit'] = manifest.get('sync_audit', {})
    synchronization['missing_policy'] = 'all sources: hold last observed value when stale, source_valid=0; before first sample use zero numeric/black image placeholders with source_available=0, source_valid=0 and source_time/source_age=-1; no future samples used'
    synchronization['auxiliary_policy'] = 'causal last observed value; never gates row selection; source_available=0 and value=0/source_time=-1/source_age=-1 before first sample; source_valid=0 when missing or stale; stale velocity is not asserted to remain active'
    synchronization.update({'clock': 'per stream: ROS header stamp when every message has a unique stamp within -20 ms..+2 s of its MCAP log_time, otherwise MCAP log_time (wall receive time); see sync_audit.source_clocks', 'selection': 'one recording is one episode, full MCAP log_time span on fixed FPS grid; causal last sample, no rows deleted for missing/stale values', 'state_action_max_age_seconds': 0.2, 'camera_max_age_seconds': 0.25, 'source_mcap_preserved': True, 'images': 'H.264 MP4 with no B frames, CRF 23; short episodes stream-concatenated without padding or dropping frames', 'joint_position_units': 'source ROS JointState units (revolute radians, prismatic meters)' })
    if 'live_camera_keys' in manifest:
        synchronization.update(clock='per sample: plausible ROS header stamp, otherwise MCAP receive time',
                               grid='integer ROS epoch, nearest nanosecond (half up)',
                               timestamp_units='absolute ROS nanoseconds; missing source=-1',
                               missing_policy='last finite value and source stamp per axis; before first use zero and -1; validity masks mark missing/stale axes',
                               auxiliary_policy='training columns follow the recorded body, optional selected hands, explicit extra action sources and declared RGB cameras')
    if manifest.get('capture_snapshot'):
        synchronization.update(clock='MCAP log_time (wall receive time), preserved for all snapshot streams',
            selection='original state log_time samples, original frame count and frequency',
            missing_policy='absent optional streams remain absent; unsupported per-frame gaps stop export',
            auxiliary_policy='actual causal samples only; no numeric or image placeholders')
    if manifest.get('correction'):
        synchronization.update(correction=manifest['correction'], initial_observations=manifest['initial_observations'],
            clock='MCAP log_time equals original source header or final HC send time; receive provenance retained in source',
            selection='one correction identity; continuous valid subepisodes split at gaps; recording_time preserves original offsets',
            intervention_semantics='true means human action, not success; outcome unknown')
    if manifest.get('session'):
        synchronization.update(session=manifest['session'],
            intervention_semantics='true during human-control intervals; false outside; not a success label')
    (destination / 'meta' / 'teleop_sync.json').write_text(json.dumps(synchronization, ensure_ascii=False, indent=2))
    return {'path': str(destination), 'format': 'lerobot', 'frames': len(loaded), 'episodes': len(manifest['episodes']), 'fps': manifest['fps'], 'cameras': list(manifest['dimensions']), 'episode_lengths': [len(episode) for episode in manifest['episodes']], 'image_storage': 'mp4_h264', 'sync_audit': manifest.get('sync_audit', {})}


def export_mcap(source, destination, fps=20, task='teleoperation', robot_type='x1', action_sources=None, record_hands=None, training_schema='auto') -> dict:
    executable = ensure_export_available()
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if isinstance(fps, bool) or not isinstance(fps, int) or not 1 <= fps <= 120:
        raise ValueError('fps must be an integer between 1 and 120')
    if destination.exists():
        raise FileExistsError(f'Destination already exists: {destination}')
    destination.parent.mkdir(parents=True, exist_ok=True)
    progress = ExportProgress(progress_path(destination))
    progress.update('等待转换', 0, force=True, reset=True)
    with tempfile.TemporaryDirectory(prefix='.lerobot-staging-', dir=destination.parent) as directory:
        manifest = prepare_mcap(source, Path(directory), fps, task, robot_type, progress, action_sources, record_hands, training_schema)
        staged_dataset = Path(directory) / 'dataset'
        writer_env = {key: value for key, value in os.environ.items() if key not in ('PYTHONPATH', 'PYTHONHOME')}
        process = subprocess.run([executable, str(Path(__file__).resolve()), '--write-manifest', str(manifest), str(staged_dataset)], capture_output=True, text=True, env=writer_env)
        if process.returncode:
            raise RuntimeError('LeRobot writer failed: ' + process.stderr[-6000:])
        result = json.loads(process.stdout.strip().splitlines()[-1])
        prepared = json.loads(manifest.read_text())
        if prepared.get('capture_snapshot'):
            from humanoid_upload.capture_plan import validate_lerobot_info
            from humanoid_upload.common import atomic_json, read_json
            snapshot = prepared['capture_snapshot']
            presence = validate_lerobot_info(snapshot, read_json(staged_dataset / 'meta/info.json'))
            atomic_json(staged_dataset / 'capture-context.json', {'schema_version': 'hc-capture-context/v1',
                'recording_id': snapshot['recording_id'], 'capture_snapshot': snapshot, 'feature_presence': presence})
        from middleware.core.takeover_log import takeover_file
        takeovers = takeover_file(source)
        if takeovers:
            shutil.copyfile(takeovers, staged_dataset / 'meta/takeovers.json')
        if destination.exists():
            raise FileExistsError(f'Destination was created during export: {destination}')
        staged_dataset.rename(destination)
        result['path'] = str(destination)
        from middleware.core.artifact_state import atomic_json
        atomic_json(destination / '.hc-export-complete.json', {'source':str(source), 'frames':result['frames'], 'complete':True})
        progress.update('转换完成', 100, result['frames'], result['frames'], '帧', state='completed', force=True)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    parser.add_argument('--fps', type=int, default=20)
    parser.add_argument('--task', default='teleoperation')
    parser.add_argument('--robot-type', default='x1')
    parser.add_argument('--action-sources', default='[]', help='JSON action topic/name/unit/age mappings')
    parser.add_argument('--training-schema', choices=('auto', 'shared'), default='auto',
                        help='shared projects historical native X1 into the ordinary ServoJ training schema; unverifiable body labels are masked')
    parser.add_argument('--record-hands', choices=('none', 'left', 'right', 'both'), help='binary hand axes (default: infer configured hand action sources)')
    parser.add_argument('--write-manifest', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        result = write_dataset(Path(args.source), Path(args.destination)) if args.write_manifest else export_mcap(args.source, args.destination, args.fps, args.task, args.robot_type, json.loads(args.action_sources), args.record_hands, args.training_schema)
        print(json.dumps(result, ensure_ascii=False))
    except Exception as error:
        if not args.write_manifest:
            ExportProgress(progress_path(Path(args.destination).resolve())).update('转换失败', 0, state='error', error=str(error)[-2000:], force=True)
        print(str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
