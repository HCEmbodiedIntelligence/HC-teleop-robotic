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
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
INSTALL_COMMAND = "./tools/setup_lerobot.sh"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / 'upload_client') not in sys.path:
    sys.path.insert(0, str(ROOT / 'upload_client'))
from middleware.core.export_progress import ExportProgress, progress_path


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


def joint_vector(message: Any, names: list[str]) -> list[float]:
    if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
        raise ValueError("JointState names/positions malformed")
    values = dict(zip(message.name, message.position))
    if not all(name in values for name in names):
        raise ValueError("JointState changed its joint set")
    result = [float(values[name]) for name in names]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("JointState contains non-finite positions")
    return result


def latest_sample(samples: list[tuple[int, Any]], stamp: int, max_age_ns: int | None):
    index = bisect.bisect_right(samples, stamp, key=lambda sample: sample[0]) - 1
    if index < 0 or (max_age_ns is not None and stamp - samples[index][0] > max_age_ns):
        return None
    return samples[index]


def required_stream(key: str) -> bool:
    return key in ('observation.state', 'action') or key.startswith('observation.images.')


def sample_max_age(key: str) -> int:
    return 250_000_000 if key.startswith('observation.images.') else 200_000_000


def synchronized_episodes(streams: dict[str, list], fps: int, audit: dict | None = None,
                          time_range: tuple[int, int] | None = None, max_ages: dict | None = None):
    """One recording, one episode: retain every grid point, with causal values.

    Missing and stale values are represented by validity/availability/age fields
    in the manifest. Never borrow future samples or delete time to close a gap.
    """
    required = {key: samples for key, samples in streams.items() if required_stream(key)}
    if not required or any(not samples for samples in required.values()):
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
    current = []
    for index in range(int((end - start) * fps // 1_000_000_000) + 1):
        stamp = start + round(index * 1_000_000_000 / fps)
        selected = {key: latest_sample(samples, stamp, None) for key, samples in streams.items()}
        invalid = [key for key, sample in selected.items()
                   if sample is None or stamp - sample[0] > (max_ages or {}).get(key, sample_max_age(key))]
        if audit is not None:
            audit['candidate_frames'] += 1
            audit['retained_frames'] += 1
            audit['invalid_frames'] += bool(invalid)
            audit['invalid_required_frames'] += any(key in required for key in invalid)
            for key in invalid:
                audit['invalid_by_stream'][key] = audit['invalid_by_stream'].get(key, 0) + 1
        current.append((stamp, selected))
    yield current


def preserved_episode(streams, fps, audit):
    """Shared plans preserve source state timestamps/count; never synthesize frames."""
    from humanoid_upload.common import UploadError
    timeline = streams['observation.state']
    stamps = [sample[0] for sample in timeline]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise UploadError('CAPTURE_STRUCTURE_MISMATCH', '状态时间戳不是严格递增')
    if len(stamps) > 1:
        actual_hz = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0])
        if abs(actual_hz - fps) > max(0.5, fps * .05):
            raise UploadError('CAPTURE_FREQUENCY_MISMATCH', f'原始状态 {actual_hz:.3f} Hz 与方案 {fps} Hz 不一致')
    rows = []
    for stamp, _ in timeline:
        selected = {key: latest_sample(samples, stamp, sample_max_age(key)) for key, samples in streams.items()}
        missing = [key for key, value in selected.items() if value is None]
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


def prepare_mcap(source: Path, staging: Path, fps: int, task: str, robot_type: str, progress: ExportProgress | None = None, action_sources: list | None = None) -> Path:
    import cv2
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from .topic_recorder import read_recording_metadata
    metadata = read_recording_metadata(source)
    if metadata.get('capture_snapshot'):
        from humanoid_upload.capture_plan import validate_snapshot
        validate_snapshot(json.loads(metadata['capture_snapshot']), recording_id=metadata.get('recording_id'))
    from .capture_formats import lerobot_snapshot
    snapshot = lerobot_snapshot(metadata)
    if snapshot:
        from humanoid_upload.capture_plan import validate_snapshot
        from humanoid_upload.common import UploadError
        expected_id = metadata.get('recording_id')
        if metadata.get('capture_snapshots'):
            expected_id += '-lerobot'
        validate_snapshot(snapshot, source_format='LEROBOT_V3', recording_id=expected_id)
        if snapshot['output']['frequency_hz'] != fps:
            raise UploadError('CAPTURE_FREQUENCY_MISMATCH', '导出频率与已固定方案不同；不自动重采样')

    progress = progress or ExportProgress(None)
    progress.update("读取 MCAP", 0, force=True)
    action_sources = action_sources or []
    action_map = {item["topic"]: item for item in action_sources}
    extra_keys = {topic: "action.source." + str(index) for index, topic in enumerate(action_map)}
    max_ages = {extra_keys[topic]: int(item["max_age_sec"]*1e9) for topic, item in action_map.items()}
    streams: dict[str, list] = {}
    names = None
    dimensions = {}
    source_topics = {}
    source_frame_ids = {}
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
    recording_start = recording_end = None
    with source.open('rb') as handle:
        reader = make_reader(handle)
        # Prefer compressed over raw when both versions of a camera are recorded.
        summary = reader.get_summary()
        topics = {channel.topic for channel in summary.channels.values()} if summary else set()
        message_total = summary.statistics.message_count if summary and summary.statistics else 0
        for message_index, (schema, channel, message) in enumerate(reader.iter_messages(log_time_order=True), 1):
            if recording_start is None:
                recording_start = message.log_time
            recording_end = message.log_time
            progress.update('读取 MCAP', 25 * message_index / max(1, message_total) if message_total else 0,
                            message_index, message_total, '条消息')
            topic = channel.topic
            if channel.message_encoding != 'cdr' or schema is None:
                continue
            kind = schema.name
            joint_key = {'/observation/joint_states': 'observation.state', '/action/joint_states': 'action', **extra_keys}.get(topic)
            is_camera = kind in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage') and 'depth' not in topic.lower()
            numeric_key = numeric_topics.get(topic)
            if joint_key is None and not is_camera and numeric_key is None:
                continue
            if is_camera and topic + '/compressed' in topics:
                continue
            if kind not in message_classes:
                message_classes[kind] = get_message(kind)
            decoded = deserialize_message(message.data, message_classes[kind])
            header = getattr(decoded, 'header', None)
            if header is not None:
                frame_id = str(header.frame_id)
                previous_frame = source_frame_ids.get(topic)
                if previous_frame is not None and previous_frame != frame_id:
                    raise ValueError(f'Reference frame changed during recording: {topic}')
                source_frame_ids[topic] = frame_id
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
                source_topics[numeric_key] = topic
            elif joint_key:
                if kind != 'sensor_msgs/msg/JointState':
                    raise ValueError(f"Expected JointState on {topic}")
                if names is None and joint_key == 'observation.state':
                    names = list(decoded.name)
                    if not names:
                        raise ValueError('Feedback has no joint names')
                raw_cameras.setdefault(joint_key, []).append((message.log_time, decoded))
                source_topics[joint_key] = topic
            else:
                key = 'observation.images.' + re.sub(r'[^a-zA-Z0-9_]', '_', topic.strip('/').removesuffix('/compressed'))
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
                source_topics[key] = topic
    if not names or not raw_cameras.get('action'):
        raise ValueError('LeRobot requires recorded /observation/joint_states and /action/joint_states; missing grid samples are explicitly masked')
    first_action_names = list(raw_cameras['action'][0][1].name)
    if not first_action_names:
        raise ValueError('Action has no joint names')
    # A command may intentionally omit passive/free joints in the feedback.
    # Keep common joints in feedback order, with command-only joints appended.
    action_names = [name for name in names if name in first_action_names]
    action_names.extend(name for name in first_action_names if name not in names)
    if snapshot:
        action_names = first_action_names
    for key, samples in raw_cameras.items():
        stream_names = (names if key == 'observation.state' else action_names if key == 'action'
                        else action_map[source_topics[key]]['names'])
        if key.startswith('action.source.'):
            numeric_names[key] = stream_names
        try:
            streams[key] = [(stamp, joint_vector(message, stream_names)) for stamp, message in samples]
        except ValueError as error:
            raise ValueError(f"{source_topics[key]}: {error}") from error
    for topic, key in extra_keys.items():
        if key not in streams:
            streams[key] = []
            source_topics[key] = topic
            numeric_names[key] = action_map[topic]['names']
    progress.update("对齐时间轴", 25, force=True)
    episodes = []
    origin = recording_start
    keys = list(streams)
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
    aligned = (preserved_episode(streams, fps, audit) if snapshot else
               synchronized_episodes(streams, fps, audit, (recording_start, recording_end), max_ages))
    for episode in aligned:
        rows = []
        for stamp, selected in episode:
            values = {key: sample[1] if sample is not None else placeholders[key]
                      for key, sample in selected.items()}
            values['action'] = list(values['action']) + [v for key in extra_keys.values() for v in values[key]]
            ages = [(stamp - selected[key][0]) / 1e9 if selected[key] is not None else -1.0 for key in keys]
            rows.append({'values': values,
                         'source_times': [(selected[key][0] - origin) / 1e9 if selected[key] is not None else -1.0 for key in keys],
                         'source_available': [float(selected[key] is not None) for key in keys],
                         'source_valid': [float(selected[key] is not None and ages[i] <= max_ages.get(key, sample_max_age(key)) / 1e9) for i, key in enumerate(keys)],
                         'source_age': ages, 'grid_time': (stamp - origin) / 1e9})
        episodes.append(rows)
    if not episodes:
        raise ValueError('No recording timeline samples; check recorded topics')
    action_names = action_names + [name for item in action_sources for name in item['names']]
    if len(set(action_names)) != len(action_names):
        raise ValueError('action source joint names overlap')
    manifest = {'action_sources': action_sources, 'action_semantics': 'upstream_command_not_execution', 'fps': fps, 'task': task, 'robot_type': robot_type, 'joint_names': names, 'action_joint_names': action_names, 'dimensions': dimensions, 'numeric_names': numeric_names, 'source_keys': keys, 'source_topics': source_topics, 'source_frame_ids': source_frame_ids, 'sync_audit': audit, 'progress_path': str(progress.path) if progress.path else None, 'origin_log_time_ns': origin, 'episodes': episodes}
    if snapshot: manifest["capture_snapshot"] = snapshot
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
    for key, names in manifest.get('numeric_names', {}).items():
        features[key] = {'dtype': 'float32', 'shape': (len(names),), 'names': names}
    for key, shape in manifest['dimensions'].items():
        features[key] = {'dtype': 'video', 'shape': tuple(shape), 'names': ['height', 'width', 'channels']}
    features['observation.source_time'] = {'dtype': 'float64', 'shape': (len(manifest['source_keys']),), 'names': manifest['source_keys']}
    features['observation.recording_time'] = {'dtype': 'float64', 'shape': (1,), 'names': ['seconds_since_first_log_time']}
    if 'sync_audit' in manifest:
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
    for episode in manifest['episodes']:
        for row in episode:
            frame = {'task': manifest['task']}
            for key, value in row['values'].items():
                frame[key] = np.asarray(Image.open(value).convert('RGB')) if key in manifest['dimensions'] else np.asarray(value, dtype=features[key]['dtype'])
            frame['observation.source_time'] = np.asarray(row['source_times'], dtype=np.float64)
            frame['observation.recording_time'] = np.asarray([row['grid_time']], dtype=np.float64)
            if 'sync_audit' in manifest:
                for field in ('source_available', 'source_valid', 'source_age'):
                    frame['observation.' + field] = np.asarray(row[field], dtype=np.float32)
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
    synchronization['action_joint_names'] = joint_names['action']
    synchronization['action_sources'] = manifest.get('action_sources', [])
    synchronization['action_semantics'] = manifest.get('action_semantics', 'upstream_command_not_execution')
    synchronization['joint_alignment'] = 'observation.state uses feedback joints; action uses commanded joints; ordered by joint name, no feedback substituted for absent commands'
    synchronization['task'] = manifest['task']
    synchronization['source_frame_ids'] = manifest.get('source_frame_ids', {})
    synchronization['sync_audit'] = manifest.get('sync_audit', {})
    synchronization['missing_policy'] = 'all sources: hold last observed value when stale, source_valid=0; before first sample use zero numeric/black image placeholders with source_available=0, source_valid=0 and source_time/source_age=-1; no future samples used'
    synchronization['auxiliary_policy'] = 'causal last observed value; never gates row selection; source_available=0 and value=0/source_time=-1/source_age=-1 before first sample; source_valid=0 when missing or stale; stale velocity is not asserted to remain active'
    synchronization.update({'clock': 'MCAP log_time (wall receive time), not ROS header time', 'selection': 'one recording is one episode, full MCAP log_time span on fixed FPS grid; causal last sample, no rows deleted for missing/stale values', 'state_action_max_age_seconds': 0.2, 'camera_max_age_seconds': 0.25, 'source_mcap_preserved': True, 'images': 'H.264 MP4 with no B frames, CRF 23; short episodes stream-concatenated without padding or dropping frames', 'joint_position_units': 'source ROS JointState units (revolute radians, prismatic meters)' })
    if manifest.get('capture_snapshot'):
        synchronization.update(selection='original state log_time samples, original frame count and frequency',
            missing_policy='absent optional streams remain absent; unsupported per-frame gaps stop export',
            auxiliary_policy='actual causal samples only; no numeric or image placeholders')
    (destination / 'meta' / 'teleop_sync.json').write_text(json.dumps(synchronization, ensure_ascii=False, indent=2))
    return {'path': str(destination), 'format': 'lerobot', 'frames': len(loaded), 'episodes': len(manifest['episodes']), 'fps': manifest['fps'], 'cameras': list(manifest['dimensions']), 'episode_lengths': [len(episode) for episode in manifest['episodes']], 'image_storage': 'mp4_h264', 'sync_audit': manifest.get('sync_audit', {})}


def export_mcap(source, destination, fps=20, task='teleoperation', robot_type='x1', action_sources=None) -> dict:
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
        manifest = prepare_mcap(source, Path(directory), fps, task, robot_type, progress, action_sources)
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
    parser.add_argument('--write-manifest', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        result = write_dataset(Path(args.source), Path(args.destination)) if args.write_manifest else export_mcap(args.source, args.destination, args.fps, args.task, args.robot_type, json.loads(args.action_sources))
        print(json.dumps(result, ensure_ascii=False))
    except Exception as error:
        if not args.write_manifest:
            ExportProgress(progress_path(Path(args.destination).resolve())).update('转换失败', 0, state='error', error=str(error)[-2000:], force=True)
        print(str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
