"""Close a ROS capture as sensor MCAP plus RGB MP4, retaining every frame timestamp."""
import hashlib
import json
import os
import shutil
from contextlib import ExitStack
from fractions import Fraction
from pathlib import Path
from uuid import NAMESPACE_URL, uuid5

from humanoid_upload.bundle import checksum
from humanoid_upload.common import atomic_json, canonical, read_json, Limiter, UploadError

VERSION = 'hc-topic-video/v1'


def ensure_available():
    import av
    av.codec.Codec('libx264', 'w')


def prepare(source, target, job, *, read_bytes_per_second=16*1024**2):
    """Conversion is offline. Source MCAP is retained for recovery, never overwritten."""
    import av
    import numpy as np
    from mcap.records import Channel, Message, Metadata, Schema
    from mcap.stream_reader import StreamReader
    from mcap.writer import Writer
    from .lerobot_export import decode_rgb

    limiter = Limiter(read_bytes_per_second)
    def file_checksum(path):
        return checksum(path, limiter)

    source, target = Path(source), Path(target)
    if source.is_symlink() or any(p.is_symlink() for p in (*source.parents, target, *target.parents)):
        raise UploadError('UNSAFE_PATH')
    session_file = target / 'session.json'
    if session_file.exists():
        session = read_json(session_file)
        if session['session_id'] != job['recording_id']:
            raise UploadError('ARTIFACT_IDENTITY_CONFLICT', '原始包与录制身份不一致')
        manifest = read_json(target / 'manifest.json')
        if (manifest.get('session_sha256') != hashlib.sha256(canonical(session)).hexdigest()
                or session.get('capture_snapshot') != job.get('capture_snapshot')):
            raise UploadError('ASSET_INTEGRITY', '已整理原始包的采集信息发生变化，不能覆盖')
        for entry in manifest['committed']:
            relative = Path(entry['path'])
            if relative.is_absolute() or '..' in relative.parts or (target / relative).is_symlink():
                raise UploadError('UNSAFE_PATH')
            if file_checksum(target / entry['path']) != {k: entry[k] for k in ('size_bytes', 'sha256', 'crc64')}:
                raise UploadError('ASSET_INTEGRITY', '已整理原始包发生变化，不能覆盖')
    else:
        # A failed staging directory is retained as evidence, and a retry gets a new one.
        from uuid import uuid4
        staging = target.parent / ('.export-' + uuid4().hex)
        staging.mkdir(parents=True)
        videos, counts, channels, schemas, decoders, origins, last_pts = {}, {}, {}, {}, {}, {}, {}
        with ExitStack() as stack:
            original = stack.enter_context(source.open('rb'))
            output = stack.enter_context((staging / 'raw.mcap').open('wb'))
            writer = Writer(output, enable_crcs=True, enable_data_crcs=True)
            writer.start(profile=VERSION)
            source_schemas, source_channels = {}, {}
            # The indexed reader's insertion-order queue can buffer every chunk
            # before yielding a message. Read physical records once instead, so
            # memory follows chunk/encoder size, not the full capture size.
            for record in StreamReader(original, validate_crcs=True).records:
                if isinstance(record, Schema):
                    source_schemas[record.id] = record
                    continue
                if isinstance(record, Channel):
                    source_channels[record.id] = record
                    continue
                if isinstance(record, Metadata):
                    writer.add_metadata(record.name, record.metadata)
                    continue
                if not isinstance(record, Message):
                    continue
                message = record
                channel = source_channels[message.channel_id]
                schema = source_schemas[channel.schema_id] if channel.schema_id else None
                payload, encoding = message.data, channel.message_encoding
                is_image = schema and schema.name.endswith(('/Image', '/CompressedImage'))
                value = None
                if is_image:
                    if channel.message_encoding == 'json':
                        from types import SimpleNamespace
                        value = SimpleNamespace(**json.loads(payload))
                    else:
                        if schema.id not in decoders:
                            try:
                                from rclpy.serialization import deserialize_message
                                from rosidl_runtime_py.utilities import get_message
                                message_type = get_message(schema.name)
                                decoders[schema.id] = lambda data, cls=message_type: deserialize_message(data, cls)
                            except ImportError:
                                from mcap_ros2.decoder import DecoderFactory
                                decoders[schema.id] = DecoderFactory().decoder_for(channel.message_encoding, schema)
                        value = decoders[schema.id](payload)
                image_encoding = str(getattr(value, 'encoding', '')).lower()
                compressed_format = str(getattr(value, 'format', '')).lower()
                depth = (image_encoding in ('16uc1', 'mono16', '32fc1')
                         or compressed_format.split(';')[0].strip() in ('16uc1', 'mono16', '32fc1')
                         or 'depth' in compressed_format)
                if is_image and not depth:
                    pixels = decode_rgb(value, schema.name.endswith('/CompressedImage'))
                    if channel.id not in videos:
                        relative = 'videos/' + hashlib.sha256((channel.topic + ':' + str(channel.id)).encode()).hexdigest()[:16] + '.mp4'
                        (staging / 'videos').mkdir(exist_ok=True)
                        container = stack.enter_context(av.open(str(staging / relative), 'w'))
                        video = container.add_stream('libx264', rate=Fraction(str(job.get('video_fps', 30))))
                        video.width, video.height = pixels.shape[1], pixels.shape[0]
                        video.pix_fmt = 'yuv420p'
                        video.codec_context.thread_count = 1
                        video.time_base = video.codec_context.time_base = Fraction(1, 1_000_000)
                        video.options = {'preset': 'veryfast', 'crf': '18', 'bf': '0'}
                        videos[channel.id] = (container, video, relative)
                        counts[channel.id] = 0
                        origins[channel.id] = message.log_time
                    container, video, relative = videos[channel.id]
                    if (video.height, video.width) != pixels.shape[:2]:
                        raise UploadError('CAMERA_LAYOUT_CHANGED', '同一相机的图像尺寸在录制中改变')
                    frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(pixels), format='rgb24')
                    frame.pts = (message.log_time - origins[channel.id]) // 1000
                    frame.time_base = Fraction(1, 1_000_000)
                    if frame.pts <= last_pts.get(channel.id, -1):
                        raise UploadError('CAMERA_TIME_INVALID', '同一相机的录制时间戳重复或倒退，源 MCAP 已保留')
                    last_pts[channel.id] = frame.pts
                    for packet in video.encode(frame):
                        container.mux(packet)
                    header = getattr(value, 'header', None)
                    stamp = getattr(header, 'stamp', None)
                    source_header = ({'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                                      'frame_id': getattr(header, 'frame_id', '')} if stamp else None)
                    payload = canonical({'video_path': relative, 'frame_index': counts[channel.id],
                                         'height': video.height, 'width': video.width, 'header': source_header})
                    counts[channel.id] += 1
                    encoding = VERSION
                key = (channel.id, encoding)
                if key not in channels:
                    if schema and schema.id not in schemas:
                        schemas[schema.id] = writer.register_schema(schema.name, schema.encoding, schema.data)
                    channels[key] = writer.register_channel(channel.topic, encoding, schemas.get(channel.schema_id, 0), channel.metadata)
                writer.add_message(channels[key], message.log_time, payload, message.publish_time, message.sequence)
            for container, video, _ in videos.values():
                for packet in video.encode():
                    container.mux(packet)
            writer.finish()
        session = {'schema_version': VERSION, 'session_id': job['recording_id'],
                   'capture_started_at': job['started_at'], 'capture_ended_at': job['ended_at'],
                   'capture_snapshot': job.get('capture_snapshot'),
                   'videos': [{'path': entry[2], 'frame_count': counts[key]} for key, entry in videos.items()]}
        correction = source.with_suffix('.correction.json')
        if correction.is_file():
            shutil.copyfile(correction, staging / 'correction.json')
        session_capture = source.with_suffix('.session.json')
        if session_capture.is_file():
            shutil.copyfile(session_capture, staging / 'intervention-session.json')
        committed = [{'path': p.relative_to(staging).as_posix(), **file_checksum(p)}
                     for p in sorted(staging.rglob('*')) if p.is_file()]
        atomic_json(staging / 'manifest.json', {'state': 'completed', 'committed': committed,
                    'session_sha256': hashlib.sha256(canonical(session)).hexdigest()})
        atomic_json(staging / 'session.json', session)
        for path in staging.rglob('*'):
            if path.is_file():
                with path.open('rb') as stream:
                    os.fsync(stream.fileno())
        for path in [*sorted((p for p in staging.rglob('*') if p.is_dir()), reverse=True), staging]:
            fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
            try: os.fsync(fd)
            finally: os.close(fd)
        staging.rename(target)
        fd = os.open(target.parent, os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(fd)
        finally: os.close(fd)
    if not job.get('binding'):
        return target
    binding = job['binding']
    marker_path = target / 'recording-complete.json'
    identity = uuid5(NAMESPACE_URL, VERSION + ':' + job['recording_id'])
    assets = []
    for path in sorted(target.rglob('*')):
        if not path.is_file() or path == marker_path:
            continue
        relative, facts = path.relative_to(target).as_posix(), file_checksum(path)
        assets.append({'path': relative, 'size': facts['size_bytes'], 'sha256': facts['sha256'],
                       'crc64': int(facts['crc64']), 'part_count': 1, 'role': 'AUXILIARY',
                       'camera_id': None, 'media_type': 'video/mp4' if path.suffix == '.mp4' else
                       'application/mcap' if path.suffix == '.mcap' else 'application/json'})
    command = {'schema_version': 'openarm-raw-recording-upload/v1', 'recording_id': job['recording_id'],
               'rollout_id': 'capture-' + identity.hex, 'data_package_id': 'capture-' + identity.hex,
               'collection_job_id': 'capture-' + identity.hex, 'collection_task_id': binding['collection_task_id'],
               'robot_id': binding['robot_id'], 'device_id': job['local_robot']['local_robot_id'],
               'capture_started_at': job['started_at'], 'capture_ended_at': job['ended_at'], 'assets': assets}
    marker = {'schema_version': 'openarm-recording-complete/v2', 'export_id': str(identity),
              'session_id': job['recording_id'], 'robot_id': command['robot_id'],
              'local_robot_id': command['device_id'], 'collection_task_id': command['collection_task_id'],
              'synthetic': False, 'content_sha256': hashlib.sha256(canonical(assets)).hexdigest(),
              'recording_upload': command}
    if marker_path.exists() and read_json(marker_path) != marker:
        raise UploadError('TASK_BINDING_MISMATCH', '已封存原始包不能改变任务归属')
    if not marker_path.exists():
        atomic_json(marker_path, marker)
    return target
