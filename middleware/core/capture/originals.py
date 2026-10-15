"""Archive each driver message independently, before pairing or validation.

This is the ROS receive boundary, not evidence that USB/the driver delivered
every sensor exposure. The original CDR bytes and headers are never rewritten.
"""
from collections import Counter
import os
import shutil
import struct
import threading
import time

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer

from .buffers import ByteQueue
from .holobrain_mcap import DEFINITIONS, HEADER, HEADER_DEPS


FILENAME = 'originals.mcap'
STREAM_TYPES = {
    'rgb_frame': 'realsense2_camera_msgs/msg/ImageMetadata',
    'depth_frame': 'realsense2_camera_msgs/msg/ImageMetadata',
    'rgb': 'sensor_msgs/msg/Image', 'depth': 'sensor_msgs/msg/Image',
    'rgb_meta': 'realsense2_camera_msgs/msg/Metadata',
    'depth_meta': 'realsense2_camera_msgs/msg/Metadata',
    'rgb_info': 'sensor_msgs/msg/CameraInfo', 'depth_info': 'sensor_msgs/msg/CameraInfo',
}
SCHEMAS = {
    'sensor_msgs/msg/Image': DEFINITIONS['Image'],
    'sensor_msgs/msg/JointState': DEFINITIONS['JointState'],
    'std_msgs/msg/String': DEFINITIONS['String'],
    'realsense2_camera_msgs/msg/Metadata': HEADER + 'string json_data\n' + HEADER_DEPS,
    'sensor_msgs/msg/CameraInfo': HEADER + (
        'uint32 height\nuint32 width\nstring distortion_model\nfloat64[] d\n'
        'float64[9] k\nfloat64[9] r\nfloat64[12] p\nuint32 binning_x\nuint32 binning_y\n'
        'sensor_msgs/RegionOfInterest roi\n') + HEADER_DEPS + '\n' + '=' * 80 + (
        '\nMSG: sensor_msgs/RegionOfInterest\nuint32 x_offset\nuint32 y_offset\n'
        'uint32 height\nuint32 width\nbool do_rectify\n'),
}


def _bundle_schema():
    # Include each transitive ROS definition once (MCAP is self-contained).
    separator='\n'+'='*80+'\nMSG: '
    root='sensor_msgs/Image image\nsensor_msgs/CameraInfo camera_info\nstring json_data\n'
    definitions={}
    for typ in ('sensor_msgs/msg/Image','sensor_msgs/msg/CameraInfo'):
        parts=SCHEMAS[typ].split(separator)
        definitions[typ.replace('/msg/','/')]=parts[0]
        for part in parts[1:]:
            name,body=part.split('\n',1)
            definitions[name]=body
    return root+''.join(separator+name+'\n'+body for name,body in definitions.items())


SCHEMAS['realsense2_camera_msgs/msg/ImageMetadata']=_bundle_schema()


def camera_topics(camera,bundled=False):
    prefix = '/' + camera['namespace'] + '/' + camera['camera_name']
    if bundled:
        topics={'rgb_frame':prefix+'/color/image_metadata'}
        if camera.get('depth_enabled',True):topics['depth_frame']=prefix+'/depth/image_metadata'
        return topics
    topics = {'rgb': camera['rgb_topic'], 'rgb_meta': prefix + '/color/metadata',
              'rgb_info': prefix + '/color/camera_info'}
    if camera.get('depth_enabled', True):
        topics.update(depth=camera['depth_topic'], depth_meta=prefix + '/depth/metadata',
                      depth_info=prefix + '/depth/camera_info')
    return topics


def source_topics(cfg):
    """Freeze identity/schema once; never infer provenance from paired output."""
    result = {}
    for ident, source in cfg['sources'].items():
        for name, topic in source.get('original_topics', {}).items():
            result[topic] = {'source_id': ident, 'stream': name, 'type': STREAM_TYPES[name]}
        if source['transport'] == 'ros_joint_state':
            result[source['topic']] = {'source_id': ident, 'stream': source['kind'],
                                       'type': 'sensor_msgs/msg/JointState'}
        elif source.get('evidence_topic'):
            result[source['evidence_topic']] = {'source_id': ident, 'stream': 'action_evidence',
                                               'type': 'std_msgs/msg/String'}
    return result


def header_stamp(data, message_type):
    # All supported messages except String start with Header. An invalid header
    # must not prevent retention; its original bytes remain in the message.
    if message_type == 'std_msgs/msg/String' or len(data) < 12 or data[:2] not in (b'\0\0', b'\0\1'):
        return 0
    sec, nanosec = struct.unpack_from('<iI' if data[1] else '>iI', data, 4)
    return sec * 10**9 + nanosec if sec >= 0 and nanosec < 10**9 else 0


def compress_archive(source, destination):
    """Offline compression preserves every message, schema and both MCAP times."""
    with source.open('rb') as incoming, destination.open('xb') as outgoing:
        reader=make_reader(incoming,validate_crcs=True)
        writer=Writer(outgoing,compression=CompressionType.ZSTD,enable_crcs=True,enable_data_crcs=True)
        writer.start(profile='ros2',library='humanoid_manager/original-ros-v1')
        schemas,channels={},{}
        # Preserve empty channels too: they identify streams that never arrived.
        summary=reader.get_summary()
        if summary is None:
            raise ValueError('原始 ROS 文件缺少完成索引')
        for ident,schema in summary.schemas.items():
            schemas[ident]=writer.register_schema(schema.name,schema.encoding,schema.data)
        for ident,channel in summary.channels.items():
            channels[ident]=writer.register_channel(channel.topic,channel.message_encoding,
                schemas.get(channel.schema_id,0),metadata=channel.metadata)
        for _,channel,message in reader.iter_messages(log_time_order=False):
            writer.add_message(channels[channel.id],log_time=message.log_time,
                publish_time=message.publish_time,sequence=message.sequence,data=message.data)
        writer.finish()
        outgoing.flush()
        os.fsync(outgoing.fileno())


class OriginalArchive:
    def __init__(self, path, cfg, fail):
        self.path, self.cfg, self.fail = path, cfg, fail
        self.topics = source_topics(cfg)
        self.queue = ByteQueue(cfg['buffers']['writer_items'], cfg['buffers']['ros_assembly_bytes'])
        self.counts = Counter()
        self.error = None
        self.closed = False
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self.run, name='original-ros-writer', daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(5) or self.error:
            raise RuntimeError(self.error or '原始 ROS 写盘启动超时')

    def submit(self, topic, data, receive_ns):
        if topic not in self.topics:
            raise ValueError('未配置的原始 ROS 话题: ' + topic)
        # raw=True subscriptions supply immutable CDR bytes, without decoding or
        # copying each image pixel in this callback. No pairing cache is involved.
        data = bytes(data)
        return self.queue.offer((topic, data, receive_ns), len(data) + 128)

    def run(self):
        try:
            with (self.path / FILENAME).open('xb') as stream:
                writer = Writer(stream, compression=CompressionType.NONE, chunk_size=1024*1024,
                                enable_crcs=True, enable_data_crcs=True)
                writer.start(profile='ros2', library='humanoid_manager/original-ros-v1')
                schemas, channels = {}, {}
                for topic, spec in self.topics.items():
                    typ = spec['type']
                    if typ not in schemas:
                        schemas[typ] = writer.register_schema(typ, 'ros2msg', SCHEMAS[typ].encode())
                    channels[topic] = writer.register_channel(topic, 'cdr', schemas[typ],
                        metadata={**spec, 'capture_boundary': 'driver_ros_subscription',
                                  'association_required': 'false'})
                self.ready.set()
                last_flush = time.monotonic()
                while not self.queue.closed or self.queue.items:
                    item = self.queue.get()
                    if item is not None:
                        topic, data, receive_ns = item
                        writer.add_message(channels[topic], log_time=receive_ns,
                            publish_time=header_stamp(data, self.topics[topic]['type']),
                            sequence=self.counts[topic] % 2**32, data=data)
                        self.counts[topic] += 1
                    now = time.monotonic()
                    if now - last_flush >= self.cfg['storage']['numeric_flush_seconds']:
                        stream.flush()
                        if shutil.disk_usage(self.path).free < self.cfg['storage']['minimum_free_bytes']:
                            raise OSError('磁盘可用空间低于配置下限')
                        last_flush = now
                writer.finish()
                stream.flush()
                if self.cfg['storage']['fsync_on_commit']:
                    os.fsync(stream.fileno())
            self.closed = True
        except Exception as error:
            self.error = f'{type(error).__name__}: {error}'
            self.fail('original_storage_failure:' + self.error)
        finally:
            self.ready.set()

    def stop(self):
        self.queue.close()
        if self.thread.ident is not None:
            self.thread.join(30)
        if self.thread.is_alive():
            self.fail('original_writer_close_timeout')

    def status(self):
        return {'path': FILENAME, 'boundary': 'driver_ros_subscription',
                'counts': dict(self.counts), 'queue': self.queue.status(),
                'closed': self.closed, 'error': self.error}
