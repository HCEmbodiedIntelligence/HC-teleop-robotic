"""Write agreed ROS messages directly while recording (no post-capture converter)."""
from array import array as ros_array
import io
import json

import numpy as np
from PIL import Image

from .holobrain_fields import AXES, ACTION_TOPIC, STATE_TOPIC, camera_topic, axes

HEADER = 'std_msgs/Header header\n'
HEADER_DEPS = ('\n' + '=' * 80 + '\nMSG: std_msgs/Header\n'
               'builtin_interfaces/Time stamp\nstring frame_id\n'
               + '=' * 80 + '\nMSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n')
DEFINITIONS = {
    'JointState': HEADER + 'string[] name\nfloat64[] position\nfloat64[] velocity\nfloat64[] effort\n' + HEADER_DEPS,
    'CompressedImage': HEADER + 'string format\nuint8[] data\n' + HEADER_DEPS,
    'Image': HEADER + 'uint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data\n' + HEADER_DEPS,
    'String': 'string data\n',
}


def jpeg(pixels):
    output = io.BytesIO()
    Image.fromarray(pixels).save(output, format='JPEG', quality=95)
    return output.getvalue()


class PublicWriter:
    def __init__(self, writer, cfg):
        self.writer, self.cfg = writer, cfg
        self.channels, self.schemas = {}, {}
        profile = cfg.get('capture', {}).get('profile') if cfg.get('capture') else None
        self.names = {a['source_name']: a['name'] for a in axes(profile)} if profile else {}
        self.robot_type = profile.get('robot_type') if profile else None
        self.robot_info_written = False
        self.calibrations = {}

    def write(self, topic, message, stamp, receive, sequence):
        from rclpy.serialization import serialize_message
        name = type(message).__name__
        if name not in self.schemas:
            namespace = 'std_msgs' if name == 'String' else 'sensor_msgs'
            self.schemas[name] = self.writer.register_schema(
                namespace + '/msg/' + name, 'ros2msg', DEFINITIONS[name].encode())
        key = (topic, name)
        if key not in self.channels:
            self.channels[key] = self.writer.register_channel(topic, 'cdr', self.schemas[name])
        self.writer.add_message(self.channels[key], log_time=int(receive), publish_time=int(stamp),
                                sequence=int(sequence) % 2**32, data=serialize_message(message))

    @staticmethod
    def header(message, stamp, frame=''):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(stamp), 10**9)
        message.header.frame_id = frame
        return message

    def append(self, kind, data, raw, log_ns):
        source = self.cfg['sources'].get(data.get('source_id'), {})
        ident = data.get('source_id')
        receive = data.get('receive_time_ns', log_ns)
        sequence = data.get('source_seq', 0)
        if kind == 'raw' and self.robot_type and not self.robot_info_written:
            from std_msgs.msg import String
            self.write('io_teleop/robot_info', String(data=self.robot_type),
                       data['source_timestamp_ns'], receive, 0)
            self.robot_info_written = True
        if kind == 'raw' and source.get('kind') in {'state', 'action'}:
            from sensor_msgs.msg import JointState
            if data.get('validity_event'):
                return
            names = data.get('joint_names', [])
            values = data.get('payload', {}).get('position', [])
            if not names or len(names) != len(values):
                return
            pairs = {self.names.get(n, n): float(v) for n, v in zip(names, values)}
            order = [name for _, name in AXES if name in pairs] + [n for n in pairs if n not in dict(AXES).values()]
            stamp = data.get('header_timestamp_ns', data['source_timestamp_ns'])
            groups = {}
            for name in order:
                if source['kind'] == 'action' and name in {'R_ban', 'L_ban'}:
                    topic = '/io_teleop/joint_cmd_finger_' + ('right' if name == 'R_ban' else 'left')
                elif source['kind'] == 'state' and name in {'R_ban', 'L_ban'}:
                    topic = '/io_teleop/gripper_state'
                else:
                    topic = ACTION_TOPIC if source['kind'] == 'action' else STATE_TOPIC
                groups.setdefault(topic, []).append(name)
            for topic, selected in groups.items():
                message = self.header(JointState(), stamp, data.get('ros_header_frame_id', ''))
                message.name = selected
                message.position = [pairs[n] for n in selected]
                for field in ('velocity','effort'):
                    recorded=data.get('payload',{}).get(field,[])
                    if len(recorded)==len(names):
                        fields={self.names.get(n,n):float(v) for n,v in zip(names,recorded)}
                        setattr(message,field,[fields[n] for n in selected])
                self.write(topic, message, stamp, receive, sequence)
        elif kind in {'public_rgb', 'depth'} and source.get('kind') == 'rgbd':
            from sensor_msgs.msg import CompressedImage, Image as RosImage
            depth = kind == 'depth'
            metadata = data['depth' if depth else 'rgb']
            stamp = metadata.get('header_timestamp_ns', metadata.get('source_timestamp_ns', data['source_timestamp_ns']))
            frame = metadata.get('coordinate_frame', '')
            if depth:
                array = np.frombuffer(raw, dtype=metadata['dtype']).reshape(metadata['shape'])
                if array.dtype.kind == 'u' and array.dtype.itemsize == 2:
                    output = io.BytesIO()
                    # This runs on the shared MCAP writer for every camera.
                    # Fast lossless PNG keeps three 30 Hz streams from blocking
                    # state/depth persistence; pixel values are unchanged.
                    Image.fromarray(array.astype(np.uint16, copy=False)).save(
                        output, format='PNG', compress_level=1)
                    message = self.header(CompressedImage(), stamp, frame)
                    message.format = '16UC1; png'
                    message.data = ros_array('B', output.getvalue())
                else:
                    # Float depth is retained as float metres, never relabelled uint16.
                    message = self.header(RosImage(), stamp, frame)
                    message.height, message.width = array.shape[:2]
                    message.encoding = '32FC1'
                    array = array.astype('<f4')
                    message.step = message.width * 4
                    message.data = ros_array('B', array.tobytes())
            else:
                message = self.header(CompressedImage(), stamp, frame)
                message.format = 'jpeg'
                # ROS accepts a typed uint8 buffer directly. Passing bytes runs
                # its Python integer/range validators once per image byte.
                message.data = ros_array('B', raw)
            self.write(camera_topic(ident, depth), message, stamp, receive, metadata.get('source_seq', sequence))
            calibration = data.get('calibration')
            if calibration and calibration != self.calibrations.get(ident):
                from std_msgs.msg import String
                value = calibration_document(ident, calibration, receive)
                if value:
                    self.write(camera_topic(ident).removesuffix('/color') + '/calibration',
                               String(data=json.dumps(value, separators=(',', ':'))), stamp, receive, sequence)
                self.calibrations[ident] = calibration


def calibration_document(ident, calibration, receive):
    from .holobrain_fields import camera_name
    result = {'schema':'hc_camera_calibration/v1', 'camera':camera_name(ident),
              'source':'recorded_sensor_msgs/CameraInfo', 'received_at_ros_time_ns':int(receive),
              'resolution':{}}
    for name in ('rgb','depth'):
        info = calibration.get(name, {})
        k = info.get('k', [])
        if len(k) != 9 or k[0] <= 0 or k[4] <= 0:
            return None
        result[name] = {'fx':k[0],'fy':k[4],'cx':k[2],'cy':k[5],
                        'distortion_model':info.get('distortion_model',''),
                        'distortion_coefficients':info.get('d', [])}
        if info.get('distortion_model') == 'plumb_bob' and len(info.get('d',[])) == 5:
            result[name]['distortion_order'] = ['k1','k2','p1','p2','k3']
        result['resolution']['color' if name == 'rgb' else name] = {
            'width':info['width'],'height':info['height']}
    # CameraInfo does not contain the robot-to-camera transform or RGB/depth
    # extrinsics. Those must come from measured calibration, never an identity.
    return result
