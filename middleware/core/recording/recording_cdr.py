"""Small CDR1 readers for recorder labels, without ROS/NumPy initialization.

Only standard JointState and String are accepted. Original header bytes and
endianness are retained when selecting measured positions for a derived label.
This module has no ROS context, publisher, or control-policy dependency.
"""
from dataclasses import dataclass
import json
import os
import struct

try:
    if os.environ.get('HC_NATIVE_EXT', '1') != '0':
        from . import _recording_cdr_cpp as _native
    else:
        _native = None
except ImportError:
    _native = None

USING_NATIVE_EXT = _native is not None


class Reader:
    def __init__(self, raw):
        if not isinstance(raw, bytes) or len(raw) < 4 or raw[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('unsupported CDR representation')
        self.raw, self.offset = raw, 4
        self.endian = '<' if raw[1] else '>'

    def align(self, size):
        self.offset += (-(self.offset-4)) % size

    def uint(self):
        self.align(4)
        if self.offset+4 > len(self.raw):
            raise ValueError('truncated CDR length')
        value = struct.unpack_from(self.endian+'I', self.raw, self.offset)[0]
        self.offset += 4
        return value

    def string(self):
        count = self.uint()
        end = self.offset+count
        if not count or end > len(self.raw) or self.raw[end-1] != 0:
            raise ValueError('invalid CDR string')
        value = self.raw[self.offset:end-1].decode('utf-8')
        self.offset = end
        return value

    def doubles(self):
        count = self.uint()
        if count:
            self.align(8)
        end = self.offset+8*count
        if end > len(self.raw):
            raise ValueError('truncated CDR double sequence')
        values = struct.unpack_from(self.endian+str(count)+'d', self.raw, self.offset) if count else ()
        self.offset = end
        return values


@dataclass(frozen=True)
class JointFields:
    name: tuple
    position: tuple
    header_prefix: bytes


def read_string(raw):
    return Reader(raw).string()


def write_string(value):
    """Encode a standard ROS2 String as little-endian CDR1 without ROS imports."""
    encoded = value.encode('utf-8') + b'\0'
    return b'\x00\x01\x00\x00' + struct.pack('<I', len(encoded)) + encoded


def read_event(raw, message_encoding, schema_name=''):
    """Read historical JSON events and standard String/CDR recording events."""
    if message_encoding == 'json':
        return json.loads(raw)
    if message_encoding == 'cdr' and schema_name == 'std_msgs/msg/String':
        return json.loads(read_string(raw))
    raise ValueError('unsupported recording event encoding/schema')


def _parse_joint_state(raw):
    reader = Reader(raw)
    if len(raw) < 12:
        raise ValueError('truncated CDR Header')
    reader.offset = 12  # Header.stamp: int32 sec, uint32 nanosec.
    reader.string()  # Preserve the original frame_id, including its padding.
    prefix = raw[:reader.offset]
    count = reader.uint()
    if count > (len(raw)-reader.offset)//5:
        raise ValueError('impossible CDR joint count')
    names = tuple(reader.string() for _ in range(count))
    positions = reader.doubles()
    velocity = reader.doubles()  # Labels omit motion/effort but validate their bounds.
    effort = reader.doubles()
    # Humble's generated size calculation reserves alignment for empty arrays
    # that Fast-CDR does not write. The ROS serializer can leave up to four
    # unused trailing bytes per empty array. Never interpret them as fields.
    padding_budget = 4*sum(not values for values in (positions, velocity, effort))
    if not 0 <= len(raw)-reader.offset <= padding_budget:
        raise ValueError('unexpected trailing JointState data')
    return JointFields(names, positions, prefix), reader.offset


def read_joint_state(raw):
    return _parse_joint_state(raw)[0]


def trim_joint_state_cdr(raw):
    """Strip ROS 2 Humble over-allocated alignment bytes from JointState CDR.

    ``rclpy.serialization.serialize_message`` reserves 4-byte alignment slack
    for empty ``float64[]`` sequences and leaves those unused bytes at the end
    of the returned buffer, whereas native DDS ``Publisher.publish(msg)``
    omits them. Strict CDR readers such as Foxglove flag the extra bytes as a
    schema mismatch.
    """
    _, end_offset = _parse_joint_state(raw)
    return raw if end_offset == len(raw) else raw[:end_offset]


def position_label(header_prefix, names, positions):
    reader = Reader(header_prefix)
    if len(names) != len(positions):
        raise ValueError('joint names/positions length mismatch')
    raw = bytearray(header_prefix)

    def align(size):
        raw.extend(b'\0' * ((-(len(raw)-4)) % size))

    def uint(value):
        align(4)
        raw.extend(struct.pack(reader.endian+'I', value))

    uint(len(names))
    for name in names:
        encoded = name.encode('utf-8')+b'\0'
        uint(len(encoded)); raw.extend(encoded)
    uint(len(positions))
    if positions:
        align(8)
        raw.extend(struct.pack(reader.endian+str(len(positions))+'d', *positions))
    uint(0); uint(0)  # Empty velocity/effort: measured motion is not a hold command.
    return bytes(raw)


def serialize_joint_state(sec, nanosec, frame_id, names, positions, velocities=(), efforts=()):
    """Encode sensor_msgs/msg/JointState as trimmed little-endian CDR1 without rclpy."""
    sec_i = int(sec)
    nsec_i = int(nanosec)
    if not -2147483648 <= sec_i <= 2147483647:
        raise ValueError('JointState header.stamp.sec out of int32 range')
    if not 0 <= nsec_i <= 4294967295:
        raise ValueError('JointState header.stamp.nanosec out of uint32 range')
    raw = bytearray(b'\x00\x01\x00\x00')
    raw.extend(struct.pack('<iI', sec_i, nsec_i))

    def align(size):
        raw.extend(b'\0' * ((-(len(raw) - 4)) % size))

    def uint(value):
        align(4)
        raw.extend(struct.pack('<I', value))

    fid = frame_id.encode('utf-8') + b'\0'
    uint(len(fid))
    raw.extend(fid)

    names_seq = tuple(names)
    uint(len(names_seq))
    for name in names_seq:
        encoded = name.encode('utf-8') + b'\0'
        uint(len(encoded))
        raw.extend(encoded)

    for seq in (positions, velocities, efforts):
        vals = tuple(seq)
        uint(len(vals))
        if vals:
            align(8)
            raw.extend(struct.pack('<' + str(len(vals)) + 'd', *vals))
    return bytes(raw)


# Retain pure-Python reference implementations for parity testing & fallback.
_py_read_string = read_string
_py_write_string = write_string
_py_read_joint_state = read_joint_state
_py_trim_joint_state_cdr = trim_joint_state_cdr
_py_position_label = position_label
_py_serialize_joint_state = serialize_joint_state

if _native is not None:
    read_string = _native.read_string
    write_string = _native.write_string

    def read_joint_state(raw):  # noqa: F811
        names, positions, prefix, _ = _native.parse_joint_state(raw)
        return JointFields(names, positions, prefix)

    trim_joint_state_cdr = _native.trim_joint_state_cdr
    position_label = _native.position_label
    serialize_joint_state = _native.serialize_joint_state
