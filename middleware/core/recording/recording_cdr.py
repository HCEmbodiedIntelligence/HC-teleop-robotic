"""Small CDR1 readers for recorder labels, without ROS/NumPy initialization.

Only standard JointState and String are accepted. Original header bytes and
endianness are retained when selecting measured positions for a derived label.
This module has no ROS context, publisher, or control-policy dependency.
"""
from dataclasses import dataclass
import struct
import json


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


def read_joint_state(raw):
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
    return JointFields(names, positions, prefix)


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
