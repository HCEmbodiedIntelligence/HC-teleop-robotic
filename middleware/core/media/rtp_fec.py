"""RFC 5109 ULPFEC level-0 parity and RFC 2198 single-primary RED wire codec.

This is a wire codec, not negotiation, pacing, congestion control or SRTP. Protect
serialized *non-RED*, pre-SRTP media packets; native libwebrtc removes RED before
FEC decoding. FEC payloads have a 10-byte FEC + 4-byte short-mask ULP header.
A RED/FEC packet adds its RTP header and one RED byte on top of this payload.
Callers MUST reserve this overhead plus SRTP/IP/UDP overhead in their path MTU.

Sources (algorithm/field offsets cross-checked against native libwebrtc):
https://www.rfc-editor.org/rfc/rfc5109.html sections 7.3, 7.4, 8, 9
https://www.rfc-editor.org/rfc/rfc2198.html section 3
https://chromium.googlesource.com/external/webrtc/+/HEAD/modules/rtp_rtcp/source/ulpfec_header_reader_writer.cc
https://chromium.googlesource.com/external/webrtc/+/HEAD/modules/rtp_rtcp/source/ulpfec_receiver_impl.cc

Native libwebrtc zeros mutable header extension values before parity decoding.
Use mutable_extension_ids for negotiated abs-send-time/TWCC/transmission-offset
IDs (or explicitly canonicalize before calling). They are restored as zeros in
recovered packets, not the original per-transmission timestamps. Video-timing
has partially mutable semantics and must be canonicalized by the integration.
No native receiver interoperability is asserted without an integration test.
"""
from __future__ import annotations

import struct
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass


FEC_HEADER_SIZE = 14
SHORT_MASK_PACKETS = 16


@dataclass(frozen=True)
class RtpLayout:
    sequence_number: int
    ssrc: int
    header_size: int
    payload_end: int


def rtp_layout(packet: bytes) -> RtpLayout:
    """Validate complete plaintext RTP including CSRCs, extensions and padding."""
    if not isinstance(packet, bytes) or len(packet) < 12 or packet[0] >> 6 != 2:
        raise ValueError('Expected a complete RTP v2 bytes packet')
    header_size = 12 + (packet[0] & 15) * 4
    if header_size > len(packet):
        raise ValueError('Truncated RTP CSRC list')
    if packet[0] & 0x10:
        if header_size + 4 > len(packet):
            raise ValueError('Truncated RTP extension header')
        words = struct.unpack_from('!H', packet, header_size + 2)[0]
        header_size += 4 + words * 4
        if header_size > len(packet):
            raise ValueError('Truncated RTP extension data')
    payload_end = len(packet)
    if packet[0] & 0x20:
        padding_size = packet[-1]
        if not padding_size or padding_size > len(packet) - header_size:
            raise ValueError('Invalid RTP padding')
        payload_end -= padding_size
    return RtpLayout(struct.unpack_from('!H', packet, 2)[0],
                     struct.unpack_from('!I', packet, 8)[0], header_size, payload_end)


def zero_mutable_extensions(packet: bytes, extension_ids: Iterable[int] = ()) -> bytes:
    """Zero full extension data values by negotiated ID, preserving wire layout.

    Supports RFC 8285 one-byte (0xBEDE) and two-byte (0x100x) extension formats.
    IDs must only identify extensions that libwebrtc treats as wholly mutable.
    Unknown profiles are rejected when canonicalization was requested.
    """
    rtp_layout(packet)
    ids = frozenset(extension_ids)
    if any(isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 255 for value in ids):
        raise ValueError('Extension IDs must be integers in 1..255')
    if not ids or not packet[0] & 0x10:
        return packet
    offset = 12 + (packet[0] & 15) * 4
    profile, words = struct.unpack_from('!HH', packet, offset)
    cursor, end = offset + 4, offset + 4 + words * 4
    result = bytearray(packet)
    while cursor < end:
        element = packet[cursor]
        cursor += 1
        if element == 0:  # A padding octet in either extension format.
            continue
        if profile == 0xBEDE:
            extension_id = element >> 4
            if extension_id == 15:
                break
            length = (element & 15) + 1
        elif profile & 0xFFF0 == 0x1000:
            extension_id = element
            if cursor >= end:
                raise ValueError('Truncated two-byte RTP extension element')
            length = packet[cursor]
            cursor += 1
        else:
            raise ValueError('Cannot canonicalize unknown RTP extension profile')
        if cursor + length > end:
            raise ValueError('Truncated RTP extension element data')
        if extension_id in ids:
            result[cursor:cursor + length] = bytes(length)
        cursor += length
    return bytes(result)


def encode_ulpfec(packets: Sequence[bytes], *, mutable_extension_ids: Iterable[int] = (),
                  max_payload_size: int | None = None) -> bytes:
    """Return one full-protection ULPFEC payload for 1..16 same-SSRC packets.

    Input may be unordered and span sequence-number wrap. Sequence-number spread
    must be at most 15, including holes (e.g. 65535,0,2). Duplicate sequences,
    mixed SSRCs and malformed RTP are rejected. max_payload_size bounds only the
    returned FEC payload; reserve outer RED/RTP/SRTP overhead separately.
    """
    if not 1 <= len(packets) <= SHORT_MASK_PACKETS:
        raise ValueError('A short FEC mask protects 1..16 packets')
    ids = tuple(mutable_extension_ids)
    normalized = [zero_mutable_extensions(packet, ids) for packet in packets]
    layouts = [rtp_layout(packet) for packet in normalized]
    sequences = [layout.sequence_number for layout in layouts]
    if len(set(sequences)) != len(sequences):
        raise ValueError('Duplicate protected RTP sequence number')
    if len({layout.ssrc for layout in layouts}) != 1:
        raise ValueError('Protected RTP packets must share one SSRC')
    bases = [base for base in sequences if all(((seq - base) & 0xFFFF) < 16 for seq in sequences)]
    if not bases:
        raise ValueError('Protected sequence-number span exceeds short mask')
    base = bases[0]
    protection_length = max(len(packet) - 12 for packet in normalized)
    if protection_length > 65535:
        raise ValueError('RTP tail exceeds ULPFEC 16-bit protection length')
    if max_payload_size is not None and FEC_HEADER_SIZE + protection_length > max_payload_size:
        raise ValueError('FEC payload exceeds caller MTU budget')
    first, second, timestamp, length, mask = 0, 0, 0, 0, 0
    parity = bytearray(protection_length)
    for packet, layout in zip(normalized, layouts):
        first ^= packet[0] & 0x3F  # P/X/CC; E=L=0.
        second ^= packet[1]  # M/PT.
        timestamp ^= struct.unpack_from('!I', packet, 4)[0]
        length ^= len(packet) - 12  # Includes CSRC, extension and padding.
        mask |= 1 << (15 - ((layout.sequence_number - base) & 0xFFFF))
        for index, value in enumerate(packet[12:]):
            parity[index] ^= value
    return struct.pack('!BBHIHHH', first, second, base, timestamp, length,
                       protection_length, mask) + bytes(parity)


def protected_sequences(fec_payload: bytes) -> tuple[int, ...]:
    """Read and validate our supported one-level short-mask FEC payload."""
    if not isinstance(fec_payload, bytes) or len(fec_payload) < FEC_HEADER_SIZE:
        raise ValueError('Truncated ULPFEC payload')
    if fec_payload[0] & 0xC0:
        raise ValueError('Only E=0/L=0 ULPFEC payloads are supported')
    base, = struct.unpack_from('!H', fec_payload, 2)
    protection_length, mask = struct.unpack_from('!HH', fec_payload, 10)
    if len(fec_payload) != FEC_HEADER_SIZE + protection_length:
        raise ValueError('Truncated or multi-level ULPFEC payload')
    if not mask & 0x8000:
        raise ValueError('Level-0-only FEC mask must include sequence base')
    return tuple((base + index) & 0xFFFF for index in range(16) if mask & (1 << (15 - index)))


def recover_ulpfec(fec_payload: bytes, received_packets: Iterable[bytes] | Mapping[int, bytes],
                   ssrc: int, *, mutable_extension_ids: Iterable[int] = ()) -> bytes | None:
    """Recover exactly one missing protected RTP packet, else return None.

    SSRC comes from the enclosing RED/FEC RTP header and is not parity-coded.
    Unrelated sequence numbers are ignored; same-sequence conflicting duplicates
    and wrong SSRCs are rejected. This decoder never emits partial recoveries.
    Parity provides erasure recovery, not integrity/authentication: verify SRTP
    before supplying media/FEC input. Payload corruption is not reliably detected.
    """
    if isinstance(ssrc, bool) or not isinstance(ssrc, int) or not 0 <= ssrc <= 0xFFFFFFFF:
        raise ValueError('SSRC must be an unsigned 32-bit integer')
    protected = protected_sequences(fec_payload)
    protected_set = frozenset(protected)
    values = received_packets.values() if isinstance(received_packets, Mapping) else received_packets
    ids = tuple(mutable_extension_ids)
    received = {}
    for packet in values:
        layout = rtp_layout(packet)
        if layout.sequence_number not in protected_set:
            continue
        if layout.ssrc != ssrc:
            raise ValueError('Received protected packet has wrong SSRC')
        packet = zero_mutable_extensions(packet, ids)
        if layout.sequence_number in received and received[layout.sequence_number] != packet:
            raise ValueError('Conflicting duplicate protected RTP packet')
        received[layout.sequence_number] = packet
    missing = [seq for seq in protected if seq not in received]
    if len(missing) != 1:
        return None
    first, second, _, timestamp, length, protection_length, _ = struct.unpack_from('!BBHIHHH', fec_payload)
    parity = bytearray(fec_payload[FEC_HEADER_SIZE:])
    for packet in received.values():
        if len(packet) - 12 > protection_length:
            raise ValueError('Received RTP exceeds FEC protection length')
        first ^= packet[0] & 0x3F
        second ^= packet[1]
        timestamp ^= struct.unpack_from('!I', packet, 4)[0]
        length ^= len(packet) - 12
        for index, value in enumerate(packet[12:]):
            parity[index] ^= value
    if length > protection_length:
        raise ValueError('FEC does not fully protect recovered RTP length')
    recovered = struct.pack('!BBHII', 0x80 | first, second, missing[0], timestamp, ssrc) + bytes(parity[:length])
    rtp_layout(recovered)
    return recovered


def wrap_red_payload(payload: bytes, block_payload_type: int) -> bytes:
    """RFC 2198 F=0 single-primary payload; caller constructs outer RTP."""
    if (isinstance(block_payload_type, bool) or not isinstance(block_payload_type, int)
            or not 0 <= block_payload_type <= 127):
        raise ValueError('RED block payload type must be in 0..127')
    if not isinstance(payload, bytes):
        raise ValueError('RED payload must be bytes')
    return bytes((block_payload_type,)) + payload


def unwrap_red_payload(payload: bytes) -> tuple[int, bytes]:
    """Return (block PT, payload); reject multiple-block RED explicitly."""
    if not isinstance(payload, bytes) or not payload:
        raise ValueError('Truncated RED primary block header')
    if payload[0] & 0x80:
        raise ValueError('Only single-primary RED blocks are supported')
    return payload[0], payload[1:]


def red_wrap(packet: bytes, red_payload_type: int) -> bytes:
    """Wrap a complete RTP packet in RED, preserving CSRC/extension/padding."""
    layout = rtp_layout(packet)
    block = wrap_red_payload(packet[layout.header_size:layout.payload_end], packet[1] & 0x7F)
    wrap_red_payload(b'', red_payload_type)  # Validate outer PT, too.
    header = bytearray(packet[:layout.header_size])
    header[1] = (header[1] & 0x80) | red_payload_type
    return bytes(header) + block + packet[layout.payload_end:]


def red_unwrap(packet: bytes) -> bytes:
    """Restore complete original RTP from an identified RED RTP packet."""
    layout = rtp_layout(packet)
    payload_type, payload = unwrap_red_payload(packet[layout.header_size:layout.payload_end])
    header = bytearray(packet[:layout.header_size])
    header[1] = (header[1] & 0x80) | payload_type
    return bytes(header) + payload + packet[layout.payload_end:]
