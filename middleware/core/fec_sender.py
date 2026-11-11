"""Per-peer VP8 / RED / ULPFEC sender over aiortc's plaintext SRTP boundary.

No installed dependency is patched. Native browser/Unity libwebrtc receives
standard RFC 2198/5109 payloads; unsupported offers retain ordinary media/RTX.
Private aiortc adapters are isolated here and exercised by peer tests.
"""
from collections import OrderedDict
import copy
import struct
import time

from .rtp_fec import encode_ulpfec, red_wrap, rtp_layout, wrap_red_payload


class FecSender:
    def __init__(self, sender, red_pt, fec_pt, red_rtx_pt=None, group_size=5,
                 media_bitrate_ratio=.8, mutable_extension_ids=()):
        self.sender = sender
        self.red_pt, self.fec_pt, self.red_rtx_pt = red_pt, fec_pt, red_rtx_pt
        self.group_size = group_size
        self.media_bitrate_ratio = media_bitrate_ratio
        self.mutable_extension_ids = tuple(mutable_extension_ids)
        self.group = []
        self.cache = OrderedDict()
        self.next_sequence = None
        self.rtx_sequence = 0
        self.base_send = sender.transport._send_rtp
        self.base_feedback = sender._handle_rtcp_packet
        self.state = dict(requested=True, state='enabled', codec='VP8', scheme='RED/ULPFEC',
                          red_payload_type=red_pt, ulpfec_payload_type=fec_pt,
                          red_rtx_payload_type=red_rtx_pt, group_size=group_size,
                          media_bitrate_ratio=media_bitrate_ratio, media_packets=0,
                          fec_packets=0, media_bytes=0, fec_bytes=0, retransmissions=0,
                          ignored_nacks=0, protection_skipped=0,
                          recovery_verified=False, reason=None)

    def snapshot(self):
        result = dict(self.state)
        result['overhead_pct'] = round(100 * result['fec_bytes'] / result['media_bytes'], 2) if result['media_bytes'] else None
        return result

    def _sequence(self):
        seq = self.next_sequence
        self.next_sequence = (seq + 1) & 0xffff
        return seq

    def _remember(self, seq, packet):
        self.cache[seq] = (time.monotonic(), packet)
        while len(self.cache) > 512:
            self.cache.popitem(last=False)

    async def send(self, data):
        from aiortc.rtp import is_rtcp
        if len(data) < 12 or is_rtcp(data) or struct.unpack_from('!I', data, 8)[0] != self.sender._ssrc:
            return await self.base_send(data)
        # Only the main sender enters this wrapper. Repairs use base_send, never
        # receive a new media sequence number and never recursively generate FEC.
        layout = rtp_layout(data)
        if self.next_sequence is None:
            self.next_sequence = layout.sequence_number
        seq = self._sequence()
        primary = bytearray(data)
        struct.pack_into('!H', primary, 2, seq)
        primary = bytes(primary)
        wire = red_wrap(primary, self.red_pt)
        await self.base_send(wire)
        self._remember(seq, wire)
        self.state['media_packets'] += 1
        self.state['media_bytes'] += len(wire)
        # aiortc counts primary packets after this method returns; count only
        # additional RED octets and FEC here so SR counters include wire data.
        self.sender._RTCRtpSender__octet_count += 1
        # Reserve RTP + RED + ULPFEC overhead within a 1400-byte plaintext RTP
        # limit, leaving room for SRTP/UDP/IPv6 on an ordinary 1500-byte path.
        projected_fec_size = layout.header_size + 1 + 14 + len(primary) - 12
        if projected_fec_size > 1400:
            self.group.clear()
            self.state['protection_skipped'] += 1
            return
        self.group.append(primary)
        if len(self.group) >= self.group_size or data[1] & 0x80:
            await self._flush(primary)

    async def _flush(self, last):
        group, self.group = self.group, []
        if not group:
            return
        try:
            payload = encode_ulpfec(group, mutable_extension_ids=self.mutable_extension_ids)
        except ValueError as exc:
            self.state['protection_skipped'] += 1
            self.state['reason'] = 'Packet could not be protected: ' + str(exc)
            return
        layout = rtp_layout(last)
        # Retain negotiated MID/extensions for BUNDLE demux, but FEC is never a
        # frame marker and carries no RTP padding of its own.
        header = bytearray(last[:layout.header_size])
        header[0] &= ~0x20
        header[1] = self.red_pt
        struct.pack_into('!H', header, 2, 0)
        packet = bytes(header) + wrap_red_payload(payload, self.fec_pt)
        if len(packet) > 1400:
            self.state['protection_skipped'] += 1
            return
        packet = bytearray(packet)
        struct.pack_into('!H', packet, 2, self._sequence())
        packet = bytes(packet)
        await self.base_send(packet)
        self.state['fec_packets'] += 1
        self.state['fec_bytes'] += len(packet)
        self.sender._RTCRtpSender__packet_count += 1
        self.sender._RTCRtpSender__octet_count += len(packet) - layout.header_size

    async def retransmit(self, sequence_number):
        from aiortc.rtp import RtpPacket, wrap_rtx
        entry = self.cache.get(sequence_number)
        if entry is None or time.monotonic() - entry[0] > 2:
            # Missing parity is not media and must never be retransmitted.
            self.state['ignored_nacks'] += 1
            return
        packet = entry[1]
        if self.red_rtx_pt is not None:
            parsed = RtpPacket.parse(packet, self.sender._RTCRtpSender__rtp_header_extensions_map)
            repair = wrap_rtx(parsed, payload_type=self.red_rtx_pt,
                              sequence_number=self.rtx_sequence, ssrc=self.sender._rtx_ssrc)
            self.rtx_sequence = (self.rtx_sequence + 1) & 0xffff
            packet = repair.serialize(self.sender._RTCRtpSender__rtp_header_extensions_map)
        await self.base_send(packet)
        if self.red_rtx_pt is None:
            self.sender._RTCRtpSender__packet_count += 1
            self.sender._RTCRtpSender__octet_count += len(packet) - rtp_layout(packet).header_size
        self.state['retransmissions'] += 1

    async def feedback(self, packet):
        from aiortc.rtp import RtcpPsfbPacket, RTCP_PSFB_APP, unpack_remb_fci
        await self.base_feedback(packet)
        if isinstance(packet, RtcpPsfbPacket) and packet.fmt == RTCP_PSFB_APP:
            try:
                bitrate, ssrcs = unpack_remb_fci(packet.fci)
            except ValueError:
                return
            encoder = getattr(self.sender, '_RTCRtpSender__encoder', None)
            if self.sender._ssrc in ssrcs and encoder is not None:
                # Reserve bandwidth using observed overhead, capped by the
                # configured media share. At startup reserve at least 20%.
                total = self.state['media_bytes'] + self.state['fec_bytes']
                observed = self.state['media_bytes'] / total if total else self.media_bitrate_ratio
                share = min(self.media_bitrate_ratio, observed * .95)
                encoder.target_bitrate = int(bitrate * share)
                self.state['feedback_bitrate'] = bitrate
                self.state['media_target_bitrate'] = encoder.target_bitrate
                self.state['effective_media_ratio'] = share
                self.state['minimum_bitrate_limited'] = encoder.target_bitrate > int(bitrate * share)

    def install(self):
        self.sender.transport._send_rtp = self.send
        self.sender._retransmit = self.retransmit
        self.sender._handle_rtcp_packet = self.feedback
        return self


def configure_fec(pc, transceiver, sender, offer_sdp, options):
    """Append only peer-offered FEC formats to this send-only transceiver.

    Do this after setRemoteDescription, before createAnswer and sender start.
    Never add global CODECS entries that pretend aiortc can receive RED/FEC.
    """
    options = options or {}
    disabled = {'requested': bool(options.get('enabled', False)), 'state': 'disabled',
                'scheme': 'RED/ULPFEC', 'reason': None, 'recovery_verified': False}
    if not disabled['requested']:
        return None, disabled
    if not transceiver._codecs or transceiver._codecs[0].mimeType.lower() != 'video/vp8':
        disabled.update(state='unsupported_codec', reason='FEC requires VP8; Unity libwebrtc M116 does not support the H264+ULPFEC combination. Select camera.codec: VP8 for FEC; Jetson/NVENC H264 remains available with RTX.')
        return None, disabled
    try:
        from aiortc.sdp import SessionDescription
        remote = SessionDescription.parse(offer_sdp)
        media = next(item for item in remote.media if item.kind == 'video' and item.rtp.muxId == transceiver.mid)
        red = next(c for c in media.rtp.codecs if c.mimeType.lower() == 'video/red' and c.clockRate == 90000)
        fec = next(c for c in media.rtp.codecs if c.mimeType.lower() == 'video/ulpfec' and c.clockRate == 90000)
        # Only the single-primary RED form with no fmtp restriction is used.
        if red.parameters:
            raise ValueError('RED fmtp restrictions are not supported by this sender')
        auxiliary_pts = (red.payloadType, fec.payloadType)
        media_pts = {c.payloadType for c in transceiver._codecs if c.mimeType.lower() != 'video/rtx'}
        if (len(set(auxiliary_pts)) != 2 or any(isinstance(pt, bool) or not isinstance(pt, int) or not 96 <= pt <= 127 or pt in media_pts for pt in auxiliary_pts)):
            raise ValueError('invalid or conflicting RED/ULPFEC payload types')
        red_rtx = next((c for c in media.rtp.codecs if c.mimeType.lower() == 'video/rtx' and c.parameters.get('apt') == red.payloadType), None)
        if red_rtx is not None and (red_rtx.clockRate != 90000 or not isinstance(red_rtx.payloadType, int)
                or not 96 <= red_rtx.payloadType <= 127 or red_rtx.payloadType in media_pts | set(auxiliary_pts)):
            raise ValueError('invalid or conflicting RED RTX payload type')
    except (StopIteration, ValueError) as exc:
        disabled.update(state='peer_unsupported', reason='Peer did not offer compatible RED + ULPFEC: ' + str(exc))
        return None, disabled
    transceiver.direction = "sendonly"
    transceiver._codecs = [c for c in transceiver._codecs if c.mimeType.lower() != 'video/rtx']
    transceiver._codecs.extend(copy.deepcopy([red, fec] + ([red_rtx] if red_rtx else [])))
    mutable = [ext.id for ext in transceiver._headerExtensions if ext.uri in {
        'http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time',
        'http://www.ietf.org/id/draft-holmer-rmcat-transport-wide-cc-extensions-01',
        'urn:ietf:params:rtp-hdrext:toffset'}]
    adapter = FecSender(sender, red.payloadType, fec.payloadType,
                        red_rtx.payloadType if red_rtx else None,
                        options.get('group_size', 5), options.get('media_bitrate_ratio', .8), mutable).install()
    return adapter, adapter.snapshot()
