"""Regression tests at aiortc's serialized plaintext RTP/RTCP boundary."""
import struct
from types import MethodType, SimpleNamespace
import unittest

from aiortc.codecs.vpx import Vp8Encoder
from aiortc.rtcrtpsender import RTCRtpSender
from aiortc.rtcrtpparameters import RTCRtpCodecParameters, RTCRtpHeaderExtensionParameters, RTCRtpParameters
from aiortc.rtp import (HeaderExtensionsMap, RtcpPsfbPacket, RtcpRtpfbPacket,
                       RtpPacket, RTCP_PSFB_APP, RTCP_RTPFB_NACK, pack_remb_fci)

from middleware.core.fec_sender import FecSender, configure_fec
from middleware.core.rtp_fec import (protected_sequences, recover_ulpfec, red_unwrap,
                                    rtp_layout, unwrap_red_payload, zero_mutable_extensions)

SSRC, RTX_SSRC = 0x11223344, 0x55667788
MEDIA_PT, RED_PT, FEC_PT, RED_RTX_PT = 96, 116, 117, 118
ABS_SEND_TIME = 'http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time'


class BoundarySender:
    """Minimal private state, with real aiortc feedback dispatch and RTP maps."""
    def __init__(self):
        self.wire = []
        self.transport = SimpleNamespace(_send_rtp=self._send)
        self._ssrc, self._rtx_ssrc = SSRC, RTX_SSRC
        self._RTCRtpSender__octet_count = 0
        self._RTCRtpSender__packet_count = 0
        self._RTCRtpSender__encoder = Vp8Encoder()
        self._RTCRtpSender__log_debug = lambda *args: None
        self._RTCRtpSender__rtp_header_extensions_map = HeaderExtensionsMap()
        self._RTCRtpSender__rtp_header_extensions_map.configure(RTCRtpParameters(
            headerExtensions=[RTCRtpHeaderExtensionParameters(id=3, uri=ABS_SEND_TIME)]))
        self._handle_rtcp_packet = MethodType(RTCRtpSender._handle_rtcp_packet, self)

    async def _send(self, data):
        self.wire.append(data)

    def media(self, seq, payload=b'\x10vp8-media', *, marker=False, abs_send_time=None):
        packet = RtpPacket(payload_type=MEDIA_PT, sequence_number=seq, timestamp=9000)
        packet.ssrc = self._ssrc
        packet.marker = int(marker)
        packet.payload = payload
        packet.extensions.abs_send_time = abs_send_time
        return packet.serialize(self._RTCRtpSender__rtp_header_extensions_map)

    def parsed(self, data):
        return RtpPacket.parse(data, self._RTCRtpSender__rtp_header_extensions_map)


def red_block(data):
    layout = rtp_layout(data)
    return unwrap_red_payload(data[layout.header_size:layout.payload_end])


class FecSenderTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.sender = BoundarySender()

    async def group(self, adapter, first=100, *, abs_send_time=None):
        for index in range(5):
            await self.sender.transport._send_rtp(self.sender.media(
                (first + index) & 0xFFFF, payload=b'\x10' + bytes([index]) * (index + 5),
                abs_send_time=abs_send_time))

    async def test_skipped_parity_does_not_allocate_unsent_sequence(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT, group_size=2).install()
        # First packet has the largest protected tail. Last packet has a longer
        # RTP extension header. Each per-packet estimate fits, combined parity
        # exceeds the cap and must be skipped without leaving a sequence hole.
        await self.sender.transport._send_rtp(self.sender.media(10, b'x'*1366))
        await self.sender.transport._send_rtp(self.sender.media(11, b'y', abs_send_time=123))
        self.assertEqual(adapter.snapshot()['protection_skipped'], 1)
        await self.sender.transport._send_rtp(self.sender.media(12, b'z'))
        self.assertEqual([self.sender.parsed(p).sequence_number for p in self.sender.wire], [10,11,12])

    async def test_same_ssrc_repair_increments_sender_report_counters(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        await self.sender.transport._send_rtp(self.sender.media(100))
        before_count = self.sender._RTCRtpSender__packet_count
        before_bytes = self.sender._RTCRtpSender__octet_count
        await adapter.retransmit(100)
        self.assertEqual(self.sender._RTCRtpSender__packet_count, before_count+1)
        self.assertEqual(self.sender._RTCRtpSender__octet_count-before_bytes,
                         len(self.sender.wire[-1])-rtp_layout(self.sender.wire[-1]).header_size)

    async def test_five_media_one_parity_share_contiguous_wire_sequence_across_wrap(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        await self.group(adapter, 65532)
        self.assertEqual([self.sender.parsed(data).sequence_number for data in self.sender.wire],
                         [65532, 65533, 65534, 65535, 0, 1])
        self.assertEqual([red_block(data)[0] for data in self.sender.wire], [MEDIA_PT] * 5 + [FEC_PT])
        self.assertTrue(all(self.sender.parsed(data).payload_type == RED_PT for data in self.sender.wire))
        await self.sender.transport._send_rtp(self.sender.media(1))
        self.assertEqual(self.sender.parsed(self.sender.wire[-1]).sequence_number, 2)
        self.assertEqual(adapter.snapshot()['fec_packets'], 1)
        self.assertFalse(adapter.snapshot()['recovery_verified'])
        self.assertEqual(self.sender._RTCRtpSender__packet_count, 1)  # Only extra FEC counted here.

    async def test_generated_fec_recovers_any_one_missing_media_and_not_two(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        await self.group(adapter)
        media = [red_unwrap(data) for data in self.sender.wire[:5]]
        _, fec = red_block(self.sender.wire[5])
        self.assertEqual(protected_sequences(fec), tuple(range(100, 105)))
        for missing in range(5):
            received = media[:missing] + media[missing + 1:]
            self.assertEqual(recover_ulpfec(fec, received, SSRC), media[missing])
        self.assertIsNone(recover_ulpfec(fec, media[2:], SSRC))

    async def test_real_nack_dispatch_repairs_cached_red_media_never_parity(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        await self.group(adapter)
        cached_wire = self.sender.wire[2]
        await self.sender._handle_rtcp_packet(RtcpRtpfbPacket(
            fmt=RTCP_RTPFB_NACK, ssrc=99, media_ssrc=SSRC, lost=[102, 105, 999]))
        self.assertEqual(len(self.sender.wire), 7)
        self.assertEqual(self.sender.wire[-1], cached_wire)
        self.assertEqual(adapter.snapshot()['retransmissions'], 1)
        self.assertEqual(adapter.snapshot()['ignored_nacks'], 2)
        self.assertNotIn(105, adapter.cache)
        self.assertEqual(adapter.snapshot()['fec_packets'], 1)

    async def test_red_rtx_uses_red_apt_payload_and_wire_osn(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT, RED_RTX_PT).install()
        await self.group(adapter)
        original_red = self.sender.parsed(self.sender.wire[4])
        await self.sender._retransmit(104)
        repair = self.sender.parsed(self.sender.wire[-1])
        self.assertEqual(repair.payload_type, RED_RTX_PT)
        self.assertEqual(repair.ssrc, RTX_SSRC)
        self.assertEqual(repair.sequence_number, 0)
        self.assertEqual(struct.unpack('!H', repair.payload[:2])[0], 104)
        self.assertEqual(repair.payload[2:], original_red.payload)
        self.assertEqual(repair.payload[2], MEDIA_PT)  # RTX apt is RED, not original VP8.
        self.assertEqual(repair.timestamp, original_red.timestamp)
        await self.sender._retransmit(100)
        self.assertEqual(self.sender.parsed(self.sender.wire[-1]).sequence_number, 1)

    async def test_mutable_extension_matches_libwebrtc_canonicalized_media(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT, mutable_extension_ids=[3]).install()
        await self.group(adapter, abs_send_time=0x123456)
        media = [red_unwrap(data) for data in self.sender.wire[:5]]
        _, fec = red_block(self.sender.wire[5])
        self.assertEqual(self.sender.parsed(media[2]).extensions.abs_send_time, 0x123456)
        recovered = recover_ulpfec(fec, media[:2] + media[3:], SSRC, mutable_extension_ids=[3])
        self.assertEqual(recovered, zero_mutable_extensions(media[2], [3]))
        self.assertEqual(self.sender.parsed(recovered).extensions.abs_send_time, 0)
        self.assertEqual(self.sender.parsed(recovered).payload, self.sender.parsed(media[2]).payload)

    async def test_remb_reserves_parity_bandwidth_and_reports_encoder_floor(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        await self.group(adapter)
        await self.sender._handle_rtcp_packet(RtcpPsfbPacket(
            fmt=RTCP_PSFB_APP, ssrc=99, media_ssrc=0,
            fci=pack_remb_fci(1_000_000, [SSRC])))
        state = adapter.snapshot()
        self.assertLessEqual(state['effective_media_ratio'], .8)
        self.assertEqual(state['media_target_bitrate'], int(1_000_000 * state['effective_media_ratio']))
        self.assertLess(state['media_target_bitrate'], 1_000_000)
        self.assertFalse(state['minimum_bitrate_limited'])
        await self.sender._handle_rtcp_packet(RtcpPsfbPacket(
            fmt=RTCP_PSFB_APP, ssrc=99, media_ssrc=0,
            fci=pack_remb_fci(100_000, [SSRC])))
        self.assertTrue(adapter.snapshot()['minimum_bitrate_limited'])
        self.assertEqual(self.sender._RTCRtpSender__encoder.target_bitrate, 250_000)

    async def test_large_media_skips_protection_without_inventing_parity_or_sequence_gap(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        await self.sender.transport._send_rtp(self.sender.media(100, payload=b'\x10' * 1380, marker=True))
        self.assertEqual(len(self.sender.wire), 1)
        self.assertEqual(adapter.snapshot()['protection_skipped'], 1)
        self.assertEqual(adapter.snapshot()['fec_packets'], 0)
        self.assertFalse(adapter.snapshot()['recovery_verified'])
        self.assertEqual(adapter.group, [])
        for index in range(4):
            await self.sender.transport._send_rtp(self.sender.media(101 + index))
        self.assertEqual(len(self.sender.wire), 5)
        self.assertEqual(adapter.snapshot()['fec_packets'], 0)
        self.assertEqual([self.sender.parsed(data).sequence_number for data in self.sender.wire], list(range(100, 105)))

    async def test_negotiation_selects_red_rtx_apt_and_preserves_peer_payload_types(self):
        transceiver = SimpleNamespace(mid='0', direction='sendrecv', _codecs=[
            RTCRtpCodecParameters(mimeType='video/VP8', clockRate=90000, payloadType=96),
            RTCRtpCodecParameters(mimeType='video/rtx', clockRate=90000, payloadType=97,
                                  parameters={'apt': 96})],
            _headerExtensions=[RTCRtpHeaderExtensionParameters(id=3, uri=ABS_SEND_TIME)])
        offer = '\r\n'.join(['v=0', 'o=- 0 0 IN IP4 127.0.0.1', 's=-', 't=0 0',
                                'm=video 9 UDP/TLS/RTP/SAVPF 96 97 116 117 118', 'a=mid:0',
                                'a=rtpmap:96 VP8/90000', 'a=rtpmap:97 rtx/90000',
                                'a=fmtp:97 apt=96', 'a=rtpmap:116 red/90000',
                                'a=rtpmap:117 ulpfec/90000', 'a=rtpmap:118 rtx/90000',
                                'a=fmtp:118 apt=116', ''])
        adapter, state = configure_fec(None, transceiver, self.sender, offer, {'enabled': True})
        self.assertEqual(state['state'], 'enabled')
        self.assertEqual(transceiver.direction, 'sendonly')
        self.assertEqual([codec.payloadType for codec in transceiver._codecs], [96, 116, 117, 118])
        self.assertEqual(transceiver._codecs[-1].parameters['apt'], RED_PT)
        self.assertEqual(adapter.red_rtx_pt, RED_RTX_PT)
        self.assertEqual(adapter.mutable_extension_ids, (3,))
        await self.group(adapter)
        await self.sender._retransmit(100)
        self.assertEqual(self.sender.parsed(self.sender.wire[-1]).payload_type, RED_RTX_PT)

    async def test_other_ssrc_and_rtcp_bypass_wrapper(self):
        adapter = FecSender(self.sender, RED_PT, FEC_PT).install()
        other = RtpPacket(payload_type=MEDIA_PT, sequence_number=1, timestamp=9000)
        other.ssrc, other.payload = RTX_SSRC, b'other'
        wire = other.serialize()
        await self.sender.transport._send_rtp(wire)
        rtcp = bytes(RtcpRtpfbPacket(fmt=RTCP_RTPFB_NACK, ssrc=99, media_ssrc=SSRC, lost=[1]))
        await self.sender.transport._send_rtp(rtcp)
        self.assertEqual(self.sender.wire, [wire, rtcp])
        self.assertEqual(adapter.snapshot()['media_packets'], 0)


if __name__ == '__main__':
    unittest.main()
