"""Independent RFC vectors plus optional native GStreamer ULPFEC interop.

Run the native check where GI is installed, e.g. /usr/bin/python3 -m unittest
 discover -s tests -p test_rtp_fec.py -v. No aiortc or GPU dependencies required.
"""
import struct
import threading
import unittest

from middleware.core.rtp_fec import (
    encode_ulpfec, protected_sequences, recover_ulpfec, red_unwrap, red_wrap,
    rtp_layout, unwrap_red_payload, wrap_red_payload, zero_mutable_extensions,
)


A = bytes.fromhex('806000010102030411223344010203')
B = bytes.fromhex('80e1000205060708112233441020')
SSRC = 0x11223344
# RFC5109 field calculation independent of encode/recover implementation:
# P/X/CC=0; M/PT=0x60 xor 0xe1=0x81; SNbase=1;
# TS=0x01020304 xor 0x05060708=0x0404040c;
# length=3 xor 2=1; protection=3; bits 0,1=>0xc000;
# payload=01 02 03 xor 10 20 00=>11 22 03.
KNOWN_FEC = bytes.fromhex('008100010404040c00010003c000112203')


def packet(sequence, tail=b'payload', first=0x80, second=0x60, timestamp=0, ssrc=SSRC):
    return struct.pack('!BBHII', first, second, sequence, timestamp, ssrc) + tail


class RtpFecTests(unittest.TestCase):
    def test_independent_rfc_wire_vector(self):
        self.assertEqual(encode_ulpfec([A, B]), KNOWN_FEC)
        self.assertEqual(protected_sequences(KNOWN_FEC), (1, 2))
        self.assertEqual(recover_ulpfec(KNOWN_FEC, [A], SSRC), B)
        self.assertEqual(recover_ulpfec(KNOWN_FEC, [B], SSRC), A)

    def test_recovery_preserves_csrc_extension_padding_and_variable_length(self):
        # P=X=1, CC=1: CSRC + BEDE extension (id3,3 bytes) + payload + padding.
        complex_packet = packet(2, bytes.fromhex('99887766bede000132aabbcc') + b'vp8' + b'\0\0\0\x04',
                                first=0xB1, second=0xE0, timestamp=0xFEDCBA98)
        others = [packet(1, b''), packet(3, bytes(range(250)))]
        fec = encode_ulpfec([others[0], complex_packet, others[1]])
        self.assertEqual(recover_ulpfec(fec, others, SSRC), complex_packet)
        self.assertEqual(red_unwrap(red_wrap(complex_packet, 116)), complex_packet)

    def test_sequence_wrap_unordered_and_holes(self):
        data = [packet(2), packet(65535), packet(0)]
        fec = encode_ulpfec(data)
        self.assertEqual(protected_sequences(fec), (65535, 0, 2))
        self.assertEqual(recover_ulpfec(fec, data[:1] + data[2:], SSRC), data[1])
        data = [packet((65530 + index) & 0xFFFF) for index in range(16)]
        self.assertEqual(len(protected_sequences(encode_ulpfec(data))), 16)
        with self.assertRaises(ValueError):
            encode_ulpfec([packet(65530), packet(10)])

    def test_multiple_or_no_missing_returns_none(self):
        self.assertIsNone(recover_ulpfec(KNOWN_FEC, [], SSRC))
        self.assertIsNone(recover_ulpfec(KNOWN_FEC, [A, B], SSRC))
        self.assertEqual(recover_ulpfec(KNOWN_FEC, {1: A, 999: packet(999)}, SSRC), B)

    def test_rejects_wrong_ssrc_duplicates_and_invalid_rtp(self):
        invalid = [b'', b'\x80' * 11, packet(1, b'', first=0x81),
                   packet(1, b'\x00', first=0xA0), packet(1, b'\x10\x00', first=0x90)]
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(ValueError):
                encode_ulpfec([data])
        for packets in ([], [A, A], [A, packet(2, ssrc=9)]):
            with self.assertRaises(ValueError):
                encode_ulpfec(packets)
        with self.assertRaises(ValueError):
            recover_ulpfec(KNOWN_FEC, [packet(1, ssrc=9)], SSRC)
        with self.assertRaises(ValueError):
            recover_ulpfec(KNOWN_FEC, [A, packet(1, b'changed')], SSRC)

    def test_mtu_budget_and_bad_fec_rejected(self):
        with self.assertRaises(ValueError):
            encode_ulpfec([A, B], max_payload_size=16)
        self.assertEqual(encode_ulpfec([A, B], max_payload_size=17), KNOWN_FEC)
        for value in (KNOWN_FEC[:-1], KNOWN_FEC + b'\0', bytes([0x40]) + KNOWN_FEC[1:],
                      KNOWN_FEC[:12] + b'\0\0' + KNOWN_FEC[14:]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                protected_sequences(value)

    def test_red_primary_vector_and_multiblock_rejected(self):
        self.assertEqual(wrap_red_payload(b'abc', 96), b'\x60abc')
        self.assertEqual(unwrap_red_payload(b'\x60abc'), (96, b'abc'))
        self.assertEqual(red_wrap(A, 116), bytes.fromhex('80740001010203041122334460010203'))
        with self.assertRaises(ValueError):
            unwrap_red_payload(b'\x80\0\0\0')
        with self.assertRaises(ValueError):
            wrap_red_payload(b'a', 128)

    def test_native_mutable_extension_canonicalization(self):
        one = packet(1, bytes.fromhex('bede000132aabbcc') + b'vp8', first=0x90)
        two = packet(2, bytes.fromhex('100000020303aabbcc0101dd') + b'vp8', first=0x90)
        self.assertEqual(zero_mutable_extensions(one, [3])[17:20], b'\0\0\0')
        self.assertEqual(zero_mutable_extensions(two, [3])[18:21], b'\0\0\0')
        fec = encode_ulpfec([one, two], mutable_extension_ids=[3])
        self.assertEqual(recover_ulpfec(fec, [one], SSRC, mutable_extension_ids=[3]),
                         zero_mutable_extensions(two, [3]))
        self.assertEqual(zero_mutable_extensions(two, [3])[-3:], b'vp8')


try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GObject
    Gst.init(None)
    HAS_GST = all(Gst.ElementFactory.find(name) for name in ('appsrc', 'appsink', 'rtpstorage', 'rtpulpfecdec'))
except (ImportError, ValueError):
    HAS_GST = False


@unittest.skipUnless(HAS_GST, 'GStreamer GI + RTP plugins unavailable')
class NativeGStreamerInteropTests(unittest.TestCase):
    def test_native_decoder_recovers_generated_parity(self):
        # Native decoder (no call to our recover_ulpfec): lose seq2, retain seq1
        # and FEC seq3 in rtpstorage, then signal the standard lost-packet event.
        # rtpulpfecdec rewrites outgoing sequence numbers, so compare every other
        # byte and separately require its native recovered counter to increment.
        media = [packet(1, b'\x10vp8-one', timestamp=9000),
                 packet(2, b'\x10vp8-two-longer', second=0xE0, timestamp=9000)]
        fec = packet(3, encode_ulpfec(media), second=122, timestamp=9000)
        pipeline = Gst.parse_launch('appsrc name=source is-live=true format=time ! '
                                    'rtpstorage name=storage size-time=2000000000 ! '
                                    'rtpulpfecdec name=decoder pt=122 ! '
                                    'appsink name=sink sync=false async=false')
        source, storage, decoder, sink = (pipeline.get_by_name(name)
                                         for name in ('source', 'storage', 'decoder', 'sink'))
        source.set_property('caps', Gst.Caps.from_string(
            f'application/x-rtp,media=(string)video,encoding-name=(string)VP8,'
            f'clock-rate=(int)90000,payload=(int)96,ssrc=(uint){SSRC}'))
        decoder.set_property('storage', storage.get_property('internal-storage'))
        fec_seen = threading.Event()

        def observed(_pad, probe):
            buf = probe.get_buffer()
            if buf is not None and buf.extract_dup(0, buf.get_size()) == fec:
                fec_seen.set()
            return Gst.PadProbeReturn.OK

        storage.get_static_pad('src').add_probe(Gst.PadProbeType.BUFFER, observed)
        pipeline.set_state(Gst.State.PLAYING)
        try:
            for index, data in enumerate((media[0], fec)):
                buf = Gst.Buffer.new_wrapped(data)
                buf.pts = index * 10 * Gst.MSECOND
                buf.duration = 10 * Gst.MSECOND
                self.assertEqual(source.emit('push-buffer', buf), Gst.FlowReturn.OK)
            self.assertTrue(fec_seen.wait(2), 'Native pipeline did not receive FEC')
            structure = Gst.Structure.new_empty('GstRTPPacketLost')
            structure.set_value('seqnum', GObject.Value(GObject.TYPE_UINT, 2))
            structure.set_value('timestamp', GObject.Value(GObject.TYPE_UINT64, 10 * Gst.MSECOND))
            structure.set_value('duration', GObject.Value(GObject.TYPE_UINT64, 10 * Gst.MSECOND))
            event = Gst.Event.new_custom(Gst.EventType.CUSTOM_DOWNSTREAM, structure)
            self.assertTrue(storage.get_static_pad('src').push_event(event))
            self.assertEqual(decoder.get_property('recovered'), 1)
            output = []
            for _ in range(4):
                sample = sink.emit('try-pull-sample', 100 * Gst.MSECOND)
                if sample is None:
                    break
                buf = sample.get_buffer()
                output.append(buf.extract_dup(0, buf.get_size()))
            self.assertTrue(any(data[:2] + data[4:] == media[1][:2] + media[1][4:] for data in output),
                            'Native recovered packet differs from source media')
        finally:
            pipeline.set_state(Gst.State.NULL)


if __name__ == '__main__':
    unittest.main()
