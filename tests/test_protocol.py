import json
import struct
import unittest

from hc_teleop_middleware.protocol import (
    LEGACY_PACKET_FORMAT,
    LEGACY_PACKET_SIZE,
    PACKET_FORMAT,
    PACKET_SIZE,
    PacketError,
    decode_pose_packet,
    encode_json_packet,
    is_sequence_newer,
    packet_loss_count,
)


class ProtocolTests(unittest.TestCase):
    def test_decodes_v2_pico_packet_and_controller_events(self):
        values = [float(number) for number in range(21)]
        left_input = (9, 1, 2, 0.75, 0.25, 0.1, -0.2, 0.3, -0.4)
        right_input = (4, 4, 0, 0.5, 0.8, -0.1, 0.2, -0.3, 0.4)
        raw = struct.pack(
            PACKET_FORMAT,
            b"PICO",
            2,
            42,
            12.5,
            7,
            *values,
            *left_input,
            *right_input,
        )
        packet = decode_pose_packet(raw)
        self.assertEqual(len(raw), PACKET_SIZE)
        self.assertEqual(PACKET_SIZE, 162)
        self.assertEqual(packet.protocol_version, 2)
        self.assertEqual(packet.sequence, 42)
        self.assertTrue(packet.tracked("head"))
        self.assertEqual(packet.left.position, (7.0, 8.0, 9.0))
        self.assertEqual(
            packet.left_input.decode_buttons(packet.left_input.held_mask),
            ["primary", "trigger_button"],
        )
        self.assertEqual(packet.left_input.decode_buttons(packet.left_input.pressed_mask), ["primary"])
        self.assertEqual(packet.left_input.decode_buttons(packet.left_input.released_mask), ["secondary"])
        self.assertAlmostEqual(packet.left_input.trigger, 0.75)
        self.assertEqual(packet.right_input.joy_buttons()[2], 1)

    def test_decodes_legacy_v1_packet(self):
        values = [float(number) for number in range(21)]
        raw = struct.pack(LEGACY_PACKET_FORMAT, b"PICO", 1, 7, 1.0, 7, *values)
        packet = decode_pose_packet(raw)
        self.assertEqual(len(raw), LEGACY_PACKET_SIZE)
        self.assertEqual(packet.protocol_version, 1)
        self.assertEqual(packet.left_input.held_mask, 0)

    def test_invalid_packet_is_rejected(self):
        with self.assertRaises(PacketError):
            decode_pose_packet(b"short")

    def test_sequence_wrap_and_loss(self):
        self.assertTrue(is_sequence_newer(0, 0xFFFFFFFF))
        self.assertEqual(packet_loss_count(2, 0xFFFFFFFF), 2)
        self.assertFalse(is_sequence_newer(4, 4))

    def test_json_datagram_is_utf8(self):
        encoded = encode_json_packet({"message": "急停"})
        self.assertEqual(json.loads(encoded), {"message": "急停"})


if __name__ == "__main__":
    unittest.main()
