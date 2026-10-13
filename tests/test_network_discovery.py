import socket
import struct
import sys
import unittest
from middleware.core.network import enable_packet_info, receive_packet, reply_packet


class DiscoveryInterfaceTests(unittest.TestCase):
    @unittest.skipUnless(sys.platform.startswith('linux'), 'Linux IP_PKTINFO')
    def test_reply_preserves_received_local_address(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
            server.bind(('0.0.0.0', 0))
            server.settimeout(1)
            client.settimeout(1)
            enabled = enable_packet_info(server)
            client.sendto(b'PICO_DISCOVER_V1', ('127.0.0.2', server.getsockname()[1]))
            payload, peer, info = receive_packet(server, enabled)
            self.assertEqual(payload, b'PICO_DISCOVER_V1')
            self.assertEqual(socket.inet_ntoa(struct.unpack('=I4s4s', info)[1]), '127.0.0.2')
            reply_packet(server, b'PICO_RECEIVER_V1|5005', peer, info)
            response, source = client.recvfrom(256)
            self.assertEqual(source[0], '127.0.0.2')
            self.assertEqual(response, b'PICO_RECEIVER_V1|5005')
