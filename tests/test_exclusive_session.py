import socket
import time
import unittest
from middleware.core.vr_gateway import VrGateway
from middleware.core.protocol import DISCOVERY_REQUEST, RELEASE_REQUEST, PACKET_FORMAT, MAGIC, PROTOCOL_VERSION


def make_dummy_pose_packet(sequence=1):
    import struct
    # "<4sBIdB21f3H6f3H6f"
    head = [0.0] * 7
    left = [0.0] * 7
    right = [0.0] * 7
    poses = head + left + right # 21f
    left_input = (0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) # 3H6f
    right_input = (0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) # 3H6f
    return struct.pack(
        PACKET_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        sequence,
        time.time(),
        7, # flags: head, left, right tracked
        *poses,
        *left_input,
        *right_input
    )


class ExclusiveSessionTests(unittest.TestCase):
    def test_exclusive_lease_and_release(self):
        poses_received = []
        events_received = []
        timeouts = []

        cfg = {
            "enabled": True,
            "listen_host": "127.0.0.1",
            "pose_port": 58911,
            "discovery_port": 58912,
            "outbound_host": "",
            "outbound_port": 58913,
            "pose_timeout_ms": 2000,
            "exclusive_mode": True,
            "lease_timeout_s": 1.0,
        }

        gateway = VrGateway(
            cfg,
            on_pose=lambda pose, d: poses_received.append(pose),
            on_event=lambda ev, dest: events_received.append(ev),
            on_timeout=lambda msg: timeouts.append(msg),
        )
        gateway.start()
        time.sleep(0.1)

        try:
            client1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            client1.settimeout(1.0)
            client2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            client2.settimeout(1.0)

            # 1. Initially free: client1 discovers gateway
            client1.bind(("127.0.0.1", 0))
            client2.bind(("127.0.0.2", 0))

            client1.sendto(DISCOVERY_REQUEST, ("127.0.0.1", 58912))
            resp, _ = client1.recvfrom(256)
            self.assertEqual(resp, b"PICO_RECEIVER_V1|58911")

            # 2. Client1 sends pose -> acquires exclusive lease
            pkt1 = make_dummy_pose_packet(1)
            client1.sendto(pkt1, ("127.0.0.1", 58911))
            time.sleep(0.05)

            st = gateway.status()
            self.assertEqual(st["active_peer"], "127.0.0.1")
            self.assertTrue(st["is_locked"])
            self.assertGreaterEqual(st["received"], 1)

            # 3. Client2 discovers gateway while Client1 has lease -> BUSY response
            client2.sendto(DISCOVERY_REQUEST, ("127.0.0.1", 58912))
            resp2, _ = client2.recvfrom(256)
            self.assertEqual(resp2, b"PICO_RECEIVER_BUSY|58911|127.0.0.1")

            # 4. Client2 sends pose while Client1 has lease -> rejected / dropped
            pkt2 = make_dummy_pose_packet(1)
            client2.sendto(pkt2, ("127.0.0.1", 58911))
            time.sleep(0.05)

            st = gateway.status()
            self.assertEqual(st["unauthorized_dropped"], 1)
            self.assertEqual(st["active_peer"], "127.0.0.1")

            # 5. Client1 sends release -> lease cleared
            client1.sendto(RELEASE_REQUEST, ("127.0.0.1", 58911))
            time.sleep(0.05)

            st = gateway.status()
            self.assertIsNone(st["active_peer"])

            # 6. Client2 discovers gateway again -> now free!
            client2.sendto(DISCOVERY_REQUEST, ("127.0.0.1", 58912))
            resp, _ = client2.recvfrom(256)
            self.assertEqual(resp, b"PICO_RECEIVER_V1|58911")

            # 7. Client2 sends pose -> Client2 acquires lease
            pkt2_new = make_dummy_pose_packet(2)
            client2.sendto(pkt2_new, ("127.0.0.1", 58911))
            time.sleep(0.05)

            st = gateway.status()
            self.assertEqual(st["active_peer"], "127.0.0.2")

            client1.close()
            client2.close()
        finally:
            gateway.stop()


if __name__ == "__main__":
    unittest.main()
