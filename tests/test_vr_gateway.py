import socket
import struct
import threading
import time
import unittest

from middleware.core.protocol import LEGACY_PACKET_FORMAT
from middleware.core.vr_gateway import VrGateway


def _unused_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def _pose_packet(sequence: int, timestamp: float) -> bytes:
    # Three identity poses: position xyz + quaternion xyzw.
    pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    return struct.pack(
        LEGACY_PACKET_FORMAT,
        b"PICO",
        1,
        sequence,
        timestamp,
        0b111,
        *(pose * 3),
    )


class VrGatewayTests(unittest.TestCase):
    def check_sequences(self, sequences, expected, old, *, switch_peer=False):
        received=[]
        gateway=VrGateway(dict(enabled=True,listen_host='127.0.0.1',pose_port=_unused_udp_port(),
            discovery_port=_unused_udp_port(),outbound_host='',outbound_port=_unused_udp_port(),
            pose_timeout_ms=1000,processing_queue_size=32,web_pose_hz=20),
            lambda pose:received.append(pose.sequence),lambda *_:None,lambda _:None)
        gateway.start()
        senders=[socket.socket(socket.AF_INET,socket.SOCK_DGRAM) for _ in range(2)]
        try:
            deadline=time.monotonic()+2
            while gateway.status()['state']!='running' and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertEqual(gateway.status()['state'],'running')
            for i,seq in enumerate(sequences):
                senders[int(switch_peer and i>0)].sendto(_pose_packet(seq,time.monotonic()),
                    ('127.0.0.1',gateway.config['pose_port']))
                time.sleep(.01)
            deadline=time.monotonic()+2
            while len(received)<len(expected) and time.monotonic()<deadline:
                time.sleep(.005)
            self.assertEqual(received,expected)
            self.assertEqual(gateway.status()['old'],old)
            self.assertEqual(gateway.status()['lost'],0)
        finally:
            gateway.stop()
            for sender in senders: sender.close()

    def test_uint32_wrap_duplicate_old_and_half_range_packets(self):
        self.check_sequences([0xfffffffe,0xffffffff,0,1,1,0,0x80000001,2],
                             [0xfffffffe,0xffffffff,0,1,2],3)

    def test_new_udp_sender_can_restart_its_sequence(self):
        self.check_sequences([1000,0,1],[1000,0,1],0,switch_peer=True)

    def test_slow_consumer_does_not_block_receive_watchdog(self) -> None:
        pose_port = _unused_udp_port()
        discovery_port = _unused_udp_port()
        timeouts: list[str] = []

        gateway = VrGateway(
            {
                "enabled": True,
                "listen_host": "127.0.0.1",
                "pose_port": pose_port,
                "discovery_port": discovery_port,
                "outbound_host": "",
                "outbound_port": _unused_udp_port(),
                "pose_timeout_ms": 120,
                "processing_queue_size": 4,
                "web_pose_hz": 20,
            },
            # Deliberately slower than the safety timeout.  Before reception
            # and processing were separated, this caused a false emergency.
            lambda _pose: time.sleep(0.2),
            lambda _event, _outputs: None,
            timeouts.append,
        )
        gateway.start()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            time.sleep(0.03)
            deadline = time.monotonic() + 0.55
            sequence = 0
            while time.monotonic() < deadline:
                sender.sendto(
                    _pose_packet(sequence, time.monotonic()),
                    ("127.0.0.1", pose_port),
                )
                sequence += 1
                time.sleep(0.01)
        finally:
            gateway.stop()
            sender.close()

        status = gateway.status()
        self.assertGreaterEqual(status["received"], 40)
        self.assertGreater(status["processing_dropped"], 0)
        self.assertFalse(
            any(reason.startswith("no VR pose data") for reason in timeouts),
            timeouts,
        )

    def test_repeated_sample_timestamp_triggers_stale_safety(self) -> None:
        pose_port = _unused_udp_port()
        discovery_port = _unused_udp_port()
        timeouts: list[str] = []

        gateway = VrGateway(
            {
                "enabled": True,
                "listen_host": "127.0.0.1",
                "pose_port": pose_port,
                "discovery_port": discovery_port,
                "outbound_host": "",
                "outbound_port": _unused_udp_port(),
                "pose_timeout_ms": 100,
                "processing_queue_size": 4,
                "web_pose_hz": 20,
            },
            lambda _pose: None,
            lambda _event, _outputs: None,
            timeouts.append,
        )
        gateway.start()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            time.sleep(0.03)
            # The transport remains healthy and sequence numbers advance, but
            # every packet repeats one Unity main-thread sample timestamp.
            deadline = time.monotonic() + 0.24
            sequence = 0
            while time.monotonic() < deadline:
                sender.sendto(
                    _pose_packet(sequence, 123.0),
                    ("127.0.0.1", pose_port),
                )
                sequence += 1
                time.sleep(0.01)
        finally:
            gateway.stop()
            sender.close()

        status = gateway.status()
        self.assertGreaterEqual(status["received"], 15)
        self.assertTrue(status["sample_stale"])
        self.assertTrue(
            any(reason.startswith("VR pose sample stale") for reason in timeouts),
            timeouts,
        )


if __name__ == "__main__":
    unittest.main()
