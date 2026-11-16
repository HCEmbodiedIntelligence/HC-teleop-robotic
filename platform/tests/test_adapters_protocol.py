import asyncio
import json
import math
import socket

import pytest

from hc_platform.adapters.protocol import V1, V2, ProtocolError, decode_packet, is_newer
from hc_platform.adapters.vr import VrGateway, VrSession


def packet(sequence=1, timestamp=1.0, flags=7, version=2, left=None, right=None):
    poses = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] * 3
    codec = V1 if version == 1 else V2
    inputs = (left or [4, 4, 0, 0.2, 0.8, 0.1, -0.2, 0.0, 0.0]) + (
        right or [0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    return codec.pack(
        b"PICO",
        version,
        sequence,
        timestamp,
        flags,
        *poses,
        *(inputs if version == 2 else []),
    )


def test_wire_compatibility_and_offsets():
    assert V1.size == 102 and V2.size == 162
    result = decode_packet(packet())
    assert result.version == 2 and result.left_input.held == 4
    assert result.left_input.trigger == pytest.approx(0.2)
    assert result.left_input.grip == pytest.approx(0.8)
    assert result.right.quaternion == (0, 0, 0, 1)
    assert decode_packet(packet(version=1)).left_input.grip == 0


@pytest.mark.parametrize("value", [b"", b"PICO", b"x" * 162])
def test_reject_wrong_wire(value):
    with pytest.raises(ProtocolError):
        decode_packet(value)


def test_nonfinite_and_analog_ranges_are_rejected():
    with pytest.raises(ProtocolError):
        decode_packet(packet(timestamp=float("nan")))
    with pytest.raises(ProtocolError):
        decode_packet(packet(left=[0, 0, 0, 1.5, 0, 0, 0, 0, 0]))


def test_wraparound_and_duplicate_ordering():
    assert is_newer(0, 0xFFFFFFFF)
    assert not is_newer(0xFFFFFFFF, 0)
    assert not is_newer(1, 1)
    assert not is_newer(0x80000001, 1)


def test_exclusive_ip_lease_and_release():
    session = VrSession()
    first = session.accept(packet(), ("10.0.0.1", 1234), 1_000_000_000)
    assert first.new_session
    assert session.accept(packet(2, 2), ("10.0.0.2", 1234), 1_010_000_000) is None
    assert not session.release("10.0.0.2")
    assert session.release("10.0.0.1")
    assert session.accept(packet(1, 1), ("10.0.0.2", 3456), 1_020_000_000).new_session


def test_sequence_progress_does_not_hide_frozen_sample():
    session = VrSession()
    first = session.accept(packet(), ("10.0.0.1", 1), 1_000_000_000)
    second = session.accept(packet(2, 1), ("10.0.0.1", 1), 1_300_000_000)
    assert second.sample_seq == first.sample_seq and not second.fresh_sample
    assert not session.status(1_300_000_000)["fresh"]
    assert session.status(1_300_000_000)["packet_age_s"] == 0
    third = session.accept(packet(3, 1.2), ("10.0.0.1", 1), 1_310_000_000)
    assert third.fresh_sample and session.status(1_310_000_000)["fresh"]


def test_duplicates_and_old_timestamp_do_not_renew_lease():
    session = VrSession()
    session.accept(packet(), ("127.0.0.1", 1), 1_000_000_000)
    assert session.accept(packet(), ("127.0.0.1", 1), 1_100_000_000) is None
    assert session.accept(packet(2, 0.5), ("127.0.0.1", 1), 1_100_000_000) is None
    assert session.last_packet_ns == 1_000_000_000
    assert session.expire(3_100_000_000)


def test_stationary_pose_with_advancing_timestamp_is_fresh():
    session = VrSession()
    a = session.accept(packet(), ("127.0.0.1", 1), 0)
    b = session.accept(packet(2, 1.01), ("127.0.0.1", 1), 10_000_000)
    assert a.frame.left == b.frame.left and b.sample_seq == a.sample_seq + 1


def test_udp_discovery_pose_events_and_release():
    async def exercise():
        frames = []
        stops = []
        loop = asyncio.get_running_loop()
        peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer.bind(("127.0.0.1", 0))
        peer.setblocking(False)
        events = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        events.bind(("127.0.0.1", 0))
        events.setblocking(False)
        gateway = VrGateway(
            frames.append,
            stops.append,
            host="127.0.0.1",
            pose_port=0,
            discovery_port=0,
            event_port=events.getsockname()[1],
        )
        await gateway.start()
        try:
            peer.sendto(b"PICO_DISCOVER_V1", ("127.0.0.1", gateway.discovery_port))
            response = await asyncio.wait_for(loop.sock_recv(peer, 512), 1)
            assert response == f"PICO_RECEIVER_V1|{gateway.pose_port}".encode()
            peer.sendto(packet(), ("127.0.0.1", gateway.pose_port))
            await asyncio.sleep(0.02)
            assert len(frames) == 1 and gateway.status()["active_peer"] == "127.0.0.1"
            gateway.send_event("safety_stop", {"reason": "test"})
            response = await asyncio.wait_for(loop.sock_recv(events, 8192), 1)
            assert json.loads(response)["payload"]["reason"] == "test"
            peer.sendto(b"PICO_RELEASE_V1", ("127.0.0.1", gateway.discovery_port))
            await asyncio.sleep(0.02)
            assert not gateway.status()["is_locked"] and stops == [
                "VR_SESSION_RELEASED"
            ]
            # Also accept PICO_RELEASE_V1 sent directly to the pose port.
            peer.sendto(packet(sequence=2, timestamp=2.0), ("127.0.0.1", gateway.pose_port))
            await asyncio.sleep(0.02)
            assert gateway.status()["active_peer"] == "127.0.0.1"
            invalid_before = gateway.status()["counts"].get("invalid", 0)
            peer.sendto(b"PICO_RELEASE_V1", ("127.0.0.1", gateway.pose_port))
            await asyncio.sleep(0.02)
            assert not gateway.status()["is_locked"]
            assert gateway.status()["counts"].get("invalid", 0) == invalid_before
        finally:
            await gateway.close()
            peer.close()
            events.close()

    asyncio.run(exercise())
