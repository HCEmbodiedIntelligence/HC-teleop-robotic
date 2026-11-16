"""Bounded asyncio UDP receiver, exclusive IP lease, and headset feedback.

Datagram callbacks never solve IK or await. Only new producer samples refresh
sample freshness; sequence-only retransmissions cannot conceal sampler freezes.
"""

import asyncio
from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Callable, Optional, Tuple

from .protocol import DISCOVER, RELEASE, ProtocolError, VrFrame, decode_packet, is_newer


@dataclass(frozen=True)
class AcceptedFrame:
    frame: VrFrame
    peer: str
    received_ns: int
    sample_mono_ns: int
    sample_seq: int
    fresh_sample: bool
    new_session: bool


class VrSession:
    def __init__(self, input_timeout_s=0.2, lease_timeout_s=2.0):
        if not 0 < input_timeout_s < lease_timeout_s:
            raise ValueError(
                "input timeout must be positive and shorter than lease timeout"
            )
        self.input_timeout_ns = int(input_timeout_s * 1e9)
        self.lease_timeout_ns = int(lease_timeout_s * 1e9)
        self.peer = None
        self.last_sequence = None
        self.last_sample_time = None
        self.last_packet_ns = None
        self.last_sample_ns = None
        self.sample_seq = 0
        self.session_generation = 0
        self._source_offset_ns = None
        self.counts = {
            key: 0
            for key in (
                "accepted",
                "invalid",
                "busy",
                "old",
                "lost",
                "frozen",
                "releases",
            )
        }

    def release(self, peer=None):
        if self.peer is None or (peer is not None and peer != self.peer):
            return False
        self.peer = self.last_sequence = self.last_sample_time = None
        self.last_packet_ns = self.last_sample_ns = None
        self._source_offset_ns = None
        self.counts["releases"] += 1
        return True

    def expire(self, now_ns):
        if (
            self.peer is not None
            and now_ns - self.last_packet_ns > self.lease_timeout_ns
        ):
            return self.release()
        return False

    def accept(
        self, packet: bytes, address: Tuple[str, int], now_ns: int
    ) -> Optional[AcceptedFrame]:
        try:
            frame = decode_packet(packet)
        except ProtocolError:
            self.counts["invalid"] += 1
            return None
        self.expire(now_ns)
        if self.peer is not None and address[0] != self.peer:
            self.counts["busy"] += 1
            return None
        new = self.peer is None
        if new and not frame.tracked("head"):
            return None  # shutdown packets do not claim idle receivers
        if not is_newer(frame.sequence, self.last_sequence):
            self.counts["old"] += 1
            return None
        if (
            self.last_sample_time is not None
            and frame.sample_time < self.last_sample_time
        ):
            self.counts["old"] += 1
            return None
        if self.last_sequence is not None:
            self.counts["lost"] += max(
                0, ((frame.sequence - self.last_sequence) & 0xFFFFFFFF) - 1
            )
        if new:
            self.peer = address[0]
            self.session_generation += 1
        fresh = (
            self.last_sample_time is None or frame.sample_time > self.last_sample_time
        )
        self.last_sequence = frame.sequence
        self.last_packet_ns = now_ns
        if fresh:
            self.last_sample_time = frame.sample_time
            offset = now_ns - int(frame.sample_time * 1e9)
            self._source_offset_ns = (
                offset
                if self._source_offset_ns is None
                else min(offset, self._source_offset_ns)
            )
            # A minimum-delay clock offset prevents a later kernel/socket
            # backlog from turning old advancing samples into fresh input.
            self.last_sample_ns = min(
                now_ns, int(frame.sample_time * 1e9) + self._source_offset_ns
            )
            self.sample_seq += 1
        else:
            self.counts["frozen"] += 1
        self.counts["accepted"] += 1
        return AcceptedFrame(
            frame, self.peer, now_ns, self.last_sample_ns, self.sample_seq, fresh, new
        )

    def status(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        age = (
            None
            if self.last_sample_ns is None
            else (now_ns - self.last_sample_ns) / 1e9
        )
        packet_age = (
            None
            if self.last_packet_ns is None
            else (now_ns - self.last_packet_ns) / 1e9
        )
        return {
            "is_locked": self.peer is not None,
            "active_peer": self.peer,
            "session_generation": self.session_generation,
            "sample_age_s": age,
            "packet_age_s": packet_age,
            "fresh": age is not None and age <= self.input_timeout_ns / 1e9,
            "identity": "ip_lease_compatibility",
            "event_ack_supported": False,
            "counts": dict(self.counts),
        }


class _Datagrams(asyncio.DatagramProtocol):
    def __init__(self, receiver):
        self.receiver = receiver

    def datagram_received(self, data, addr):
        self.receiver(data, addr)


class VrGateway:
    def __init__(
        self,
        on_frame: Callable,
        on_disconnect: Optional[Callable] = None,
        *,
        host="0.0.0.0",
        pose_port=5005,
        discovery_port=5006,
        event_port=5007,
        input_timeout_s=0.2,
        lease_timeout_s=2.0,
    ):
        self.on_frame = on_frame
        self.on_disconnect = on_disconnect
        self.host, self.pose_port = host, pose_port
        self.discovery_port, self.event_port = discovery_port, event_port
        self.session = VrSession(input_timeout_s, lease_timeout_s)
        self.pose_transport = self.discovery_transport = None
        self.task = None
        self._latched_events = {}
        self._last_event_timestamp = 0.0
        self._disconnected = False
        self._event_sequence = 0
        self.callback_error = None
        self.protocol_version = None

    async def start(self):
        loop = asyncio.get_running_loop()
        self.pose_transport, _ = await loop.create_datagram_endpoint(
            lambda: _Datagrams(self._pose), local_addr=(self.host, self.pose_port)
        )
        self.pose_port = self.pose_transport.get_extra_info("sockname")[1]
        try:
            self.discovery_transport, _ = await loop.create_datagram_endpoint(
                lambda: _Datagrams(self._discovery),
                local_addr=(self.host, self.discovery_port),
            )
            self.discovery_port = self.discovery_transport.get_extra_info("sockname")[1]
        except BaseException:
            self.pose_transport.close()
            raise
        self.task = asyncio.create_task(self._maintenance(), name="vr-lease-watchdog")

    def _pose(self, data, address):
        if data == RELEASE:
            if self.session.release(address[0]):
                self._disconnect("VR_SESSION_RELEASED")
            return
        accepted = self.session.accept(data, address, time.monotonic_ns())
        if accepted is None:
            return
        if accepted.fresh_sample:
            self._disconnected = False
        self.protocol_version = accepted.frame.version
        try:
            self.on_frame(accepted)
        except Exception as error:
            self.callback_error = str(error)
            self._disconnect("VR_INPUT_CONSUMER_FAILED")

    def _discovery(self, data, address):
        if data == RELEASE:
            if self.session.release(address[0]):
                self._disconnect("VR_SESSION_RELEASED")
            return
        if data != DISCOVER:
            return
        self.session.expire(time.monotonic_ns())
        peer = self.session.peer
        if peer is not None and address[0] != peer:
            response = "PICO_RECEIVER_BUSY|{}|{}".format(self.pose_port, peer)
        else:
            response = "PICO_RECEIVER_V1|{}".format(self.pose_port)
        self.discovery_transport.sendto(response.encode("ascii"), address)

    def _disconnect(self, reason):
        if not self._disconnected:
            self._disconnected = True
            if self.on_disconnect is not None:
                self.on_disconnect(reason)

    async def _maintenance(self):
        ticks = 0
        while True:
            await asyncio.sleep(0.02)
            now = time.monotonic_ns()
            status = self.session.status(now)
            if status["is_locked"] and not status["fresh"]:
                self._disconnect("VR_SAMPLE_STALE")
            if self.session.expire(now):
                self._disconnect("VR_LEASE_EXPIRED")
            ticks += 1
            if ticks % 25 == 0:
                # Existing headset does not ACK; periodic same-timestamp state
                # reconciliation is compatible with its safety-event ordering.
                for event in self._latched_events.values():
                    self._send(event)

    def send_event(self, kind, payload, *, latch=None):
        self._event_sequence += 1
        stamp = max(time.time(), self._last_event_timestamp + 1e-6)
        self._last_event_timestamp = stamp
        event = {
            "kind": kind,
            "source": "middleware",
            "timestamp": stamp,
            "event_id": self._event_sequence,
            "payload": payload,
        }
        if latch is None:
            latch = "safety" if kind in ("safety_stop", "safety_resume") else None
        if latch:
            if len(self._latched_events) >= 16 and latch not in self._latched_events:
                raise ValueError("too many latched headset states")
            self._latched_events[latch] = event
        self._send(event)
        return event

    def _send(self, event):
        if self.session.peer and self.pose_transport:
            data = json.dumps(event, separators=(",", ":"), allow_nan=False).encode(
                "utf-8"
            )
            if len(data) > 8192:
                raise ValueError("headset event exceeds 8192 bytes")
            self.pose_transport.sendto(data, (self.session.peer, self.event_port))

    def status(self):
        result = self.session.status()
        result.update(
            pose_port=self.pose_port,
            discovery_port=self.discovery_port,
            callback_error=self.callback_error,
            protocol_version=self.protocol_version,
            controller_inputs_available=self.protocol_version == 2,
        )
        return result

    async def close(self):
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass
        for transport in (self.pose_transport, self.discovery_transport):
            if transport:
                transport.close()
        self.session.release()
