"""One platform lease owns a peer, its camera track, and its input admission."""

import asyncio
from collections import deque
import json
import logging
import time
from uuid import UUID

from aiortc import (
    MediaStreamTrack,
    RTCConfiguration,
    RTCIceServer,
    RTCPeerConnection,
    RTCSessionDescription,
    RTCRtpSender,
)

from .input import InputWindow

LOG = logging.getLogger("platform-device")


class WatchedVideo(MediaStreamTrack):
    kind = "video"

    def __init__(self, source):
        super().__init__()
        self.source = source
        self.last = float("-inf")

    async def recv(self):
        frame = await self.source.recv()
        self.last = time.monotonic()
        return frame

    def stop(self):
        self.source.stop()
        super().stop()


class Peer:
    def __init__(self, session_id, servers, track_factory, sink, settings):
        self.id = session_id
        self.lease_valid = lambda: True
        self.pc = RTCPeerConnection(
            RTCConfiguration([RTCIceServer(**s) for s in servers])
        )
        self.track = WatchedVideo(track_factory())
        self.sender = self.pc.addTrack(self.track)
        video = next(t for t in self.pc.getTransceivers() if t.sender == self.sender)
        codecs = RTCRtpSender.getCapabilities("video").codecs
        h264 = [c for c in codecs if c.mimeType.lower() == "video/h264"]
        if h264:
            video.setCodecPreferences(h264 + [c for c in codecs if c not in h264])
        self.sink = sink
        self.window = InputWindow(settings["max_lag_ms"], settings["watchdog_ms"])
        self.events = self.motion = self.profile = None
        self.closed = False
        self.enable_ids = deque(maxlen=256)
        self.ambiguous_restart = False
        self.pending = None
        self.flush_handle = None
        self.coalesced = self.invalid = 0
        self.last_ping = float("-inf")
        self.monitor = asyncio.create_task(self.watch())
        self.pc.on("datachannel", self.channel)
        self.pc.on("connectionstatechange", self.connection_change)

    async def answer(self, offer):
        await self.pc.setRemoteDescription(RTCSessionDescription(offer, "offer"))
        await self.pc.setLocalDescription(await self.pc.createAnswer())
        return self.pc.localDescription.sdp

    def ready(self):
        return (
            self.lease_valid()
            and self.sink.ready()
            and time.monotonic() - self.track.last < 0.5
        )

    def connection_change(self):
        if self.pc.connectionState in ("failed", "closed", "disconnected"):
            self.stop("peer lost")

    def channel(self, ch):
        if (
            ch.label == "hc-operator.v1"
            and self.events is None
            and ch.ordered
            and ch.maxRetransmits is None
            and ch.maxPacketLifeTime is None
        ):
            self.events = ch
            ch.on("message", self.command)
        elif (
            ch.label == "vr-control.v1"
            and self.motion is None
            and not ch.ordered
            and ch.maxRetransmits == 0
        ):
            self.motion = ch
            ch.on("message", self.receive)
        else:
            ch.close()
            return
        ch.on("close", lambda: self.stop("data channel closed"))

    def reply(self, kind, rid="", reason=""):
        if self.events and self.events.readyState == "open":
            self.events.send(
                json.dumps(
                    dict(
                        type=kind,
                        version=1,
                        session_id=self.id,
                        request_id=rid,
                        input_profile=self.profile,
                        reason=reason,
                    )
                )
            )

    def command(self, data):
        try:
            if not isinstance(data, str) or len(data) > 4096 or self.closed:
                return
            msg = json.loads(data)
            if msg["version"] != 1 or msg["session_id"] != self.id:
                return
            kind, rid, profile = msg["type"], msg["request_id"], msg["input_profile"]
            UUID(rid)
            if kind == "latency_probe" and time.monotonic() - self.last_ping >= 0.1:
                self.last_ping = time.monotonic()
                self.reply("latency_echo", rid)
            elif kind == "disable":
                self.stop("operator disabled", rid, notify=True)
            elif kind == "hello" and profile in self.sink.profiles:
                if self.profile and self.profile != profile:
                    self.stop("profile changed")
                self.profile = profile
                self.reply("ready", rid)
            elif kind == "enable" and self.profile and profile == self.profile:
                if rid in self.enable_ids or self.window.enabled:
                    return
                self.enable_ids.append(rid)
                if self.ambiguous_restart:
                    self.reply(
                        "disabled", rid, "reconnect after an enable with no input"
                    )
                    return
                if (
                    not self.motion
                    or self.motion.readyState != "open"
                    or not self.ready()
                ):
                    self.reply("disabled", rid, "device not ready")
                    return
                self.window.arm()
                self.reply("enabled", rid)
        except (ValueError, TypeError, KeyError, AttributeError, RecursionError):
            self.stop("invalid control message")

    def receive(self, data):
        if not self.window.enabled or self.closed:
            return
        arrived = time.monotonic()
        try:
            if not isinstance(data, str) or len(data) > 8192:
                raise ValueError("invalid frame size")
            msg = json.loads(data)
            if msg["session_id"] != self.id or msg["version"] != 1:
                return
            seq = msg["seq"]
            if type(seq) is not int:
                raise ValueError("invalid sequence")
            if self.pending and seq <= self.pending[0]["seq"]:
                self.window.reordered += 1
                return
            if self.pending:
                self.coalesced += 1
            self.pending = (msg, arrived)
            if self.flush_handle is None:
                self.flush_handle = asyncio.get_running_loop().call_soon(self.flush)
        except (ValueError, TypeError, KeyError, RecursionError):
            self.invalid += 1
            self.stop("malformed frame")

    def flush(self):
        (msg, arrived), self.pending = self.pending, None
        self.flush_handle = None
        if not self.window.enabled or self.closed:
            return
        try:
            if not self.ready():
                raise RuntimeError("local control interlock")
            packet = self.window.sample(
                msg, self.profile, getattr(self.sink, "reflect_z", False)
            )
            if packet:
                self.sink.send(packet)
                self.window.handling.add((time.monotonic() - arrived) * 1000)
        except (
            TypeError,
            KeyError,
            IndexError,
            ValueError,
            RuntimeError,
            TimeoutError,
            OSError,
            OverflowError,
        ) as exc:
            self.invalid += 1
            self.stop(str(exc))

    def stop(self, reason, rid="", notify=False):
        active = self.window.enabled
        self.window.enabled = False
        self.pending = None
        if self.flush_handle:
            self.flush_handle.cancel()
            self.flush_handle = None
        if active:
            if self.window.anchor is None:
                self.ambiguous_restart = True
            self.sink.stop()
            LOG.info("input stopped: %s", reason)
        if active or notify:
            self.reply("disabled", rid, reason)

    async def watch(self):
        while True:
            await asyncio.sleep(0.01)
            if self.window.expire() or (self.window.enabled and not self.ready()):
                self.stop("watchdog or local control interlock")

    def metrics(self):
        w = self.window
        return dict(
            session=self.id,
            peer=self.pc.connectionState,
            enabled=w.enabled,
            accepted=w.accepted,
            sequence_gaps=w.skipped,
            reordered=w.reordered,
            coalesced=self.coalesced,
            expired=w.expired,
            invalid=self.invalid,
            input_age_upper_ms=w.age.report(),
            input_gap_ms=w.gaps.report(),
            device_handler_ms=w.handling.report(),
            last_input=w.last_values,
        )

    async def close(self):
        self.closed = True
        self.stop("session released")
        self.monitor.cancel()
        await asyncio.gather(self.monitor, return_exceptions=True)
        self.track.stop()
        await self.pc.close()
