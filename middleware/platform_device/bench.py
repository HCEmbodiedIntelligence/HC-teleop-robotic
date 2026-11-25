"""Local WebRTC performance and failure-injection bench. Never opens robot I/O."""

import argparse
import asyncio
import json
import os
import time
import uuid

from aiohttp import web
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription

from middleware.core.protocol import decode_pose_packet
from .__main__ import TestPattern
from .agent import Agent
from .input import Quantiles
from .sink import Observe


def frame(sid, seq, stamp):
    pose = {"position": [0, 0.3, 0], "orientation": [0, 0, 0, 1]}
    hands = [
        dict(
            hand=h,
            profile="hc-pico-v2",
            grip=pose,
            buttons=[0.75, 0, 0, 0, 0, 0],
            axes=[0] * 4,
        )
        for h in ("left", "right")
    ]
    return dict(
        version=1, session_id=sid, seq=seq, time_ms=stamp, head=pose, controllers=hands
    )


async def until(test, timeout=10):
    async def wait():
        while not test():
            await asyncio.sleep(0.005)

    await asyncio.wait_for(wait(), timeout)


class BenchSink(Observe):
    def __init__(self):
        self.packets = 0
        self.last_seq = 0
        self.latency = Quantiles()

    def send(self, data):
        packet = decode_pose_packet(data)
        assert abs(packet.right_input.trigger - 0.75) < 0.001
        self.packets += 1
        self.last_seq = packet.sequence
        self.latency.add((time.monotonic() - packet.vr_timestamp) * 1000)


class Rendezvous:
    def __init__(self):
        self.sid = str(uuid.uuid4())
        self.offer = self.answer = None
        self.enabled = True
        self.calls = 0
        self.token = uuid.uuid4().hex

    async def sync(self, request):
        if request.headers.get("Authorization") != "Bearer " + self.token:
            raise web.HTTPUnauthorized()
        m = await request.json()
        self.calls += 1
        if m.get("answer"):
            assert m["session_id"] == self.sid
            self.answer = m["answer"]
        if not self.enabled or m.get("release"):
            return web.json_response({})
        return web.json_response(
            dict(
                session_id=self.sid,
                offer=None if self.answer else self.offer,
                ice_servers=[],
            )
        )

    async def start(self):
        app = web.Application()
        app.router.add_post("/api/v1/robot-ingest/teleoperation/sync", self.sync)
        self.runner = web.AppRunner(app, access_log=None)
        await self.runner.setup()
        site = web.TCPSite(self.runner, "127.0.0.1", 0)
        await site.start()
        return "http://127.0.0.1:" + str(self.runner.addresses[0][1])


async def run(seconds, rate):
    server = Rendezvous()
    base = await server.start()
    cfg = dict(name="bench", base_url=base, watchdog_ms=180, max_lag_ms=100)
    previous = os.environ.get("HC_TELEOP_ROBOT_TOKEN")
    os.environ["HC_TELEOP_ROBOT_TOKEN"] = server.token
    sink = BenchSink()
    agent = Agent(cfg, TestPattern, sink)
    if previous is None:
        os.environ.pop("HC_TELEOP_ROBOT_TOKEN")
    else:
        os.environ["HC_TELEOP_ROBOT_TOKEN"] = previous
    pc = RTCPeerConnection(RTCConfiguration([]))
    pc.addTransceiver("video", direction="recvonly")
    events = pc.createDataChannel("hc-operator.v1")
    motion = pc.createDataChannel("vr-control.v1", ordered=False, maxRetransmits=0)
    replies = []
    events.on("message", lambda text: replies.append(json.loads(text)))
    video_gaps = Quantiles()
    frames = 0

    async def drain(track):
        nonlocal frames
        last = None
        while True:
            await track.recv()
            now = time.monotonic()
            if last is not None:
                video_gaps.add((now - last) * 1000)
            frames += 1
            last = now

    drains = []
    pc.on("track", lambda track: drains.append(asyncio.create_task(drain(track))))

    def cmd(kind):
        rid = str(uuid.uuid4())
        events.send(
            json.dumps(
                dict(
                    version=1,
                    type=kind,
                    session_id=server.sid,
                    input_profile="hc-pico-v2",
                    request_id=rid,
                )
            )
        )
        return rid

    async def exchange(kind, want):
        rid = cmd(kind)
        await until(
            lambda: any(r["type"] == want and r["request_id"] == rid for r in replies)
        )

    def send(seq, stamp=None):
        motion.send(
            json.dumps(
                frame(
                    server.sid,
                    seq,
                    stamp if stamp is not None else time.monotonic() * 1000,
                )
            )
        )

    task = asyncio.create_task(agent.run())
    start = time.monotonic()
    try:
        await pc.setLocalDescription(await pc.createOffer())
        server.offer = pc.localDescription.sdp
        await until(lambda: server.answer is not None)
        await pc.setRemoteDescription(RTCSessionDescription(server.answer, "answer"))
        await until(lambda: events.readyState == "open" and frames >= 2)
        setup = (time.monotonic() - start) * 1000
        await exchange("hello", "ready")
        send(1)
        await asyncio.sleep(0.04)
        assert sink.packets == 0
        await exchange("enable", "enabled")
        seq = 0
        origin = time.monotonic()
        for seq in range(1, int(seconds * rate) + 1):
            await asyncio.sleep(max(0, origin + (seq - 1) / rate - time.monotonic()))
            send(seq)
        await until(lambda: sink.packets >= int(seq * 0.95), timeout=1)
        await asyncio.sleep(0.02)
        baseline = agent.peer.metrics()
        latency = sink.latency.report()
        send(seq)
        await until(lambda: agent.peer.window.reordered > 0)
        send(seq + 1, time.monotonic() * 1000 + 5000)
        await until(lambda: not agent.peer.window.enabled)
        await exchange("enable", "enabled")
        await asyncio.sleep(0.03)
        send(seq + 2)
        await until(lambda: agent.peer.window.prev_seq == seq + 2)
        await asyncio.sleep(0.125)
        send(seq + 3, agent.peer.window.base_stamp + 1)
        await until(lambda: not agent.peer.window.enabled)
        await exchange("enable", "enabled")
        await asyncio.sleep(0.06)
        send(seq + 4)
        await until(lambda: agent.peer.window.prev_seq == seq + 4)
        stop_at = agent.peer.window.last
        await until(lambda: not agent.peer.window.enabled)
        watchdog = (time.monotonic() - stop_at) * 1000
        await exchange("enable", "enabled")
        await exchange("disable", "disabled")
        server.enabled = False
        await until(lambda: agent.peer is None)
        assert (
            latency["p99"] < cfg["max_lag_ms"] and watchdog < cfg["watchdog_ms"] + 100
        )
        return dict(
            sent=seq,
            received=baseline["accepted"],
            rate_hz=rate,
            webrtc_input_to_device_ms=latency,
            receiver=baseline,
            watchdog_ms=round(watchdog, 2),
            first_video_ms=round(setup, 2),
            decoded_video_frames=frames,
            video_frame_interval_ms=video_gaps.report(),
            checks=[
                "preview",
                "ack",
                "mapped pose",
                "duplicate",
                "future timestamp",
                "late sample",
                "watchdog",
                "disable",
                "release",
            ],
        )
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await pc.close()
        for d in drains:
            d.cancel()
        await asyncio.gather(*drains, return_exceptions=True)
        await server.runner.cleanup()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=10)
    p.add_argument("--hz", type=int, default=90)
    a = p.parse_args()
    if not 1 <= a.seconds <= 600 or not 1 <= a.hz <= 240:
        p.error("invalid duration or rate")
    print(json.dumps(asyncio.run(run(a.seconds, a.hz)), indent=2))


if __name__ == "__main__":
    main()
