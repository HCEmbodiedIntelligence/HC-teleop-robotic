"""Device-initiated rendezvous. HTTP is off the video/input hot path."""

import asyncio
import json
import logging
import os
import stat
import time
import uuid
from pathlib import Path

import aiohttp

from middleware.platform_sessions import PlatformHTTPError, RobotPlatform, platform_url

from .peer import Peer

LOG = logging.getLogger("platform-device")


def settings(path):
    cfg = json.loads(Path(path).expanduser().read_text())
    cfg["base_url"] = platform_url(cfg["base_url"])
    cfg.setdefault("max_lag_ms", 100)
    cfg.setdefault("watchdog_ms", 180)
    if not 10 <= cfg["max_lag_ms"] <= cfg["watchdog_ms"] <= 1000:
        raise ValueError("expected 10 <= max_lag_ms <= watchdog_ms <= 1000")
    if not isinstance(cfg.get("name"), str) or not 1 <= len(cfg["name"]) <= 120:
        raise ValueError("set the device display name")
    if cfg.get("mode", "observe") not in ("observe", "live"):
        raise ValueError("mode must be observe or live")
    return cfg


def credential(cfg):
    if cfg.get("token_file"):
        p = Path(cfg["token_file"]).expanduser()
        if p.stat().st_mode & (stat.S_IRWXG | stat.S_IRWXO):
            raise ValueError("restrict token_file permissions to the device account")
        token = p.read_text().strip()
    else:
        token = os.environ.get("HC_TELEOP_ROBOT_TOKEN", "").strip()
    if not token:
        raise ValueError("set HC_TELEOP_ROBOT_TOKEN or token_file")
    return token


class Agent:
    def __init__(self, cfg, track, sink):
        self.cfg, self.track, self.sink = cfg, track, sink
        self.instance = str(uuid.uuid4())
        self.peer = self.negotiation = self.answer = None
        self.confirmed = 0
        self.status = {"state": "starting"}
        self.session_id = None
        self.ack_sent = False
        self.platform = RobotPlatform(cfg["base_url"], credential(cfg))

    async def sync(self, **fields):
        return await self.platform.sync(
            self.instance, self.cfg["name"], **self.sink.capabilities(), **fields
        )

    async def drop(self):
        if self.negotiation:
            self.negotiation.cancel()
            await asyncio.gather(self.negotiation, return_exceptions=True)
        if self.peer:
            LOG.info("final metrics %s", json.dumps(self.peer.metrics()))
            await self.peer.close()
        if self.session_id:
            await self.sink.end(self.session_id)
        self.session_id = None
        self.peer = self.negotiation = self.answer = None
        self.ack_sent = False

    async def poll(self):
        fields = {}
        if self.peer and self.negotiation and self.negotiation.done():
            try:
                self.answer = self.negotiation.result()
            except Exception:
                sid = self.peer.id
                await self.drop()
                await self.sync(session_id=sid, release=True)
                raise
            self.negotiation = None
        if self.peer and self.peer.pc.connectionState in ("closed", "failed"):
            sid = self.peer.id
            await self.drop()
            await self.sync(session_id=sid, release=True)
            return
        if self.peer and self.answer and not self.ack_sent:
            fields.update(session_id=self.peer.id, answer=self.answer)
        receipt = self.sink.receipt()
        if receipt and not fields:
            fields.update(session_id=receipt[0], capture_result=receipt[1])
        else:
            receipt = None
        start = time.monotonic()
        try:
            reply = await self.sync(**fields)
        except PlatformHTTPError as exc:
            if exc.status != 409:
                raise
            if not receipt:
                await self.drop()
                raise
            # Distinguish a superseded recording slot from a competing instance.
            # Only retire a stale receipt after this identity can still sync.
            reply = await self.sync()
        self.confirmed = start
        if fields.get("answer"):
            self.ack_sent = True
        sid = reply.get("session_id")
        if self.session_id != sid:
            await self.drop()
            self.session_id = sid
        if receipt:
            await self.sink.ack(receipt)
        if sid and reply.get("offer") and not self.peer:
            self.peer = Peer(
                sid, reply.get("ice_servers", []), self.track, self.sink, self.cfg
            )
            self.peer.lease_valid = lambda: time.monotonic() - self.confirmed < 4
            self.negotiation = asyncio.create_task(
                asyncio.wait_for(self.peer.answer(reply["offer"]), 12)
            )
        command = reply.get("capture_command")
        if command and sid and command.get("session_id") == sid:
            self.sink.command(command)

    async def run(self):
        async with self.platform:
            await self.sink.open()
            last_log = 0
            try:
                while True:
                    try:
                        await self.poll()
                        if self.status.get("state") in ("starting", "unavailable"):
                            LOG.info(
                                "registered %s, instance %s",
                                self.cfg["name"],
                                self.instance,
                            )
                        self.status = {
                            "state": "connected" if self.peer else "online",
                            "instance": self.instance,
                            "metrics": self.status.get("metrics")
                            if self.peer
                            else None,
                        }
                    except (
                        aiohttp.ClientError,
                        TimeoutError,
                        ValueError,
                        RuntimeError,
                        OSError,
                    ) as exc:
                        LOG.warning("rendezvous: %s", exc)
                        self.status = {"state": "unavailable", "error": str(exc)[:240]}
                        if self.session_id and time.monotonic() - self.confirmed > 4:
                            await self.drop()
                    if self.peer and time.monotonic() - last_log > 2:
                        self.status["metrics"] = self.peer.metrics()
                        LOG.info(
                            "metrics %s",
                            json.dumps(self.status["metrics"], separators=(",", ":")),
                        )
                        last_log = time.monotonic()
                    await asyncio.sleep(0.25 if self.peer and not self.ack_sent else 1)
            finally:
                sid = self.session_id
                await self.drop()
                if sid:
                    try:
                        await self.sync(session_id=sid, release=True)
                    except (
                        aiohttp.ClientError,
                        TimeoutError,
                        RuntimeError,
                        ValueError,
                        OSError,
                    ):
                        pass
                await self.sink.close()
