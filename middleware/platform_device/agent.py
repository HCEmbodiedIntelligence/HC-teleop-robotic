"""Device-initiated rendezvous. HTTP is off the video/input hot path."""

import asyncio
import json
import logging
import os
import stat
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit

import aiohttp

from .peer import Peer

LOG = logging.getLogger("platform-device")


def settings(path):
    cfg = json.loads(Path(path).expanduser().read_text())
    base = cfg["base_url"].rstrip("/")
    url = urlsplit(base)
    if (
        (
            url.scheme != "https"
            and not (
                url.scheme == "http"
                and url.hostname in ("localhost", "127.0.0.1", "::1")
            )
        )
        or not url.hostname
        or url.username
        or url.password
        or url.query
        or url.fragment
    ):
        raise ValueError("base_url must be an HTTPS API origin or loopback test server")
    cfg["base_url"] = base
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
        self.http = None
        self.ack_sent = False
        self.token = credential(cfg)

    async def sync(self, **fields):
        payload = dict(
            instance_id=self.instance,
            name=self.cfg["name"],
            **self.sink.capabilities(),
            **fields,
        )
        async with self.http.post(
            self.cfg["base_url"] + "/api/v1/robot-ingest/teleoperation/sync",
            json=payload,
            headers={"Authorization": "Bearer " + self.token},
            allow_redirects=False,
        ) as res:
            if res.status != 200:
                raise RuntimeError(
                    f"robot sync HTTP {res.status} (check API ingress and robot identity)"
                )
            return await res.json()

    async def drop(self):
        if self.negotiation:
            self.negotiation.cancel()
            await asyncio.gather(self.negotiation, return_exceptions=True)
        if self.peer:
            LOG.info("final metrics %s", json.dumps(self.peer.metrics()))
            await self.peer.close()
            await self.sink.end(self.peer.id)
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
        if receipt and (not fields or fields.get("session_id") == receipt[0]):
            fields.update(session_id=receipt[0], capture_result=receipt[1])
        else:
            receipt = None
        start = time.monotonic()
        try:
            reply = await self.sync(**fields)
        except RuntimeError as exc:
            if "HTTP 409" in str(exc):
                if receipt:
                    await self.sink.ack(receipt)
                else:
                    await self.drop()
            raise
        self.confirmed = start
        if receipt:
            await self.sink.ack(receipt)
        if fields.get("answer"):
            self.ack_sent = True
        sid = reply.get("session_id")
        if self.peer and self.peer.id != sid:
            await self.drop()
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
        async with aiohttp.ClientSession(
            timeout=aiohttp.ClientTimeout(total=3)
        ) as self.http:
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
                        if self.peer and time.monotonic() - self.confirmed > 4:
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
                sid = self.peer.id if self.peer else None
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
