"""Robot-local control/capture transport. Observation mode never opens a pose socket."""

import asyncio
import logging
import os
from pathlib import Path
import socket
import struct
import time

import aiohttp

LOG = logging.getLogger("platform-device")


def local_directory():
    parent = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp"))
    directory = parent / f"hc-platform-device-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    st = directory.lstat()
    if (
        not directory.is_dir()
        or directory.is_symlink()
        or st.st_uid != os.getuid()
        or st.st_mode & 0o077
    ):
        raise ValueError("unsafe local control directory")
    return directory


class Observe:
    profiles = ("hc-pico-v2", "webxr")

    async def open(self):
        pass

    async def close(self):
        pass

    def ready(self):
        return True

    def send(self, packet):
        pass

    def stop(self):
        pass

    def capabilities(self):
        return {}

    def receipt(self):
        return None

    async def ack(self, receipt):
        pass

    async def end(self, sid):
        pass

    def command(self, command):
        pass


class Live:
    # Desktop sends the existing native PICO pose convention unchanged.
    # Immersive WebXR uses a different reference frame; calibrate before native output.
    profiles = ("hc-pico-v2",)

    def __init__(self, cfg=None):
        transform = (cfg or {}).get("webxr_to_native")
        if transform not in (None, "identity", "reflect-z"):
            raise ValueError("unknown WebXR calibration basis")
        self.reflect_z = transform == "reflect-z"
        self.profiles = ("hc-pico-v2", "webxr") if transform else ("hc-pico-v2",)
        self.wire_seq = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self.state = {}
        self.checked = 0
        self.timer = self.capture = self.notify = self.delivery = None
        self.result = None
        self.udp = None
        self.http = None

    async def open(self):
        self.http = aiohttp.ClientSession(
            connector=aiohttp.UnixConnector(
                path=str(local_directory() / "device.sock")
            ),
            timeout=aiohttp.ClientTimeout(total=2),
        )
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.2", 0))
        self.udp.setblocking(False)
        self.timer = asyncio.create_task(self.refresh())
        self.delivery = asyncio.create_task(self.read_receipts())

    async def refresh(self):
        while True:
            try:
                async with self.http.get("http://localhost/status") as r:
                    r.raise_for_status()
                    self.state = await r.json()
                self.checked = time.monotonic()
            except (OSError, aiohttp.ClientError, TimeoutError, ValueError):
                self.state = {}
            await asyncio.sleep(0.1)

    def ready(self):
        return (
            self.state.get("ready", False)
            and time.monotonic() - self.checked < 0.4
            and not self.notify
        )

    def send(self, packet):
        native = bytearray(packet)
        self.wire_seq = (self.wire_seq + 1) & 0xFFFFFFFF
        struct.pack_into("<Id", native, 5, self.wire_seq, time.monotonic())
        # Local A/stick-click shortcuts resume safety or mark a recording in the
        # legacy gateway. Those actions have separate, acknowledged controls here.
        for offset in (102, 132):
            held, pressed, released = struct.unpack_from("<3H", native, offset)
            struct.pack_into(
                "<3H", native, offset, held & 12, pressed & 12, released & 12
            )
        self.udp.sendto(native, ("127.0.0.1", self.state["pose_port"]))

    def stop(self):
        if self.notify is None:
            self.notify = asyncio.create_task(self.halt())

    async def halt(self):
        try:
            async with self.http.post("http://localhost/halt") as r:
                r.raise_for_status()
        except (OSError, aiohttp.ClientError, TimeoutError):
            LOG.warning("local halt unconfirmed; native pose watchdog remains active")
        finally:
            self.notify = None

    def capabilities(self):
        return (
            self.state.get("capture", {})
            if time.monotonic() - self.checked < 0.4
            else {}
        )

    def receipt(self):
        return self.result

    async def read_receipts(self):
        while True:
            if self.result is None:
                try:
                    async with self.http.get("http://localhost/receipt") as r:
                        r.raise_for_status()
                        item = await r.json()
                    if item and self.result is None:
                        self.result = tuple(item)
                except (OSError, aiohttp.ClientError, TimeoutError, ValueError):
                    pass
            await asyncio.sleep(1)

    async def ack(self, receipt):
        async with self.http.post("http://localhost/confirm", json=receipt[1]) as r:
            r.raise_for_status()
        if self.result == receipt:
            self.result = None

    async def end(self, sid):
        try:
            async with self.http.post(
                "http://localhost/end", json={"session_id": sid}
            ) as r:
                r.raise_for_status()
        except (OSError, aiohttp.ClientError, TimeoutError):
            LOG.warning("session close receipt unconfirmed for %s", sid)

    def command(self, cmd):
        if self.capture and not self.capture.done():
            return
        self.capture = asyncio.create_task(self.perform(cmd))

    async def perform(self, cmd):
        try:
            async with self.http.post(
                "http://localhost/capture",
                json=cmd,
                timeout=aiohttp.ClientTimeout(total=120),
            ) as r:
                r.raise_for_status()
                receipt = await r.json()
            self.result = (cmd["session_id"], receipt)
        except (OSError, aiohttp.ClientError, TimeoutError, ValueError):
            LOG.warning(
                "capture result unavailable; request %s can be retried",
                cmd.get("request_id"),
            )

    async def close(self):
        self.timer.cancel()
        self.delivery.cancel()
        await asyncio.gather(self.timer, self.delivery, return_exceptions=True)
        if self.capture and not self.capture.done():
            self.capture.cancel()
            await asyncio.gather(self.capture, return_exceptions=True)
        if self.notify:
            await self.notify
        self.udp.close()
        await self.http.close()
