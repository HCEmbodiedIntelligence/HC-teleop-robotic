"""Bounded asynchronous client for the local control authority."""

from __future__ import annotations

import asyncio
import contextlib
import json
import socket
import uuid


class CoreError(RuntimeError):
    def __init__(self, code: str, message: str = "", state=None):
        super().__init__(message or code)
        self.code, self.state = code, state


class CoreClient:
    def __init__(self, path: str, role="api", event_queue_size: int = 256):
        self.path, self.role = str(path), role
        self.socket = None
        self.reader = None
        self.pending = {}
        self.state = {}
        self.events = asyncio.Queue(maxsize=max(1, int(event_queue_size)))
        self.lock = asyncio.Lock()
        self.send_lock = asyncio.Lock()
        self.dropped_events = 0

    async def connect(self):
        async with self.lock:
            if self.reader and not self.reader.done():
                return
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            sock.setblocking(False)
            try:
                await asyncio.get_running_loop().sock_connect(sock, self.path)
            except BaseException:
                sock.close()
                raise
            self.socket = sock
            self.reader = asyncio.create_task(self._read(), name=f"core-{self.role}")
            await self.request("hello", role=self.role, _connected=True)

    async def request(self, op, _connected=False, **parameters):
        if not _connected:
            await self.connect()
        if len(self.pending) >= 64:
            raise CoreError("CLIENT_BUSY")
        request_id = uuid.uuid4().hex
        payload = json.dumps(
            {**parameters, "op": op, "id": request_id}, allow_nan=False
        ).encode()
        if len(payload) > 65536:
            raise CoreError("REQUEST_TOO_LARGE")
        future = asyncio.get_running_loop().create_future()
        self.pending[request_id] = future
        try:
            async with self.send_lock:
                await asyncio.wait_for(
                    asyncio.get_running_loop().sock_sendall(self.socket, payload), 1
                )
            response = await asyncio.wait_for(future, 2)
            if not response.get("ok", False):
                raise CoreError(
                    response.get("code", "REJECTED"),
                    response.get("message", ""),
                    response.get("state"),
                )
            return response
        finally:
            self.pending.pop(request_id, None)

    async def _read(self):
        try:
            while True:
                raw = await asyncio.get_running_loop().sock_recv(self.socket, 65536)
                if not raw:
                    raise ConnectionError("Control core disconnected")
                message = json.loads(raw)
                if message.get("type") == "state":
                    self.state = message.get("state", message)
                elif isinstance(message.get("state"), dict):
                    self.state = message["state"]
                if message.get("type") == "response":
                    future = self.pending.get(str(message.get("id")))
                    if future and not future.done():
                        future.set_result(message)
                else:
                    try:
                        self.events.put_nowait(message)
                    except asyncio.QueueFull:
                        self.dropped_events += 1
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            for future in tuple(self.pending.values()):
                if not future.done():
                    future.set_exception(CoreError("CORE_DISCONNECTED", str(exc)))
        finally:
            if self.socket:
                self.socket.close()

    async def close(self):
        if self.reader:
            self.reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.reader
        self.reader = None
        if self.socket:
            self.socket.close()
            self.socket = None
