"""Small nonblocking SOCK_SEQPACKET client shared by adapter processes."""

import asyncio
import json
import socket


class CoreConnection:
    def __init__(self, path, role, on_message, capabilities=None):
        self.path, self.role, self.on_message = str(path), role, on_message
        self.capabilities = capabilities or {}
        self.socket = None
        self.reader = None
        self._send_lock = asyncio.Lock()
        self.state = {}
        self.closed = asyncio.Event()
        self.error = None

    async def start(self):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.setblocking(False)
        await asyncio.wait_for(
            asyncio.get_running_loop().sock_connect(self.socket, self.path), 5
        )
        self.reader = asyncio.create_task(self._read(), name=self.role + "-ipc-reader")
        await self.send(
            {
                "op": "hello",
                "id": "hello",
                "role": self.role,
                "capabilities": self.capabilities,
            }
        )

    async def send(self, message):
        raw = json.dumps(message, separators=(",", ":"), allow_nan=False).encode()
        if len(raw) > 65536:
            raise ValueError("IPC message too large")
        if self.closed.is_set() or self.socket is None:
            raise ConnectionError("control core disconnected")
        async with self._send_lock:
            await asyncio.wait_for(
                asyncio.get_running_loop().sock_sendall(self.socket, raw), 0.5
            )

    async def _read(self):
        try:
            while True:
                raw = await asyncio.get_running_loop().sock_recv(self.socket, 65537)
                if not raw:
                    break
                if len(raw) > 65536:
                    raise ValueError("oversized core packet")
                message = json.loads(
                    raw,
                    parse_constant=lambda x: (_ for _ in ()).throw(
                        ValueError("nonfinite JSON")
                    ),
                )
                if not isinstance(message, dict):
                    raise ValueError("core message must be object")
                if message.get("type") == "state":
                    self.state = message.get("state", message)
                if (
                    message.get("type") == "response"
                    and message.get("id") == "hello"
                    and not message.get("ok")
                ):
                    raise ConnectionError(
                        "core rejected role: " + str(message.get("code"))
                    )
                result = self.on_message(message)
                if asyncio.iscoroutine(result):
                    await result
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.error = str(error)
        finally:
            self.closed.set()

    async def close(self):
        if self.reader:
            self.reader.cancel()
            try:
                await self.reader
            except asyncio.CancelledError:
                pass
        if self.socket:
            self.socket.close()
        self.closed.set()
