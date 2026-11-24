"""WebSocket 事件广播与高频遥测合并器。"""
from __future__ import annotations

import asyncio
import json
from typing import Any, Awaitable, Callable

from aiohttp import web


def _json_default(obj: Any) -> Any:
    if isinstance(obj, bytes):
        return obj.decode("utf-8", errors="replace")
    if hasattr(obj, "tolist"):
        return obj.tolist()
    return str(obj)


class WebSocketBroadcaster:
    """管理 WebSocket 连接集合，并对高频遥测做最新帧合并以防阻塞事件循环。"""

    def __init__(
        self,
        send_event: Callable[[dict[str, Any]], Awaitable[None]] | None = None,
    ) -> None:
        self.clients: set[web.WebSocketResponse] = set()
        self.pending: dict[str, dict[str, Any]] = {}
        self._drain_task: asyncio.Task | None = None
        self._send_event = send_event

    def _sender(self) -> Callable[[dict[str, Any]], Awaitable[None]]:
        return self._send_event if self._send_event is not None else self.broadcast

    def schedule(self, event: dict[str, Any]) -> None:
        """合并高频遥测事件，同时确保状态变更事件不被丢弃。"""
        kind = str(event.get("kind", ""))
        key = ""
        if kind == "vr_pose":
            key = kind
        elif kind == "ros_message":
            key = f"ros_message:{event.get('topic', '')}"
        if not key:
            asyncio.create_task(self._sender()(event))
            return
        self.pending[key] = event
        if self._drain_task is None or self._drain_task.done():
            self._drain_task = asyncio.create_task(self._drain_pending())

    async def _drain_pending(self) -> None:
        send = self._sender()
        while self.pending:
            batch = tuple(self.pending.values())
            self.pending.clear()
            for event in batch:
                await send(event)
                # send_str 可能同步完成；让出事件循环以免遥测持续回填饿死 RTP/定时器。
                await asyncio.sleep(0)

    async def broadcast(self, event: dict[str, Any]) -> None:
        if not self.clients:
            return
        safe_event = {k: v for k, v in event.items() if k != "_raw"}
        try:
            message = json.dumps(
                safe_event,
                ensure_ascii=False,
                separators=(",", ":"),
                default=_json_default,
            )
        except Exception:
            return
        dead = []
        for ws in tuple(self.clients):
            try:
                await ws.send_str(message)
            except (ConnectionError, RuntimeError):
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def stop(self) -> None:
        if self._drain_task is not None:
            self._drain_task.cancel()
            try:
                await self._drain_task
            except asyncio.CancelledError:
                pass
            self._drain_task = None
        self.pending.clear()

    async def close_all(
        self, *, code: int = 1001, message: bytes = b"server shutdown"
    ) -> None:
        for ws in tuple(self.clients):
            await ws.close(code=code, message=message)

