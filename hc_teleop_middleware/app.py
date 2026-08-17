from __future__ import annotations

import asyncio
import json
import socket
import time
from collections import deque
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web

from .camera import CameraService
from .config import ConfigError, ConfigStore
from .protocol import envelope
from .ros_bridge import RosBridge
from .vr_gateway import VrGateway


def local_ip() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


class MiddlewareRuntime:
    def __init__(self, config: dict[str, Any]):
        self.config = config
        self.ros: RosBridge | None = None
        self.vr: VrGateway | None = None
        self.camera: CameraService | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.websockets: set[web.WebSocketResponse] = set()
        self.events: deque[dict[str, Any]] = deque(maxlen=300)
        self.started_at = time.time()
        self._restart_lock = asyncio.Lock()

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        self.ros = RosBridge(self.config["ros"], self.emit)
        self.vr = VrGateway(
            self.config["vr"], self._on_pose, self.emit, self._on_safety_event
        )
        self.camera = CameraService(self.config["camera"])
        self.ros.start()
        self.vr.start()
        self.camera.start()
        self._log("info", "runtime started")
        if self.config["safety"].get("stop_on_startup", True):
            self._on_safety_event("middleware startup")

    async def stop(self) -> None:
        if self.vr is not None:
            await asyncio.to_thread(self.vr.stop)
        if self.ros is not None:
            await asyncio.to_thread(self.ros.stop)
        if self.camera is not None:
            await self.camera.stop()
        self._log("info", "runtime stopped")

    async def restart(self, config: dict[str, Any]) -> None:
        async with self._restart_lock:
            await self.stop()
            self.config = config
            await self.start()

    def emit(self, event: dict[str, Any], outputs: list[str]) -> None:
        self.events.append(event)
        if "udp" in outputs and self.vr is not None:
            self.vr.send_event(event)
        if "websocket" in outputs and self.loop is not None:
            self.loop.call_soon_threadsafe(
                lambda: asyncio.create_task(self._broadcast(event))
            )

    async def _broadcast(self, event: dict[str, Any]) -> None:
        if not self.websockets:
            return
        message = json.dumps(event, ensure_ascii=False, separators=(",", ":"))
        dead = []
        for ws in tuple(self.websockets):
            try:
                await ws.send_str(message)
            except (ConnectionError, RuntimeError):
                dead.append(ws)
        for ws in dead:
            self.websockets.discard(ws)

    def _on_pose(self, packet: Any) -> None:
        if self.ros is None:
            return
        if self.config["vr"].get("publish_pose_to_ros", True):
            topics = self.config["vr"]["pose_topics"]
            for name in ("head", "left", "right"):
                if packet.tracked(name):
                    self.ros.publish_pose(
                        topics[name], getattr(packet, name), packet.sequence
                    )
        if (
            packet.protocol_version >= 2
            and self.config["vr"].get("publish_input_to_ros", True)
        ):
            topics = self.config["vr"]["input_topics"]
            for side in ("left", "right"):
                self.ros.publish_controller_input(
                    topics[side],
                    self.config["vr"]["event_topic"],
                    side,
                    getattr(packet, f"{side}_input"),
                    packet.sequence,
                    packet.vr_timestamp,
                )

    def _on_safety_event(self, reason: str) -> None:
        self._log("warning", reason)
        if self.ros is not None and self.config["safety"].get("enabled", True):
            self.ros.emergency_stop(self.config["safety"]["stop_topic"], reason)
        self.emit(envelope("safety_stop", "middleware", {"reason": reason}), ["websocket", "udp"])

    def _log(self, level: str, message: str) -> None:
        self.events.append(
            envelope("log", "middleware", {"level": level, "message": message})
        )

    def status(self) -> dict[str, Any]:
        return {
            "status": "ok",
            "version": 1,
            "uptime_seconds": round(time.time() - self.started_at, 1),
            "websocket_clients": len(self.websockets),
            "ros": self.ros.status() if self.ros else {"state": "stopped"},
            "vr": self.vr.status() if self.vr else {"state": "stopped"},
            "camera": self.camera.status() if self.camera else {"state": "stopped"},
        }


@web.middleware
async def cors_middleware(request: web.Request, handler: Any) -> web.StreamResponse:
    if request.method == "OPTIONS":
        response: web.StreamResponse = web.Response(status=204)
    else:
        response = await handler(request)
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Access-Control-Allow-Methods"] = "GET,POST,PUT,OPTIONS"
    return response


def create_app(store: ConfigStore) -> web.Application:
    config = store.load()
    runtime = MiddlewareRuntime(config)
    app = web.Application(
        client_max_size=4 * 1024 * 1024, middlewares=[cors_middleware]
    )
    app["store"] = store
    app["runtime"] = runtime
    static_dir = Path(__file__).parent / "static"

    async def index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(static_dir / "index.html")

    async def get_status(_request: web.Request) -> web.Response:
        return web.json_response(runtime.status())

    async def get_config(_request: web.Request) -> web.Response:
        return web.json_response(store.value)

    async def put_config(request: web.Request) -> web.Response:
        try:
            proposed = await request.json()
            old_server = dict(store.value["server"])
            saved = store.save(proposed)
            await runtime.restart(saved)
            restart_required = saved["server"] != old_server
            return web.json_response(
                {"ok": True, "config": saved, "server_restart_required": restart_required}
            )
        except (ConfigError, json.JSONDecodeError, TypeError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def get_topics(_request: web.Request) -> web.Response:
        status = runtime.ros.status() if runtime.ros else {}
        return web.json_response(status.get("discovered_topics", []))

    async def publish(request: web.Request) -> web.Response:
        data = await request.json()
        required = ("topic", "type", "data")
        if any(key not in data for key in required) or not isinstance(data["data"], dict):
            raise web.HTTPBadRequest(text="topic, type and object data are required")
        if runtime.ros is None or runtime.ros.status()["state"] != "running":
            raise web.HTTPServiceUnavailable(text="ROS bridge is not running")
        accepted = runtime.ros.publish(data["topic"], data["type"], data["data"])
        return web.json_response({"accepted": accepted}, status=202 if accepted else 429)

    async def emergency_stop(request: web.Request) -> web.Response:
        data = await request.json() if request.can_read_body else {}
        runtime._on_safety_event(str(data.get("reason", "dashboard emergency stop")))
        return web.json_response({"accepted": True}, status=202)

    async def get_events(request: web.Request) -> web.Response:
        try:
            limit = min(max(int(request.query.get("limit", "100")), 1), 300)
        except ValueError:
            raise web.HTTPBadRequest(text="limit must be an integer")
        return web.json_response(list(runtime.events)[-limit:])

    async def websocket(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=1024 * 1024)
        await ws.prepare(request)
        runtime.websockets.add(ws)
        await ws.send_json(envelope("connected", "middleware", runtime.status()))
        try:
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(message.data)
                    except json.JSONDecodeError:
                        await ws.send_json({"error": "invalid JSON"})
                        continue
                    if data.get("kind") == "ping":
                        await ws.send_json(envelope("pong", "middleware", {}))
                    elif data.get("kind") == "ros_publish":
                        accepted = bool(
                            runtime.ros
                            and runtime.ros.publish(
                                data.get("topic", ""),
                                data.get("msg_type", ""),
                                data.get("payload", {}),
                            )
                        )
                        await ws.send_json({"kind": "publish_ack", "accepted": accepted})
                elif message.type == WSMsgType.ERROR:
                    break
        finally:
            runtime.websockets.discard(ws)
        return ws

    async def webrtc_offer(request: web.Request) -> web.Response:
        if runtime.camera is None:
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        try:
            answer = await runtime.camera.offer(await request.json())
            return web.json_response(answer)
        except (RuntimeError, KeyError, TypeError) as exc:
            raise web.HTTPServiceUnavailable(text=str(exc)) from exc

    async def startup(_app: web.Application) -> None:
        await runtime.start()

    async def shutdown(_app: web.Application) -> None:
        for ws in tuple(runtime.websockets):
            await ws.close(code=1001, message=b"server shutdown")
        await runtime.stop()

    app.router.add_get("/", index)
    app.router.add_get("/dashboard/", index)
    app.router.add_static("/static/", static_dir)
    app.router.add_get("/api/status", get_status)
    app.router.add_get("/health", get_status)
    app.router.add_get("/api/config", get_config)
    app.router.add_put("/api/config", put_config)
    app.router.add_get("/api/ros/topics", get_topics)
    app.router.add_post("/api/ros/publish", publish)
    app.router.add_post("/api/safety/stop", emergency_stop)
    app.router.add_get("/api/events", get_events)
    app.router.add_get("/ws", websocket)
    app.router.add_post("/api/webrtc/offer", webrtc_offer)
    app.router.add_post("/offer", webrtc_offer)
    app.router.add_options("/{tail:.*}", lambda _request: web.Response(status=204))
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    return app
