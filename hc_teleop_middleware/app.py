from __future__ import annotations

import asyncio
import copy
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
from .robot_profiles import RobotProfileError, RobotProfileManager, STANDARD_TOPICS
from .ros_bridge import RosBridge
from .topic_recorder import TopicRecorder
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
    def __init__(self, config: dict[str, Any], config_dir: Path):
        self.config = config
        self.config_dir = config_dir
        self.ros: RosBridge | None = None
        self.vr: VrGateway | None = None
        self.camera: CameraService | None = None
        self.recorder: TopicRecorder | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.websockets: set[web.WebSocketResponse] = set()
        self.events: deque[dict[str, Any]] = deque(maxlen=300)
        self.started_at = time.time()
        self._restart_lock = asyncio.Lock()

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        self.recorder = TopicRecorder(self.config["ros"]["recording"], self.config_dir)
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
        if self.recorder is not None:
            await asyncio.to_thread(self.recorder.stop)
        self._log("info", "runtime stopped")

    async def restart(self, config: dict[str, Any]) -> None:
        async with self._restart_lock:
            await self.stop()
            self.config = config
            await self.start()

    def emit(self, event: dict[str, Any], outputs: list[str]) -> None:
        self.events.append(event)
        if "record" in outputs and self.recorder is not None:
            self.recorder.record(event)
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
        if self.config["vr"].get("publish_to_ros", True):
            self.ros.publish(
                self.config["vr"]["data_topic"],
                "std_msgs/msg/String",
                {
                    "data": json.dumps(
                        packet.as_dict(),
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                },
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
            "recording": self.recorder.status() if self.recorder else {"recording": False, "enabled": False},
        }


@web.middleware
async def cors_middleware(request: web.Request, handler: Any) -> web.StreamResponse:
    if request.method == "OPTIONS":
        response: web.StreamResponse = web.Response(status=204)
    else:
        response = await handler(request)
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Access-Control-Allow-Methods"] = "GET,POST,PUT,DELETE,OPTIONS"
    return response


def create_app(store: ConfigStore) -> web.Application:
    config = store.load()
    runtime = MiddlewareRuntime(config, store.path.parent)
    profile_root = Path(config["robot_profiles"]["root"]).expanduser()
    if not profile_root.is_absolute():
        profile_root = store.path.parent / profile_root
    profiles = RobotProfileManager(profile_root)
    app = web.Application(
        client_max_size=100 * 1024 * 1024, middlewares=[cors_middleware]
    )
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
            old_profile_root = store.value["robot_profiles"]["root"]
            saved = store.save(proposed)
            await runtime.restart(saved)
            restart_required = (
                saved["server"] != old_server
                or saved["robot_profiles"]["root"] != old_profile_root
            )
            return web.json_response(
                {"ok": True, "config": saved, "server_restart_required": restart_required}
            )
        except (ConfigError, json.JSONDecodeError, TypeError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def get_robot_profiles(_request: web.Request) -> web.Response:
        active = str(store.value["robot_profiles"].get("active", ""))
        values = profiles.list()
        for value in values:
            value["active"] = value["id"] == active
        return web.json_response(
            {
                "active": active,
                "root": str(profiles.root),
                "profiles": values,
                "standard_topics": STANDARD_TOPICS,
            }
        )

    async def import_robot_profile(request: web.Request) -> web.Response:
        if not request.content_type.startswith("multipart/"):
            raise web.HTTPBadRequest(text="multipart form data is required")
        fields: dict[str, str] = {}
        uploads: dict[str, tuple[str, bytes]] = {}
        try:
            reader = await request.multipart()
            async for part in reader:
                if part.name in {"archive", "file", "zip", "urdf", "config"}:
                    if not part.filename:
                        raise RobotProfileError(f"{part.name} file is required")
                    uploads[part.name] = (
                        part.filename,
                        await part.read(decode=False),
                    )
                elif part.name in {"id", "display_name"}:
                    fields[part.name] = (await part.text()).strip()

            archive_key = next((k for k in ("archive", "file", "zip") if k in uploads), None)
            if archive_key:
                archive_name, archive_payload = uploads[archive_key]
                profile = await asyncio.to_thread(
                    profiles.import_archive,
                    fields.get("id", ""),
                    fields.get("display_name", ""),
                    archive_payload,
                    archive_name,
                )
            elif "urdf" in uploads and "config" in uploads:
                urdf_name, urdf_payload = uploads["urdf"]
                config_name, config_payload = uploads["config"]
                profile = await asyncio.to_thread(
                    profiles.import_profile,
                    fields.get("id", ""),
                    fields.get("display_name", ""),
                    urdf_name,
                    urdf_payload,
                    config_name,
                    config_payload,
                )
            else:
                raise RobotProfileError("robot zip archive is required")

            return web.json_response({"ok": True, "profile": profile}, status=201)
        except RobotProfileError as exc:
            if "already exists" in str(exc):
                raise web.HTTPConflict(text=str(exc)) from exc
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def activate_robot_profile(request: web.Request) -> web.Response:
        profile_id = request.match_info["profile_id"]
        try:
            profile = profiles.get(profile_id)
        except RobotProfileError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        if profile.get("schema") == "invalid":
            raise web.HTTPBadRequest(text="invalid robot profile cannot be activated")
        previous = str(store.value["robot_profiles"].get("active", ""))
        if profile_id != previous:
            proposed = copy.deepcopy(store.value)
            proposed["robot_profiles"]["active"] = profile_id
            saved = store.save(proposed)
            runtime.config = saved
            runtime._on_safety_event(
                f"robot profile changed from {previous or 'none'} to {profile_id}; restart teleop"
            )
        return web.json_response(
            {
                "ok": True,
                "active": profile_id,
                "profile": profile,
                "restart_simulation_required": profile_id != previous,
            }
        )

    async def delete_robot_profile(request: web.Request) -> web.Response:
        profile_id = request.match_info["profile_id"]
        try:
            await asyncio.to_thread(profiles.delete_profile, profile_id)
            active = str(store.value["robot_profiles"].get("active", ""))
            cleared = profile_id == active
            if cleared:
                proposed = copy.deepcopy(store.value)
                proposed["robot_profiles"]["active"] = ""
                saved = store.save(proposed)
                runtime.config = saved
            return web.json_response({"ok": True, "deleted": profile_id, "cleared_active": cleared})
        except RobotProfileError as exc:
            if "does not exist" in str(exc):
                raise web.HTTPNotFound(text=str(exc)) from exc
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

    async def start_recording(request: web.Request) -> web.Response:
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder is not initialized")
        data = {}
        if request.can_read_body:
            try:
                data = await request.json()
            except Exception:
                pass
        filename = str(data.get("filename", "")).strip()
        path = await asyncio.to_thread(runtime.recorder.start, filename)
        return web.json_response({"ok": True, "recording": True, "path": path})

    async def stop_recording(_request: web.Request) -> web.Response:
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder is not initialized")
        rec_status = await asyncio.to_thread(runtime.recorder.stop)
        return web.json_response({"ok": True, "recording": False, "status": rec_status})

    async def get_recording_status(_request: web.Request) -> web.Response:
        if runtime.recorder is None:
            return web.json_response({"recording": False, "enabled": False})
        return web.json_response(runtime.recorder.status())

    async def get_recordings(_request: web.Request) -> web.Response:
        if runtime.recorder is None:
            return web.json_response({
                "files": [],
                "directory": "",
                "total_count": 0,
                "total_size_bytes": 0,
                "total_size_human": "0 B",
            })
        files = await asyncio.to_thread(runtime.recorder.list_recordings)
        total_bytes = sum(f["size_bytes"] for f in files)
        if total_bytes < 1024:
            human = f"{total_bytes} B"
        elif total_bytes < 1024 * 1024:
            human = f"{total_bytes / 1024:.1f} KB"
        elif total_bytes < 1024 * 1024 * 1024:
            human = f"{total_bytes / (1024 * 1024):.2f} MB"
        else:
            human = f"{total_bytes / (1024 * 1024 * 1024):.2f} GB"
        return web.json_response({
            "files": files,
            "directory": str(runtime.recorder.directory),
            "total_count": len(files),
            "total_size_bytes": total_bytes,
            "total_size_human": human,
        })

    async def download_recording(request: web.Request) -> web.FileResponse:
        filename = Path(request.match_info["filename"]).name
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder not available")
        target = (runtime.recorder.directory / filename).resolve()
        if not target.is_relative_to(runtime.recorder.directory) or not target.is_file():
            raise web.HTTPNotFound(text=f"recording not found: {filename}")
        return web.FileResponse(
            target,
            headers={
                "Content-Disposition": f'attachment; filename="{filename}"',
                "Content-Type": "application/octet-stream",
            },
        )

    async def delete_recording(request: web.Request) -> web.Response:
        filename = Path(request.match_info["filename"]).name
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder not available")
        try:
            await asyncio.to_thread(runtime.recorder.delete_recording, filename)
            return web.json_response({"ok": True, "deleted": filename})
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        except ValueError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def batch_delete_recordings(request: web.Request) -> web.Response:
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder not available")
        data = await request.json() if request.can_read_body else {}
        filenames = list(data.get("filenames", []))
        deleted = []
        failed = []
        for fname in filenames:
            try:
                await asyncio.to_thread(runtime.recorder.delete_recording, fname)
                deleted.append(fname)
            except Exception as exc:
                failed.append({"filename": fname, "error": str(exc)})
        return web.json_response({"ok": True, "deleted": deleted, "failed": failed})

    async def startup(_app: web.Application) -> None:
        await runtime.start()

    async def shutdown(_app: web.Application) -> None:
        for ws in tuple(runtime.websockets):
            await ws.close(code=1001, message=b"server shutdown")
        await runtime.stop()

    async def options(_request: web.Request) -> web.Response:
        return web.Response(status=204)

    app.router.add_get("/", index)
    app.router.add_get("/dashboard/", index)
    app.router.add_static("/static/", static_dir)
    app.router.add_get("/api/status", get_status)
    app.router.add_get("/health", get_status)
    app.router.add_get("/api/config", get_config)
    app.router.add_put("/api/config", put_config)
    app.router.add_get("/api/robot-profiles", get_robot_profiles)
    app.router.add_post("/api/robot-profiles/import", import_robot_profile)
    app.router.add_post(
        "/api/robot-profiles/{profile_id}/activate", activate_robot_profile
    )
    app.router.add_delete(
        "/api/robot-profiles/{profile_id}", delete_robot_profile
    )
    app.router.add_get("/api/ros/topics", get_topics)
    app.router.add_post("/api/ros/publish", publish)
    app.router.add_post("/api/safety/stop", emergency_stop)
    app.router.add_post("/api/recording/start", start_recording)
    app.router.add_post("/api/recording/stop", stop_recording)
    app.router.add_get("/api/recording/status", get_recording_status)
    app.router.add_get("/api/recordings", get_recordings)
    app.router.add_get("/api/recordings/{filename}/download", download_recording)
    app.router.add_delete("/api/recordings/{filename}", delete_recording)
    app.router.add_post("/api/recordings/batch-delete", batch_delete_recordings)
    app.router.add_get("/ws", websocket)
    app.router.add_post("/api/webrtc/offer", webrtc_offer)
    app.router.add_post("/offer", webrtc_offer)
    app.router.add_options("/{tail:.*}", options)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    return app
