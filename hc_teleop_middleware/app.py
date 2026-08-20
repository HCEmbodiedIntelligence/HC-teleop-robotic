from __future__ import annotations

import asyncio
import copy
import json
import os
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
from .topic_player import TopicPlayer
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
        self.player: TopicPlayer | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.websockets: set[web.WebSocketResponse] = set()
        self.events: deque[dict[str, Any]] = deque(maxlen=300)
        self.started_at = time.time()
        self._restart_lock = asyncio.Lock()
        self._last_x_held = False
        self._last_y_held = False

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        domain_id = int(self.config.get("ros", {}).get("domain_id", 13))
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init(args=[], domain_id=domain_id)
        except Exception:
            pass

        self.recorder = TopicRecorder(self.config["ros"]["recording"], self.config_dir)
        self.player = TopicPlayer(self.recorder.directory, lambda: self.ros)
        self.camera = CameraService(
            self.config.get("camera", {}),
            domain_id=domain_id,
        )
        self.ros = RosBridge(
            self.config["ros"],
            self.emit,
            on_frame=self.camera.handle_ros_message,
        )
        self.vr = VrGateway(
            self.config["vr"], self._on_pose, self.emit, self._on_safety_event
        )
        self.ros.start()
        self.vr.start()
        self.camera.start()
        self._log("info", "runtime started")
        if self.config["safety"].get("stop_on_startup", True):
            self._on_safety_event("middleware startup")

    async def stop(self) -> None:
        if self.player is not None:
            await asyncio.to_thread(self.player.stop)
        if self.vr is not None:
            await asyncio.to_thread(self.vr.stop)
        if self.camera is not None:
            await self.camera.stop()
        if self.ros is not None:
            await asyncio.to_thread(self.ros.stop)
        if self.recorder is not None:
            await asyncio.to_thread(self.recorder.stop)
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
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

        def _json_default(obj: Any) -> Any:
            if isinstance(obj, bytes):
                return obj.decode("utf-8", errors="replace")
            if hasattr(obj, "tolist"):
                return obj.tolist()
            return str(obj)

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
        for ws in tuple(self.websockets):
            try:
                await ws.send_str(message)
            except (ConnectionError, RuntimeError):
                dead.append(ws)
        for ws in dead:
            self.websockets.discard(ws)

    def _on_pose(self, packet: Any) -> None:
        if hasattr(packet, "right_input") and packet.right_input is not None:
            if (packet.right_input.pressed_mask & 1) or (packet.right_input.held_mask & 1):
                self._on_safety_resume("VR controller A button pressed")

        if hasattr(packet, "left_input") and packet.left_input is not None:
            left_held = int(getattr(packet.left_input, "held_mask", 0))
            left_pressed = int(getattr(packet.left_input, "pressed_mask", 0))

            # X button on left controller: bit 0 (1 << 0) -> Start recording
            x_down = bool(left_pressed & 1) or (bool(left_held & 1) and not self._last_x_held)
            self._last_x_held = bool(left_held & 1)

            # Y button on left controller: bit 1 (1 << 1) -> Stop recording
            y_down = bool(left_pressed & 2) or (bool(left_held & 2) and not self._last_y_held)
            self._last_y_held = bool(left_held & 2)

            if x_down:
                self._handle_vr_record_start()
            elif y_down:
                self._handle_vr_record_stop()

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

    def _handle_vr_record_start(self) -> None:
        if self.recorder is None:
            return
        status = self.recorder.status()
        if status.get("recording", False):
            self._log("info", "VR requested record start, but already recording")
            self.emit(
                envelope(
                    "recording_status",
                    "middleware",
                    {
                        "recording": True,
                        "filename": status.get("active_file", ""),
                        "action": "already_recording",
                        "message": "Already recording",
                    },
                ),
                ["websocket", "udp"],
            )
            return
        try:
            filename = f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.mcap"
            path = self.recorder.start(filename)
            self._log("info", f"Recording started by VR X button: {filename}")
            if self.ros is not None:
                self.ros.publish(
                    "/teleop/recording_state",
                    "std_msgs/msg/String",
                    {"data": json.dumps({"recording": True, "filename": filename})},
                )
            self.emit(
                envelope(
                    "recording_started",
                    "middleware",
                    {
                        "recording": True,
                        "filename": filename,
                        "path": str(path),
                        "action": "start",
                        "message": "Recording started",
                    },
                ),
                ["websocket", "udp"],
            )
        except Exception as exc:
            self._log("error", f"Failed to start recording on VR X button: {exc}")
            self.emit(
                envelope(
                    "recording_error",
                    "middleware",
                    {
                        "recording": False,
                        "error": str(exc),
                        "action": "start_failed",
                    },
                ),
                ["websocket", "udp"],
            )

    def _handle_vr_record_stop(self) -> None:
        if self.recorder is None:
            return
        status = self.recorder.status()
        if not status.get("recording", False):
            self._log("info", "VR requested record stop, but not currently recording")
            self.emit(
                envelope(
                    "recording_status",
                    "middleware",
                    {
                        "recording": False,
                        "action": "not_recording",
                        "message": "Not recording",
                    },
                ),
                ["websocket", "udp"],
            )
            return
        try:
            stopped_status = self.recorder.stop()
            self._log("info", f"Recording stopped by VR Y button: {stopped_status}")
            if self.ros is not None:
                self.ros.publish(
                    "/teleop/recording_state",
                    "std_msgs/msg/String",
                    {"data": json.dumps({"recording": False, "status": stopped_status})},
                )
            self.emit(
                envelope(
                    "recording_stopped",
                    "middleware",
                    {
                        "recording": False,
                        "status": stopped_status,
                        "action": "stop",
                        "message": "Recording stopped",
                    },
                ),
                ["websocket", "udp"],
            )
        except Exception as exc:
            self._log("error", f"Failed to stop recording on VR Y button: {exc}")
            self.emit(
                envelope(
                    "recording_error",
                    "middleware",
                    {
                        "error": str(exc),
                        "action": "stop_failed",
                    },
                ),
                ["websocket", "udp"],
            )

    def _on_safety_event(self, reason: str) -> None:
        if self.ros is not None and self.config["safety"].get("enabled", True):
            self.ros.emergency_stop(self.config["safety"]["stop_topic"], reason)
        self.emit(envelope("safety_stop", "middleware", {"reason": reason}), ["websocket", "udp"])

    def _on_safety_resume(self, reason: str) -> None:
        if self.ros is not None:
            stop_topic = self.config["safety"].get("stop_topic", "/teleop/emergency_stop")
            self.ros.publish(stop_topic, "std_msgs/msg/Bool", {"data": False})
            self.ros.publish("/teleop/arm/enabled", "std_msgs/msg/Bool", {"data": True})
        self.emit(envelope("safety_resume", "middleware", {"reason": reason}), ["websocket", "udp"])

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
            "replay": self.player.status() if self.player else {"state": "idle", "is_active": False},
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
        active_topics = dict(STANDARD_TOPICS)
        if active:
            try:
                active_topics = profiles.get_profile_topics(active)
            except Exception:
                pass
        for value in values:
            value["active"] = value["id"] == active
        return web.json_response(
            {
                "active": active,
                "root": str(profiles.root),
                "profiles": values,
                "standard_topics": active_topics,
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

    async def safety_resume(request: web.Request) -> web.Response:
        data = await request.json() if request.can_read_body else {}
        runtime._on_safety_resume(str(data.get("reason", "dashboard safety resume")))
        return web.json_response({"accepted": True}, status=202)

    async def teleop_home(request: web.Request) -> web.Response:
        if runtime.ros is not None:
            runtime.ros.publish("/teleop/arm/home", "std_msgs/msg/Bool", {"data": True})
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
            runtime._log("error", "WebRTC offer received but camera service is unavailable")
            print("[WebRTC] ERROR: Camera service is unavailable", flush=True)
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        peer_ip = request.remote or "unknown"
        try:
            body = await request.json()
            print(f"[WebRTC] Offer received from {peer_ip}", flush=True)
            runtime._log("info", f"WebRTC offer received from {peer_ip}")
            answer = await runtime.camera.offer(body)
            print(f"[WebRTC] Answer generated successfully for {peer_ip}", flush=True)
            runtime._log("info", f"WebRTC answer generated successfully for {peer_ip}")
            return web.json_response(answer)
        except Exception as exc:
            import traceback
            tb = traceback.format_exc()
            print(f"[WebRTC] Offer failed for {peer_ip}: {exc}\n{tb}", flush=True)
            runtime._log("error", f"WebRTC offer failed for {peer_ip}: {exc}\n{tb}")
            raise web.HTTPServiceUnavailable(text=f"WebRTC offer failed: {exc}") from exc

    async def get_camera_status(_request: web.Request) -> web.Response:
        status = runtime.camera.status() if runtime.camera else {"state": "disabled"}
        return web.json_response(status)

    async def get_camera_snapshot(_request: web.Request) -> web.Response:
        if runtime.camera is None:
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        frame = runtime.camera.latest()
        if frame is None:
            raise web.HTTPNotFound(text="no frame available")
        try:
            import cv2
            success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not success:
                raise RuntimeError("failed to encode frame")
            return web.Response(body=encoded.tobytes(), content_type="image/jpeg")
        except Exception as exc:
            raise web.HTTPInternalServerError(text=str(exc))

    async def get_camera_stream(_request: web.Request) -> web.StreamResponse:
        if runtime.camera is None:
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        response = web.StreamResponse(
            status=200,
            reason="OK",
            headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame"},
        )
        await response.prepare(_request)
        try:
            import cv2
            while True:
                frame = runtime.camera.latest()
                if frame is not None:
                    success, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if success:
                        data = encoded.tobytes()
                        await response.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
                        )
                await asyncio.sleep(0.033)
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        return response

    async def precheck_recording(_request: web.Request) -> web.Response:
        subs = store.value.get("ros", {}).get("subscriptions", [])
        recording_topics = [
            item for item in subs
            if item.get("enabled", True) and "record" in item.get("outputs", [])
        ]
        topic_health = runtime.ros.get_topic_health() if runtime.ros is not None else {}

        checked = []
        issues = []
        for item in recording_topics:
            topic = item["topic"]
            info = topic_health.get(topic)
            if info is None:
                from .ros_bridge import DEFAULT_TOPIC_STANDARDS
                std = DEFAULT_TOPIC_STANDARDS.get(topic, {"target_hz": 10.0, "min_hz": 1.0})
                target_hz = float(item.get("target_hz", 0) or std["target_hz"])
                min_hz = float(item.get("min_hz", 0) or std["min_hz"])
                item_stat = {
                    "topic": topic,
                    "type": item.get("type", "std_msgs/msg/String"),
                    "messages": 0,
                    "hz": 0.0,
                    "target_hz": target_hz,
                    "min_hz": min_hz,
                    "has_data": False,
                    "state": "no_data",
                    "message": "未检测到消息发布 (0 Hz)",
                }
            else:
                item_stat = dict(info)
            checked.append(item_stat)
            if item_stat["state"] == "no_data":
                issues.append({
                    "topic": topic,
                    "state": "no_data",
                    "hz": 0.0,
                    "min_hz": item_stat["min_hz"],
                    "reason": f"{topic}: 未检测到消息发布 (0 Hz，标准: ≥ {item_stat['min_hz']:.1f} Hz)",
                })
            elif item_stat["state"] == "low_rate":
                issues.append({
                    "topic": topic,
                    "state": "low_rate",
                    "hz": item_stat["hz"],
                    "min_hz": item_stat["min_hz"],
                    "reason": f"{topic}: 消息频率偏低 ({item_stat['hz']:.1f} Hz < 标准 {item_stat['min_hz']:.1f} Hz)",
                })

        ready = len(issues) == 0 and len(recording_topics) > 0
        return web.json_response({
            "ok": True,
            "ready": ready,
            "topic_count": len(recording_topics),
            "topics": checked,
            "issues": issues,
            "reason": "" if ready else ("未勾选任何录制话题" if not recording_topics else f"{len(issues)} 个待录制话题未达标"),
        })

    async def start_recording(request: web.Request) -> web.Response:
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder is not initialized")
        data = {}
        if request.can_read_body:
            try:
                data = await request.json()
            except Exception:
                pass
        force = bool(data.get("force", False))
        filename = str(data.get("filename", "")).strip()

        if not force:
            subs = store.value.get("ros", {}).get("subscriptions", [])
            recording_topics = [
                item for item in subs
                if item.get("enabled", True) and "record" in item.get("outputs", [])
            ]
            if not recording_topics:
                return web.json_response({
                    "ok": False,
                    "can_force": True,
                    "reason": "尚未勾选任何需要录制的话题，请先在下方勾选录制话题",
                    "issues": [{"topic": "none", "reason": "未勾选录制话题"}],
                }, status=400)

            topic_health = runtime.ros.get_topic_health() if runtime.ros is not None else {}
            issues = []
            for item in recording_topics:
                topic = item["topic"]
                info = topic_health.get(topic)
                if not info or info.get("state") != "ok":
                    hz = info.get("hz", 0.0) if info else 0.0
                    from .ros_bridge import DEFAULT_TOPIC_STANDARDS
                    std = DEFAULT_TOPIC_STANDARDS.get(topic, {"target_hz": 10.0, "min_hz": 1.0})
                    min_hz = float(item.get("min_hz", 0) or (info.get("min_hz") if info else std["min_hz"]))
                    issues.append({
                        "topic": topic,
                        "state": info.get("state", "no_data") if info else "no_data",
                        "hz": hz,
                        "min_hz": min_hz,
                        "reason": f"{topic}: 未检测到消息发布 (0 Hz)" if hz == 0 else f"{topic}: 频率不足 ({hz:.1f} Hz < 标准 {min_hz:.1f} Hz)",
                    })
            if issues:
                return web.json_response({
                    "ok": False,
                    "can_force": True,
                    "reason": "待录制话题未检测到消息或频率未达标",
                    "issues": issues,
                }, status=400)

        path = await asyncio.to_thread(runtime.recorder.start, filename)
        if runtime.ros is not None:
            runtime.ros.publish(
                "/teleop/recording_state",
                "std_msgs/msg/String",
                {"data": json.dumps({"recording": True, "filename": filename})},
            )
        runtime.emit(
            envelope(
                "recording_started",
                "middleware",
                {
                    "recording": True,
                    "filename": filename,
                    "path": str(path),
                    "action": "start",
                    "message": "Recording started",
                },
            ),
            ["websocket", "udp"],
        )
        return web.json_response({"ok": True, "recording": True, "path": path})

    async def stop_recording(_request: web.Request) -> web.Response:
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder is not initialized")
        rec_status = await asyncio.to_thread(runtime.recorder.stop)
        if runtime.ros is not None:
            runtime.ros.publish(
                "/teleop/recording_state",
                "std_msgs/msg/String",
                {"data": json.dumps({"recording": False, "status": rec_status})},
            )
        runtime.emit(
            envelope(
                "recording_stopped",
                "middleware",
                {
                    "recording": False,
                    "status": rec_status,
                    "action": "stop",
                    "message": "Recording stopped",
                },
            ),
            ["websocket", "udp"],
        )
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

    async def start_replay(request: web.Request) -> web.Response:
        if runtime.player is None:
            raise web.HTTPServiceUnavailable(text="player is not initialized")
        data = await request.json() if request.can_read_body else {}
        filename = str(data.get("filename", "")).strip()
        if not filename:
            raise web.HTTPBadRequest(text="filename is required")
        speed = float(data.get("speed", 1.0))
        loop = bool(data.get("loop", False))
        mode = str(data.get("mode", "drive"))
        selected_topics = data.get("topics")
        topic_remap = data.get("remap")
        try:
            status = await asyncio.to_thread(
                runtime.player.play,
                filename,
                speed=speed,
                loop=loop,
                mode=mode,
                selected_topics=selected_topics,
                topic_remap=topic_remap,
            )
            return web.json_response({"ok": True, "replay": status})
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        except Exception as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def pause_replay(_request: web.Request) -> web.Response:
        if runtime.player is None:
            raise web.HTTPServiceUnavailable(text="player is not initialized")
        status = await asyncio.to_thread(runtime.player.pause)
        return web.json_response({"ok": True, "replay": status})

    async def resume_replay(_request: web.Request) -> web.Response:
        if runtime.player is None:
            raise web.HTTPServiceUnavailable(text="player is not initialized")
        status = await asyncio.to_thread(runtime.player.resume)
        return web.json_response({"ok": True, "replay": status})

    async def stop_replay(_request: web.Request) -> web.Response:
        if runtime.player is None:
            raise web.HTTPServiceUnavailable(text="player is not initialized")
        status = await asyncio.to_thread(runtime.player.stop)
        return web.json_response({"ok": True, "replay": status})

    async def get_replay_status(_request: web.Request) -> web.Response:
        if runtime.player is None:
            return web.json_response({"state": "idle", "is_active": False})
        return web.json_response(runtime.player.status())

    async def import_mcap_file(request: web.Request) -> web.Response:
        if not request.can_read_body:
            raise web.HTTPBadRequest(text="multipart form data required")
        reader = await request.multipart()
        rec_dir = runtime.recorder.directory if runtime.recorder else store.path.parent / "runtime/topic_recordings"
        rec_dir.mkdir(parents=True, exist_ok=True)
        uploaded_name = ""
        while True:
            field = await reader.next()
            if field is None:
                break
            if field.name in {"file", "archive"}:
                filename = field.filename or "imported.mcap"
                if not filename.endswith(".mcap"):
                    filename += ".mcap"
                filename = Path(filename).name
                target_path = rec_dir / filename
                if target_path.exists():
                    stem = target_path.stem
                    target_path = rec_dir / f"{stem}_{int(time.time())}.mcap"
                with target_path.open("wb") as f:
                    while True:
                        chunk = await field.read_chunk()
                        if not chunk:
                            break
                        f.write(chunk)
                uploaded_name = target_path.name

        if not uploaded_name:
            raise web.HTTPBadRequest(text="no file uploaded")
        return web.json_response({"ok": True, "filename": uploaded_name}, status=201)

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
    app.router.add_post("/api/safety/resume", safety_resume)
    app.router.add_post("/api/teleop/home", teleop_home)
    app.router.add_post("/api/recording/precheck", precheck_recording)
    app.router.add_get("/api/recording/precheck", precheck_recording)
    app.router.add_post("/api/recording/start", start_recording)
    app.router.add_post("/api/recording/stop", stop_recording)
    app.router.add_get("/api/recording/status", get_recording_status)
    app.router.add_get("/api/recordings", get_recordings)
    app.router.add_post("/api/recordings/upload", import_mcap_file)
    app.router.add_get("/api/recordings/{filename}/download", download_recording)
    app.router.add_delete("/api/recordings/{filename}", delete_recording)
    app.router.add_post("/api/recordings/batch-delete", batch_delete_recordings)
    app.router.add_post("/api/replay/start", start_replay)
    app.router.add_post("/api/replay/pause", pause_replay)
    app.router.add_post("/api/replay/resume", resume_replay)
    app.router.add_post("/api/replay/stop", stop_replay)
    app.router.add_get("/api/replay/status", get_replay_status)
    app.router.add_get("/ws", websocket)
    app.router.add_post("/api/webrtc/offer", webrtc_offer)
    app.router.add_post("/offer", webrtc_offer)
    app.router.add_get("/api/camera/status", get_camera_status)
    app.router.add_get("/api/camera/snapshot", get_camera_snapshot)
    app.router.add_get("/api/camera/stream", get_camera_stream)
    app.router.add_options("/{tail:.*}", options)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(shutdown)
    return app
