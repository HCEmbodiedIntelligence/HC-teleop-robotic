"""相机流构建及 WebRTC / 快照 / MJPEG HTTP 路由。"""
from __future__ import annotations

import asyncio
import json
import traceback
from typing import Any

from aiohttp import web

from .camera_process import CameraProcessService as CameraService


def build_camera_services(
    camera_config: dict[str, Any], domain_id: int
) -> tuple[dict[str, CameraService], dict[str, list[CameraService]], CameraService | None]:
    """根据相机配置构建按 ID 与按话题索引的 CameraService 集合。"""
    base_config = {
        key: value for key, value in camera_config.items() if key != "streams"
    }
    configured_streams = camera_config.get("streams", [])
    if not configured_streams:
        configured_streams = [
            {
                "id": "main",
                "name": "主相机",
                "topic": base_config.get("custom_topic", "")
                or base_config.get(
                    "topic", "/observation/cameras/head/color/compressed"
                ),
                "enabled": base_config.get("enabled", False),
            }
        ]

    cameras: dict[str, CameraService] = {}
    cameras_by_topic: dict[str, list[CameraService]] = {}
    for stream in configured_streams:
        if not base_config.get("enabled", False) or not stream.get("enabled", True):
            continue
        stream_config = {**base_config, **stream, "enabled": True}
        stream_id = str(stream_config.get("id", "main"))
        service = CameraService(stream_config, domain_id=domain_id)
        cameras[stream_id] = service
        active_topic = str(
            stream_config.get("custom_topic", "")
            or stream_config.get("topic", "/observation/cameras/head/color/compressed")
        )
        cameras_by_topic.setdefault(active_topic, []).append(service)

    primary = next(iter(cameras.values()), None)
    return cameras, cameras_by_topic, primary


def register_camera_routes(app: web.Application, runtime: Any) -> None:
    """注册 WebRTC 信令、接收端质量报告、相机状态、快照与 MJPEG 视频流路由。"""

    def camera_for_request(request: web.Request) -> tuple[str, CameraService | None]:
        camera_id = str(request.match_info.get("camera_id", "")).strip()
        if camera_id:
            return camera_id, runtime.cameras.get(camera_id)
        default_id = next(iter(runtime.cameras), "")
        return default_id, runtime.cameras.get(default_id)

    async def webrtc_offer(request: web.Request) -> web.Response:
        camera_id, camera = camera_for_request(request)
        if camera is None:
            runtime._log("error", "WebRTC offer received but camera service is unavailable")
            print("[WebRTC] ERROR: Camera service is unavailable", flush=True)
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        peer_ip = request.remote or "unknown"
        try:
            body = await request.json()
            print(f"[WebRTC:{camera_id}] Offer received from {peer_ip}", flush=True)
            runtime._log("info", f"WebRTC {camera_id} offer received from {peer_ip}")
            answer = await camera.offer(body)
            print(f"[WebRTC:{camera_id}] Answer generated for {peer_ip}", flush=True)
            runtime._log("info", f"WebRTC {camera_id} answer generated for {peer_ip}")
            return web.json_response(answer)
        except Exception as exc:
            tb = traceback.format_exc()
            print(f"[WebRTC] Offer failed for {peer_ip}: {exc}\n{tb}", flush=True)
            runtime._log("error", f"WebRTC offer failed for {peer_ip}: {exc}\n{tb}")
            raise web.HTTPServiceUnavailable(text=f"WebRTC offer failed: {exc}") from exc

    async def receiver_report(request: web.Request) -> web.Response:
        _camera_id, camera = camera_for_request(request)
        if camera is None:
            raise web.HTTPNotFound(text="camera stream not found")
        if request.content_length is not None and request.content_length > 16384:
            raise web.HTTPRequestEntityTooLarge(
                max_size=16384, actual_size=request.content_length
            )
        raw = bytearray()
        async for chunk in request.content.iter_chunked(4096):
            raw.extend(chunk)
            if len(raw) > 16384:
                raise web.HTTPRequestEntityTooLarge(
                    max_size=16384, actual_size=len(raw)
                )
        try:
            payload = json.loads(raw)
            accepted = await camera.receiver_report(payload)
        except (ValueError, TypeError, RuntimeError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        return web.json_response({"accepted": accepted})

    async def get_camera_status(request: web.Request) -> web.Response:
        camera_id = str(request.match_info.get("camera_id", "")).strip()
        if camera_id:
            camera = runtime.cameras.get(camera_id)
            if camera is None:
                raise web.HTTPNotFound(text=f"camera stream not found: {camera_id}")
            return web.json_response(camera.status())
        streams = [camera.status() for camera in runtime.cameras.values()]
        status = dict(streams[0]) if streams else {"state": "disabled"}
        status["streams"] = streams
        return web.json_response(status)

    async def get_camera_snapshot(request: web.Request) -> web.Response:
        _camera_id, camera = camera_for_request(request)
        if camera is None:
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        frame = await asyncio.to_thread(camera.latest)
        if frame is None:
            raise web.HTTPNotFound(text="no frame available")
        try:
            import cv2

            success, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            if not success:
                raise RuntimeError("failed to encode frame")
            return web.Response(body=encoded.tobytes(), content_type="image/jpeg")
        except Exception as exc:
            raise web.HTTPInternalServerError(text=str(exc))

    async def get_camera_stream(request: web.Request) -> web.StreamResponse:
        _camera_id, camera = camera_for_request(request)
        if camera is None:
            raise web.HTTPServiceUnavailable(text="camera service is unavailable")
        response = web.StreamResponse(
            status=200,
            reason="OK",
            headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame"},
        )
        await response.prepare(request)
        try:
            import cv2

            while True:
                frame = await asyncio.to_thread(camera.latest)
                if frame is not None:
                    success, encoded = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                    )
                    if success:
                        await response.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                            + encoded.tobytes()
                            + b"\r\n"
                        )
                await asyncio.sleep(0.033)
        except (asyncio.CancelledError, ConnectionResetError):
            pass
        return response

    app.router.add_post("/api/webrtc/offer", webrtc_offer)
    app.router.add_post("/api/webrtc/offer/{camera_id}", webrtc_offer)
    app.router.add_post("/offer", webrtc_offer)
    app.router.add_post("/offer/{camera_id}", webrtc_offer)
    app.router.add_post("/api/webrtc/receiver/{camera_id}", receiver_report)
    app.router.add_get("/api/camera/status", get_camera_status)
    app.router.add_get("/api/camera/status/{camera_id}", get_camera_status)
    app.router.add_get("/api/camera/snapshot", get_camera_snapshot)
    app.router.add_get("/api/camera/snapshot/{camera_id}", get_camera_snapshot)
    app.router.add_get("/api/camera/stream", get_camera_stream)
    app.router.add_get("/api/camera/stream/{camera_id}", get_camera_stream)

