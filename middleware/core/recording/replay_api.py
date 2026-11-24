"""轨迹回放控制 HTTP 路由与状态文案格式化。"""
from __future__ import annotations

import asyncio
import os
from typing import Any

from aiohttp import web

_REPLAY_MESSAGES = {
    "replay_started": "正在重放；重放结束后请按 A 恢复遥操作",
    "replay_paused": "重放已暂停",
    "replay_resumed": "重放已继续",
    "replay_stopped": "重放已停止；请按 A 恢复遥操作",
    "replay_completed": "重放完成；请按 A 恢复遥操作",
    "replay_error": "重放异常；请按 A 恢复遥操作",
    "replay_reset": "已恢复实时遥操作",
    "replay_home_requested": "回初始位姿指令已提交",
}


def format_replay_state_payload(
    event_type: str, status: dict[str, Any]
) -> dict[str, Any]:
    return {
        **status,
        "state": str(status.get("state", "idle")),
        "message": _REPLAY_MESSAGES.get(event_type, event_type),
    }


def _require_player(runtime: Any) -> Any:
    if runtime.player is None:
        raise web.HTTPServiceUnavailable(text="player is not initialized")
    return runtime.player


def register_replay_routes(app: web.Application, runtime: Any) -> None:
    """注册轨迹回放的启动、暂停、继续、停止、回零与状态查询路由。"""

    async def start_replay(request: web.Request) -> web.Response:
        player = _require_player(runtime)
        data = await request.json() if request.can_read_body else {}
        filename = str(data.get("filename", "")).strip()
        if not filename:
            raise web.HTTPBadRequest(text="filename is required")
        try:
            status = await asyncio.to_thread(
                player.play,
                filename,
                speed=float(data.get("speed", 1.0)),
                loop=bool(data.get("loop", False)),
                mode=str(data.get("mode", "drive")),
                selected_topics=data.get("topics"),
                topic_remap=data.get("remap"),
            )
            return web.json_response({"ok": True, "replay": status})
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        except Exception as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def pause_replay(_request: web.Request) -> web.Response:
        status = await asyncio.to_thread(_require_player(runtime).pause)
        return web.json_response({"ok": True, "replay": status})

    async def resume_replay(_request: web.Request) -> web.Response:
        status = await asyncio.to_thread(_require_player(runtime).resume)
        return web.json_response({"ok": True, "replay": status})

    async def stop_replay(_request: web.Request) -> web.Response:
        status = await asyncio.to_thread(_require_player(runtime).stop)
        return web.json_response({"ok": True, "replay": status})

    async def replay_home(_request: web.Request) -> web.Response:
        ros = runtime.ros
        if runtime.player is None or ros is None or ros.status().get("state") != "running":
            raise web.HTTPServiceUnavailable(text="回放或 ROS 控制进程尚未就绪")
        if os.environ.get("HC_CONTROL_BACKEND") == "motion":
            raise web.HTTPConflict(text="当前 Motion 后端请使用现有的回零流程")
        control_status = runtime.control_runtime.status()
        if runtime._safety_reason:
            raise web.HTTPConflict(text="安全停机仍有效，请先人工恢复控制")
        if (
            runtime._stopping
            or runtime.control_runtime.enabled
            and control_status["state"] != "running"
        ):
            raise web.HTTPConflict(text="控制进程尚未就绪")
        isolated = getattr(ros, "isolated_control_process", False) is True
        fence = ros.authorization_fence() if isolated else None

        def request_home(output_resumed: bool) -> Any:
            if runtime._stopping or runtime._safety_reason:
                raise ValueError("安全停机仍有效")
            options = {"expected_fence": fence} if isolated else {}
            return ros.request_replay_home(output_resumed=output_resumed, **options)

        try:
            status = await asyncio.to_thread(runtime.player.return_home, request_home)
        except (ValueError, RuntimeError) as exc:
            raise web.HTTPConflict(text=str(exc)) from exc
        return web.json_response({"accepted": True, "replay": status}, status=202)

    async def get_replay_status(_request: web.Request) -> web.Response:
        if runtime.player is None:
            return web.json_response({"state": "idle", "is_active": False})
        return web.json_response(runtime.player.status())

    app.router.add_post("/api/replay/start", start_replay)
    app.router.add_post("/api/replay/pause", pause_replay)
    app.router.add_post("/api/replay/resume", resume_replay)
    app.router.add_post("/api/replay/stop", stop_replay)
    app.router.add_post("/api/replay/home", replay_home)
    app.router.add_get("/api/replay/status", get_replay_status)

