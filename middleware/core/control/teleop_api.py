"""ROS 发布、安全急停/恢复、遥操回零与控制源切换 HTTP 路由。"""
from __future__ import annotations

import asyncio
import copy
from typing import Any

from aiohttp import web

from ..config import ConfigStore
from ..protocol import envelope


def register_teleop_routes(
    app: web.Application, store: ConfigStore, runtime: Any
) -> None:
    """注册基础 ROS 话题发布、安全控制与遥操控制源接口。"""

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
        if (
            runtime.control_runtime.enabled
            and runtime.control_runtime.status()["state"] != "running"
        ):
            return web.json_response(
                {
                    "accepted": False,
                    "error": "控制进程尚未就绪，不能恢复输出",
                    "control_runtime": runtime.control_runtime.status(),
                },
                status=409,
            )
        isolated = getattr(runtime.ros, "isolated_control_process", False) is True
        fence = runtime.ros.authorization_fence() if isolated else None
        data = await request.json() if request.can_read_body else {}
        reason = str(data.get("reason", "dashboard safety resume"))
        accepted = (
            await asyncio.to_thread(
                runtime._on_safety_resume, reason, expected_fence=fence
            )
            if isolated
            else runtime._on_safety_resume(reason)
        )
        return web.json_response({"accepted": accepted}, status=202 if accepted else 409)

    async def teleop_home(_request: web.Request) -> web.Response:
        if runtime.ros and runtime.ros.ownership.active:
            if not runtime.ros.request_manual_home():
                raise web.HTTPConflict(
                    text="请先握住右 Grip 完成人工接管，确认硬件就绪后再回零"
                )
            return web.json_response({"accepted": True}, status=202)
        source = (
            runtime.ros.status().get("command_mux", {}).get("source", "vr")
            if runtime.ros
            else store.value.get("ros", {}).get("command_mux", {}).get("source", "vr")
        )
        if source != "vr":
            raise web.HTTPConflict(text="回零仅在 VR/IK 控制源下可用")
        if runtime.ros is not None:
            runtime.ros.publish("/teleop/arm/home", "std_msgs/msg/Bool", {"data": True})
        return web.json_response({"accepted": True}, status=202)

    async def set_control_source(request: web.Request) -> web.Response:
        data = await request.json()
        source = str(data.get("source", "")).strip().lower()
        if source not in {"vr", "exoskeleton", "motion"}:
            raise web.HTTPBadRequest(text="source must be vr, exoskeleton or motion")
        if runtime.ros is None or runtime.ros.status().get("state") != "running":
            raise web.HTTPServiceUnavailable(text="ROS bridge is not running")
        if runtime.ros.ownership.active:
            raise web.HTTPConflict(text="人工干预会话正在管理控制来源")

        proposed = copy.deepcopy(store.value)
        proposed["ros"]["command_mux"]["source"] = source
        store.save(proposed)
        runtime.config["ros"]["command_mux"]["source"] = source
        accepted = runtime.ros.set_control_source(source)
        runtime.ros.publish(
            "/control/teleop/enabled",
            "std_msgs/msg/Bool",
            {"data": source == "vr"},
        )
        runtime.emit(
            envelope("control_source", "middleware", {"source": source}),
            ["websocket"],
        )
        return web.json_response(
            {"accepted": accepted, "source": source},
            status=202 if accepted else 429,
        )

    app.router.add_get("/api/ros/topics", get_topics)
    app.router.add_post("/api/ros/publish", publish)
    app.router.add_post("/api/safety/stop", emergency_stop)
    app.router.add_post("/api/safety/resume", safety_resume)
    app.router.add_post("/api/teleop/home", teleop_home)
    app.router.add_post("/api/teleop/source", set_control_source)

