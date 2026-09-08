from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation

from .ros_bridge import DashboardRosBridge


def _static_directory() -> Path:
    return Path(get_package_share_directory("hc_dashboard")) / "static"


def create_app(node: DashboardRosBridge) -> Any:
    try:
        from aiohttp import WSMsgType, web
    except ImportError as error:
        raise RuntimeError(
            "aiohttp is missing; run ./bootstrap_colcon.sh deps to install python3-aiohttp"
        ) from error

    static_dir = _static_directory()
    if not (static_dir / "index.html").is_file():
        raise RuntimeError(f"dashboard static assets are missing: {static_dir}")

    async def index(request: Any) -> Any:
        return web.FileResponse(static_dir / "index.html")

    async def status(request: Any) -> Any:
        return web.json_response(node.model.snapshot())

    async def runtime(request: Any) -> Any:
        from .inspection import profiles
        return web.json_response({
            "profiles": await asyncio.to_thread(profiles, node.profile),
            "active": node.robot_id,
            "server": {"host": node.host, "port": node.port},
            "ros": {"domain_id": int(os.environ.get("ROS_DOMAIN_ID", "0")),
                    "node_name": node.get_name(), "namespace": node.get_namespace()},
            "topics": [{"name": name, "types": types}
                       for name, types in node.get_topic_names_and_types()],
            "ws_clients": len(sockets),
        })

    async def dataset_list(request: Any) -> Any:
        from .inspection import datasets
        return web.json_response(await asyncio.to_thread(datasets, node.dataset_root))

    sockets: set[Any] = set()

    async def health(request: Any) -> Any:
        snapshot = node.model.snapshot()
        return web.json_response(
            {
                "ok": True,
                "robot_id": snapshot["robot_id"],
                "vr_stale": snapshot["streams"]["vr"]["stale"],
                "joints_stale": snapshot["streams"]["joints"]["stale"],
            }
        )

    async def set_enabled(request: Any) -> Any:
        try:
            body = await request.json()
            if not isinstance(body, dict) or not isinstance(body.get("enabled"), bool):
                raise ValueError("enabled must be a boolean")
            result = await asyncio.to_thread(node.set_enabled, body["enabled"])
            return web.json_response(result, status=200 if result["success"] else 409)
        except (ValueError, json.JSONDecodeError) as error:
            return web.json_response({"success": False, "reason": str(error)}, status=400)
        except (RuntimeError, TimeoutError) as error:
            return web.json_response({"success": False, "reason": str(error)}, status=503)

    async def reset_fault(request: Any) -> Any:
        try:
            result = await asyncio.to_thread(node.reset_fault)
            return web.json_response(result, status=200 if result["success"] else 409)
        except (RuntimeError, TimeoutError) as error:
            return web.json_response({"success": False, "reason": str(error)}, status=503)

    async def websocket(request: Any) -> Any:
        socket = web.WebSocketResponse(heartbeat=10.0, max_msg_size=4096)
        await socket.prepare(request)
        sockets.add(socket)
        try:
            await stream_socket(socket)
        finally:
            sockets.discard(socket)
        return socket

    async def stream_socket(socket: Any) -> None:
        while not socket.closed:
            snapshot = node.model.snapshot()
            snapshot["ws_clients"] = len(sockets)
            await socket.send_json(snapshot)
            try:
                message = await socket.receive(timeout=0.25)
            except asyncio.TimeoutError:
                continue
            if message.type in {WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.ERROR}:
                break

    app = web.Application(client_max_size=64 * 1024)
    app.router.add_get("/", index)
    app.router.add_get("/dashboard/", index)
    app.router.add_get("/api/v1/status", status)
    app.router.add_get("/api/v1/health", health)
    app.router.add_get("/api/v1/runtime", runtime)
    app.router.add_get("/api/v1/datasets", dataset_list)
    app.router.add_post("/api/v1/safety/enabled", set_enabled)
    app.router.add_post("/api/v1/safety/reset", reset_fault)
    app.router.add_get("/ws", websocket)
    # colcon --symlink-install intentionally exposes package assets through
    # symlinks. aiohttp rejects those files unless following links is enabled;
    # the root itself is still the installed, trusted package static directory.
    app.router.add_static(
        "/static/", static_dir, show_index=False, follow_symlinks=True
    )
    return app


async def _serve(node: DashboardRosBridge) -> None:
    from aiohttp import web

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    import threading

    def spin_ros() -> None:
        try:
            executor.spin()
        except (ExternalShutdownException, rclpy_implementation.RCLError):
            pass

    ros_thread = threading.Thread(target=spin_ros, name="hc-dashboard-ros", daemon=True)
    ros_thread.start()
    runner = web.AppRunner(create_app(node), access_log=None)
    try:
        await runner.setup()
        site = web.TCPSite(runner, host=node.host, port=node.port)
        await site.start()
        node.get_logger().info(
            f"HC dashboard ready: http://{node.host}:{node.port}/dashboard/ "
            f"robot={node.robot_id}"
        )
        while rclpy.ok():
            await asyncio.sleep(0.25)
    finally:
        await runner.cleanup()
        executor.shutdown(timeout_sec=2.0)
        ros_thread.join(timeout=2.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: DashboardRosBridge | None = None
    try:
        node = DashboardRosBridge()
        asyncio.run(_serve(node))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f"hc_dashboard: {type(error).__name__}: {error}", file=sys.stderr)
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
