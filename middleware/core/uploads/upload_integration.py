"""可选上传模块的运行时初始化、错误转换与 HTTP 路由集成。"""
from __future__ import annotations

import asyncio
import copy
import json
from pathlib import Path
from typing import Any

from aiohttp import web

from ..config import ConfigStore


def upload_failure(exc: Exception) -> dict[str, Any] | None:
    """识别可选上传组件抛出的 UploadError，而不把上传模块变为强依赖。"""
    try:
        from humanoid_upload.common import UploadError
    except ImportError:
        return None
    if not isinstance(exc, UploadError):
        return None
    return {"ok": False, "code": exc.code, "reason": exc.action}


def upload_failure_status(code: str) -> int:
    return (
        409
        if code in {"ARTIFACT_BUSY", "WORKER_ALREADY_RUNNING", "BINDING_REQUIRED"}
        else 400
    )


async def start_runtime_uploads(runtime: Any) -> None:
    """启动可选上传服务；依赖不可用时不得影响本地录制。"""
    runtime.uploads = None
    runtime.upload_error = None
    try:
        from .upload_service import UploadService

        upload_config = copy.deepcopy(runtime.config.get("uploads", {"enabled": False}))
        queue_path = Path(
            upload_config.get("queue_directory", "../runtime/upload_queue")
        ).expanduser()
        upload_config["queue_directory"] = str(
            (runtime.config_dir / queue_path).resolve()
            if not queue_path.is_absolute()
            else queue_path
        )
        service = UploadService(
            upload_config,
            artifact_state_dir=runtime.recorder.artifact_state_dir,
            data_directory=runtime.recorder.directory,
        )
        try:
            await asyncio.to_thread(service.start)
        except Exception:
            await asyncio.to_thread(service.stop)
            raise
        runtime.uploads = service
        runtime.recorder.upload_service = service
        if upload_config.get("enabled", False) and upload_config.get("auto_enqueue", False):
            runtime.recorder.binding_snapshot = service.binding_snapshot
    except Exception as exc:
        runtime.upload_error = str(exc)
        runtime._log("error", "upload unavailable: " + str(exc))


class UploadProxy:
    """将上传路由动态转发至当前 runtime.uploads 实例。"""

    def __init__(self, runtime: Any) -> None:
        self._runtime = runtime

    def _unavailable(self) -> web.HTTPServiceUnavailable:
        return web.HTTPServiceUnavailable(
            text=json.dumps(
                {
                    "enabled": False,
                    "state": "unavailable",
                    "error": self._runtime.upload_error or "upload service unavailable",
                }
            ),
            content_type="application/json",
        )

    def status(self) -> dict[str, Any]:
        platform = self._runtime.platform
        if platform is not None and not platform.state.get("uploads_enabled", False):
            return {
                "enabled": False,
                "running": False,
                "auto_enqueue": False,
                "jobs": [],
                "scan_errors": {},
                "platform_backoffs": {},
                "processes": {
                    "artifacts": {"running": False},
                    "upload": {"running": False},
                },
            }
        if self._runtime.uploads is None:
            raise self._unavailable()
        return self._runtime.uploads.status()

    def __getattr__(self, key: str) -> Any:
        if self._runtime.uploads is None:
            raise self._unavailable()
        return getattr(self._runtime.uploads, key)


def register_runtime_upload_routes(
    app: web.Application, store: ConfigStore, runtime: Any
) -> None:
    """注册上传接口；未安装上传模块时降级为 503 提示路由。"""
    try:
        from .upload_api import register_upload_routes
    except ImportError:
        async def upload_unavailable(_request: web.Request) -> web.Response:
            return web.json_response(
                {
                    "enabled": False,
                    "state": "unavailable",
                    "error": "upload module is not installed",
                },
                status=503,
            )

        app.router.add_get("/api/uploads", upload_unavailable)
        app.router.add_post("/api/uploads/{operation}", upload_unavailable)
        return

    proxy = UploadProxy(runtime)

    async def update_upload_settings(body: dict[str, Any]) -> dict[str, Any]:
        from humanoid_upload.common import UploadError
        from .upload_service import UploadService

        if set(body) != {"enabled", "auto_enqueue"} or any(
            type(v) is not bool for v in body.values()
        ):
            raise ValueError()
        async with runtime._restart_lock:
            if runtime.recorder is None:
                raise UploadError("UPLOAD_UNAVAILABLE", "录制服务尚未就绪")
            if runtime.recorder.status().get("recording"):
                raise UploadError("ARTIFACT_BUSY", "请停止当前录制后修改上传设置")
            proposed = copy.deepcopy(store.value)
            proposed["uploads"].update(body)
            if runtime.platform is not None:
                saved = store.save(proposed)
                runtime.config["uploads"] = copy.deepcopy(saved["uploads"])
                platform = runtime.platform
                await asyncio.to_thread(platform.set_enabled, body["enabled"])
                if body["enabled"]:
                    await asyncio.to_thread(platform.set_auto, body["auto_enqueue"])
                return {**proxy.status(), **body}

            upload_config = copy.deepcopy(proposed["uploads"])
            queue_path = Path(upload_config["queue_directory"]).expanduser()
            upload_config["queue_directory"] = str(
                (store.path.parent / queue_path).resolve()
            )
            candidate = UploadService(
                upload_config,
                artifact_state_dir=runtime.recorder.artifact_state_dir,
                data_directory=runtime.recorder.directory,
            )
            old = runtime.uploads
            if old is not None:
                stopped = await asyncio.to_thread(old.stop)
                if not stopped.get("stopped"):
                    raise UploadError("ARTIFACT_BUSY", "上传请求尚未结束，请稍后重试")
            try:
                saved = store.save(proposed)
            except Exception:
                if old is not None:
                    old.start()
                raise
            if old is not None:
                await asyncio.to_thread(old.close)
            runtime.config["uploads"] = copy.deepcopy(saved["uploads"])
            runtime.uploads = candidate
            runtime.upload_error = None
            runtime.recorder.upload_service = candidate
            runtime.recorder.binding_snapshot = (
                candidate.binding_snapshot
                if body["enabled"] and body["auto_enqueue"]
                else None
            )
            candidate.start()
            return candidate.status()

    register_upload_routes(app, proxy, update_settings=update_upload_settings)

