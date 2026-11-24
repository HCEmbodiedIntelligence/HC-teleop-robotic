"""录制控制与数据集管理 HTTP 路由。"""
from __future__ import annotations

import asyncio
from datetime import datetime
import json
from pathlib import Path
import time
from typing import Any
from urllib.parse import quote

from aiohttp import web

from ..config import ConfigStore
from ..uploads.upload_integration import upload_failure, upload_failure_status
from .recording_controller import RecordingController, check_recording_topics
from .recording_downloads import register_download_routes


def _require_recorder(runtime: Any, text: str = "recorder not available") -> Any:
    if runtime.recorder is None:
        raise web.HTTPServiceUnavailable(text=text)
    return runtime.recorder


def _format_size(total_bytes: int) -> str:
    if total_bytes < 1024:
        return f"{total_bytes} B"
    if total_bytes < 1024 * 1024:
        return f"{total_bytes / 1024:.1f} KB"
    if total_bytes < 1024 * 1024 * 1024:
        return f"{total_bytes / (1024 * 1024):.2f} MB"
    return f"{total_bytes / (1024 * 1024 * 1024):.2f} GB"


async def _stream_zip_archive(
    request: web.Request, archive: Path, content_disposition: str
) -> web.StreamResponse:
    """流式回传临时 ZIP 归档并在结束后清理临时文件。"""
    try:
        response = web.StreamResponse(
            headers={
                "Content-Disposition": content_disposition,
                "Content-Type": "application/zip",
                "Content-Length": str(archive.stat().st_size),
            }
        )
        await response.prepare(request)
        with archive.open("rb") as handle:
            while chunk := await asyncio.to_thread(handle.read, 256 * 1024):
                await response.write(chunk)
        await response.write_eof()
        return response
    finally:
        archive.unlink(missing_ok=True)


def _recording_controller(runtime: Any) -> RecordingController:
    controller = getattr(runtime, "recording_controller", None)
    return controller if controller is not None else RecordingController(runtime)


def register_recording_routes(
    app: web.Application, store: ConfigStore, runtime: Any
) -> None:
    """注册录制预检、启停、数据集列举、导入、下载与批量操作路由。"""

    async def precheck_recording(_request: web.Request) -> web.Response:
        if runtime.recording_ros is not None:
            try:
                await asyncio.to_thread(runtime.recording_ros.prepare_recording)
                # 频率统计需要多个样本，在后台预热的同时避免阻塞媒体线程。
                await asyncio.sleep(1.9)
            except (RuntimeError, TimeoutError) as exc:
                return web.json_response(
                    {"ok": False, "ready": False, "reason": str(exc), "issues": []},
                    status=503,
                )
        subs = store.value.get("ros", {}).get("subscriptions", [])
        recording_topics, checked, issues = check_recording_topics(
            subs, runtime.topic_health(), detailed=True
        )
        ready = len(issues) == 0 and len(recording_topics) > 0
        return web.json_response(
            {
                "ok": True,
                "ready": ready,
                "topic_count": len(recording_topics),
                "topics": checked,
                "issues": issues,
                "reason": (
                    ""
                    if ready
                    else (
                        "未勾选任何录制话题"
                        if not recording_topics
                        else f"{len(issues)} 个待录制话题未达标"
                    )
                ),
            }
        )

    async def start_recording(request: web.Request) -> web.Response:
        recorder = _require_recorder(runtime, "recorder is not initialized")
        data: dict[str, Any] = {}
        if request.can_read_body:
            try:
                data = await request.json()
            except Exception:
                pass
        if runtime.recording_ros is not None:
            try:
                await asyncio.to_thread(runtime.recording_ros.prepare_recording)
            except (RuntimeError, TimeoutError) as exc:
                return web.json_response({"ok": False, "reason": str(exc)}, status=503)
        if runtime.platform is None and runtime.upload_error:
            if (data.get("selection") or {}).get("mode", "local") != "local":
                return web.json_response(
                    {
                        "ok": False,
                        "code": "PLATFORM_UNAVAILABLE",
                        "reason": "平台模块尚未安装，可暂不关联平台任务录制到本机",
                    },
                    status=409,
                )
        force = bool(data.get("force", False))
        filename = str(data.get("filename", "")).strip()

        if not force:
            subs = store.value.get("ros", {}).get("subscriptions", [])
            recording_topics, _checked, issues = check_recording_topics(
                subs, runtime.topic_health(), detailed=False
            )
            if not recording_topics:
                return web.json_response(
                    {
                        "ok": False,
                        "can_force": True,
                        "reason": "尚未勾选任何需要录制的话题，请先在下方勾选录制话题",
                        "issues": [{"topic": "none", "reason": "未勾选录制话题"}],
                    },
                    status=400,
                )
            if issues:
                return web.json_response(
                    {
                        "ok": False,
                        "can_force": True,
                        "reason": "待录制话题未检测到消息或频率未达标",
                        "issues": issues,
                    },
                    status=400,
                )

        try:
            path = await asyncio.to_thread(
                recorder.start,
                filename,
                runtime.session_profile_metadata,
                *([data.get("selection")] if runtime.platform is not None else []),
            )
        except (ValueError, RuntimeError, FileExistsError, ImportError) as exc:
            return web.json_response({"ok": False, "reason": str(exc)}, status=400)
        except Exception as exc:
            failure = upload_failure(exc)
            if failure is None:
                raise
            return web.json_response(
                failure, status=upload_failure_status(failure["code"])
            )
        status = recorder.status()
        if not status.get("recording") or status.get("path") != path:
            return web.json_response(
                {"ok": True, "recording": False, "path": path, "replayed": True}
            )
        _recording_controller(runtime).notify_started(filename, path)
        return web.json_response({"ok": True, "recording": True, "path": path})

    async def stop_recording(_request: web.Request) -> web.Response:
        recorder = _require_recorder(runtime, "recorder is not initialized")
        rec_status = await asyncio.to_thread(recorder.stop)
        _recording_controller(runtime).notify_stopped(rec_status)
        return web.json_response(
            {"ok": True, "recording": False, "status": rec_status}
        )

    async def get_recording_status(_request: web.Request) -> web.Response:
        if runtime.recorder is None:
            return web.json_response({"recording": False, "enabled": False})
        return web.json_response(runtime.recorder.status())

    async def get_recordings(request: web.Request) -> web.Response:
        if runtime.recorder is None:
            return web.json_response(
                {
                    "files": [],
                    "directory": "",
                    "total_count": 0,
                    "total_size_bytes": 0,
                    "total_size_human": "0 B",
                }
            )
        format_filter = request.query.get("format", "all")
        identifier = request.query.get("identifier", "")
        compact = request.query.get("compact") == "1"
        try:
            files, format_counts = await asyncio.gather(
                asyncio.to_thread(
                    runtime.recorder.list_recordings, format_filter, identifier, compact
                ),
                asyncio.to_thread(runtime.recorder.recording_format_counts),
            )
        except ValueError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        if compact:
            files = [
                {
                    key: value
                    for key, value in item.items()
                    if key not in {"channels", "videos"}
                }
                for item in files
            ]
        total_bytes = sum(f["size_bytes"] for f in files)
        return web.json_response(
            {
                "files": files,
                "directory": str(runtime.recorder.directory),
                "total_count": len(files),
                "total_size_bytes": total_bytes,
                "total_size_human": _format_size(total_bytes),
                "format": format_filter,
                "format_counts": format_counts,
            }
        )

    async def import_mcap_file(request: web.Request) -> web.Response:
        if not request.can_read_body:
            raise web.HTTPBadRequest(text="multipart form data required")
        reader = await request.multipart()
        rec_dir = (
            runtime.recorder.directory
            if runtime.recorder
            else store.path.parent / "runtime/topic_recordings"
        )
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
                    target_path = rec_dir / f"{target_path.stem}_{int(time.time())}.mcap"
                with target_path.open("wb") as handle:
                    while chunk := await field.read_chunk():
                        handle.write(chunk)
                uploaded_name = target_path.name

        if not uploaded_name:
            raise web.HTTPBadRequest(text="no file uploaded")
        return web.json_response({"ok": True, "filename": uploaded_name}, status=201)

    async def download_recording(request: web.Request) -> web.StreamResponse:
        from .recording_datasets import archive_dataset, archive_datasets, resolve_dataset
        from .takeover_log import takeover_file

        recorder = _require_recorder(runtime)
        identifier = request.match_info["filename"]
        try:
            target = resolve_dataset(recorder.directory, identifier)
        except (ValueError, FileNotFoundError) as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        if target.is_file():
            if recorder.is_recording() and recorder.path == target:
                raise web.HTTPConflict(text="recording is still being written")
            if request.query.get("with_tags") != "1" or not takeover_file(target):
                return web.FileResponse(
                    target,
                    headers={
                        "Content-Disposition": f"attachment; filename*=UTF-8''{quote(target.name)}",
                        "Content-Type": "application/octet-stream",
                    },
                )
        try:
            if target.is_file():
                archive = await asyncio.to_thread(
                    archive_datasets, target.parent, [identifier]
                )
            else:
                archive = await asyncio.to_thread(archive_dataset, target)
        except ValueError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        return await _stream_zip_archive(
            request,
            archive,
            f"attachment; filename*=UTF-8''{quote(target.name + '.zip')}",
        )

    async def get_dataset_video(request: web.Request) -> web.FileResponse:
        from .recording_datasets import dataset_video, resolve_dataset

        recorder = _require_recorder(runtime)
        identifier = request.match_info["filename"]
        if not identifier.startswith(("lerobot:", "bundle:")):
            raise web.HTTPNotFound(text="not a LeRobot dataset")
        try:
            target = resolve_dataset(recorder.directory, identifier)
            video = dataset_video(target, request.query.get("path", ""))
        except (ValueError, FileNotFoundError) as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        return web.FileResponse(video, headers={"Content-Type": "video/mp4"})

    async def delete_recording(request: web.Request) -> web.Response:
        recorder = _require_recorder(runtime)
        filename = request.match_info["filename"]
        try:
            await asyncio.to_thread(recorder.delete_recording, filename)
            return web.json_response({"ok": True, "deleted": filename})
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        except ValueError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        except Exception as exc:
            failure = upload_failure(exc)
            if failure is None:
                raise
            return web.json_response(
                failure, status=upload_failure_status(failure["code"])
            )

    async def batch_delete_recordings(request: web.Request) -> web.Response:
        recorder = _require_recorder(runtime)
        data = await request.json() if request.can_read_body else {}
        filenames = list(data.get("filenames", []))
        deleted = []
        failed = []
        for fname in filenames:
            try:
                await asyncio.to_thread(recorder.delete_recording, fname)
                deleted.append(fname)
            except Exception as exc:
                failure = upload_failure(exc)
                failed.append(
                    {
                        "filename": fname,
                        "error": str(exc),
                        **(
                            {"code": failure["code"], "reason": failure["reason"]}
                            if failure
                            else {}
                        ),
                    }
                )
        return web.json_response({"ok": True, "deleted": deleted, "failed": failed})

    async def batch_convert_lerobot(request: web.Request) -> web.Response:
        recorder = _require_recorder(runtime)
        data = await request.json() if request.can_read_body else {}
        try:
            result = await asyncio.to_thread(
                recorder.convert_recordings_to_lerobot, data.get("filenames")
            )
            return web.json_response(
                result, status=202 if result.get("accepted") else 200
            )
        except (ValueError, RuntimeError, FileNotFoundError) as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)

    async def batch_download_recordings(request: web.Request) -> web.StreamResponse:
        from .recording_datasets import archive_datasets, validate_download_selection

        recorder = _require_recorder(runtime)
        if request.content_type == "application/json":
            data = await request.json() if request.can_read_body else {}
            filenames = data.get("filenames") if isinstance(data, dict) else None
        else:
            form = await request.post()
            try:
                filenames = json.loads(str(form.get("filenames", "[]")))
            except (TypeError, ValueError):
                filenames = None
        try:
            filenames = validate_download_selection(filenames)
            if (
                recorder.is_recording()
                and recorder.path is not None
                and recorder.path.name in filenames
            ):
                raise web.HTTPConflict(text="recording is still being written")
            archive = await asyncio.to_thread(
                archive_datasets, recorder.directory, filenames
            )
        except (ValueError, FileNotFoundError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        return await _stream_zip_archive(
            request,
            archive,
            f'attachment; filename="hc-datasets-{datetime.now().strftime("%Y%m%d-%H%M%S")}.zip"',
        )

    app.router.add_post("/api/recording/precheck", precheck_recording)
    app.router.add_get("/api/recording/precheck", precheck_recording)
    app.router.add_post("/api/recording/start", start_recording)
    app.router.add_post("/api/recording/stop", stop_recording)
    app.router.add_get("/api/recording/status", get_recording_status)
    app.router.add_get("/api/recordings", get_recordings)
    app.router.add_post("/api/recordings/upload", import_mcap_file)
    register_download_routes(app, lambda: runtime.recorder)
    app.router.add_get("/api/recordings/{filename}/download", download_recording)
    app.router.add_get("/api/recordings/{filename}/video", get_dataset_video)
    app.router.add_delete("/api/recordings/{filename}", delete_recording)
    app.router.add_post("/api/recordings/batch-delete", batch_delete_recordings)
    app.router.add_post("/api/recordings/batch-convert-lerobot", batch_convert_lerobot)
    app.router.add_post("/api/recordings/batch-download", batch_download_recordings)

