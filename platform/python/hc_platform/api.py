"""HTTP and WebSocket projection. Robot authority remains in the C++ process."""

from __future__ import annotations
import argparse
import asyncio
from collections import OrderedDict
import contextlib
import hmac
import io
import json
import os
from pathlib import Path
import tempfile
import tarfile
import time
import uuid
import zipfile

from fastapi import (
    FastAPI,
    Request,
    UploadFile,
    File,
    HTTPException,
    WebSocket,
    WebSocketDisconnect,
    WebSocketException,
    Query,
    Depends,
)
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.requests import HTTPConnection
from pydantic import BaseModel, ConfigDict, Field
import uvicorn

from .configuration import ConfigurationError, atomic_json, within
from .ipc import CoreError
from .runtime import Runtime
from .topics import TopicRegistry, public_publish_allowed
from .data import DataError
from .replay import ReplayManager, ReplayError, ACTIVE as REPLAY_ACTIVE
from .motion import MotionService, create_router as motion_router, MotionError


class Operation(BaseModel):
    model_config = ConfigDict(extra="forbid")
    op: str
    request_id: str = Field(default_factory=lambda: uuid.uuid4().hex, max_length=63)
    expected_boot_id: str | None = None
    expected_epoch: int | None = None
    mode: str | None = None
    groups: list[str] | None = None
    dagger_enabled: bool = False
    enabled: bool | None = None


class PlatformServer(uvicorn.Server):
    """Fence actuators before HTTP connections are allowed to drain."""

    def __init__(self, config, runtime):
        super().__init__(config)
        self.runtime = runtime
        # MJPEG, WebSocket and large downloads must not indefinitely delay
        # lifespan shutdown and reaping of the independent control processes.
        self.config.timeout_graceful_shutdown = 2

    async def shutdown(self, sockets=None):
        self.runtime.shutdown_requested = True
        self.runtime.management_generation += 1
        with contextlib.suppress(Exception):
            await asyncio.wait_for(
                self.runtime.core.request("stop", reason="SERVER_SHUTDOWN"), 0.75
            )
        try:
            await super().shutdown(sockets)
        finally:
            # Also clean up on a second interrupt, when Uvicorn skips lifespan.
            await self.runtime.close()


def create_app(runtime: Runtime):
    runtime.replay = ReplayManager(runtime)
    runtime.motion = MotionService(runtime)

    @contextlib.asynccontextmanager
    async def lifespan(app):
        try:
            await runtime.start()
            yield
        finally:
            await runtime.close()

    async def accepting_operations(connection: HTTPConnection):
        # Dependency runs after request-body parsing as well as middleware's
        # ingress check, covering an upload that began before shutdown.
        if runtime.shutdown_requested:
            if connection.scope["type"] == "websocket":
                raise WebSocketException(code=1012)
            if connection.scope.get("method") not in ("GET", "HEAD", "OPTIONS"):
                raise HTTPException(503, "Runtime is shutting down")

    app = FastAPI(
        title="HC Robot Platform",
        version="0.1.0",
        lifespan=lifespan,
        dependencies=[Depends(accepting_operations)],
    )
    app.include_router(motion_router(runtime.motion))
    operation_cache = OrderedDict()
    token = os.environ.get("HC_PLATFORM_TOKEN", "")
    read_only_posts = {"/offer", "/api/webrtc/offer"}
    request_rates = {}
    viewers = set()

    @app.middleware("http")
    async def access(request, call_next):
        if runtime.shutdown_requested and request.method not in (
            "GET",
            "HEAD",
            "OPTIONS",
        ):
            return JSONResponse({"code": "RUNTIME_SHUTTING_DOWN"}, 503)
        if (
            request.headers.get("content-type", "").startswith("application/json")
            and int(request.headers.get("content-length", 0)) > 1024 * 1024
        ):
            return JSONResponse({"code": "JSON_REQUEST_TOO_LARGE"}, 413)
        if (
            request.method not in ("GET", "HEAD", "OPTIONS")
            and request.url.path not in read_only_posts
            and not request.url.path.startswith(("/offer/", "/api/webrtc/receiver/"))
        ):
            peer = request.client.host if request.client else ""
            supplied = request.headers.get("authorization", "").removeprefix("Bearer ")
            if token:
                if not hmac.compare_digest(supplied, token):
                    return JSONResponse({"code": "CONTROL_AUTH_REQUIRED"}, 401)
            elif peer not in ("127.0.0.1", "::1", "testclient"):
                return JSONResponse(
                    {
                        "code": "REMOTE_CONTROL_DISABLED",
                        "message": "Configure HC_PLATFORM_TOKEN for remote operations",
                    },
                    403,
                )
            origin = request.headers.get("origin")
            if origin and origin.rstrip("/") != str(request.base_url).rstrip("/"):
                return JSONResponse({"code": "ORIGIN_REJECTED"}, 403)
            now = time.monotonic()
            count, started = request_rates.get(peer, (0, now))
            if now - started >= 1:
                count, started = 0, now
            if count >= 200:
                return JSONResponse({"code": "RATE_LIMITED"}, 429)
            request_rates[peer] = count + 1, started
        try:
            return await call_next(request)
        except (CoreError, ConfigurationError, ValueError) as exc:
            return JSONResponse(
                {
                    "code": getattr(exc, "code", "INVALID_REQUEST"),
                    "message": str(exc),
                    "state": getattr(exc, "state", None),
                },
                409,
            )

    @app.exception_handler(CoreError)
    async def core_error(request, exc):
        return JSONResponse(
            {"code": exc.code, "message": str(exc), "state": exc.state}, 409
        )

    @app.exception_handler(DataError)
    async def data_error(request, exc):
        return JSONResponse(exc.as_dict(), 409)

    @app.exception_handler(ConfigurationError)
    async def config_error(request, exc):
        return JSONResponse({"code": "CONFIGURATION_INVALID", "message": str(exc)}, 422)

    @app.exception_handler(ValueError)
    async def value_error(request, exc):
        return JSONResponse({"code": "INVALID_REQUEST", "message": str(exc)}, 422)

    @app.exception_handler(ReplayError)
    @app.exception_handler(MotionError)
    async def execution_error(request, exc):
        return JSONResponse({"code": exc.code, "message": str(exc)}, 409)

    @app.get("/health")
    async def health():
        return {"status": "ok", "runtime": "process-v1", "robot": runtime.robot}

    @app.get("/api/status")
    async def status():
        return await runtime.status()

    @app.get("/api/control/state")
    async def control_state():
        return (await runtime.core.request("status"))["state"]

    @app.get("/api/robot/info")
    async def robot_info():
        status = await runtime.status()
        config = runtime.config
        hardware = status["backend"]
        capabilities = {}
        for name, configured in {
            "arms.teleop": bool(config["arms"]),
            "grippers.open_close": config["enable_grippers"],
            "body.base": config["enable_base"],
            "body.waist": "body" in config["allowed_groups"],
            "body.neck": bool(
                config["teleop"].get("body", {}).get("neck_tracking_enabled")
            ),
            "dagger": True,
            "joint_target_stream": True,
        }.items():
            capabilities[name] = {
                "declared": configured,
                "configured": configured,
                "backend_supported": configured,
                "runtime_ready": configured
                and status["control"].get("safety") == "ready",
                "verification": config["verification"],
            }
        configured_arms = [name for name in config["groups"] if name.endswith("_arm")]
        authorized_arms = [
            name for name in configured_arms if name in config["allowed_groups"]
        ]
        available_arms = [
            name
            for name in configured_arms
            if name in status["control"].get("available_groups", [])
        ]
        ready_arms = sorted(set(authorized_arms) & set(available_arms))
        capabilities["arms.teleop"].update(
            configured_groups=configured_arms,
            authorized_groups=authorized_arms,
            available_groups=available_arms,
            ready_groups=ready_arms,
            runtime_ready=bool(ready_arms)
            and status["control"].get("safety") == "ready",
        )
        sides = [
            side
            for side in ("left", "right")
            if side + "_gripper" in config["hardware"]["allowed_groups"]
        ]
        hands = {
            side: bool(hardware.get("ready"))
            if runtime.mode == "sim"
            else bool(hardware.get("gripper_ready", {}).get(side))
            for side in sides
        }
        hand_ready = bool(sides) and all(hands.values())
        capabilities["grippers.open_close"].update(
            authorized_sides=sides,
            side_ready=hands,
            runtime_ready=hand_ready and status["control"].get("safety") == "ready",
            blocking_reasons=[]
            if hand_ready
            else ["GRIPPER_FEEDBACK_UNAVAILABLE"]
            if sides
            else ["GROUP_NOT_AUTHORIZED"],
        )
        motion = await runtime.motion.capabilities()
        capabilities.update(motion.get("capabilities", motion))
        return {
            "schema": "hc-platform-robot/v1",
            "robot_id": runtime.robot,
            "config_hash": config["config_hash"],
            "command_interface_version": config["command_interface_version"],
            "groups": config["groups"],
            "joints": [
                {
                    "name": n,
                    "unit": unit,
                    "lower": lo,
                    "upper": hi,
                    "velocity": v,
                    "acceleration": a,
                }
                for n, unit, lo, hi, v, a in zip(
                    config["joint_names"],
                    config["joint_units"],
                    config["lower"],
                    config["upper"],
                    config["velocity"],
                    config["acceleration"],
                )
            ],
            "frame_convention": "ROS right-handed SI, quaternion xyzw",
            "capabilities": capabilities,
            "hardware": hardware,
            "applied_allowed_groups": config["hardware"]["allowed_groups"],
        }

    @app.post("/api/control/operations", status_code=202)
    async def control(operation: Operation):
        valid = {
            "recover",
            "stop",
            "pause",
            "resume",
            "session_start",
            "session_end",
            "set_dagger",
            "reference_reset",
            "home",
        }
        if operation.op not in valid:
            raise HTTPException(400, "Unknown control operation")
        if operation.op == "stop":
            # Stop reaches the authority before waiting for any CPU job or recorder cleanup.
            runtime.management_generation += 1
            response = await runtime.core.request(
                "stop", reason="OPERATOR_SOFTWARE_STOP"
            )
            await runtime.replay.stop()
            await runtime.motion.close()
            return {
                "operation_id": operation.request_id,
                "status": "accepted",
                **response,
            }
        # Bind authority at ingress, before waiting for configuration / recording
        # work. A stop while status or the lifecycle lock is awaited cancels it.
        generation = runtime.management_generation
        values = operation.model_dump(exclude_none=True)
        request_id = values.pop("request_id")
        fingerprint = json.dumps(values, sort_keys=True)
        current = (await runtime.core.request("status"))["state"]
        values.setdefault("expected_boot_id", current["boot_id"])
        values.setdefault("expected_epoch", current["epoch"])
        cache_key = (current["boot_id"], request_id)
        async with runtime.mutation_lock:
            if cache_key in operation_cache:
                previous, response = operation_cache[cache_key]
                if previous != fingerprint:
                    raise HTTPException(
                        409, "Idempotency key reused with another operation"
                    )
                return response
            current = (await runtime.core.request("status"))["state"]
            if generation != runtime.management_generation or any(
                values[key] != current[state_key]
                for key, state_key in (
                    ("expected_boot_id", "boot_id"),
                    ("expected_epoch", "epoch"),
                )
            ):
                raise CoreError("STALE_OPERATION", state=current)
            if (await runtime.replay.status()).get("phase") in REPLAY_ACTIVE:
                raise HTTPException(
                    409, "Stop managed replay before changing live control"
                )
            enabling_dagger = (
                operation.op == "session_start"
                and operation.mode == "program"
                and operation.dagger_enabled
            ) or (operation.op == "set_dagger" and operation.enabled is True)
            if (
                enabling_dagger
                and runtime.active_recording
                and runtime.recording_mode == "ordinary"
            ):
                raise HTTPException(
                    409, "Stop ordinary recording before enabling DAgger"
                )
            if operation.op == "home":
                values.update(await runtime.motion.validate_home(operation.groups))
            started_recording = None
            try:
                if (
                    operation.op == "session_start"
                    and operation.mode == "program"
                    and runtime.recording_policy["enabled"]
                ):
                    await runtime.start_recording(
                        {
                            "format": "mcap",
                            "mode": "dagger_" + runtime.recording_policy["mode"],
                            "auto_record": True,
                        },
                        request_id + "-record",
                    )
                    started_recording = runtime.active_recording
                if generation != runtime.management_generation:
                    raise CoreError("STALE_OPERATION", state=runtime.core.state)
                response = await runtime.core.request(values.pop("op"), **values)
                if operation.op in ("recover", "resume"):
                    mark_reset = getattr(runtime.replay, "mark_reset", None)
                    if callable(mark_reset):
                        await mark_reset()
            except BaseException:
                if started_recording and runtime.active_recording == started_recording:
                    await runtime.stop_recording()
                raise
            result = {"operation_id": request_id, "status": "accepted", **response}
            operation_cache[cache_key] = (fingerprint, result)
            while len(operation_cache) > 256:
                operation_cache.popitem(last=False)
            if (
                operation.op == "session_end"
                and runtime.active_recording
                and runtime.recording_mode.startswith("dagger")
            ):
                await runtime.stop_recording()
            return result

    @app.post("/api/control/command")
    async def command(payload: dict):
        source = payload.get("source")
        if source not in ("program", "exo"):
            raise HTTPException(403, "Use the managed replay API for replay")
        if source == "program" and runtime.motion.busy:
            raise HTTPException(409, "A Motion task owns the program source")
        required = {
            "boot_id",
            "epoch",
            "lease_id",
            "reference_epoch",
            "seq",
            "ttl_ms",
            "joint_names",
            "positions",
        }
        if not required.issubset(payload):
            raise HTTPException(
                422,
                "A command must carry current lease, epochs, names, sequence and TTL",
            )
        payload = await runtime.motion.validate_external(payload)
        return await runtime.source_clients[source].request(
            "command", **{k: v for k, v in payload.items() if k not in ("op", "id")}
        )

    @app.put("/api/dagger/recording")
    async def dagger_recording(payload: dict):
        async with runtime.mutation_lock:
            if (await runtime.core.request("status"))["state"].get("mode") != "idle":
                raise HTTPException(409, "Change recording policy between sessions")
            if payload.get("mode", "corrections") not in ("corrections", "full"):
                raise HTTPException(422, "Expected corrections or full")
            runtime.recording_policy = {
                "enabled": bool(payload.get("enabled")),
                "mode": payload.get("mode", "corrections"),
                "required": bool(payload.get("required", False)),
            }
            await asyncio.to_thread(
                atomic_json,
                runtime.state_dir / "dagger-recording.json",
                runtime.recording_policy,
            )
            return runtime.recording_policy

    @app.get("/api/robots")
    async def robots():
        return await asyncio.to_thread(runtime.registry.list)

    @app.post("/api/robots/{robot_id}/import", status_code=201)
    async def import_robot(robot_id: str, file: UploadFile = File(...)):
        fd, name = tempfile.mkstemp(dir=runtime.state_dir, suffix=".zip")
        total = 0
        try:
            with os.fdopen(fd, "wb") as stream:
                while chunk := await file.read(1024 * 1024):
                    total += len(chunk)
                    if total > 512 * 1024**2:
                        raise HTTPException(413, "Compressed package exceeds 512 MiB")
                    await asyncio.to_thread(stream.write, chunk)
            return await asyncio.to_thread(
                runtime.registry.import_zip, Path(name), robot_id
            )
        finally:
            Path(name).unlink(missing_ok=True)

    @app.post("/api/robots/{robot_id}/apply")
    async def apply_robot(robot_id: str):
        return await runtime.apply(robot_id)

    @app.get("/api/robots/{robot_id}/export")
    async def export_robot(robot_id: str):
        output = runtime.state_dir / f"export-{robot_id}.zip"
        await asyncio.to_thread(runtime.registry.export_zip, robot_id, output)
        return FileResponse(output, filename=f"{robot_id}.zip")

    @app.delete("/api/robots/{robot_id}")
    async def delete_robot(robot_id: str):
        async with runtime.mutation_lock:
            offset, references = 0, []
            while True:
                datasets = await runtime.data.list_datasets(limit=1000, offset=offset)
                if any(
                    item.get("summary", {}).get("robot_id") == robot_id
                    for item in datasets
                ):
                    raise ConfigurationError(
                        "Robot package is referenced by a saved dataset"
                    )
                references.extend(item.get("config_hash") for item in datasets)
                if len(datasets) < 1000:
                    break
                offset += len(datasets)
            await asyncio.to_thread(runtime.registry.delete, robot_id, references)
        return {"deleted": robot_id}

    @app.get("/api/config")
    async def config():
        return {
            "applied": runtime.config,
            "overrides": runtime.overrides(),
            "robot_id": runtime.robot,
            "runtime_mode": runtime.mode,
            "ros_domain_id": runtime.domain,
        }

    @app.put("/api/config")
    async def apply_config(payload: dict):
        return await runtime.apply(runtime.robot, payload)

    @app.post("/api/config/validate")
    async def validate_config(payload: dict):
        value = await asyncio.to_thread(runtime.compile, runtime.robot, payload)
        return {"valid": True, "effective": value}

    @app.get("/api/topics")
    async def topics():
        try:
            media = (await runtime.media.get("/status")).json()
        except Exception as exc:
            media = {"reason": str(exc)}
        return {
            "configured": runtime.topics.items,
            "discovered": media.get("topics", []),
            "metrics": media.get("topic_metrics", {}),
            "reason": media.get("reason"),
        }

    @app.get("/api/topics/catalog")
    async def topic_catalog():
        robot = runtime.robot
        package = await asyncio.to_thread(runtime.registry.compile, robot)
        return {
            "robot_id": robot,
            "topics": package.get("topics", []),
            "source": "robot_package",
        }

    @app.put("/api/topics")
    async def save_topic(item: dict):
        async with runtime.mutation_lock:
            if runtime.active_recording or (await runtime.status_data.status()).get(
                "active_session_id"
            ):
                raise HTTPException(409, "Recording topic selection is frozen")
            result = await asyncio.to_thread(runtime.topics.put, item)
            runtime.config["topics"] = runtime.topics.items
            await asyncio.to_thread(
                atomic_json, runtime.run_dir / "config.json", runtime.config
            )
            await runtime.supervisor.stop("media")
            await runtime.start_media()
            return result

    @app.delete("/api/topics")
    async def delete_topic(topic: str):
        async with runtime.mutation_lock:
            if runtime.active_recording or (await runtime.status_data.status()).get(
                "active_session_id"
            ):
                raise HTTPException(409, "Recording topic selection is frozen")
            await asyncio.to_thread(runtime.topics.delete, topic)
            runtime.config["topics"] = runtime.topics.items
            await asyncio.to_thread(
                atomic_json, runtime.run_dir / "config.json", runtime.config
            )
            await runtime.supervisor.stop("media")
            await runtime.start_media()
            return {"deleted": topic}

    @app.post("/api/messages/publish")
    async def publish(payload: dict):
        if not public_publish_allowed(payload.get("topic", "")):
            raise HTTPException(
                403, "Control and hardware topics cannot use generic publication"
            )
        response = await runtime.media.post("/publish", json=payload)
        return Response(
            response.content,
            status_code=response.status_code,
            media_type="application/json",
        )

    @app.post("/api/recordings", status_code=201)
    async def start_recording(spec: dict):
        async with runtime.mutation_lock:
            if (await runtime.core.request("status"))["state"].get("mode") == "program":
                raise HTTPException(
                    409,
                    "Configure DAgger recording before starting an autonomous session",
                )
            return await runtime.start_recording(
                spec, spec.pop("request_id", uuid.uuid4().hex)
            )

    @app.post("/api/recordings/preflight")
    async def recording_preflight(spec: dict):
        return await runtime.recording_preflight(spec)

    @app.post("/api/recordings/stop")
    async def stop_recording():
        async with runtime.mutation_lock:
            return await runtime.stop_recording()

    @app.post("/api/recordings/mark")
    async def mark(payload: dict):
        if not runtime.active_recording:
            raise HTTPException(409, "No recording")
        label = str(payload.get("label", "operator_mark"))[:1024]
        sample = await runtime.audit_data.append_sample(
            session_id=runtime.active_recording,
            topic="/capture/marks",
            payload={
                "label": label,
                "task_success": payload.get("task_success"),
                "source": "operator",
            },
        )
        runtime.notify_vr(
            "recording_marked",
            {
                "marked": True,
                "session_id": runtime.active_recording,
                "filename": runtime.active_recording,
                "message": label,
            },
        )
        return sample

    @app.get("/api/datasets")
    async def datasets(
        limit: int = Query(100, ge=1, le=1000), offset: int = Query(0, ge=0)
    ):
        return await runtime.data.list_datasets(limit=limit, offset=offset)

    @app.post("/api/datasets/download")
    async def download_many(payload: dict):
        identifiers = payload.get("artifact_ids", [])
        if (
            not isinstance(identifiers, list)
            or not 1 <= len(identifiers) <= 100
            or len(set(identifiers)) != len(identifiers)
        ):
            raise HTTPException(422, "Select 1-100 distinct datasets")
        owner = "batch-download-" + uuid.uuid4().hex
        archives = []
        descriptor, output_name = tempfile.mkstemp(suffix=".tar", dir=runtime.state_dir)
        os.close(descriptor)

        async def cleanup():
            for identifier, archive in archives:
                with contextlib.suppress(Exception):
                    await runtime.data.release_artifact(identifier, owner)
                Path(archive["path"]).unlink(missing_ok=True)
            Path(output_name).unlink(missing_ok=True)

        try:
            for identifier in identifiers:
                archives.append(
                    (identifier, await runtime.data.prepare_download(identifier, owner))
                )

            def package():
                with tarfile.open(output_name, "w") as stream:
                    for identifier, archive in archives:
                        stream.add(
                            archive["path"],
                            arcname=identifier + ".tar",
                            recursive=False,
                        )

            await asyncio.to_thread(package)
        except BaseException:
            await cleanup()
            raise
        from starlette.background import BackgroundTask

        return FileResponse(
            output_name,
            media_type="application/x-tar",
            filename="datasets.tar",
            background=BackgroundTask(cleanup),
        )

    @app.get("/api/datasets/{artifact_id}")
    async def dataset(artifact_id: str):
        return await runtime.data.get_dataset(artifact_id)

    @app.post("/api/datasets/{artifact_id}/convert", status_code=202)
    async def convert(artifact_id: str, payload: dict):
        return await runtime.data.enqueue_conversion(
            artifact_id,
            payload.get("request_id", uuid.uuid4().hex),
            payload.get("options", {}),
        )

    @app.post("/api/datasets/{artifact_id}/upload", status_code=202)
    async def upload(artifact_id: str, payload: dict):
        target = dict(payload.get("target", {}))
        if "task_id" in target:
            target["collection_task_id"] = target.pop("task_id")
        platform = await runtime.data.platform_status()
        target.setdefault(
            "robot_id",
            platform.get("robot_id", platform.get("capabilities", {}).get("robot_id")),
        )
        return await runtime.data.enqueue_upload(
            artifact_id, payload.get("request_id", uuid.uuid4().hex), target
        )

    @app.delete("/api/datasets/{artifact_id}")
    async def delete_dataset(artifact_id: str):
        return await runtime.data.delete_dataset(artifact_id)

    @app.get("/api/datasets/{artifact_id}/download")
    async def download(artifact_id: str):
        owner = "download-" + uuid.uuid4().hex
        artifact = await runtime.data.prepare_download(artifact_id, owner)

        async def stream():
            temporary = None
            try:
                path = Path(artifact["path"])
                if not path.is_absolute() or not path.exists():
                    raise RuntimeError("Artifact location unavailable")
                if path.is_dir():
                    fd, temporary = tempfile.mkstemp(
                        suffix=".zip", dir=runtime.state_dir
                    )
                    os.close(fd)

                    def package():
                        with zipfile.ZipFile(
                            temporary, "w", zipfile.ZIP_STORED
                        ) as output:
                            for file in path.rglob("*"):
                                if file.is_file() and not file.is_symlink():
                                    output.write(file, file.relative_to(path))

                    await asyncio.to_thread(package)
                    path = Path(temporary)
                with path.open("rb") as file:
                    while chunk := await asyncio.to_thread(file.read, 1024 * 1024):
                        yield chunk
            finally:
                if temporary:
                    Path(temporary).unlink(missing_ok=True)
                await runtime.data.release_artifact(artifact_id, owner)
                Path(artifact["path"]).unlink(missing_ok=True)

        return StreamingResponse(
            stream(),
            media_type="application/x-tar",
            headers={
                "Content-Disposition": f'attachment; filename="{artifact_id}.tar"'
            },
        )

    @app.post("/api/datasets/import", status_code=201)
    async def import_mcap(file: UploadFile = File(...)):
        fd, name = tempfile.mkstemp(suffix=".mcap", dir=runtime.state_dir)
        try:
            total = 0
            with os.fdopen(fd, "wb") as output:
                while chunk := await file.read(1024 * 1024):
                    total += len(chunk)
                    if total > 32 * 1024**3:
                        raise HTTPException(413, "Import exceeds 32 GiB")
                    await asyncio.to_thread(output.write, chunk)
            return await runtime.data.import_mcap(
                name, uuid.uuid4().hex, {"imported_filename": file.filename}
            )
        finally:
            Path(name).unlink(missing_ok=True)

    @app.get("/api/datasets/{artifact_id}/asset")
    async def asset(artifact_id: str, path: str, request: Request):
        owner = "preview-" + uuid.uuid4().hex
        await runtime.data.acquire_artifact(artifact_id, owner, 3600)
        try:
            result = await runtime.data.resolve_asset(artifact_id, path, owner)
        except BaseException:
            await runtime.data.release_artifact(artifact_id, owner)
            raise
        from starlette.background import BackgroundTask

        return FileResponse(
            result["path"],
            background=BackgroundTask(
                runtime.data.release_artifact, artifact_id, owner
            ),
        )

    @app.get("/api/platform/status")
    async def platform_status():
        return await runtime.data.platform_status()

    @app.put("/api/platform/config")
    async def platform_config(payload: dict):
        if "link" in payload:
            return await runtime.data.connect_platform(
                payload["link"],
                runtime.robot,
                payload.get("request_id", uuid.uuid4().hex),
                payload.get("allow_insecure_lan", False),
            )
        payload = dict(payload)
        if "base_url" in payload:
            payload["platform_base_url"] = payload.pop("base_url")
        reference = payload.pop("credential_ref", "")
        if reference.startswith("env:"):
            payload["credential_env"] = reference[4:]
        elif reference.startswith("file:"):
            payload["credential_file"] = reference[5:]
        return await runtime.data.configure_platform(payload)

    @app.get("/api/platform/tasks")
    async def platform_tasks(q: str = ""):
        return await runtime.data.platform_tasks(
            {"q": q, "local_robot": runtime.robot, "source_format": "MCAP"}
        )

    @app.post("/api/platform/preflight")
    async def platform_preflight(payload: dict):
        return await runtime.data.platform_preflight(
            payload["task_id"],
            "LEROBOT_V3" if payload.get("format", "mcap") == "lerobot" else "MCAP",
            runtime.robot,
        )

    @app.get("/api/jobs")
    async def jobs():
        return await runtime.data.list_jobs()

    @app.post("/api/jobs/{job_id}/cancel")
    async def cancel_job(job_id: str):
        return await runtime.data.cancel_job(job_id)

    @app.post("/api/jobs/{job_id}/retry")
    async def retry_job(job_id: str):
        return await runtime.data.retry_job(job_id)

    @app.post("/api/jobs/{job_id}/refresh")
    async def refresh_job(job_id: str):
        return await runtime.data.refresh_upload(job_id)

    @app.get("/api/replay/status")
    async def replay_status():
        return await runtime.replay.status()

    @app.post("/api/replay/start", status_code=202)
    async def replay_start(payload: dict):
        async with runtime.mutation_lock:
            return await runtime.replay.start(**payload)

    @app.post("/api/replay/{action}")
    async def replay_action(action: str, payload: dict | None = None):
        if action in ("pause", "resume", "stop"):
            return await getattr(runtime.replay, action)()
        if action == "speed":
            return await runtime.replay.set_speed((payload or {}).get("speed", 1))
        raise HTTPException(404, "Unknown replay operation")

    @app.get("/api/cameras")
    async def cameras():
        return (await runtime.media.get("/status")).json()

    def projection_path(robot):
        return runtime.state_dir / ("projection-" + robot + ".json")

    @app.get("/api/camera/status")
    async def headset_cameras():
        robot = runtime.robot
        status = (await runtime.media.get("/status")).json()
        enabled = {
            stream["id"]: stream
            for stream in status.get("streams", [])
            if stream.get("enabled", True)
        }

        def read_selection():
            path = projection_path(robot)
            return (
                json.loads(path.read_text()).get("camera_ids")
                if path.exists()
                else None
            )

        reason = None
        try:
            selected = await asyncio.to_thread(read_selection)
        except (OSError, ValueError, AttributeError):
            selected, reason = None, "SAVED_PROJECTION_UNREADABLE"
        if selected is not None and (
            not isinstance(selected, list)
            or not 1 <= len(selected) <= 4
            or any(
                not isinstance(identifier, str) or identifier not in enabled
                for identifier in selected
            )
            or len(set(selected)) != len(selected)
        ):
            selected, reason = None, "SAVED_PROJECTION_HAS_UNAVAILABLE_CAMERAS"
        if selected is None:
            selected = list(enabled)[:4]
        if not selected:
            reason = "NO_ENABLED_CAMERAS"
        return {
            **status,
            "streams": [enabled[identifier] for identifier in selected],
            "robot_id": robot,
            "projection": {"camera_ids": selected, "fallback_reason": reason},
        }

    @app.put("/api/cameras/projection")
    async def set_projection(payload: dict):
        identifiers = payload.get("camera_ids")
        if (
            not isinstance(identifiers, list)
            or not 1 <= len(identifiers) <= 4
            or any(not isinstance(value, str) for value in identifiers)
            or len(set(identifiers)) != len(identifiers)
        ):
            raise HTTPException(422, "Select 1-4 different camera IDs")
        async with runtime.mutation_lock:
            enabled = {
                stream["id"]
                for stream in runtime.config.get("camera", {}).get("streams", [])
                if stream.get("enabled", True)
            }
            if not set(identifiers).issubset(enabled):
                raise HTTPException(
                    422,
                    "Projection requires enabled cameras from the applied configuration",
                )
            value = {"robot_id": runtime.robot, "camera_ids": identifiers}
            await asyncio.to_thread(atomic_json, projection_path(runtime.robot), value)
            return value

    @app.get("/api/cameras/{identifier}/snapshot")
    async def snapshot(identifier: str):
        response = await runtime.media.get(f"/cameras/{identifier}/snapshot")
        return Response(
            response.content,
            response.status_code,
            media_type=response.headers.get("content-type", "image/jpeg"),
        )

    @app.get("/api/cameras/{identifier}/preview")
    async def preview(identifier: str):
        status = (await runtime.media.get("/status")).json()
        if not any(stream.get("id") == identifier for stream in status.get("streams", [])):
            raise HTTPException(404, "Unknown camera")

        async def frames():
            async with runtime.media.stream(
                "GET", f"/cameras/{identifier}/preview", timeout=None
            ) as response:
                async for chunk in response.aiter_bytes():
                    yield chunk

        return StreamingResponse(
            frames(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

    def authorize_video(request):
        peer = request.client.host if request.client else ""
        origin = request.headers.get("origin")
        if origin and origin.rstrip("/") != str(request.base_url).rstrip("/"):
            raise HTTPException(403, "ORIGIN_REJECTED")
        supplied = request.headers.get("authorization", "").removeprefix("Bearer ")
        if peer in ("127.0.0.1", "::1", "testclient") or (
            token and hmac.compare_digest(supplied, token)
        ):
            return
        vr = runtime.read_adapter_state("vr")
        if (
            vr.get("process_alive")
            and not vr.get("status_stale", True)
            and vr.get("is_locked")
            and vr.get("active_peer") == peer
        ):
            return
        raise HTTPException(403, "VIDEO_SESSION_NOT_AUTHORIZED")

    @app.post("/offer")
    @app.post("/api/webrtc/offer")
    async def offer(payload: dict, request: Request):
        authorize_video(request)
        response = await runtime.media.post("/offer", json=payload, timeout=15)
        return Response(
            response.content, response.status_code, media_type="application/json"
        )

    @app.post("/offer/{identifier}")
    async def stream_offer(identifier: str, payload: dict, request: Request):
        return await offer({**payload, "camera_ids": [identifier]}, request)

    @app.post("/api/webrtc/receiver/{identifier}")
    async def receiver_telemetry(identifier: str, payload: dict, request: Request):
        authorize_video(request)
        response = await runtime.media.post("/receiver/" + identifier, json=payload)
        return Response(
            response.content, response.status_code, media_type="application/json"
        )

    @app.get("/api/logs")
    async def logs(
        after: int = 0,
        level: str = "",
        subsystem: str = "",
        q: str = "",
        session: str = "",
    ):
        return await asyncio.to_thread(
            runtime.supervisor.query_logs,
            session=session,
            after=after,
            level=level,
            subsystem=subsystem,
            q=q,
        )

    @app.get("/api/logs/export")
    async def export_logs(
        session: str = "", level: str = "", subsystem: str = "", q: str = ""
    ):
        items = await asyncio.to_thread(
            runtime.supervisor.query_logs,
            session=session,
            level=level,
            subsystem=subsystem,
            q=q,
            limit=10000,
        )
        return Response(
            "\n".join(json.dumps(item, ensure_ascii=False) for item in items),
            media_type="application/x-ndjson",
            headers={
                "Content-Disposition": 'attachment; filename="runtime-logs.jsonl"'
            },
        )

    @app.get("/api/log-sessions")
    async def log_sessions():
        return [
            {"id": p.stem, "size": p.stat().st_size}
            for p in sorted(runtime.supervisor.log_dir.glob("*.jsonl"), reverse=True)
        ]

    @app.get("/api/diagnostics")
    async def diagnostics():
        state = await runtime.status()
        try:
            media = (await runtime.media.get("/status")).json()
        except Exception as exc:
            media = {"reason": str(exc)}
        import shutil

        disk = shutil.disk_usage(runtime.state_dir)
        return {
            "runtime": state,
            "media": media,
            "storage": {"free_bytes": disk.free, "total_bytes": disk.total},
            "ports": {
                "vr_udp": runtime.vr_port,
                "discovery_udp": runtime.discovery_port,
                "headset_status_udp": 5007,
            },
            "isolation": {
                "ros_domain_id": runtime.domain,
                "ros_localhost_only": runtime.mode == "sim",
            },
            "control_audit_dropped": runtime.audit.dropped_events,
            "log_dropped": runtime.supervisor.logs_dropped,
            "physical_verification": "not_performed",
            "software_stop_boundary": "application_and_adapter_gate",
        }

    @app.get("/api/observations")
    async def observations():
        media = (await runtime.media.get("/status")).json()
        return {
            "control": (await runtime.status())["control"],
            "topics": media.get("observations", {}),
            "camera_metadata": {
                stream["id"]: stream.get("metadata", {})
                for stream in media.get("streams", [])
            },
            "camera_status": media.get("streams", []),
        }

    @app.get("/api/model/{resource:path}")
    async def model_resource(resource: str):
        path = within(Path(runtime.config["package_root"]), resource)
        if path.suffix.lower() not in (
            ".urdf",
            ".stl",
            ".dae",
            ".obj",
            ".png",
            ".jpg",
            ".mtl",
        ):
            raise HTTPException(403, "Not a model resource")
        return FileResponse(path)

    @app.post("/api/runtime/restart")
    async def restart():
        return await runtime.apply(runtime.robot)

    @app.post("/api/runtime/simulation/{action}")
    async def simulation(action: str):
        if runtime.mode != "sim":
            raise HTTPException(409, "Simulation requires an isolated sim launch")
        if action == "stop":
            async with runtime.mutation_lock:
                if runtime.active_recording:
                    raise HTTPException(409, "Stop recording first")
                await runtime.stop_control()
            return {"stopped": True}
        if action in ("start", "restart"):
            return await runtime.apply(runtime.robot)
        raise HTTPException(404, "Unknown simulation action")

    @app.websocket("/api/ws")
    async def websocket(ws: WebSocket):
        if len(viewers) >= 16:
            await ws.close(code=1013)
            return
        await ws.accept()
        viewers.add(ws)
        try:
            while True:
                # Latest snapshots: a slow viewer never queues control history.
                await asyncio.wait_for(ws.send_json(await runtime.status()), 2)
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, asyncio.TimeoutError):
            pass
        finally:
            viewers.discard(ws)

    @app.websocket("/api/topics/ws")
    async def topic_websocket(ws: WebSocket):
        if len(viewers) >= 16:
            await ws.close(code=1013)
            return
        await ws.accept()
        viewers.add(ws)
        try:
            while True:
                value = (await runtime.media.get("/status")).json()
                await asyncio.wait_for(
                    ws.send_json(
                        {
                            "type": "observations",
                            "topics": value.get("observations", {}),
                            "metrics": value.get("topic_metrics", {}),
                        }
                    ),
                    2,
                )
                await asyncio.sleep(0.05)
        except (WebSocketDisconnect, asyncio.TimeoutError):
            pass
        finally:
            viewers.discard(ws)

    @app.websocket("/api/messages/ws")
    async def publish_websocket(ws: WebSocket):
        await ws.accept()
        try:
            peer = ws.client.host if ws.client else ""
            origin = ws.headers.get("origin")
            if (
                origin
                and origin.rstrip("/")
                != str(ws.url)
                .replace("ws://", "http://")
                .replace("wss://", "https://")
                .split("/api/", 1)[0]
            ):
                await ws.close(code=1008)
                return
            if token:
                auth = await asyncio.wait_for(ws.receive_json(), 2)
                if not hmac.compare_digest(
                    str(auth.get("authorization", "")).removeprefix("Bearer "), token
                ):
                    await ws.close(code=1008)
                    return
            elif peer not in ("127.0.0.1", "::1", "testclient"):
                await ws.close(code=1008)
                return
            await ws.send_json({"authenticated": True, "allowed_prefix": "/external/"})
            while True:
                text = await ws.receive_text()
                if len(text) > 65536:
                    await ws.close(code=1009)
                    return
                payload = json.loads(text)
                if not isinstance(payload, dict) or not public_publish_allowed(
                    payload.get("topic", "")
                ):
                    await ws.send_json({"accepted": False, "code": "TOPIC_FORBIDDEN"})
                    continue
                response = await runtime.media.post("/publish", json=payload)
                try:
                    result = response.json()
                except ValueError:
                    result = {
                        "accepted": False,
                        "status_code": response.status_code,
                        "detail": response.text,
                    }
                if response.status_code >= 400 and isinstance(result, dict):
                    result.setdefault("accepted", False)
                    result.setdefault("status_code", response.status_code)
                await ws.send_json(result)
                await asyncio.sleep(0.02)
        except (WebSocketDisconnect, asyncio.TimeoutError, ValueError):
            with contextlib.suppress(Exception):
                await ws.close(code=1008)

    @app.websocket("/ws")
    async def legacy_websocket(ws: WebSocket):
        """Legacy compatibility WebSocket endpoint for VR headset and tools."""
        if len(viewers) >= 16:
            await ws.close(code=1013)
            return
        await ws.accept()
        viewers.add(ws)

        def _envelope(kind: str, payload: dict):
            return {
                "version": 1,
                "type": "event",
                "kind": kind,
                "source": "middleware",
                "timestamp": time.time(),
                "payload": payload,
            }

        try:
            initial = await runtime.status()
            last_state = (
                initial.get("control", {}).get("safety"),
                initial.get("control", {}).get("mode"),
            )
            await ws.send_json(_envelope("connected", initial))
            while not runtime.shutdown_requested:
                try:
                    raw = await asyncio.wait_for(ws.receive_text(), 0.5)
                    try:
                        data = json.loads(raw)
                    except ValueError:
                        await ws.send_json({"error": "invalid JSON"})
                        continue
                    if isinstance(data, dict):
                        if data.get("kind") == "ping":
                            await ws.send_json(_envelope("pong", {}))
                        elif data.get("type") == "simulation_viewer":
                            await ws.send_json(
                                {
                                    "type": "simulation_viewer_ack",
                                    "data": {"registered": False},
                                }
                            )
                except asyncio.TimeoutError:
                    current = await runtime.status()
                    control = current.get("control", {})
                    pair = (control.get("safety"), control.get("mode"))
                    if pair != last_state:
                        last_state = pair
                        kind = "safety_resume" if pair[0] == "ready" else "safety_stop"
                        await ws.send_json(
                            _envelope(
                                kind,
                                {
                                    "reason": control.get("reason", ""),
                                    "safety": pair[0],
                                    "mode": pair[1],
                                },
                            )
                        )
        except (WebSocketDisconnect, asyncio.TimeoutError, OSError, RuntimeError):
            pass
        finally:
            viewers.discard(ws)

    web_root = runtime.repo / "platform/web/dist"
    if web_root.exists():

        class _HttpStaticFiles(StaticFiles):
            async def __call__(self, scope, receive, send):
                if scope["type"] != "http":
                    if scope["type"] == "websocket":
                        await send({"type": "websocket.close", "code": 1008})
                    return
                await super().__call__(scope, receive, send)

        app.mount(
            "/", _HttpStaticFiles(directory=web_root, html=True), name="console"
        )
    return app


def main():
    parser = argparse.ArgumentParser(description="HC process-isolated runtime")
    parser.add_argument("--repo", default=str(Path(__file__).resolve().parents[3]))
    parser.add_argument("--state-dir")
    parser.add_argument("--robot")
    parser.add_argument("--mode", choices=("sim", "teleop"), default="sim")
    parser.add_argument(
        "--domain", type=int, default=int(os.environ.get("ROS_DOMAIN_ID", 14))
    )
    parser.add_argument(
        "--groups", help="Comma-separated explicitly enabled resource groups"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7876)
    parser.add_argument("--vr-port", type=int, default=5005)
    parser.add_argument("--discovery-port", type=int, default=5006)
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    runtime = Runtime(
        repo,
        Path(args.state_dir) if args.state_dir else repo / "runtime/platform",
        robot=args.robot,
        mode=args.mode,
        domain=args.domain,
        groups=args.groups.split(",") if args.groups else None,
        vr_port=args.vr_port,
        discovery_port=args.discovery_port,
    )
    runtime.api_url = f"http://127.0.0.1:{args.port}"
    if args.host in ("127.0.0.1", "localhost", "::1"):
        print(
            f"[hc_platform] HTTP/WebRTC bound to {args.host}:{args.port} "
            "(LAN VR headset video projection requires --host 0.0.0.0)",
            flush=True,
        )
    os.umask(0o077)
    config = uvicorn.Config(
        create_app(runtime),
        host=args.host,
        port=args.port,
        log_level="info",
        ws_max_size=65536,
        ws_max_queue=16,
        access_log=False,
    )
    PlatformServer(config, runtime).run()


if __name__ == "__main__":
    main()
