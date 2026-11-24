"""中间件组合根（MiddlewareRuntime）与 HTTP/WebSocket 应用装配入口（create_app）。"""
from __future__ import annotations

import asyncio
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import logging
import os
from pathlib import Path
import socket
import threading
import time
from typing import Any

from aiohttp import WSMsgType, web

from adapters.core.runtime_config import unified_profile
from .config import (
    ConfigError,
    ConfigStore,
    isolated_control_process_enabled,
    validate_config,
)
from .control.intervention_api import register_intervention_routes
from .control.ros_bridge import RosBridge
from .control.safety_controller import SafetyController
from .control.teleop_api import register_teleop_routes
from .diagnostic_logs import DiagnosticLogHandler, DiagnosticLogs, register_log_routes
from .media.camera_api import build_camera_services, register_camera_routes
from .media.camera_process import CameraProcessService as CameraService
from .media.vr_gateway import VrGateway
from .media.vr_pose_handler import VrPoseHandler
from .media.ws_broadcaster import WebSocketBroadcaster
from .protocol import envelope
from .recording.recording_api import register_recording_routes
from .recording.recording_controller import (
    RecordingController,
    configure_publication_capture,
)
from .recording.replay_api import format_replay_state_payload, register_replay_routes
from .recording.ros_recording_executor import (
    RosRecordingExecutor,
    recording_subscriptions,
)
from .recording.takeover_log import TakeoverLog
from .recording.topic_player import TopicPlayer
from .recording.topic_recorder import TopicRecorder
from .robot.profile_api import register_profile_routes
from .robot.profile_metadata import load_session_profile_metadata
from .robot.robot_api import RobotApiProfile, selected_profile, selected_profile_root
from .robot.robot_profiles import RobotProfileManager
from .robot.simulation import SimulationService, register_simulation_routes
from .runtime_mode import configure_environment
from .uploads.platform_api import register_platform_routes
from .uploads.upload_integration import (
    register_runtime_upload_routes,
    start_runtime_uploads,
    upload_failure as _upload_failure,
    upload_failure_status as _upload_failure_status,
)


def control_bridge_class(ros_config: dict[str, Any]):
    if isolated_control_process_enabled(ros_config):
        from .control.control_process import IsolatedRosBridge

        return IsolatedRosBridge
    return RosBridge


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
    """中间件运行时组合根：负责子系统生命周期编排，并将具体逻辑委托给各领域协作者。"""

    def __init__(self, config: dict[str, Any], config_dir: Path):
        self.config = copy.deepcopy(config)
        self.config_dir = config_dir
        self.ros: RosBridge | None = None
        self.recording_ros: RosRecordingExecutor | None = None
        self.vr: VrGateway | None = None
        self.camera: CameraService | None = None
        self.cameras: dict[str, CameraService] = {}
        self._cameras_by_topic: dict[str, list[CameraService]] = {}
        self.recorder: TopicRecorder | None = None
        self.platform = None
        self._record_commands = None
        self._record_command_slots = threading.BoundedSemaphore(8)
        self._control_commands = None
        self._control_command_slots = threading.BoundedSemaphore(4)
        self._stopping = False
        self.uploads = None
        self.upload_error = None
        self.player: TopicPlayer | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.broadcaster = WebSocketBroadcaster(
            send_event=lambda event: self._broadcast(event)
        )
        self.websockets: set[web.WebSocketResponse] = self.broadcaster.clients
        self._broadcast_pending: dict[str, dict[str, Any]] = self.broadcaster.pending
        self.events: deque[dict[str, Any]] = deque(maxlen=300)
        self.started_at = time.time()
        self._restart_lock = asyncio.Lock()
        self._intervention_lifecycle_lock = asyncio.Lock()
        self._safety_sync_task = None
        self._safety_reason = ""
        self._safety_event_lock = threading.RLock()
        self._safety_event_generation = 0
        self.session_profile_metadata: dict[str, str] = {}
        self.control_runtime = None
        self._control_watch_task = None

        # 领域协作者（组合替代上帝类）
        self.vr_pose_handler = VrPoseHandler(self)
        self.recording_controller = RecordingController(self)
        self.safety_controller = SafetyController(self)

    def _load_session_profile_metadata(self) -> dict[str, str]:
        return load_session_profile_metadata(self.config, self.config_dir)

    async def _init_recorder_and_uploads(self) -> TopicRecorder:
        base_recorder = TopicRecorder(
            self.config["ros"]["recording"],
            self.config_dir,
            topics=recording_subscriptions(self.config["ros"]),
        )
        self.recorder = base_recorder
        try:
            from .uploads.platform_connection import PlatformConnection
            from .uploads.task_recording import TaskRecorder

            self.platform = PlatformConnection(
                self.config, self.config_dir, base_recorder.directory
            )
            self.recorder = TaskRecorder(base_recorder, self.platform)
            self.platform.activity_provider = (
                lambda: "COLLECTING"
                if self.recorder.is_recording()
                else (
                    "UPLOADING"
                    if any(
                        job["state"] == "uploading"
                        for job in self.platform.upload.jobs()
                    )
                    else "IDLE"
                )
            )
            self.platform.start()
            self.uploads = self.platform.upload
        except ImportError as exc:
            logging.getLogger(__name__).warning(
                "Platform connection component could not load: %s",
                exc.name or type(exc).__name__,
            )
            self.platform = None
            await self._start_uploads()
            self.recorder.binding_snapshot = None
        return base_recorder

    def _configure_profile_on_ros_config(self) -> None:
        profile_dir = (
            selected_profile(self.config, self.config_dir)
            if os.environ.get("HC_ROBOT_NAME")
            or self.config.get("robot_profiles", {}).get("active")
            else None
        )
        self.config["ros"].pop("robot_api_profile", None)
        self.config["ros"].pop("legacy_control_profile", None)
        if profile_dir is not None and unified_profile(profile_dir):
            RobotApiProfile(profile_dir)
            self.config["ros"]["robot_api_profile"] = str(profile_dir)
        elif profile_dir is not None and self.config["ros"].get("intervention", {}).get(
            "legacy_topics", False
        ):
            self.config["ros"]["legacy_control_profile"] = str(profile_dir)

    def _wire_ros_and_recorder(self, base_recorder: TopicRecorder) -> None:
        dataset_config = self.config["ros"].get("dataset_topics", {})
        if dataset_config.get("enabled", False):
            from .media.camera_metadata import load_camera_metadata
            from .recording.dataset_topics import load_joint_groups
            from .recording.x1_recording import is_x1

            dataset_config["x1_recording_fk"] = is_x1(self.config["ros"])
            dataset_config["joint_groups"] = load_joint_groups(
                self.config, self.config_dir
            )
            dataset_config["camera_metadata"] = load_camera_metadata(
                self.config, self.config_dir
            )
        self.ros = control_bridge_class(self.config["ros"])(
            self.config["ros"],
            self.emit,
            on_frame=self._handle_camera_ros_message,
            camera_topics=list(self._cameras_by_topic),
        )
        self.recorder.configure_corrections(self.config["ros"])
        base_recorder.takeovers = TakeoverLog(self.ros.ownership.ros_clock)
        self.ros.ownership.manual_changed = base_recorder.takeovers.manual_changed
        self.ros.ownership.dagger_changed = base_recorder.takeovers.dagger_changed
        self.ros.ownership.open_segment = self.recorder.correction_open
        self.ros.ownership.close_segment = self.recorder.correction_close

        def capture_ready() -> bool:
            capture = self.recorder.corrections
            return bool(capture and capture.status()["ready"])

        self.ros.ownership.capture_ready = capture_ready
        self.ros._action_sink = self.recorder.correction_action
        self._configure_publication_capture(base_recorder)
        self.recording_ros = RosRecordingExecutor(self.config["ros"], self.recorder)
        self.recorder.prepare_recording = self.recording_ros.prepare_recording
        if self.platform is not None:
            self.recorder.capture_health = self.recording_ros.get_topic_health

    async def start(self) -> None:
        self._stopping = False
        self.loop = asyncio.get_running_loop()
        configured_domain_id = int(self.config.get("ros", {}).get("domain_id", 14))
        domain_id = configure_environment(configured_domain_id)
        # 所有 ROS 上下文（VR、安全、录制、相机）统一使用当前运行时域 ID。
        self.config.setdefault("ros", {})["domain_id"] = domain_id
        try:
            import rclpy

            if not rclpy.ok():
                rclpy.init(args=[], domain_id=domain_id)
        except Exception:
            pass

        self._record_commands = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="recording-command"
        )
        self._control_commands = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="control-command"
        )
        self.session_profile_metadata = self._load_session_profile_metadata()
        self._configure_profile_on_ros_config()
        base_recorder = await self._init_recorder_and_uploads()
        self.player = TopicPlayer(
            self.recorder.directory,
            lambda: self.ros,
            self._on_replay_state,
            lambda: self.session_profile_metadata.get("profile_id", ""),
            lambda: self.session_profile_metadata,
        )
        self.cameras, self._cameras_by_topic, self.camera = build_camera_services(
            self.config.get("camera", {}), domain_id=domain_id
        )
        self._wire_ros_and_recorder(base_recorder)
        self.vr = VrGateway(
            self.config["vr"], self._on_pose, self.emit, self._on_vr_timeout
        )
        self.ros.start()
        self.recording_ros.start()
        self.vr.start()
        for camera in self.cameras.values():
            camera.start()
        self._log("info", "runtime started")
        if self.config["safety"].get("stop_on_startup", True):
            self._on_safety_event("middleware startup")
        self._safety_sync_task = asyncio.create_task(self._sync_safety_state())
        if self.control_runtime is not None and self.control_runtime.enabled:
            self._on_safety_event(
                "loading control configuration; explicit resume required", force=True
            )
            await self.control_runtime.start(self.config)
            if self.control_runtime.status()["state"] != "running":
                self._on_safety_event(self.control_runtime.error, force=True)
            else:
                self._control_watch_task = asyncio.create_task(self._watch_control())

    async def _watch_control(self) -> None:
        process = self.control_runtime.process
        await process.wait()
        self.control_runtime.status()
        self._on_safety_event(
            "control process exited; see control runtime status", force=True
        )

    async def _start_uploads(self) -> None:
        await start_runtime_uploads(self)

    async def stop(self) -> None:
        self._stopping = True
        # 在排空排队授权前立即撤销输出，隔离核心栅栏也会拒绝头显已提交的恢复请求。
        if self.ros is not None and getattr(self.ros, "isolated_control_process", False) is True:
            self._on_safety_event(
                "middleware stopping; explicit resume required", force=True
            )
        elif self.ros is not None and getattr(self.ros, "ownership", None) and self.ros.ownership.active:
            self.ros.ownership.disable()
        # 持有生命周期锁等待在途的介入采集准备/清理完成；急停本身绝不竞争此锁。
        async with self._intervention_lifecycle_lock:
            await self._stop_after_intervention()

    async def _stop_after_intervention(self) -> None:
        if self.control_runtime is not None and self.control_runtime.enabled:
            self._on_safety_event(
                "control configuration reload; explicit resume required", force=True
            )
            if self._control_watch_task is not None:
                self._control_watch_task.cancel()
                await asyncio.gather(self._control_watch_task, return_exceptions=True)
                self._control_watch_task = None
            await self.control_runtime.stop()
        await self.broadcaster.stop()
        if self.ros is not None and getattr(self.ros, "ownership", None) and self.ros.ownership.active:
            if getattr(self.ros, "isolated_control_process", False) is True:
                try:
                    await asyncio.to_thread(self.ros.ownership.disable)
                except RuntimeError as exc:
                    self._log("warning", str(exc))
            else:
                self.ros.ownership.disable()
        if self._safety_sync_task is not None:
            self._safety_sync_task.cancel()
            try:
                await self._safety_sync_task
            except asyncio.CancelledError:
                pass
            self._safety_sync_task = None
        if self.player is not None:
            await asyncio.to_thread(self.player.stop, False)
        if self.vr is not None:
            await asyncio.to_thread(self.vr.stop)
        if self._control_commands is not None:
            await asyncio.to_thread(
                self._control_commands.shutdown, wait=True, cancel_futures=True
            )
            self._control_commands = None
        for camera in self.cameras.values():
            await camera.stop()
        self.cameras.clear()
        self._cameras_by_topic.clear()
        self.camera = None
        # 在关闭 ROS 录制读取器与控制核心之前，先排空所有排队的 VR 录制启停命令。
        if self._record_commands is not None:
            await asyncio.to_thread(self._record_commands.shutdown, wait=True)
            self._record_commands = None
        if (
            self.recorder is not None
            and self.ros is not None
            and getattr(self.ros, "isolated_control_process", False) is True
        ):
            try:
                await asyncio.to_thread(self.recorder.stop)
            except (RuntimeError, OSError) as exc:
                self._log("warning", f"recording stop during shutdown: {exc}")
        if self.recording_ros is not None:
            await asyncio.to_thread(self.recording_ros.stop)
        if self.ros is not None:
            await asyncio.to_thread(self.ros.stop)
        if self.recorder is not None:
            await asyncio.to_thread(self.recorder.shutdown)
        if self.platform is not None:
            await asyncio.to_thread(self.platform.stop)
        elif self.uploads is not None:
            await asyncio.to_thread(getattr(self.uploads, "close", self.uploads.stop))
        self.uploads = None
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self._log("info", "runtime stopped")

    def _handle_camera_ros_message(self, topic: str, message: Any) -> None:
        for camera in self._cameras_by_topic.get(topic, []):
            camera.handle_ros_message(topic, message)

    async def restart(self, config: dict[str, Any]) -> None:
        async with self._restart_lock:
            await self.stop()
            self.config = copy.deepcopy(config)
            if self.control_runtime is not None and self.control_runtime.enabled:
                os.environ["ROS_DOMAIN_ID"] = str(config["ros"]["domain_id"])
            await self.start()

    def _configure_publication_capture(self, recorder: Any) -> None:
        configure_publication_capture(self, recorder)

    def emit(self, event: dict[str, Any], outputs: list[str]) -> None:
        self.events.append(event)
        diagnostics = getattr(self, "diagnostics", None)
        if diagnostics is not None:
            diagnostics.emit(event)
        if "record" in outputs and self.recorder is not None:
            self.recorder.record(event)
        if "udp" in outputs and self.vr is not None:
            self.vr.send_event(event)
        if "websocket" in outputs and self.loop is not None:
            self.loop.call_soon_threadsafe(self._schedule_broadcast, event)

    def _schedule_broadcast(self, event: dict[str, Any]) -> None:
        self.broadcaster.schedule(event)

    async def _broadcast(self, event: dict[str, Any]) -> None:
        await self.broadcaster.broadcast(event)

    def _on_pose(
        self, packet: Any, pose_dict: dict[str, Any] | None = None
    ) -> None:
        self.vr_pose_handler.handle(packet, pose_dict)

    def _dispatch_recording(self, operation: Any) -> None:
        self.recording_controller.dispatch(operation)

    def _dispatch_safety_resume(self, reason: str) -> None:
        SafetyController.dispatch_resume(self, reason)

    def _handle_vr_record_start(self) -> None:
        self.recording_controller.vr_start()

    def _handle_vr_record_stop(self) -> None:
        self.recording_controller.vr_stop()

    def _handle_vr_record_mark(self) -> None:
        self.recording_controller.vr_mark()

    def _send_safety_snapshot(self) -> None:
        SafetyController.send_snapshot(self)

    async def _sync_safety_state(self) -> None:
        await SafetyController.sync_loop(self)

    def _on_vr_timeout(self, reason: str) -> None:
        SafetyController.handle_vr_timeout(self, reason)

    def _on_safety_event(
        self,
        reason: str,
        *,
        force: bool = False,
        cause_event_id: str | None = None,
    ) -> None:
        SafetyController.trigger_stop(
            self, reason, force=force, cause_event_id=cause_event_id
        )

    def _on_safety_resume(self, reason: str, *, expected_fence: Any = None) -> bool:
        return SafetyController.resume(self, reason, expected_fence=expected_fence)

    def _on_replay_state(self, event_type: str, status: dict[str, Any]) -> None:
        self.emit(
            envelope(
                event_type, "middleware", format_replay_state_payload(event_type, status)
            ),
            ["websocket", "udp"],
        )

    def _log(self, level: str, message: str) -> None:
        self.emit(
            envelope("log", "middleware", {"level": level, "message": message}),
            ["websocket"],
        )

    def topic_health(self) -> dict[str, Any]:
        recording_health = (
            self.recording_ros.get_topic_health()
            if self.recording_ros is not None
            else {}
        )
        bridge_health = self.ros.get_topic_health() if self.ros is not None else {}
        # 独立原始订阅器直接统计话题到达率，避免受控制、WS 序列化或相机回调干扰。
        return {**bridge_health, **recording_health}

    def status(self) -> dict[str, Any]:
        recording_executor_status = (
            self.recording_ros.status()
            if self.recording_ros
            else {"state": "stopped", "topic_health": {}}
        )
        ros_status = self.ros.status() if self.ros else {"state": "stopped"}
        ros_status["topic_health"] = {
            **ros_status.get("topic_health", {}),
            **recording_executor_status.get("topic_health", {}),
        }
        camera_streams = [camera.status() for camera in self.cameras.values()]
        camera_status = (
            dict(camera_streams[0]) if camera_streams else {"state": "stopped"}
        )
        camera_status["streams"] = camera_streams
        return {
            "status": "ok",
            "version": 1,
            "runtime_profile": self.session_profile_metadata,
            "control_runtime": (
                self.control_runtime.status()
                if self.control_runtime
                else {"state": "unmanaged"}
            ),
            "control_backend": os.environ.get("HC_CONTROL_BACKEND") or None,
            "uptime_seconds": round(time.time() - self.started_at, 1),
            "websocket_clients": len(self.websockets),
            "safety_stop_active": bool(self._safety_reason),
            "ros": ros_status,
            "recording_executor": recording_executor_status,
            "vr": self.vr.status() if self.vr else {"state": "stopped"},
            "camera": camera_status,
            "recording": (
                self.recorder.status()
                if self.recorder
                else {"recording": False, "enabled": False}
            ),
            "replay": (
                self.player.status()
                if self.player
                else {"state": "idle", "is_active": False}
            ),
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


def _build_simulation_guard_middleware(
    store: ConfigStore, runtime: MiddlewareRuntime, simulation: SimulationService
):
    @web.middleware
    async def serialize_simulation_changes(request: web.Request, handler: Any):
        # 串行化配置/型号写入与仿真启动，避免运行中模型读到半导入资产或不一致的配置。
        if request.method in {"POST", "PUT", "DELETE"} and (
            request.path.startswith("/api/robot-profiles")
            or request.path == "/api/config"
        ):
            if runtime.recorder is not None and runtime.recorder.is_recording():
                raise web.HTTPConflict(text="请先停止录制，再修改配置或机器人型号")
            if (
                runtime.ros
                and getattr(runtime.ros, "ownership", None)
                and runtime.ros.ownership.active
            ):
                raise web.HTTPConflict(text="请先退出人工干预会话，再修改配置或机器人型号")
            async with simulation.lock:
                stop_required = True
                if request.path == "/api/config":
                    try:
                        proposed = validate_config(await request.json())
                        stop_required = (
                            proposed["robot_profiles"] != store.value["robot_profiles"]
                            or proposed["ros"]["domain_id"] != store.value["ros"]["domain_id"]
                            or proposed["simulation"] != store.value["simulation"]
                        )
                    except (ConfigError, ValueError, TypeError):
                        stop_required = False
                elif request.path.endswith("/activate"):
                    stop_required = True
                elif request.method == "DELETE":
                    stop_required = (
                        request.match_info["profile_id"] == simulation.profile_id
                    )
                if simulation.processes and stop_required:
                    await simulation.stop()
                return await handler(request)
        return await handler(request)

    return serialize_simulation_changes


async def _stream_simulation_frames(simulation: SimulationService) -> None:
    last_frame = None
    while True:
        await asyncio.sleep(1.0 / 30.0)
        frame = simulation.latest_frame
        if frame is last_frame or frame is None or simulation.state != "running":
            continue
        last_frame = frame
        event = {
            "type": "simulation_frame",
            "data": {**frame, "profile_id": simulation.profile_id},
        }
        for viewer in tuple(simulation.viewers):
            try:
                await asyncio.wait_for(viewer.send_json(event), timeout=0.5)
            except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                simulation.set_viewer(viewer, False)
                try:
                    await asyncio.wait_for(viewer.close(), timeout=0.2)
                except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                    pass


def create_app(store: ConfigStore) -> web.Application:
    config = store.load()
    runtime = MiddlewareRuntime(config, store.path.parent)
    hardware_logs = Path(
        os.environ.get("HC_X1_LOG_ROOT", str(Path.home() / "HC_X1" / "runtime"))
    )
    diagnostics = DiagnosticLogs(
        Path(__file__).resolve().parents[2],
        os.environ.get("HC_MIDDLEWARE_LOG_DIR"),
        external_roots=[("hardware", hardware_logs)],
    )
    runtime.diagnostics = diagnostics
    diagnostic_handler = DiagnosticLogHandler(diagnostics)
    diagnostic_handler.setLevel(logging.WARNING)

    from .control.control_runtime import ControlRuntime

    runtime.control_runtime = ControlRuntime(store.path)
    profiles = RobotProfileManager(selected_profile_root(config, store.path.parent))
    simulation = SimulationService(store, profiles)

    app = web.Application(
        client_max_size=100 * 1024 * 1024,
        middlewares=[
            cors_middleware,
            _build_simulation_guard_middleware(store, runtime, simulation),
        ],
    )
    static_dir = Path(__file__).parent / "static"

    async def index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(static_dir / "index.html")

    async def get_status(_request: web.Request) -> web.Response:
        return web.json_response(runtime.status())

    async def get_config(_request: web.Request) -> web.Response:
        return web.json_response(store.value)

    async def put_config(request: web.Request) -> web.Response:
        if runtime.recorder is not None and runtime.recorder.is_recording():
            return web.json_response(
                {
                    "ok": False,
                    "code": "RECORDING_ACTIVE",
                    "reason": "请先停止当前录制，再保存配置；新保存方式用于下一段录制",
                },
                status=409,
            )
        try:
            proposed = await request.json()
            old_server = dict(store.value["server"])
            old_profile_root = store.value["robot_profiles"]["root"]
            saved = store.save(proposed)
            await runtime.restart(saved)
            control_status = runtime.control_runtime.status()
            if control_status["state"] == "failed":
                return web.json_response(
                    {
                        "ok": False,
                        "config": saved,
                        "error": "配置已保存，但控制进程启动失败；输出保持停止",
                        "control_runtime": control_status,
                    },
                    status=503,
                )
            restart_required = (
                saved["server"] != old_server
                or saved["robot_profiles"]["root"] != old_profile_root
            )
            return web.json_response(
                {
                    "ok": True,
                    "config": saved,
                    "server_restart_required": restart_required,
                    "control_runtime": control_status,
                }
            )
        except (ConfigError, json.JSONDecodeError, TypeError) as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc

    async def websocket(request: web.Request) -> web.WebSocketResponse:
        # 遥测与 RTP 共享事件循环；同步 deflate 压缩会卡顿视频。
        ws = web.WebSocketResponse(
            heartbeat=20, max_msg_size=1024 * 1024, compress=False
        )
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
                    elif data.get("type") == "simulation_viewer":
                        if data.get("data", {}).get("enabled", True):
                            simulation.set_viewer(
                                ws, True, data.get("data", {}).get("render_mode", "native")
                            )
                        else:
                            simulation.set_viewer(ws, False)
                        await ws.send_json(
                            {
                                "type": "simulation_viewer_ack",
                                "data": {"registered": ws in simulation.viewers},
                            }
                        )
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
            simulation.set_viewer(ws, False)
        return ws

    viewer_sender: asyncio.Task | None = None

    async def startup(_app: web.Application) -> None:
        nonlocal viewer_sender
        await asyncio.to_thread(diagnostics.start)
        logging.getLogger().addHandler(diagnostic_handler)
        try:
            await runtime.start()
            viewer_sender = asyncio.create_task(_stream_simulation_frames(simulation))
        except BaseException:
            logging.getLogger(__name__).exception("middleware startup failed")
            logging.getLogger().removeHandler(diagnostic_handler)
            await asyncio.to_thread(diagnostics.close)
            raise

    async def shutdown(_app: web.Application) -> None:
        if viewer_sender is not None:
            viewer_sender.cancel()
            await asyncio.gather(viewer_sender, return_exceptions=True)
        await runtime.broadcaster.close_all()
        async with simulation.lock:
            await simulation.stop()
        await runtime.stop()

    async def close_diagnostics(_app: web.Application) -> None:
        logging.getLogger().removeHandler(diagnostic_handler)
        await asyncio.to_thread(diagnostics.close)

    async def options(_request: web.Request) -> web.Response:
        return web.Response(status=204)

    # 核心页面、状态、配置与 WebSocket
    app.router.add_get("/", index)
    app.router.add_get("/dashboard/", index)
    app.router.add_static("/static/", static_dir)
    app.router.add_get("/api/status", get_status)
    app.router.add_get("/health", get_status)
    app.router.add_get("/api/config", get_config)
    app.router.add_put("/api/config", put_config)
    app.router.add_get("/ws", websocket)

    # 按领域模块注册其余 API 路由
    register_log_routes(app, diagnostics)
    register_intervention_routes(app, runtime)
    register_simulation_routes(app, simulation)
    register_profile_routes(app, store, runtime, profiles)
    register_teleop_routes(app, store, runtime)
    register_runtime_upload_routes(app, store, runtime)
    register_platform_routes(app, runtime)
    register_recording_routes(app, store, runtime)
    register_replay_routes(app, runtime)
    register_camera_routes(app, runtime)

    app.router.add_options("/{tail:.*}", options)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(shutdown)
    app.on_cleanup.append(close_diagnostics)
    return app
