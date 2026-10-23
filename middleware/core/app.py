from __future__ import annotations

import asyncio
import threading
import copy
import json
import logging
import os
import socket
import time
from collections import deque
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web
import yaml

from .camera import CameraService
from .simulation import SimulationService, register_simulation_routes
from .config import ConfigError, ConfigStore, validate_config
from .profile_api import register_profile_routes
from .protocol import envelope
from .robot_profiles import RobotProfileManager
from .robot_api import RobotApiProfile, selected_profile, selected_profile_root
from adapters.core.runtime_config import unified_profile
from .ros_bridge import RosBridge
from .ros_recording_executor import RosRecordingExecutor
from .topic_player import TopicPlayer
from .topic_recorder import TopicRecorder
from .vr_gateway import VrGateway
from .runtime_mode import configure_environment


def _upload_failure(exc: Exception) -> dict[str, Any] | None:
    """Identify optional uploader errors without making uploads a hard dependency."""
    try:
        from humanoid_upload.common import UploadError
    except ImportError:
        return None
    if not isinstance(exc, UploadError):
        return None
    return {"ok": False, "code": exc.code, "reason": exc.action}


def _upload_failure_status(code: str) -> int:
    return 409 if code in {"ARTIFACT_BUSY", "WORKER_ALREADY_RUNNING", "BINDING_REQUIRED"} else 400


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
        self.uploads = None
        self.upload_error = None
        self.player: TopicPlayer | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.websockets: set[web.WebSocketResponse] = set()
        self.events: deque[dict[str, Any]] = deque(maxlen=300)
        self.started_at = time.time()
        self._restart_lock = asyncio.Lock()
        self._safety_sync_task = None
        self._safety_reason = ""
        self._last_x_held = False
        self._last_y_held = False
        self._last_a_held = False
        self._last_both_sticks_held = False
        self._last_replay_status_sent_at = 0.0
        self.session_profile_metadata: dict[str, str] = {}

    def _load_session_profile_metadata(self) -> dict[str, str]:
        """Describe the Profile actually selected when this runtime starts."""
        profile_config = self.config.get("robot_profiles", {})
        profile_id = str(
            os.environ.get("HC_ROBOT_NAME")
            or profile_config.get("active", "")
        ).strip()
        if not profile_id:
            return {}

        metadata: dict[str, str] = {
            "profile_id": profile_id,
            "profile_display_name": profile_id,
            "robot_name": profile_id,
            "controller_config": "",
            "free_joints": "[]",
            "ros_domain_id": str(self.config.get("ros", {}).get("domain_id", 14)),
        }
        profile_dir = selected_profile(self.config, self.config_dir)
        try:
            profile = yaml.safe_load(
                (profile_dir / "profile.yaml").read_text(encoding="utf-8")
            ) or {}
            metadata["profile_display_name"] = str(
                profile.get("display_name") or profile_id
            )
            metadata["robot_name"] = str(profile.get("robot_name") or profile_id)
            controller_name = str(
                profile.get("primary_config") or "controller_v23.yml"
            )
            metadata["controller_config"] = controller_name
            controller = yaml.safe_load(
                (profile_dir / controller_name).read_text(encoding="utf-8")
            ) or {}
            free_joints = controller.get("model", {}).get("free_joints", [])
            if isinstance(free_joints, list):
                metadata["free_joints"] = json.dumps(
                    [str(name) for name in free_joints],
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
        except (OSError, TypeError, yaml.YAMLError):
            pass
        if unified_profile(profile_dir):
            api = RobotApiProfile(profile_dir)
            metadata.update(robot_api_schema="hc-robot-api/v1", robot_api_revision=api.revision,
                            robot_api_info=json.dumps(api.describe(), separators=(",", ":")),
                            free_joints=json.dumps([j["public_name"] for j in api.joints.values()]))
        return metadata

    async def start(self) -> None:
        self.loop = asyncio.get_running_loop()
        configured_domain_id = int(self.config.get("ros", {}).get("domain_id", 14))
        domain_id = configure_environment(configured_domain_id)
        # Every ROS context (VR, safety, recording and camera) uses the selected
        # runtime domain, including after a Dashboard configuration reload.
        self.config.setdefault("ros", {})["domain_id"] = domain_id
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init(args=[], domain_id=domain_id)
        except Exception:
            pass

        from concurrent.futures import ThreadPoolExecutor
        self._record_commands = ThreadPoolExecutor(max_workers=1, thread_name_prefix="recording-command")
        self.session_profile_metadata = self._load_session_profile_metadata()
        profile_dir = (selected_profile(self.config, self.config_dir)
                       if os.environ.get("HC_ROBOT_NAME") or self.config.get("robot_profiles", {}).get("active") else None)
        self.config["ros"].pop("robot_api_profile", None)
        if profile_dir is not None and unified_profile(profile_dir):
            RobotApiProfile(profile_dir)
            self.config["ros"]["robot_api_profile"] = str(profile_dir)
        recorder = TopicRecorder(self.config["ros"]["recording"], self.config_dir)
        self.recorder = recorder
        try:
            from .platform_connection import PlatformConnection
            from .task_recording import TaskRecorder
            self.platform = PlatformConnection(self.config, self.config_dir, recorder.directory)
            self.recorder = TaskRecorder(recorder, self.platform)
            self.platform.activity_provider = lambda: "COLLECTING" if self.recorder.is_recording() else (
                "UPLOADING" if any(job['state'] == 'uploading' for job in self.platform.upload.jobs()) else "IDLE")
            self.platform.start()
            self.uploads = self.platform.upload
        except ImportError as exc:
            logging.getLogger(__name__).warning(
                "Platform connection component could not load: %s", exc.name or type(exc).__name__
            )
            self.platform = None
            await self._start_uploads()
            self.recorder.binding_snapshot = None
        self.player = TopicPlayer(
            self.recorder.directory,
            lambda: self.ros,
            self._on_replay_state,
            lambda: self.session_profile_metadata.get("profile_id", ""),
            lambda: self.session_profile_metadata,
        )
        camera_config = self.config.get("camera", {})
        base_camera_config = {
            key: value for key, value in camera_config.items() if key != "streams"
        }
        configured_streams = camera_config.get("streams", [])
        if not configured_streams:
            configured_streams = [
                {
                    "id": "main",
                    "name": "主相机",
                    "topic": base_camera_config.get(
                        "custom_topic", ""
                    ) or base_camera_config.get(
                        "topic", "/observation/cameras/head/color/compressed"
                    ),
                    "enabled": base_camera_config.get("enabled", False),
                }
            ]

        self.cameras = {}
        self._cameras_by_topic = {}
        for stream in configured_streams:
            if not base_camera_config.get("enabled", False) or not stream.get(
                "enabled", True
            ):
                continue
            stream_config = {**base_camera_config, **stream, "enabled": True}
            stream_id = str(stream_config.get("id", "main"))
            service = CameraService(stream_config, domain_id=domain_id)
            self.cameras[stream_id] = service
            active_topic = str(
                stream_config.get("custom_topic", "")
                or stream_config.get(
                    "topic", "/observation/cameras/head/color/compressed"
                )
            )
            self._cameras_by_topic.setdefault(active_topic, []).append(service)

        self.camera = next(iter(self.cameras.values()), None)
        dataset_config = self.config["ros"].get("dataset_topics", {})
        if dataset_config.get("enabled", False):
            from .dataset_topics import load_joint_groups
            dataset_config["joint_groups"] = load_joint_groups(self.config, self.config_dir)
        self.ros = RosBridge(
            self.config["ros"],
            self.emit,
            on_frame=self._handle_camera_ros_message,
            camera_topics=list(self._cameras_by_topic),
        )
        self.recorder.configure_corrections(self.config['ros'])
        self.ros.ownership.open_segment = self.recorder.correction_open
        self.ros.ownership.close_segment = self.recorder.correction_close
        def capture_ready():
            capture = self.recorder.corrections
            return bool(capture and capture.status()['ready'])
        self.ros.ownership.capture_ready = capture_ready
        self.ros._action_sink = self.recorder.correction_action
        self.recording_ros = RosRecordingExecutor(self.config["ros"], self.recorder)
        self.recorder.prepare_recording = self.recording_ros.prepare_recording
        if self.platform is not None:
            self.recorder.capture_health = self.recording_ros.get_topic_health
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

    async def _start_uploads(self) -> None:
        """Optional uploader; unavailable dependencies must not prevent local recording."""
        self.uploads = None
        self.upload_error = None
        try:
            from .upload_service import UploadService
            upload_config = copy.deepcopy(self.config.get("uploads", {"enabled": False}))
            queue_path = Path(upload_config.get("queue_directory", "../runtime/upload_queue")).expanduser()
            upload_config["queue_directory"] = str(
                (self.config_dir / queue_path).resolve() if not queue_path.is_absolute() else queue_path
            )
            service = UploadService(
                upload_config, artifact_state_dir=self.recorder.artifact_state_dir,
                data_directory=self.recorder.directory,
            )
            try:
                await asyncio.to_thread(service.start)
            except Exception:
                await asyncio.to_thread(service.stop)
                raise
            self.uploads = service
            self.recorder.upload_service = service
            if upload_config.get("enabled", False) and upload_config.get("auto_enqueue", False):
                self.recorder.binding_snapshot = service.binding_snapshot
        except Exception as exc:
            self.upload_error = str(exc)
            self._log("error", "upload unavailable: " + str(exc))

    async def stop(self) -> None:
        if self.ros is not None and getattr(self.ros, 'ownership', None) and self.ros.ownership.active:
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
        for camera in self.cameras.values():
            await camera.stop()
        self.cameras.clear()
        self._cameras_by_topic.clear()
        self.camera = None
        if self.recording_ros is not None:
            await asyncio.to_thread(self.recording_ros.stop)
        if self.ros is not None:
            await asyncio.to_thread(self.ros.stop)
        if self._record_commands is not None:
            await asyncio.to_thread(self._record_commands.shutdown, wait=True)
            self._record_commands = None
        if self.recorder is not None:
            await asyncio.to_thread(self.recorder.shutdown)
        if self.platform is not None:
            await asyncio.to_thread(self.platform.stop)
        elif self.uploads is not None:
            await asyncio.to_thread(getattr(self.uploads, 'close', self.uploads.stop))
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
        left_input = getattr(packet, "left_input", None)
        right_input = getattr(packet, "right_input", None)

        left_held = int(getattr(left_input, "held_mask", 0))
        left_pressed = int(getattr(left_input, "pressed_mask", 0))
        right_held = int(getattr(right_input, "held_mask", 0))
        right_pressed = int(getattr(right_input, "pressed_mask", 0))

        # A resumes command output only once per physical press.  The old
        # implementation called this for every packet while A was held.
        a_held = bool(right_held & 1)
        a_down = bool(right_pressed & 1) or (a_held and not self._last_a_held)
        self._last_a_held = a_held
        if a_down:
            self._on_safety_resume("VR controller A button pressed")

        # Pressing both thumbsticks marks the active recording, or the most
        # recently completed recording.  Use the combined held edge so the
        # gesture still works when the two sticks are pressed a few frames apart.
        stick_bit = 1 << 5
        both_sticks_held = bool(left_held & stick_bit) and bool(right_held & stick_bit)
        both_sticks_pressed = bool(left_pressed & stick_bit) and bool(right_pressed & stick_bit)
        mark_down = both_sticks_pressed or (
            both_sticks_held and not self._last_both_sticks_held
        )
        self._last_both_sticks_held = both_sticks_held
        if mark_down:
            self._dispatch_recording(self._handle_vr_record_mark)

        # Replay control events use UDP and can be lost.  While replay is
        # active or waiting for the A-button acknowledgement, periodically
        # resend a compact snapshot so the headset UI converges reliably.
        if self.player is not None and self.vr is not None:
            replay = self.player.status()
            now = time.monotonic()
            if (replay.get("is_active") or replay.get("requires_reset")) and (
                now - self._last_replay_status_sent_at >= 1.0
            ):
                replay_message = (
                    "重放结束；请按 A 恢复遥操作"
                    if replay.get("requires_reset")
                    else "正在重放"
                )
                self.vr.send_event(
                    envelope(
                        "replay_status",
                        "middleware",
                        {**replay, "message": replay_message},
                    )
                )
                self._last_replay_status_sent_at = now

        if left_input is not None:
            # X button on left controller: bit 0 (1 << 0) -> Start recording
            x_down = bool(left_pressed & 1) or (bool(left_held & 1) and not self._last_x_held)
            self._last_x_held = bool(left_held & 1)

            # Y button on left controller: bit 1 (1 << 1) -> Stop recording
            y_down = bool(left_pressed & 2) or (bool(left_held & 2) and not self._last_y_held)
            self._last_y_held = bool(left_held & 2)

            if x_down:
                self._dispatch_recording(self._handle_vr_record_start)
            elif y_down:
                self._dispatch_recording(self._handle_vr_record_stop)

        if self.ros is None:
            return
        if self.config["vr"].get("publish_to_ros", True):
            self.ros.publish(
                self.config["vr"]["data_topic"],
                "std_msgs/msg/String",
                {
                    "data": json.dumps(
                        {**packet.as_dict(), 'hc_received_ns': time.time_ns()},
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                },
            )

    def _dispatch_recording(self, operation):
        if self._record_commands is None:
            operation()
            return
        if not self._record_command_slots.acquire(blocking=False):
            self._log('error', 'recording command queue is full')
            return
        try:
            future = self._record_commands.submit(operation)
        except Exception:
            self._record_command_slots.release()
            raise
        future.add_done_callback(lambda _: self._record_command_slots.release())

    def _handle_vr_record_start(self) -> None:
        if self.recorder is None:
            return
        if getattr(self.recorder, 'corrections', None):
            self._log('info', 'DAgger 会话按所选范围录制，X/Y 普通录制已停用')
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
            if self.platform is None and self.recording_ros is not None:
                self.recording_ros.prepare_recording()
            filename = f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.mcap"
            path = self.recorder.start(filename, self.session_profile_metadata)
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
            upload_failure = _upload_failure(exc)
            error_text = (f"{upload_failure['code']}: {upload_failure['reason']}"
                          if upload_failure else str(exc))
            self._log("error", f"Failed to start recording on VR X button: {error_text}")
            self.emit(
                envelope(
                    "recording_error",
                    "middleware",
                    {
                        "recording": False,
                        "error": error_text,
                        "action": "start_failed",
                        **({"code": upload_failure["code"], "reason": upload_failure["reason"]}
                           if upload_failure else {}),
                    },
                ),
                ["websocket", "udp"],
            )

    def _handle_vr_record_stop(self) -> None:
        if self.recorder is not None and getattr(self.recorder, 'corrections', None):
            self._log('info', '纠正采集使用 Grip 自动分段，X/Y 不结束纠正会话')
            return
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

    def _handle_vr_record_mark(self) -> None:
        if self.recorder is None:
            return
        try:
            result = self.recorder.mark_current_or_latest("vr_dual_thumbstick")
            filename = str(result.get("filename", ""))
            message = f"已标记录制文件: {filename}"
            self._log("info", message)
            if self.ros is not None:
                self.ros.publish(
                    "/teleop/recording_state",
                    "std_msgs/msg/String",
                    {"data": json.dumps(result, ensure_ascii=False)},
                )
            self.emit(
                envelope(
                    "recording_marked",
                    "middleware",
                    {**result, "action": "mark", "message": message},
                ),
                ["websocket", "udp"],
            )
        except Exception as exc:
            self._log("error", f"Failed to mark recording from VR: {exc}")
            self.emit(
                envelope(
                    "recording_mark_error",
                    "middleware",
                    {
                        "action": "mark_failed",
                        "error": str(exc),
                        "message": "没有可标记的录制文件",
                    },
                ),
                ["websocket", "udp"],
            )

    def _send_safety_snapshot(self) -> None:
        """Repair lost UDP events / headset reconnects without changing actuator state."""
        if self.ros is None or self.vr is None:
            return
        status = self.ros.status()
        if status.get("state") != "running":
            return  # Unknown state must never clear the headset latch.
        enabled = status.get("command_mux", {}).get("output_enabled")
        if not isinstance(enabled, bool):
            return
        self.vr.send_event(envelope(
            "safety_resume" if enabled else "safety_stop", "middleware",
            {"reason": "middleware state synchronization" if enabled else
             (self._safety_reason or "command output disabled; explicit resume required"),
             "state_snapshot": True},
        ))

    async def _sync_safety_state(self) -> None:
        while True:
            self._send_safety_snapshot()
            await asyncio.sleep(0.5)

    def _on_vr_timeout(self, reason: str) -> None:
        if self.ros and getattr(self.ros, 'ownership', None) and self.ros.ownership.active:
            self.ros.ownership.fault(reason)
            return
        source = (self.ros.status().get("command_mux", {}).get("source") if self.ros else None)
        if source is None:
            source = self.config.get("ros", {}).get("command_mux", {}).get("source", "vr")
        if source == "vr":
            self._on_safety_event(reason)
        else:
            # VR link health cannot revoke another explicitly selected source.
            if self.ros:
                self.ros.publish("/control/teleop/enabled", "std_msgs/msg/Bool", {"data": False})
            self.emit(envelope("vr_timeout", "middleware", {"reason": reason}), ["websocket", "udp"])

    def _on_safety_event(self, reason: str) -> None:
        self._safety_reason = reason
        if self.ros is not None and self.config["safety"].get("enabled", True):
            self.ros.emergency_stop(self.config["safety"]["stop_topic"], reason)
        self.emit(envelope("safety_stop", "middleware", {"reason": reason}), ["websocket", "udp"])

    def _on_safety_resume(self, reason: str) -> None:
        if self.ros is not None and getattr(self.ros, 'ownership', None) and self.ros.ownership.active:
            try:
                self.ros.recover_intervention()
            except ValueError as exc:
                self._log('warning', str(exc))
            return
        self._safety_reason = ""
        if self.ros is not None:
            stop_topic = self.config["safety"].get("stop_topic", "/control/emergency_stop")
            self.ros.set_command_output_enabled(True)
            self.ros.publish(stop_topic, "std_msgs/msg/Bool", {"data": False})
            source = self.config.get("ros", {}).get("command_mux", {}).get(
                "source", "vr"
            )
            self.ros.publish(
                "/control/teleop/enabled",
                "std_msgs/msg/Bool",
                {"data": source == "vr"},
            )
        if self.player is not None:
            self.player.acknowledge_reset()
        self.emit(envelope("safety_resume", "middleware", {"reason": reason}), ["websocket", "udp"])

    def _on_replay_state(self, event_type: str, status: dict[str, Any]) -> None:
        state = str(status.get("state", "idle"))
        messages = {
            "replay_started": "正在重放；重放结束后请按 A 恢复遥操作",
            "replay_paused": "重放已暂停",
            "replay_resumed": "重放已继续",
            "replay_stopped": "重放已停止；请按 A 恢复遥操作",
            "replay_completed": "重放完成；请按 A 恢复遥操作",
            "replay_error": "重放异常；请按 A 恢复遥操作",
            "replay_reset": "已恢复实时遥操作",
        }
        payload = {
            **status,
            "state": state,
            "message": messages.get(event_type, event_type),
        }
        self.emit(envelope(event_type, "middleware", payload), ["websocket", "udp"])

    def _log(self, level: str, message: str) -> None:
        self.events.append(
            envelope("log", "middleware", {"level": level, "message": message})
        )

    def topic_health(self) -> dict[str, Any]:
        recording_health = (
            self.recording_ros.get_topic_health()
            if self.recording_ros is not None
            else {}
        )
        bridge_health = (
            self.ros.get_topic_health() if self.ros is not None else {}
        )
        return {**recording_health, **bridge_health}

    def status(self) -> dict[str, Any]:
        recording_executor_status = (
            self.recording_ros.status()
            if self.recording_ros
            else {"state": "stopped", "topic_health": {}}
        )
        ros_status = self.ros.status() if self.ros else {"state": "stopped"}
        ros_status["topic_health"] = {
            **recording_executor_status.get("topic_health", {}),
            **ros_status.get("topic_health", {}),
        }
        camera_streams = [camera.status() for camera in self.cameras.values()]
        camera_status = (
            dict(camera_streams[0])
            if camera_streams
            else {"state": "stopped"}
        )
        camera_status["streams"] = camera_streams
        return {
            "status": "ok",
            "version": 1,
            "runtime_profile": self.session_profile_metadata,
            "control_backend": os.environ.get("HC_CONTROL_BACKEND") or None,
            "uptime_seconds": round(time.time() - self.started_at, 1),
            "websocket_clients": len(self.websockets),
            "ros": ros_status,
            "recording_executor": recording_executor_status,
            "vr": self.vr.status() if self.vr else {"state": "stopped"},
            "camera": camera_status,
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
    profile_root = selected_profile_root(config, store.path.parent)
    profiles = RobotProfileManager(profile_root)
    app = web.Application(
        client_max_size=100 * 1024 * 1024, middlewares=[cors_middleware]
    )
    simulation = SimulationService(store, profiles)

    @web.middleware
    async def serialize_simulation_changes(request, handler):
        # Serialize profile/config writes against launches so a running model can
        # never observe half-imported assets or a different active selection.
        if request.method in {"POST", "PUT", "DELETE"} and (
            request.path.startswith("/api/robot-profiles") or request.path == "/api/config"
        ):
            if runtime.ros and getattr(runtime.ros, 'ownership', None) and runtime.ros.ownership.active:
                raise web.HTTPConflict(text='请先退出人工干预会话，再修改配置或机器人型号')
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
                        stop_required = False  # Let the config handler report validation errors.
                elif request.path.endswith("/activate"):
                    # Even the same ID may now contain newly imported assets/settings.
                    stop_required = True
                elif request.method == "DELETE":
                    stop_required = request.match_info["profile_id"] == simulation.profile_id
                if simulation.processes and stop_required:
                    await simulation.stop()
                return await handler(request)
        return await handler(request)

    app.middlewares.append(serialize_simulation_changes)
    from .intervention_api import register_intervention_routes
    register_intervention_routes(app, runtime)
    register_simulation_routes(app, simulation)
    static_dir = Path(__file__).parent / "static"

    async def index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(static_dir / "index.html")

    async def get_status(_request: web.Request) -> web.Response:
        return web.json_response(runtime.status())

    async def get_config(_request: web.Request) -> web.Response:
        return web.json_response(store.value)

    async def put_config(request: web.Request) -> web.Response:
        if runtime.recorder is not None and runtime.recorder.is_recording():
            return web.json_response({'ok': False, 'code': 'RECORDING_ACTIVE',
                'reason': '请先停止当前录制，再保存配置；新保存方式用于下一段录制'}, status=409)
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
        if runtime.ros and runtime.ros.ownership.active:
            raise web.HTTPConflict(text='请退出人工干预会话后再回零')
        source = store.value.get("ros", {}).get("command_mux", {}).get(
            "source", "vr"
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
            raise web.HTTPConflict(text='人工干预会话正在管理控制来源')

        proposed = copy.deepcopy(store.value)
        proposed["ros"]["command_mux"]["source"] = source
        saved = store.save(proposed)
        runtime.config["ros"]["command_mux"]["source"] = source
        accepted = runtime.ros.set_control_source(source)
        runtime.ros.publish(
            "/control/teleop/enabled",
            "std_msgs/msg/Bool",
            {"data": source == "vr"},
        )
        runtime.emit(
            envelope(
                "control_source",
                "middleware",
                {"source": source},
            ),
            ["websocket"],
        )
        return web.json_response(
            {"accepted": accepted, "source": source},
            status=202 if accepted else 429,
        )

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
                    elif data.get("type") == "simulation_viewer":
                        if data.get("data", {}).get("enabled", True):
                            simulation.set_viewer(ws, True, data.get("data", {}).get("render_mode", "native"))
                        else:
                            simulation.set_viewer(ws, False)
                        await ws.send_json({"type": "simulation_viewer_ack", "data": {"registered": ws in simulation.viewers}})
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
            import traceback
            tb = traceback.format_exc()
            print(f"[WebRTC] Offer failed for {peer_ip}: {exc}\n{tb}", flush=True)
            runtime._log("error", f"WebRTC offer failed for {peer_ip}: {exc}\n{tb}")
            raise web.HTTPServiceUnavailable(text=f"WebRTC offer failed: {exc}") from exc

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
        frame = camera.latest()
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
        _camera_id, camera = camera_for_request(_request)
        if camera is None:
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
                frame = camera.latest()
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
        if runtime.recording_ros is not None:
            try:
                await asyncio.to_thread(runtime.recording_ros.prepare_recording)
                # Frequency checks need more than one sample. Keep this work off
                # the WebRTC/media path while the recorder warms up.
                await asyncio.sleep(1.9)
            except (RuntimeError, TimeoutError) as exc:
                return web.json_response(
                    {"ok": False, "ready": False, "reason": str(exc), "issues": []},
                    status=503,
                )
        subs = store.value.get("ros", {}).get("subscriptions", [])
        recording_topics = [
            item for item in subs
            if item.get("enabled", True) and "record" in item.get("outputs", [])
        ]
        topic_health = runtime.topic_health()

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
        if runtime.recording_ros is not None:
            try:
                await asyncio.to_thread(runtime.recording_ros.prepare_recording)
            except (RuntimeError, TimeoutError) as exc:
                return web.json_response({"ok": False, "reason": str(exc)}, status=503)
        if runtime.platform is None and runtime.upload_error:
            if (data.get('selection') or {}).get('mode', 'local') != 'local':
                return web.json_response({'ok': False, 'code': 'PLATFORM_UNAVAILABLE',
                    'reason': '平台模块尚未安装，可暂不关联平台任务录制到本机'}, status=409)
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

            topic_health = runtime.topic_health()
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

        try:
            path = await asyncio.to_thread(
                runtime.recorder.start,
                filename,
                runtime.session_profile_metadata,
                *([data.get("selection")] if runtime.platform is not None else []),
            )
        except (ValueError, RuntimeError, FileExistsError, ImportError) as exc:
            return web.json_response({"ok": False, "reason": str(exc)}, status=400)
        except Exception as exc:
            failure = _upload_failure(exc)
            if failure is None:
                raise
            return web.json_response(failure, status=_upload_failure_status(failure["code"]))
        status = runtime.recorder.status()
        if not status.get("recording") or status.get("path") != path:
            return web.json_response({"ok": True, "recording": False, "path": path, "replayed": True})
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

    async def download_recording(request: web.Request) -> web.StreamResponse:
        from .recording_datasets import resolve_dataset, archive_dataset
        from urllib.parse import quote
        identifier = request.match_info["filename"]
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder not available")
        try:
            target = resolve_dataset(runtime.recorder.directory, identifier)
        except (ValueError, FileNotFoundError) as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        if target.is_file():
            if runtime.recorder.is_recording() and runtime.recorder.path == target:
                raise web.HTTPConflict(text="recording is still being written")
            return web.FileResponse(target, headers={
                "Content-Disposition": f"attachment; filename*=UTF-8''{quote(target.name)}",
                "Content-Type": "application/octet-stream",
            })
        try:
            archive = await asyncio.to_thread(archive_dataset, target)
        except ValueError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        try:
            response = web.StreamResponse(headers={
                "Content-Disposition": f"attachment; filename*=UTF-8''{quote(target.name + '.zip')}",
                "Content-Type": "application/zip",
                "Content-Length": str(archive.stat().st_size),
            })
            await response.prepare(request)
            with archive.open('rb') as handle:
                while chunk := await asyncio.to_thread(handle.read, 256 * 1024):
                    await response.write(chunk)
            await response.write_eof()
            return response
        finally:
            archive.unlink(missing_ok=True)

    async def get_dataset_video(request: web.Request) -> web.FileResponse:
        from .recording_datasets import resolve_dataset, dataset_video
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder not available")
        identifier = request.match_info["filename"]
        if not identifier.startswith(("lerobot:", "bundle:")):
            raise web.HTTPNotFound(text="not a LeRobot dataset")
        try:
            target = resolve_dataset(runtime.recorder.directory, identifier)
            video = dataset_video(target, request.query.get("path", ""))
        except (ValueError, FileNotFoundError) as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        return web.FileResponse(video, headers={"Content-Type": "video/mp4"})

    async def delete_recording(request: web.Request) -> web.Response:
        filename = request.match_info["filename"]
        if runtime.recorder is None:
            raise web.HTTPServiceUnavailable(text="recorder not available")
        try:
            await asyncio.to_thread(runtime.recorder.delete_recording, filename)
            return web.json_response({"ok": True, "deleted": filename})
        except FileNotFoundError as exc:
            raise web.HTTPNotFound(text=str(exc)) from exc
        except ValueError as exc:
            raise web.HTTPBadRequest(text=str(exc)) from exc
        except Exception as exc:
            failure = _upload_failure(exc)
            if failure is None:
                raise
            return web.json_response(failure, status=_upload_failure_status(failure["code"]))

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
                failure = _upload_failure(exc)
                failed.append({"filename": fname, "error": str(exc),
                               **({"code": failure["code"], "reason": failure["reason"]}
                                  if failure else {})})
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

    async def stream_simulation_frames():
        last_frame = None
        while True:
            await asyncio.sleep(1.0 / 30.0)
            frame = simulation.latest_frame
            if frame is last_frame or frame is None or simulation.state != 'running':
                continue
            last_frame = frame
            event = {'type': 'simulation_frame', 'data': {**frame, 'profile_id': simulation.profile_id}}
            for viewer in tuple(simulation.viewers):
                try:
                    await asyncio.wait_for(viewer.send_json(event), timeout=0.5)
                except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                    simulation.set_viewer(viewer, False)
                    try:
                        await asyncio.wait_for(viewer.close(), timeout=0.2)
                    except (ConnectionError, RuntimeError, asyncio.TimeoutError):
                        pass

    viewer_sender = None

    async def startup(_app: web.Application) -> None:
        nonlocal viewer_sender
        await runtime.start()
        viewer_sender = asyncio.create_task(stream_simulation_frames())

    async def shutdown(_app: web.Application) -> None:
        if viewer_sender is not None:
            viewer_sender.cancel()
            await asyncio.gather(viewer_sender, return_exceptions=True)
        for ws in tuple(runtime.websockets):
            await ws.close(code=1001, message=b"server shutdown")
        async with simulation.lock:
            await simulation.stop()
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
    register_profile_routes(app, store, runtime, profiles)
    app.router.add_get("/api/ros/topics", get_topics)
    app.router.add_post("/api/ros/publish", publish)
    app.router.add_post("/api/safety/stop", emergency_stop)
    app.router.add_post("/api/safety/resume", safety_resume)
    app.router.add_post("/api/teleop/home", teleop_home)
    app.router.add_post("/api/teleop/source", set_control_source)
    try:
        from .upload_api import register_upload_routes

        class UploadProxy:
            def __getattr__(self, key):
                if runtime.uploads is None:
                    raise web.HTTPServiceUnavailable(
                        text=json.dumps({"enabled": False, "state": "unavailable",
                                         "error": runtime.upload_error or "upload service unavailable"}),
                        content_type="application/json",
                    )
                return getattr(runtime.uploads, key)

        async def update_upload_settings(body):
            from humanoid_upload.common import UploadError
            from .upload_service import UploadService
            if set(body) != {"enabled", "auto_enqueue"} or any(type(v) is not bool for v in body.values()):
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
                    runtime.config['uploads'] = copy.deepcopy(saved['uploads'])
                    platform = runtime.platform
                    with platform._lock:
                        platform.state.update(worker_enabled=body['enabled'], auto_upload=body['auto_enqueue'])
                        platform.save()
                    if body['enabled']: platform.upload.start()
                    else: await asyncio.to_thread(platform.upload.stop)
                    return {**platform.upload.status(), **body}
                upload_config = copy.deepcopy(proposed["uploads"])
                queue_path = Path(upload_config["queue_directory"]).expanduser()
                upload_config["queue_directory"] = str((store.path.parent / queue_path).resolve())
                candidate = UploadService(upload_config,
                    artifact_state_dir=runtime.recorder.artifact_state_dir,
                    data_directory=runtime.recorder.directory)
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
                runtime.recorder.binding_snapshot = (candidate.binding_snapshot
                    if body["enabled"] and body["auto_enqueue"] else None)
                candidate.start()
                return candidate.status()

        register_upload_routes(app, UploadProxy(), update_settings=update_upload_settings)
    except ImportError:
        async def upload_unavailable(request):
            return web.json_response(
                {"enabled": False, "state": "unavailable", "error": "upload module is not installed"},
                status=503,
            )
        app.router.add_get("/api/uploads", upload_unavailable)
        app.router.add_post("/api/uploads/{operation}", upload_unavailable)
    app.router.add_post("/api/recording/precheck", precheck_recording)
    app.router.add_get("/api/recording/precheck", precheck_recording)
    app.router.add_post("/api/recording/start", start_recording)
    app.router.add_post("/api/recording/stop", stop_recording)
    from .platform_api import register_platform_routes
    register_platform_routes(app, runtime)
    app.router.add_get("/api/recording/status", get_recording_status)
    app.router.add_get("/api/recordings", get_recordings)
    app.router.add_post("/api/recordings/upload", import_mcap_file)
    app.router.add_get("/api/recordings/{filename}/download", download_recording)
    app.router.add_get("/api/recordings/{filename}/video", get_dataset_video)
    app.router.add_delete("/api/recordings/{filename}", delete_recording)
    app.router.add_post("/api/recordings/batch-delete", batch_delete_recordings)
    app.router.add_post("/api/replay/start", start_replay)
    app.router.add_post("/api/replay/pause", pause_replay)
    app.router.add_post("/api/replay/resume", resume_replay)
    app.router.add_post("/api/replay/stop", stop_replay)
    app.router.add_get("/api/replay/status", get_replay_status)
    app.router.add_get("/ws", websocket)
    app.router.add_post("/api/webrtc/offer", webrtc_offer)
    app.router.add_post("/api/webrtc/offer/{camera_id}", webrtc_offer)
    app.router.add_post("/offer", webrtc_offer)
    app.router.add_post("/offer/{camera_id}", webrtc_offer)
    app.router.add_get("/api/camera/status", get_camera_status)
    app.router.add_get("/api/camera/status/{camera_id}", get_camera_status)
    app.router.add_get("/api/camera/snapshot", get_camera_snapshot)
    app.router.add_get("/api/camera/snapshot/{camera_id}", get_camera_snapshot)
    app.router.add_get("/api/camera/stream", get_camera_stream)
    app.router.add_get("/api/camera/stream/{camera_id}", get_camera_stream)
    app.router.add_options("/{tail:.*}", options)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(shutdown)
    return app
