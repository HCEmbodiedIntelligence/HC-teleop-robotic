"""录制控制业务逻辑：统一话题预检、状态广播、VR 按键录制与发布点采集挂载。"""
from __future__ import annotations

import json
import time
from typing import Any

from ..protocol import envelope
from ..uploads.upload_integration import upload_failure


def check_recording_topics(
    subscriptions: list[dict[str, Any]],
    topic_health: dict[str, Any],
    *,
    detailed: bool = False,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """统一检查已勾选录制话题的健康度与频率达标情况。

    返回 ``(recording_topics, checked, issues)``，供 ``/api/recording/precheck``
    与 ``/api/recording/start`` 共同复用，避免两套检查逻辑分叉。
    """
    from ..control.ros_bridge import DEFAULT_TOPIC_STANDARDS

    recording_topics = [
        item
        for item in subscriptions
        if item.get("enabled", True) and "record" in item.get("outputs", [])
    ]
    checked: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []

    for item in recording_topics:
        topic = item["topic"]
        info = topic_health.get(topic)
        std = DEFAULT_TOPIC_STANDARDS.get(topic, {"target_hz": 10.0, "min_hz": 1.0})

        if info is None:
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
        state = item_stat.get("state", "no_data")

        if detailed:
            if state == "no_data":
                issues.append(
                    {
                        "topic": topic,
                        "state": "no_data",
                        "hz": 0.0,
                        "min_hz": item_stat["min_hz"],
                        "reason": f"{topic}: 未检测到消息发布 (0 Hz，标准: ≥ {item_stat['min_hz']:.1f} Hz)",
                    }
                )
            elif state == "low_rate":
                issues.append(
                    {
                        "topic": topic,
                        "state": "low_rate",
                        "hz": item_stat["hz"],
                        "min_hz": item_stat["min_hz"],
                        "reason": f"{topic}: 消息频率偏低 ({item_stat['hz']:.1f} Hz < 标准 {item_stat['min_hz']:.1f} Hz)",
                    }
                )
        elif state != "ok":
            hz = float(info.get("hz", 0.0)) if info else 0.0
            min_hz = float(item.get("min_hz", 0) or (info.get("min_hz") if info else std["min_hz"]))
            issues.append(
                {
                    "topic": topic,
                    "state": state,
                    "hz": hz,
                    "min_hz": min_hz,
                    "reason": (
                        f"{topic}: 未检测到消息发布 (0 Hz)"
                        if hz == 0
                        else f"{topic}: 频率不足 ({hz:.1f} Hz < 标准 {min_hz:.1f} Hz)"
                    ),
                }
            )

    return recording_topics, checked, issues


def configure_publication_capture(runtime: Any, recorder: Any) -> None:
    """将发布点采集钩子挂载到基础录制写入器（而非 TaskRecorder 代理）。"""
    action_max_hz = next(
        (
            float(item.get("max_hz", 0))
            for item in runtime.config["ros"].get("subscriptions", [])
            if item.get("topic") == "/action/joint_states"
        ),
        0.0,
    )
    action_capture = bool(
        runtime.config["ros"].get("legacy_control_profile") and action_max_hz <= 0
    )
    runtime.config["ros"]["record_action_at_publication"] = action_capture
    from .recording_action_labels import action_label_policy

    recorder.action_label_policy = (
        action_label_policy(runtime.config["ros"]) if action_capture else None
    )
    if action_capture:
        selected = lambda: recorder.records_topic(
            "/action/joint_states", "sensor_msgs/msg/JointState"
        )
        factory = getattr(runtime.ros, "create_publication_capture", None)
        if callable(factory):
            recorder.publication_capture = factory(recorder, selected=selected)
        else:
            from .publication_capture import PublicationCapture

            recorder.publication_capture = PublicationCapture(recorder, selected=selected)
            runtime.ros._publication_capture = recorder.publication_capture


class RecordingController:
    """统一收口 VR 手柄与 HTTP API 共享的录制控制、状态发布与命令派发逻辑。"""

    def __init__(self, runtime: Any) -> None:
        self._runtime = runtime

    def dispatch(self, operation: Any) -> None:
        if self._runtime._record_commands is None or self._runtime._stopping:
            # 绝不在位姿接收线程上执行写入器初始化或 IPC 栅栏。
            return
        if not self._runtime._record_command_slots.acquire(blocking=False):
            self._runtime._log("error", "recording command queue is full")
            return
        try:
            future = self._runtime._record_commands.submit(operation)
        except Exception:
            self._runtime._record_command_slots.release()
            raise
        future.add_done_callback(lambda _: self._runtime._record_command_slots.release())

    def publish_state(self, payload: dict[str, Any], *, ensure_ascii: bool = True) -> None:
        if self._runtime.ros is not None:
            self._runtime.ros.publish(
                "/teleop/recording_state",
                "std_msgs/msg/String",
                {"data": json.dumps(payload, ensure_ascii=ensure_ascii)},
            )

    def notify_started(self, filename: str, path: Any) -> None:
        self.publish_state({"recording": True, "filename": filename})
        self._runtime.emit(
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

    def notify_stopped(self, stopped_status: Any) -> None:
        self.publish_state({"recording": False, "status": stopped_status})
        self._runtime.emit(
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

    def vr_start(self) -> None:
        recorder = self._runtime.recorder
        if recorder is None:
            return
        if getattr(recorder, "corrections", None):
            self._runtime._log("info", "DAgger 会话按所选范围录制，X/Y 普通录制已停用")
            return
        status = recorder.status()
        if status.get("recording", False):
            self._runtime._log("info", "VR requested record start, but already recording")
            self._runtime.emit(
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
            if self._runtime.platform is None and self._runtime.recording_ros is not None:
                self._runtime.recording_ros.prepare_recording()
            filename = f"teleop_{time.strftime('%Y%m%d_%H%M%S')}.mcap"
            path = recorder.start(filename, self._runtime.session_profile_metadata)
            self._runtime._log("info", f"Recording started by VR X button: {filename}")
            self.notify_started(filename, path)
        except Exception as exc:
            failure = upload_failure(exc)
            error_text = (
                f"{failure['code']}: {failure['reason']}" if failure else str(exc)
            )
            self._runtime._log("error", f"Failed to start recording on VR X button: {error_text}")
            self._runtime.emit(
                envelope(
                    "recording_error",
                    "middleware",
                    {
                        "recording": False,
                        "error": error_text,
                        "action": "start_failed",
                        **(
                            {"code": failure["code"], "reason": failure["reason"]}
                            if failure
                            else {}
                        ),
                    },
                ),
                ["websocket", "udp"],
            )

    def vr_stop(self) -> None:
        recorder = self._runtime.recorder
        if recorder is not None and getattr(recorder, "corrections", None):
            self._runtime._log("info", "纠正采集使用 Grip 自动分段，X/Y 不结束纠正会话")
            return
        if recorder is None:
            return
        status = recorder.status()
        if not status.get("recording", False):
            self._runtime._log("info", "VR requested record stop, but not currently recording")
            self._runtime.emit(
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
            stopped_status = recorder.stop()
            self._runtime._log("info", f"Recording stopped by VR Y button: {stopped_status}")
            self.notify_stopped(stopped_status)
        except Exception as exc:
            self._runtime._log("error", f"Failed to stop recording on VR Y button: {exc}")
            self._runtime.emit(
                envelope(
                    "recording_error",
                    "middleware",
                    {"error": str(exc), "action": "stop_failed"},
                ),
                ["websocket", "udp"],
            )

    def vr_mark(self) -> None:
        recorder = self._runtime.recorder
        if recorder is None:
            return
        try:
            result = recorder.mark_current_or_latest("vr_dual_thumbstick")
            filename = str(result.get("filename", ""))
            message = f"已标记录制文件: {filename}"
            self._runtime._log("info", message)
            self.publish_state(result, ensure_ascii=False)
            self._runtime.emit(
                envelope(
                    "recording_marked",
                    "middleware",
                    {**result, "action": "mark", "message": message},
                ),
                ["websocket", "udp"],
            )
        except Exception as exc:
            self._runtime._log("error", f"Failed to mark recording from VR: {exc}")
            self._runtime.emit(
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

