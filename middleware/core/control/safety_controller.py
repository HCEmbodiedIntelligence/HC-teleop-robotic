"""安全急停、恢复授权、VR 超时故障链与头显状态同步控制器。"""
from __future__ import annotations

import asyncio
from contextlib import nullcontext
import uuid
from typing import Any

from ..protocol import envelope


class SafetyController:
    """集中管理安全停机/恢复状态机，避免运行时组合根混入复杂安全细节。"""

    def __init__(self, runtime: Any) -> None:
        self._runtime = runtime

    @staticmethod
    def send_snapshot(runtime: Any) -> None:
        """修复丢失的 UDP 安全事件或头显重连状态，不改变执行器使能状态。"""
        if runtime.ros is None or runtime.vr is None:
            return
        status = runtime.ros.status()
        if status.get("state") != "running":
            return  # 未知状态绝不能清除头显急停锁存。
        enabled = status.get("command_mux", {}).get("output_enabled")
        if not isinstance(enabled, bool):
            return
        ownership = status.get("ownership")
        ownership_state = ownership.get("state") if isinstance(ownership, dict) else ""
        if not enabled and ownership_state in {"TAKEOVER", "RETURNING"}:
            return  # 正常 DAgger 交接撤销旧控制源，不视为急停。
        ownership_reason = (
            str(ownership.get("reason") or "")
            if isinstance(ownership, dict) and ownership_state in {"HOLD", "FAULT", "ESTOP"}
            else ""
        )
        runtime.vr.send_event(
            envelope(
                "safety_resume" if enabled else "safety_stop",
                "middleware",
                {
                    "reason": (
                        "middleware state synchronization"
                        if enabled
                        else (
                            ownership_reason
                            or runtime._safety_reason
                            or "command output disabled; explicit resume required"
                        )
                    ),
                    "state_snapshot": True,
                },
            )
        )

    @staticmethod
    async def sync_loop(runtime: Any) -> None:
        while True:
            runtime._send_safety_snapshot()
            await asyncio.sleep(0.5)

    @staticmethod
    def handle_vr_timeout(runtime: Any, reason: str) -> None:
        # 保留原始检测原因，即使后续 watchdog 或输出关闭通知上报不同的最终停机消息。
        cause = envelope("vr_input_fault", "vr_gateway", {"reason": reason})
        cause["event_id"] = uuid.uuid4().hex
        diagnostics = getattr(runtime, "diagnostics", None)
        if diagnostics is not None:
            diagnostics.emit(cause)
        if runtime.ros and getattr(runtime.ros, "ownership", None) and runtime.ros.ownership.active:
            # 头显可用性仅约束人工接管，不影响模型推理或已授权的程序输出；人工期掉线仍触发故障。
            runtime.ros.ownership.input_lost(reason, cause_event_id=cause["event_id"])
            return
        source = (
            runtime.ros.status().get("command_mux", {}).get("source")
            if runtime.ros
            else None
        )
        if source is None:
            source = runtime.config.get("ros", {}).get("command_mux", {}).get("source", "vr")
        if source == "vr":
            runtime._on_safety_event(reason, cause_event_id=cause["event_id"])
        else:
            # VR 链路状态不能撤销用户显式选择的其他控制源。
            if runtime.ros:
                runtime.ros.publish("/control/teleop/enabled", "std_msgs/msg/Bool", {"data": False})
            runtime.emit(
                envelope("vr_timeout", "middleware", {"reason": reason}),
                ["websocket", "udp"],
            )

    @staticmethod
    def trigger_stop(
        runtime: Any,
        reason: str,
        *,
        force: bool = False,
        cause_event_id: str | None = None,
    ) -> None:
        with getattr(runtime, "_safety_event_lock", nullcontext()):
            runtime._safety_event_generation = getattr(runtime, "_safety_event_generation", 0) + 1
            runtime._safety_reason = reason
            if runtime.ros is not None and (force or runtime.config["safety"].get("enabled", True)):
                runtime.ros.emergency_stop(runtime.config["safety"]["stop_topic"], reason)
            cause = envelope("safety_trigger", "middleware", {"reason": reason})
            if cause_event_id:
                cause["payload"]["cause_event_id"] = cause_event_id
            cause["event_id"] = uuid.uuid4().hex
            runtime.emit(cause, [])
            runtime.emit(
                envelope(
                    "safety_stop",
                    "middleware",
                    {"reason": reason, "cause_event_id": cause["event_id"]},
                ),
                ["websocket", "udp"],
            )

    @staticmethod
    def resume(runtime: Any, reason: str, *, expected_fence: Any = None) -> bool:
        if getattr(runtime, "_stopping", False) or runtime.ros is None:
            return False
        control_runtime = getattr(runtime, "control_runtime", None)
        if (
            control_runtime is not None
            and control_runtime.enabled
            and control_runtime.status()["state"] != "running"
        ):
            runtime._on_safety_event(
                "control configuration is not ready; resume rejected", force=True
            )
            return False
        isolated = getattr(runtime.ros, "isolated_control_process", False) is True
        safety_generation = getattr(runtime, "_safety_event_generation", 0)
        resumed_fence = None
        if isolated and runtime.ros.status().get("state") != "running":
            runtime._log("warning", "control core is not running; explicit resume rejected")
            return False
        if isolated and expected_fence is None:
            expected_fence = runtime.ros.authorization_fence()
        if (
            runtime.ros is not None
            and getattr(runtime.ros, "ownership", None)
            and runtime.ros.ownership.active
        ):
            try:
                if isolated:
                    runtime.ros.recover_intervention(expected_fence=expected_fence)
                else:
                    runtime.ros.recover_intervention()
            except (ValueError, RuntimeError) as exc:
                runtime._log("warning", str(exc))
                return False
            return True
        if runtime.ros is not None:
            stop_topic = runtime.config["safety"].get("stop_topic", "/control/emergency_stop")
            if isolated:
                try:
                    result = runtime.ros.resume_output(stop_topic, expected_fence=expected_fence)
                    if isinstance(result, dict):
                        resumed_fence = result.get("authorization_fence")
                except (ValueError, RuntimeError) as exc:
                    runtime._log("warning", str(exc))
                    return False
            else:
                runtime.ros.set_command_output_enabled(True)
                runtime.ros.publish(stop_topic, "std_msgs/msg/Bool", {"data": False})
                source = runtime.config.get("ros", {}).get("command_mux", {}).get("source", "vr")
                runtime.ros.publish(
                    "/control/teleop/enabled", "std_msgs/msg/Bool", {"data": source == "vr"}
                )

        def notify_resumed() -> None:
            runtime._safety_reason = ""
            if runtime.player is not None:
                runtime.player.acknowledge_reset()
            runtime.emit(
                envelope("safety_resume", "middleware", {"reason": reason}),
                ["websocket", "udp"],
            )

        if isolated:
            # 较慢的 RPC 响应不得覆盖在其传输期间新到达的急停原因与 UI 事件。
            with runtime._safety_event_lock:
                if (
                    safety_generation != runtime._safety_event_generation
                    or (
                        resumed_fence is not None
                        and tuple(resumed_fence) != runtime.ros.authorization_fence()
                    )
                ):
                    return False
                notify_resumed()
        else:
            notify_resumed()
        return True

    @staticmethod
    def dispatch_resume(runtime: Any, reason: str) -> None:
        if getattr(runtime.ros, "isolated_control_process", False) is not True:
            runtime._on_safety_resume(reason)
            return
        fence = runtime.ros.authorization_fence()
        if runtime._stopping or runtime._control_commands is None:
            return
        if not runtime._control_command_slots.acquire(blocking=False):
            runtime._log("warning", "control command queue is full; press A again to resume")
            return
        try:
            future = runtime._control_commands.submit(
                runtime._on_safety_resume, reason, expected_fence=fence
            )
        except RuntimeError:
            runtime._control_command_slots.release()
            return
        future.add_done_callback(lambda _: runtime._control_command_slots.release())

