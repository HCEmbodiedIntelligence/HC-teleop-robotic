"""VR 位姿与手柄按键语义处理器。"""
from __future__ import annotations

import json
import time
from typing import Any

from ..protocol import envelope


class VrPoseHandler:
    """解码 VR 姿态与手柄按键边沿事件，协调安全恢复、录制触发、重放回传与 ROS 发布。"""

    def __init__(self, runtime: Any) -> None:
        self._runtime = runtime
        self.last_x_held = False
        self.last_y_held = False
        self.last_a_held = False
        self.last_both_sticks_held = False
        self.last_replay_status_sent_at = 0.0

    def handle(self, packet: Any, pose_dict: dict[str, Any] | None = None) -> None:
        left_input = getattr(packet, "left_input", None)
        right_input = getattr(packet, "right_input", None)
        left_held = int(getattr(left_input, "held_mask", 0))
        left_pressed = int(getattr(left_input, "pressed_mask", 0))
        right_held = int(getattr(right_input, "held_mask", 0))
        right_pressed = int(getattr(right_input, "pressed_mask", 0))

        self._handle_safety_button(right_held, right_pressed)
        self._handle_mark_gesture(left_held, left_pressed, right_held, right_pressed)
        self._sync_replay_status()
        if left_input is not None:
            self._handle_recording_buttons(left_held, left_pressed)
        self._publish_pose(packet, pose_dict)

    def _handle_safety_button(self, right_held: int, right_pressed: int) -> None:
        # 右手柄 A 键 (bit 0) 上升沿：非人工介入模式下解除软急停 / 恢复遥操作。
        a_held = bool(right_held & 1)
        a_down = bool(right_pressed & 1) or (a_held and not self.last_a_held)
        self.last_a_held = a_held

        ros = self._runtime.ros
        intervention_active = bool(
            ros is not None and getattr(ros, "ownership", None) and ros.ownership.active
        )
        if a_down and not intervention_active:
            self._runtime._dispatch_safety_resume("VR controller A button pressed")

    def _handle_mark_gesture(
        self,
        left_held: int,
        left_pressed: int,
        right_held: int,
        right_pressed: int,
    ) -> None:
        # 双摇杆下压 (bit 5 = 32) 上升沿：给当前或最近一条录制打标。
        stick_bit = 1 << 5
        both_sticks_held = bool(left_held & stick_bit) and bool(right_held & stick_bit)
        both_sticks_pressed = bool(left_pressed & stick_bit) and bool(right_pressed & stick_bit)
        mark_down = both_sticks_pressed or (
            both_sticks_held and not self.last_both_sticks_held
        )
        self.last_both_sticks_held = both_sticks_held
        if mark_down:
            self._runtime._dispatch_recording(self._runtime._handle_vr_record_mark)

    def _sync_replay_status(self) -> None:
        # UDP 可能丢包；重放期间或等待 A 键确认时每秒向 VR 回传一次状态快照。
        player = self._runtime.player
        vr = self._runtime.vr
        if player is None or vr is None:
            return
        replay = player.status()
        now = time.monotonic()
        if (replay.get("is_active") or replay.get("requires_reset")) and (
            now - self.last_replay_status_sent_at >= 1.0
        ):
            replay_message = (
                "重放结束；请按 A 恢复遥操作"
                if replay.get("requires_reset")
                else "正在重放"
            )
            vr.send_event(
                envelope(
                    "replay_status",
                    "middleware",
                    {**replay, "message": replay_message},
                )
            )
            self.last_replay_status_sent_at = now

    def _handle_recording_buttons(self, left_held: int, left_pressed: int) -> None:
        # 左手柄 X 键 (bit 0) 开始录制，Y 键 (bit 1) 停止录制。
        x_down = bool(left_pressed & 1) or (bool(left_held & 1) and not self.last_x_held)
        self.last_x_held = bool(left_held & 1)
        y_down = bool(left_pressed & 2) or (bool(left_held & 2) and not self.last_y_held)
        self.last_y_held = bool(left_held & 2)

        if x_down:
            self._runtime._dispatch_recording(self._runtime._handle_vr_record_start)
        elif y_down:
            self._runtime._dispatch_recording(self._runtime._handle_vr_record_stop)

    def _publish_pose(self, packet: Any, pose_dict: dict[str, Any] | None) -> None:
        ros = self._runtime.ros
        if ros is None or not self._runtime.config["vr"].get("publish_to_ros", True):
            return
        frame = pose_dict if pose_dict is not None else packet.as_dict()
        ros.publish(
            self._runtime.config["vr"]["data_topic"],
            "std_msgs/msg/String",
            {
                "data": json.dumps(
                    {**frame, "hc_received_ns": frame.get("hc_received_ns", time.time_ns())},
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
            },
        )

