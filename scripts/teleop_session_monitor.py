#!/usr/bin/env python3
"""HC-Teleop Session Operations & Status Logger.

Subscribes to teleoperation topics and writes structured, human-readable operational logs
to disk for real-time monitoring and debugging.
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:
    RCLError = Exception
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, CompressedImage, Image
from std_msgs.msg import Float64MultiArray, String


class TeleopSessionMonitor(Node):
    def __init__(self, log_path: str) -> None:
        super().__init__("teleop_session_monitor")
        self.log_file = open(log_path, "a", encoding="utf-8", buffering=1)
        self.last_status_str = ""
        self.last_r_clutch = False
        self.last_l_clutch = False
        self.last_r_finger = 0.0
        self.last_l_finger = 0.0
        self.last_base_moving = False
        self.last_recording_state = False

        # Camera monitoring
        self.cam_frames_count = 0
        self.last_cam_log_time = time.monotonic()
        self.last_cam_frames_sample = 0
        self.last_cam_topic = "none"

        self.log_entry("SYSTEM", f"=== 遥操作运行与操作监控已启动 (PID={os.getpid()}) ===")

        # 1. 遥操作控制器状态
        self.create_subscription(
            String,
            "/teleop/arm/status",
            self.on_arm_status,
            10,
        )

        # 2. 录制状态
        self.create_subscription(
            String,
            "/teleop/recording_state",
            self.on_recording_state,
            10,
        )

        # 3. 底盘速度指令
        self.create_subscription(
            Float64MultiArray,
            "/hc_teleop/target_base_move",
            self.on_base_cmd,
            qos_profile_sensor_data,
        )

        # 4. 灵巧手/手爪指令
        self.create_subscription(
            JointState,
            "/hc_teleop/joint_cmd_finger_right",
            self.on_r_finger_cmd,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/hc_teleop/joint_cmd_finger_left",
            self.on_l_finger_cmd,
            qos_profile_sensor_data,
        )

        # 5. 相机图像流监控
        self.create_subscription(
            CompressedImage,
            "/hc_teleop/camera_head/color/compressed",
            lambda msg: self.on_cam_frame("/hc_teleop/camera_head/color/compressed"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_head/color/compressed",
            lambda msg: self.on_cam_frame("/io_teleop/camera_head/color/compressed"),
            qos_profile_sensor_data,
        )

        # 周期性检查相机状态 (每 3 秒)
        self.create_timer(3.0, self.check_camera_health)

    def log_entry(self, category: str, message: str) -> None:
        now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"[{now_str}] [{category:<8}] {message}\n"
        self.log_file.write(line)
        self.log_file.flush()

    def on_cam_frame(self, topic: str) -> None:
        self.cam_frames_count += 1
        self.last_cam_topic = topic

    def check_camera_health(self) -> None:
        now = time.monotonic()
        dt = now - self.last_cam_log_time
        if dt > 0.5:
            delta_frames = self.cam_frames_count - self.last_cam_frames_sample
            fps = delta_frames / dt
            self.last_cam_log_time = now
            self.last_cam_frames_sample = self.cam_frames_count

            if fps > 0.5:
                self.log_entry("CAMERA", f"头部相机画面正常: {fps:.1f} FPS (源话题: {self.last_cam_topic}, 累计: {self.cam_frames_count}帧)")
            else:
                self.log_entry("CAMERA", f"⚠️ 警告: 未收到头部相机画面 (0.0 FPS, 累计: {self.cam_frames_count}帧)")

    def on_arm_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            arms = data.get("arms", {})
            r_active = bool(arms.get("right", {}).get("active", False))
            l_active = bool(arms.get("left", {}).get("active", False))
            homing = bool(data.get("homing", False))
            body_active = bool(data.get("body", {}).get("active", False))

            if r_active != self.last_r_clutch:
                self.last_r_clutch = r_active
                action = "握持 (进入双臂操控模式)" if r_active else "松开 (保持当前位姿)"
                self.log_entry("CLUTCH_R", f"右手离合(Grip) {action}")

            if body_active != self.last_l_clutch:
                self.last_l_clutch = body_active
                action = "握持 (进入头部转向与腰部控制模式)" if body_active else "松开 (腰部与底盘锁定)"
                self.log_entry("CLUTCH_L", f"左手离合(Grip) {action}")

            if homing:
                self.log_entry("HOMING", "机械臂正在执行平滑回零...")

        except Exception:
            pass

    def on_recording_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            evt_type = data.get("type", "")
            payload = data.get("payload", {})
            rec = bool(payload.get("recording", False))
            fn = payload.get("filename", "")

            if evt_type == "recording_started":
                self.last_recording_state = True
                self.log_entry("RECORD", f"▶ 数据集录制开始: {fn}")
            elif evt_type == "recording_stopped":
                self.last_recording_state = False
                dur = payload.get("status", {}).get("duration_seconds", 0.0)
                self.log_entry("RECORD", f"■ 数据集录制已停止并保存 (耗时 {dur:.1f}s)")
            elif evt_type == "recording_error":
                err = payload.get("error", "")
                self.log_entry("ERROR", f"✖ 数据集录制出错: {err}")
        except Exception:
            pass

    def on_base_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 3:
            yaw, fwd, lat = msg.data[0], msg.data[1], msg.data[2]
            is_moving = abs(yaw) > 0.001 or abs(fwd) > 0.001 or abs(lat) > 0.001
            if is_moving and not self.last_base_moving:
                self.last_base_moving = True
                self.log_entry("BASE", f"底盘开始移动: yaw={yaw:.2f} rad/s, fwd={fwd:.2f} m/s, lat={lat:.2f} m/s")
            elif not is_moving and self.last_base_moving:
                self.last_base_moving = False
                self.log_entry("BASE", "底盘停止移动")

    def on_r_finger_cmd(self, msg: JointState) -> None:
        if msg.position:
            avg = float(sum(msg.position) / len(msg.position))
            if abs(avg - self.last_r_finger) > 0.4:
                self.last_r_finger = avg
                state = "闭合抓取" if avg > 1.0 else "完全张开" if avg < 0.2 else f"开合度 {avg:.2f}"
                self.log_entry("HAND_R", f"右手食指扳机/灵巧手: {state}")

    def on_l_finger_cmd(self, msg: JointState) -> None:
        if msg.position:
            avg = float(sum(msg.position) / len(msg.position))
            if abs(avg - self.last_l_finger) > 0.4:
                self.last_l_finger = avg
                state = "闭合抓取" if avg > 1.0 else "完全张开" if avg < 0.2 else f"开合度 {avg:.2f}"
                self.log_entry("HAND_L", f"左手食指扳机/灵巧手: {state}")

    def destroy_node(self) -> bool:
        try:
            self.log_entry("SYSTEM", "=== 遥操作运行监控已安全停止 ===")
            self.log_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description="Teleop session monitor and logger")
    parser.add_argument("--log-file", required=True, help="Destination log file path")
    args = parser.parse_args()

    domain_id = int(os.environ.get("ROS_DOMAIN_ID", 13))
    rclpy.init(domain_id=domain_id)
    node = None
    try:
        node = TeleopSessionMonitor(args.log_file)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
