#!/usr/bin/env python3
"""HC Real Robot ROS 2 Bridge.

Bridges real robot driver topics (/io_teleop/*) to the standard HC teleoperation topics (/hc_teleop/*).
Ensures zero-overhead 100Hz real-time bidirectional communication.
"""
from __future__ import annotations

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
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, CompressedImage, Image
from std_msgs.msg import Float64MultiArray


class HcRealRobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("hc_real_robot_bridge")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 1. Joint States: /io_teleop/joint_states -> /hc_teleop/joint_states
        self.joint_state_pub = self.create_publisher(
            JointState, "/hc_teleop/joint_states", reliable_qos
        )
        self.create_subscription(
            JointState,
            "/io_teleop/joint_states",
            self.joint_state_pub.publish,
            qos_profile_sensor_data,
        )

        # 2. Joint Command: /hc_teleop/joint_cmd -> /io_teleop/joint_cmd
        # 方案 2：限制单拍最大位置增量 |Δq_max| <= 0.4° (0.007 rad)，叠加 12Hz 平滑滤波消除抖动
        self.joint_cmd_pub = self.create_publisher(
            JointState, "/io_teleop/joint_cmd", reliable_qos
        )
        self._last_cmd_pos: dict[str, float] = {}
        self._filtered_cmd_pos: dict[str, float] = {}
        self.MAX_DELTA_Q = 0.006981  # 0.4 deg (rad)

        def joint_cmd_cb(msg: JointState) -> None:
            out_msg = JointState()
            out_msg.header = msg.header
            out_msg.name = list(msg.name)
            out_positions = []
            alpha = 0.45  # 12Hz 一阶滤波
            for name, target_pos in zip(msg.name, msg.position):
                prev = self._last_cmd_pos.get(name, target_pos)
                delta = target_pos - prev
                if abs(delta) > self.MAX_DELTA_Q:
                    delta = math.copysign(self.MAX_DELTA_Q, delta)
                rate_limited_pos = prev + delta
                filt_prev = self._filtered_cmd_pos.get(name, rate_limited_pos)
                filt_pos = filt_prev + alpha * (rate_limited_pos - filt_prev)
                self._last_cmd_pos[name] = rate_limited_pos
                self._filtered_cmd_pos[name] = filt_pos
                out_positions.append(filt_pos)

            out_msg.position = out_positions
            if msg.velocity:
                out_msg.velocity = list(msg.velocity)
            if msg.effort:
                out_msg.effort = list(msg.effort)
            self.joint_cmd_pub.publish(out_msg)

        self.create_subscription(
            JointState,
            "/hc_teleop/joint_cmd",
            joint_cmd_cb,
            qos_profile_sensor_data,
        )

        # 3. Base Movement: /hc_teleop/target_base_move -> /io_teleop/target_base_move
        self.base_move_pub = self.create_publisher(
            Float64MultiArray, "/io_teleop/target_base_move", reliable_qos
        )
        self.create_subscription(
            Float64MultiArray,
            "/hc_teleop/target_base_move",
            self.base_move_pub.publish,
            qos_profile_sensor_data,
        )

        # 4. Grippers: /hc_teleop/joint_cmd_finger_* -> /io_teleop/joint_cmd_finger_*
        self.finger_l_pub = self.create_publisher(
            JointState, "/io_teleop/joint_cmd_finger_left", reliable_qos
        )
        self.create_subscription(
            JointState,
            "/hc_teleop/joint_cmd_finger_left",
            self.finger_l_pub.publish,
            qos_profile_sensor_data,
        )

        self.finger_r_pub = self.create_publisher(
            JointState, "/io_teleop/joint_cmd_finger_right", reliable_qos
        )
        self.create_subscription(
            JointState,
            "/hc_teleop/joint_cmd_finger_right",
            self.finger_r_pub.publish,
            qos_profile_sensor_data,
        )

        # 5. Hand feedback
        self.hand_state_pub = self.create_publisher(
            JointState, "/hc_teleop/hand_joint_states", reliable_qos
        )
        self.create_subscription(
            JointState,
            "/io_teleop/hand_joint_states",
            self.hand_state_pub.publish,
            qos_profile_sensor_data,
        )

        # 6. Cameras
        # 6.1 头部相机 (Head Camera): 优先直接转发 JPEG 压缩流，断开时自动 fallback 压缩 raw 图像
        self.cam_head_pub = self.create_publisher(
            CompressedImage, "/hc_teleop/camera_head/color/compressed", 10
        )
        self._last_head_compressed = 0.0

        def head_compressed_cb(msg: CompressedImage) -> None:
            self._last_head_compressed = time.monotonic()
            self.cam_head_pub.publish(msg)

        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_head/color/compressed",
            head_compressed_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_head/color",
            head_compressed_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            "/cameras/eye/color/compressed",
            head_compressed_cb,
            qos_profile_sensor_data,
        )
        try:
            import cv2
            import numpy as np

            def eye_raw_cb(msg: Image) -> None:
                if time.monotonic() - self._last_head_compressed < 0.5:
                    return
                try:
                    if msg.encoding in ("rgb8", "bgr8", "rgb", "bgr"):
                        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                        if "rgb" in msg.encoding:
                            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                        success, encoded = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if success:
                            cmsg = CompressedImage()
                            cmsg.header = msg.header
                            cmsg.format = "jpeg"
                            cmsg.data = encoded.tobytes()
                            self.cam_head_pub.publish(cmsg)
                except Exception:
                    pass

            self.create_subscription(
                Image,
                "/io_teleop/camera_head/color",
                eye_raw_cb,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                "/cameras/eye/color",
                eye_raw_cb,
                qos_profile_sensor_data,
            )
        except Exception:
            pass

        # 6.2 头顶/前置俯视相机 (Overhead Camera): 支持 /cameras/Bfront/color/compressed, /cameras/Bfront/color
        self.cam_overhead_pub = self.create_publisher(
            CompressedImage, "/hc_teleop/camera_overhead/color/compressed", 10
        )
        self._last_overhead_compressed = 0.0

        def overhead_compressed_cb(msg: CompressedImage) -> None:
            self._last_overhead_compressed = time.monotonic()
            self.cam_overhead_pub.publish(msg)

        self.create_subscription(
            CompressedImage,
            "/cameras/Bfront/color/compressed",
            overhead_compressed_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_overhead/color/compressed",
            overhead_compressed_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CompressedImage,
            "/cameras/overhead/color/compressed",
            overhead_compressed_cb,
            qos_profile_sensor_data,
        )
        try:
            import cv2
            import numpy as np

            def bfront_raw_cb(msg: Image) -> None:
                if time.monotonic() - self._last_overhead_compressed < 1.0:
                    return
                try:
                    if msg.encoding in ("rgb8", "bgr8", "rgb", "bgr"):
                        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                        if "rgb" in msg.encoding:
                            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                        success, encoded = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        if success:
                            cmsg = CompressedImage()
                            cmsg.header = msg.header
                            cmsg.format = "jpeg"
                            cmsg.data = encoded.tobytes()
                            self.cam_overhead_pub.publish(cmsg)
                except Exception:
                    pass

            self.create_subscription(
                Image,
                "/cameras/Bfront/color",
                bfront_raw_cb,
                qos_profile_sensor_data,
            )
        except Exception:
            pass

        # 6.3 双臂 D405 相机
        self.d405_l_pub = self.create_publisher(
            CompressedImage, "/hc_teleop/camera_d405_left/color/compressed", 10
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_d405_left/color/compressed",
            self.d405_l_pub.publish,
            qos_profile_sensor_data,
        )

        self.d405_r_pub = self.create_publisher(
            CompressedImage, "/hc_teleop/camera_d405_right/color/compressed", 10
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_d405_right/color/compressed",
            self.d405_r_pub.publish,
            qos_profile_sensor_data,
        )

        # 6.4 支持环境变量指定的自定义相机话题透传
        custom_cam_in = os.environ.get("HC_CUSTOM_CAMERA_INPUT", "").strip()
        custom_cam_out = os.environ.get("HC_CUSTOM_CAMERA_OUTPUT", "/hc_teleop/camera_custom/color/compressed").strip()
        if custom_cam_in:
            self.custom_cam_pub = self.create_publisher(CompressedImage, custom_cam_out, 10)
            self.create_subscription(
                CompressedImage,
                custom_cam_in,
                self.custom_cam_pub.publish,
                qos_profile_sensor_data,
            )

        # 7. VR Data bridge: /vrdata (JSON) -> /io_teleop/vr_data (io_msgs2/VrData)
        try:
            from io_msgs2.msg import VrData
            from std_msgs.msg import String
            import json

            self.vr_data_pub = self.create_publisher(
                VrData, "/io_teleop/vr_data", qos_profile_sensor_data
            )

            def vrdata_callback(msg: String) -> None:
                try:
                    payload = json.loads(msg.data)
                    vr_msg = VrData()
                    vr_msg.header.stamp = self.get_clock().now().to_msg()
                    inputs = payload.get("inputs", {})
                    left_input = inputs.get("left", {})
                    right_input = inputs.get("right", {})
                    vr_msg.l_index_axis = float(left_input.get("trigger", 0.0))
                    vr_msg.r_index_axis = float(right_input.get("trigger", 0.0))
                    vr_msg.l_index_trigger = 1 if vr_msg.l_index_axis > 0.5 else 0
                    vr_msg.r_index_trigger = 1 if vr_msg.r_index_axis > 0.5 else 0
                    vr_msg.l_hand_axis = float(left_input.get("grip", 0.0))
                    vr_msg.r_hand_axis = float(right_input.get("grip", 0.0))
                    vr_msg.l_hand_trigger = 1 if vr_msg.l_hand_axis > 0.5 else 0
                    vr_msg.r_hand_trigger = 1 if vr_msg.r_hand_axis > 0.5 else 0
                    left_primary = left_input.get("primary", [0.0, 0.0])
                    right_primary = right_input.get("primary", [0.0, 0.0])
                    vr_msg.l_thumb_stick_axis_x = float(left_primary[0]) if len(left_primary) > 0 else 0.0
                    vr_msg.l_thumb_stick_axis_y = float(left_primary[1]) if len(left_primary) > 1 else 0.0
                    vr_msg.r_thumb_stick_axis_x = float(right_primary[0]) if len(right_primary) > 0 else 0.0
                    vr_msg.r_thumb_stick_axis_y = float(right_primary[1]) if len(right_primary) > 1 else 0.0
                    poses = payload.get("poses", {})
                    for pose_key, pose_target in (("head", vr_msg.head_pose), ("left", vr_msg.left_pose), ("right", vr_msg.right_pose)):
                        p_data = poses.get(pose_key, {})
                        pos = p_data.get("position", [0.0, 0.0, 0.0])
                        ori = p_data.get("orientation", [0.0, 0.0, 0.0, 1.0])
                        if len(pos) >= 3:
                            pose_target.position.x = float(pos[0])
                            pose_target.position.y = float(pos[1])
                            pose_target.position.z = float(pos[2])
                        if len(ori) >= 4:
                            pose_target.orientation.x = float(ori[0])
                            pose_target.orientation.y = float(ori[1])
                            pose_target.orientation.z = float(ori[2])
                            pose_target.orientation.w = float(ori[3])
                    self.vr_data_pub.publish(vr_msg)
                except Exception:
                    pass

            self.create_subscription(String, "/vrdata", vrdata_callback, 10)
        except ImportError:
            pass

        self.get_logger().info("HC Real Robot Bridge started: /io_teleop <=> /hc_teleop")


def main() -> None:
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", 13))
    rclpy.init(domain_id=domain_id)
    node = None
    try:
        node = HcRealRobotBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
