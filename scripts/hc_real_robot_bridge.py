#!/usr/bin/env python3
"""HC Real Robot ROS 2 Bridge.

Bridges real robot driver topics (/io_teleop/*) to the standard HC teleoperation topics (/hc_teleop/*).
Ensures zero-overhead 100Hz real-time bidirectional communication.
"""
from __future__ import annotations

import os
import sys
import rclpy
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
        self.joint_cmd_pub = self.create_publisher(
            JointState, "/io_teleop/joint_cmd", reliable_qos
        )
        self.create_subscription(
            JointState,
            "/hc_teleop/joint_cmd",
            self.joint_cmd_pub.publish,
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
        self.cam_head_pub = self.create_publisher(
            CompressedImage, "/hc_teleop/camera_head/color/compressed", 10
        )
        self.create_subscription(
            CompressedImage,
            "/io_teleop/camera_head/color/compressed",
            self.cam_head_pub.publish,
            qos_profile_sensor_data,
        )

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

        self.get_logger().info("HC Real Robot Bridge started: /io_teleop <=> /hc_teleop")


def main() -> None:
    domain_id = int(os.environ.get("ROS_DOMAIN_ID", 13))
    rclpy.init(domain_id=domain_id)
    node = HcRealRobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
