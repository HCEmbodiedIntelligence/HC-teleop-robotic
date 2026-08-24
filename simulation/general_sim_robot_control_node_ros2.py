import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import time
import threading
import yaml
import numpy as np
import pybullet as p
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import argparse

from io_teleop_robot_utils.robot_module import AssembledRobot

from io_teleop_robot_utils.utils import (
    debug_draw_pose,
    multiply_transforms,
    pose_msg_to_list,
)


class SimRobotController(Node, AssembledRobot):
    def __init__(self, config_path, init_debug=True, headless=False):
        # Initialize ROS2 node
        Node.__init__(self, "sim_robot_controller_node")

        # Connect to PyBullet
        client_id = p.connect(p.DIRECT if headless else p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraPitch=-30,
            cameraYaw=180,
            cameraTargetPosition=[0, 0, 0.6],
        )

        # Start simulation
        def physics_sim():
            # p.setRealTimeSimulation(1)
            while rclpy.ok():
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

        physics_sim_thread = threading.Thread(target=physics_sim)
        physics_sim_thread.start()
        self.physics_clent_id = client_id
        config_file = os.path.abspath(os.path.expanduser(config_path))
        if not os.path.isfile(config_file):
            raise FileNotFoundError(f"simulation profile does not exist: {config_file}")
        self.robot_description_path = os.path.dirname(config_file)
        with open(config_file, encoding="utf-8") as stream:
            self.configs = yaml.safe_load(stream)
        self.wo_controller = False
        robot_urdf_path = os.path.abspath(
            os.path.join(self.robot_description_path, self.configs["urdf_path"])
        )
        with_ee_constraint = False
        
        # Initialize the AssembledRobot
        AssembledRobot.__init__(
            self,
            robot_urdf_path,
            self.configs,
            init_debug,
            with_ee_constraint,
            self.physics_clent_id,
        )
        self.reset_home()
        # Keep persistent IDs for every visual frame. The old implementation
        # drew the flange frames only once here, so they remained at the home
        # pose after the arm moved and looked like unrelated floating markers.
        self.base_frame_ids = debug_draw_pose(
            self.base.pose,
            line_width=2.0,
            line_length=0.16,
            physics_client_id=self.physics_clent_id,
        )
        self.actual_ee_frame_ids = [
            debug_draw_pose(
                arm.abs_ee_pose,
                line_width=3.0,
                line_length=0.18,
                physics_client_id=self.physics_clent_id,
            )
            for arm in self.arms
        ]
        self.target_ee_frame_ids = [None] * len(self.arms)
        
        # ROS2 publishers and subscribers
        self.joint_state_pub = self.create_publisher(
            JointState, f"/io_teleop/joint_states", 1
        )
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            f"/io_teleop/joint_cmd",
            self.joint_cmd_callback,
            1,
        )
        self.target_finger_joint_sub = self.create_subscription(
            JointState,
            f"/io_teleop/target_finger_joints",
            self.target_finger_joints_callback,
            1,
        )
        self.joint_cmd_from_vr_sub = self.create_subscription(
            JointState,
            "/io_teleop/target_joint_from_vr",
            self.joint_cmd_from_vr_callback,
            1,
        )
        self.gripper_status_sub = self.create_subscription(
            JointState,
            "/io_teleop/target_gripper_status",
            self.gripper_status_callback,
            1,
        )
        if "fixed_link" not in self.configs:
            self.target_base_move_sub = self.create_subscription(
                Float64MultiArray,
                f"/io_teleop/target_base_move",
                self.target_base_move_callback,
                1,
            )
        self.target_ee_sub = self.create_subscription(
            PoseArray,
            f"/io_teleop/target_ee_poses",
            self.target_ee_callback,
            1,
        )
        
        # Timer for publishing joint states
        self.joint_state_pub_rate = 100
        self.timer = self.create_timer(1.0 / 100, self.update_joint_state)
        
        # Service server
        self.reset_service_server = self.create_service(
            Trigger, "io_teleop_reset_robot", self.reset_service_callback
        )

    def reset_service_callback(self, request, response):
        print(
            "===========================Reset robot to home request received==========================="
        )
        self.reset_home()
        time.sleep(1)
        response.success = True
        response.message = "Service successfully triggered!"
        return response

    def update_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] == p.JOINT_FIXED:
                continue
            joint_state.name.append(joint_info[1].decode("utf-8"))
            joint_state.position.append(p.getJointState(self.robot_id, i)[0])
        self.joint_state_pub.publish(joint_state)
        self.base_frame_ids = debug_draw_pose(
            self.base.pose,
            self.base_frame_ids,
            line_width=2.0,
            line_length=0.16,
            physics_client_id=self.physics_clent_id,
        )
        for index, arm in enumerate(self.arms):
            self.actual_ee_frame_ids[index] = debug_draw_pose(
                arm.abs_ee_pose,
                self.actual_ee_frame_ids[index],
                line_width=3.0,
                line_length=0.18,
                physics_client_id=self.physics_clent_id,
            )

    def target_finger_joints_callback(self, msg):
        joint_pos = msg.position
        joint_names = msg.name
        for pos, name in zip(joint_pos, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.reset_j([joint_id], [pos])

    def joint_cmd_callback(self, msg):
        joint_pos = msg.position
        joint_names = msg.name
        for pos, name in zip(joint_pos, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.reset_j([joint_id], [pos])

    def joint_cmd_from_vr_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        for pos, name in zip(joint_positions, joint_names):
            joint_id = self.joint_name2id_dict[name]
            self.reset_j([joint_id], [pos])

    def gripper_status_callback(self, msg):
        gripper_status = msg.position
        for i, gripper in enumerate(self.grippers):
            gripper.reset(gripper_status[i])

    def target_ee_callback(self, msg):
        if "controller_indices" in self.configs:
            for i in range(len(self.arms)):
                pose = msg.poses[i]
                id = self.configs["controller_indices"]["base"][i]
                pose_list = pose_msg_to_list(pose)
                cmd_base_pose = (
                    p.getLinkState(self.robot_id, id)[4:6]
                    if id != -1
                    else self.base.pose
                )
                target_pose = multiply_transforms(cmd_base_pose, pose_list)
                if self.target_ee_frame_ids[i] is None:
                    self.target_ee_frame_ids[i] = debug_draw_pose(
                        target_pose,
                        line_width=5.0,
                        line_length=0.3,
                        physics_client_id=self.physics_clent_id,
                    )
                else:
                    self.target_ee_frame_ids[i] = debug_draw_pose(
                        target_pose,
                        self.target_ee_frame_ids[i],
                        line_width=5.0,
                        line_length=0.3,
                        physics_client_id=self.physics_clent_id,
                    )
        else:
            time.sleep(0.001)

    def target_base_move_callback(self, msg):
        delta_pos = msg.data[1]
        delta_yaw = msg.data[0]
        tar_pose = multiply_transforms(
            self.base.pose,
            [[0, delta_pos, 0], p.getQuaternionFromEuler([0, 0, delta_yaw])],
        )
        self.base.reset_base(tar_pose)

    def reset_home(self):
        for i, arm in enumerate(self.arms):
            self.reset_j(arm.arm_joint_index, arm.home_j_pos)

        if hasattr(self, "grippers"):
            for i, gripper in enumerate(self.grippers):
                gripper.reset(0)
                
        if hasattr(self, "head"):
            for i, pos in zip(self.head.head_joint_index, self.head.home_j_pos):
                if i != -1:
                    self.reset_j([i], [pos])

        if hasattr(self, "dorsal"):
            self.reset_j(
                [self.dorsal.dorsal_lift_joint_index], [self.dorsal.home_j_pos]
            )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--profile",
        required=True,
        help="Path to the imported robot profile directory or vr_configs.yml",
    )
    parser.add_argument(
        "--init_debug",
        action="store_true",
        help="Whether to use debug mode or not.",
    )
    parser.add_argument("--headless", action="store_true")

    arguments, ros_arguments = parser.parse_known_args(args)
    rclpy.init(args=ros_arguments)
    profile = os.path.abspath(os.path.expanduser(arguments.profile))
    config_path = (
        os.path.join(profile, "vr_configs.yml") if os.path.isdir(profile) else profile
    )
    
    robot = None
    try:
        robot = SimRobotController(
            config_path,
            init_debug=arguments.init_debug,
            headless=arguments.headless,
        )
        rclpy.spin(robot)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if robot is not None:
            robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
