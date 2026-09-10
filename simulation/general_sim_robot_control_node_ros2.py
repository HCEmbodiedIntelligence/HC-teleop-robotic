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
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger
import argparse
import fcntl

from robot_utils.robot_module import AssembledRobot

from robot_utils.utils import (
    debug_draw_pose,
    multiply_transforms,
    pose_msg_to_list,
)


class SimRobotController(Node, AssembledRobot):
    def __init__(self, config_path, init_debug=True, headless=False):
        # Initialize ROS2 node
        Node.__init__(self, "sim_robot_controller_node")

        config_file = os.path.abspath(os.path.expanduser(config_path))
        if not os.path.isfile(config_file):
            raise FileNotFoundError(f"simulation profile does not exist: {config_file}")
        self.robot_description_path = os.path.dirname(config_file)
        with open(config_file, encoding="utf-8") as stream:
            self.configs = yaml.safe_load(stream)
        options = "--opengl2" if self.configs.get("sim_opengl2", False) and not headless else ""
        client_id = p.connect(p.DIRECT if headless else p.GUI, options=options)
        if client_id < 0:
            raise RuntimeError("Unable to connect to PyBullet")
        self.physics_clent_id = client_id
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, int(init_debug and not headless))
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        camera = self.configs.get("sim_camera", {})
        p.resetDebugVisualizerCamera(
            cameraDistance=camera.get("distance", 1.5),
            cameraPitch=camera.get("pitch", -30),
            cameraYaw=camera.get("yaw", 180),
            cameraTargetPosition=camera.get("target_position", [0, 0, 0.6]),
        )

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
        self.profile_finger_command_subs = [
            self.create_subscription(
                JointState,
                str(topic),
                self.target_finger_joints_callback,
                1,
            )
            for topic in self.configs.get("finger_command_topics", [])
        ]
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
        
        self.hardware_ready_pub = self.create_publisher(
            Bool, f"/io_teleop/hardware_ready", 1
        )
        self.hardware_ready = True
        self.head_tracking_poses = None
        self.head_tracking_stamp = 0.0
        self.head_frame_ids = [None, None]
        self.head_text_ids = [-1, -1]
        self.head_error_id = -1
        self.head_tracking_sub = self.create_subscription(
            PoseArray, "/hc_teleop/head_tracking_poses",
            self.head_tracking_callback, 1,
        )
        self.hardware_ready_pub.publish(Bool(data=True))

        # Timer for publishing joint states
        self.joint_state_pub_rate = 100
        self.base_velocity = [0.0, 0.0, 0.0]
        self.base_command_stamp = 0.0
        self.base_update_stamp = time.monotonic()
        self.timer = self.create_timer(1.0 / 100, self.update_joint_state)
        
        # Service server
        self.reset_service_server = self.create_service(
            Trigger, "io_teleop_reset_robot", self.reset_service_callback
        )

        # Start physics only after all joints and home poses are assembled.
        self.physics_stop = threading.Event()
        def physics_sim():
            try:
                while rclpy.ok() and not self.physics_stop.is_set():
                    with self.bullet_lock:
                        if not p.isConnected(self.physics_clent_id):
                            break
                        p.stepSimulation(physicsClientId=self.physics_clent_id)
                    self.physics_stop.wait(1.0 / 240.0)
            except p.error:
                pass
            finally:
                if not self.physics_stop.is_set():
                    rclpy.try_shutdown()
        self.physics_thread = threading.Thread(target=physics_sim, daemon=True)
        self.physics_thread.start()
        if init_debug and not headless:
            self.debug_timer = self.create_timer(1.0 / 60, self.update_debug_joints)

    def close_simulation(self):
        self.physics_stop.set()
        self.physics_thread.join(timeout=2)
        if p.isConnected(self.physics_clent_id):
            p.disconnect(self.physics_clent_id)

    def reset_service_callback(self, request, response):
        print(
            "===========================Reset robot to home request received==========================="
        )
        self.hardware_ready = False
        self.hardware_ready_pub.publish(Bool(data=False))
        self.reset_home()
        time.sleep(1)
        self.hardware_ready = True
        self.hardware_ready_pub.publish(Bool(data=True))
        response.success = True
        response.message = "Service successfully triggered!"
        return response

    def update_joint_state(self):
        self._update_base_velocity()
        self._draw_head_tracking()
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
        self.hardware_ready_pub.publish(Bool(data=self.hardware_ready))
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

    def head_tracking_callback(self, msg):
        if msg.header.frame_id != "robot_base" or len(msg.poses) != 2:
            return
        poses = [pose_msg_to_list(pose) for pose in msg.poses]
        if not all(np.isfinite(position).all() and np.isfinite(orientation).all()
                   for position, orientation in poses):
            return
        self.head_tracking_poses = poses
        self.head_tracking_stamp = time.monotonic()

    def _draw_head_tracking(self):
        if self.head_tracking_poses is None:
            return
        if time.monotonic() - self.head_tracking_stamp > 0.5:
            for item in [*(self.head_frame_ids[0] or []),
                         *(self.head_frame_ids[1] or []),
                         *self.head_text_ids, self.head_error_id]:
                if item >= 0:
                    p.removeUserDebugItem(item, physicsClientId=self.physics_clent_id)
            self.head_frame_ids = [None, None]
            self.head_text_ids = [-1, -1]
            self.head_error_id = -1
            self.head_tracking_poses = None
            return
        # Match the controller's getBasePositionAndOrientation reference.
        root = p.getBasePositionAndOrientation(
            self.robot_id, physicsClientId=self.physics_clent_id)
        poses = [multiply_transforms(root, pose) for pose in self.head_tracking_poses]
        for i, (pose, label, color) in enumerate(zip(
                poses, ("HEAD ACTUAL", "HEAD TARGET"), ((0, 1, 1), (1, 0.65, 0)))):
            self.head_frame_ids[i] = debug_draw_pose(
                pose, self.head_frame_ids[i], line_width=2.0 if i == 0 else 5.0,
                line_length=0.12 if i == 0 else 0.20,
                physics_client_id=self.physics_clent_id)
            position = list(pose[0])
            position[2] += 0.06 + i * 0.06
            self.head_text_ids[i] = p.addUserDebugText(
                label, position, textColorRGB=color, textSize=1.2,
                replaceItemUniqueId=self.head_text_ids[i],
                physicsClientId=self.physics_clent_id)
        self.head_error_id = p.addUserDebugLine(
            poses[0][0], poses[1][0], lineColorRGB=[1, 0.65, 0], lineWidth=2,
            replaceItemUniqueId=self.head_error_id,
            physicsClientId=self.physics_clent_id)

    def target_base_move_callback(self, msg):
        if len(msg.data) != 3 or not np.isfinite(msg.data).all():
            self.base_velocity = [0.0, 0.0, 0.0]
            return
        if self.configs.get("base_command_mode", "delta") == "velocity":
            self.base_velocity = list(msg.data)
            self.base_command_stamp = time.monotonic()
            return
        # Historical delta profiles use local Y for forward motion.
        self._move_base(msg.data[0], msg.data[2], msg.data[1])

    def _update_base_velocity(self):
        now = time.monotonic()
        # Bound a delayed tick; never replay elapsed motion after a stall.
        dt = min(max(now - self.base_update_stamp, 0.0), 0.05)
        self.base_update_stamp = now
        if now - self.base_command_stamp > 0.2:
            self.base_velocity = [0.0, 0.0, 0.0]
        if any(self.base_velocity):
            self._move_base(*(value * dt for value in self.base_velocity))

    def _move_base(self, delta_yaw, delta_pos, delta_lateral):
        tar_pose = multiply_transforms(
            self.base.pose,
            [[delta_pos, delta_lateral, 0], p.getQuaternionFromEuler([0, 0, delta_yaw])],
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
    # One PyBullet ROS publisher per user/domain, including CLI launches.
    lock_path = f"/tmp/hc-pybullet-{os.getuid()}-{os.environ.get('ROS_DOMAIN_ID', '0')}.lock"
    simulation_lock = open(lock_path, "a")
    try:
        fcntl.flock(simulation_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("PyBullet is already running in this ROS domain; stop it before launching another model")
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
        print("HC_SIMULATION_READY", flush=True)
        rclpy.spin(robot)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        if robot is not None:
            robot.close_simulation()
            robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
