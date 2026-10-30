import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

import time
import json
import threading
import yaml
import numpy as np
import pybullet as p
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, PoseStamped
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
    def __init__(self, config_path, init_debug=True, headless=False, stream_viewer=False,
                 viewer_demand_file=None):
        # Initialize ROS2 node
        Node.__init__(self, "sim_robot_controller_node")

        config_file = os.path.abspath(os.path.expanduser(config_path))
        if not os.path.isfile(config_file):
            raise FileNotFoundError(f"simulation profile does not exist: {config_file}")
        self.robot_description_path = os.path.dirname(config_file)
        with open(config_file, encoding="utf-8") as stream:
            self.configs = yaml.safe_load(stream)
        sim_cfg = self.configs.get("simulation", {})
        if not isinstance(sim_cfg, dict):
            sim_cfg = {}
        opengl2 = self.configs.get("sim_opengl2", sim_cfg.get("sim_opengl2", True))
        options = "--opengl2" if opengl2 and not headless else ""
        forwarded_display = not headless and os.environ.get("DISPLAY", "").startswith("localhost:")
        if forwarded_display:
            options += " --width=640 --height=480"
        started = time.monotonic()
        self.get_logger().info(
            f"PyBullet connect starting: DISPLAY={os.environ.get('DISPLAY', '<unset>')} "
            f"headless={headless} options={options!r}"
        )
        client_id = p.connect(p.DIRECT if headless else p.GUI, options=options)
        if client_id < 0:
            raise RuntimeError("Unable to connect to PyBullet")
        self.physics_clent_id = client_id
        self.physics_stop = threading.Event()
        self.physics_thread = None
        self.manual_rendering = forwarded_display
        self.visualization_enabled = bool(init_debug and not headless)
        self.target_ee_poses = None
        self.stream_viewer = stream_viewer
        self.viewer_demand_file = viewer_demand_file
        self.next_viewer_frame = 0.0
        self.get_logger().info(f"PyBullet connect finished in {time.monotonic() - started:.3f}s")
        try:
            # Mesh uploads and intermediate resets must not trigger individual
            # frames over a forwarded X11 connection.
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=client_id)
            self._initialize_robot(config_file, sim_cfg, init_debug, headless)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=client_id)
            self._start_physics()
            self.get_logger().info(f"PyBullet ready in {time.monotonic() - started:.3f}s")
        except Exception:
            self.close_simulation()
            self.destroy_node()
            raise

    def _initialize_robot(self, config_file, sim_cfg, init_debug, headless):
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, int(init_debug and not headless))
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        camera = self.configs.get("sim_camera", sim_cfg.get("sim_camera", {}))
        p.resetDebugVisualizerCamera(
            cameraDistance=camera.get("distance", 1.5),
            cameraPitch=camera.get("pitch", -30),
            cameraYaw=camera.get("yaw", 180),
            cameraTargetPosition=camera.get("target_position", [0, 0, 0.6]),
        )

        self.wo_controller = False
        urdf_rel = self.configs.get("urdf_path")
        if not urdf_rel and isinstance(self.configs.get("robot"), dict):
            urdf_rel = self.configs["robot"].get("urdf_path")
        if not urdf_rel:
            raise ValueError("No urdf_path specified in configs")
        robot_urdf_path = os.path.abspath(
            os.path.join(self.robot_description_path, urdf_rel)
        )
        arm_teleop_file = os.path.join(self.robot_description_path, "arm_teleop.yaml")
        if os.path.isfile(arm_teleop_file):
            self.configs.setdefault("arm_teleop_path", arm_teleop_file)
        elif config_file.endswith("arm_teleop.yaml"):
            self.configs.setdefault("arm_teleop_path", config_file)
        with_ee_constraint = False
        
        # Initialize the AssembledRobot
        model_started = time.monotonic()
        self.get_logger().info(f"PyBullet model loading: {robot_urdf_path}")
        AssembledRobot.__init__(
            self,
            robot_urdf_path,
            self.configs,
            init_debug,
            with_ee_constraint,
            self.physics_clent_id,
        )
        self.get_logger().info(f"PyBullet model assembled in {time.monotonic() - model_started:.3f}s")
        self.reset_home()
        # Keep persistent IDs for every visual frame. The old implementation
        # drew the flange frames only once here, so they remained at the home
        # pose after the arm moved and looked like unrelated floating markers.
        self.base_frame_ids = None
        self.actual_ee_frame_ids = [None] * len(self.arms)
        self.feedback_joint_indices = []
        self.feedback_joint_names = []
        for index in range(p.getNumJoints(self.robot_id, physicsClientId=self.physics_clent_id)):
            info = p.getJointInfo(self.robot_id, index, physicsClientId=self.physics_clent_id)
            if info[2] != p.JOINT_FIXED:
                self.feedback_joint_indices.append(index)
                self.feedback_joint_names.append(info[1].decode("utf-8"))
        self.target_ee_frame_ids = [None] * len(self.arms)
        self.plane_id = None
        self.ground_frame_ids = None
        self.init_ground_plane(sim_cfg, init_debug=init_debug and not headless)
        
        # ROS2 publishers and subscribers
        from adapters.core.runtime_config import unified_profile
        unified = unified_profile(self.robot_description_path)
        if unified:
            from middleware.core.robot_api import RobotApiProfile
            RobotApiProfile(self.robot_description_path)
        self.joint_state_pub = self.create_publisher(
            JointState, "/internal/robot/joint_states" if unified else "/io_teleop/joint_states", 1
        )
        self.base_pose_pub = self.create_publisher(PoseStamped, "/observation/base/pose", 10)
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            "/internal/robot/joint_commands" if unified else "/io_teleop/joint_cmd",
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
                "/action/base/command",
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
            PoseArray, "/status/head/tracking_poses",
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

        if init_debug and not headless:
            self.debug_timer = self.create_timer(1.0 / 60, self.update_debug_joints)

    def _start_physics(self):
        # Start physics after rendering has been restored successfully.
        def physics_sim():
            try:
                next_frame = next_debug = 0.0
                while rclpy.ok() and not self.physics_stop.is_set():
                    with self.bullet_lock:
                        if not p.isConnected(self.physics_clent_id):
                            break
                        p.stepSimulation(physicsClientId=self.physics_clent_id)
                        now = time.monotonic()
                        if self.visualization_enabled and now >= next_debug:
                            self._draw_visualizations()
                            next_debug = now + 0.2
                        if self.manual_rendering and now >= next_frame:
                            # Forwarded software rendering must not run continuously
                            # or on the ROS executor handling control and feedback.
                            p.configureDebugVisualizer(
                                p.COV_ENABLE_SINGLE_STEP_RENDERING, 1,
                                physicsClientId=self.physics_clent_id)
                            # Measure the quiet period after the frame completes;
                            # a slow X11 frame must not cause continuous rendering.
                            next_frame = time.monotonic() + 0.2
                    self.physics_stop.wait(1.0 / 240.0)
            except p.error:
                pass
            finally:
                if not self.physics_stop.is_set():
                    rclpy.try_shutdown()
        self.physics_thread = threading.Thread(target=physics_sim, daemon=True)
        self.physics_thread.start()

    def close_simulation(self):
        self.physics_stop.set()
        if self.physics_thread is not None and self.physics_thread.is_alive():
            self.physics_thread.join(timeout=2)
        if p.isConnected(physicsClientId=self.physics_clent_id):
            p.disconnect(physicsClientId=self.physics_clent_id)

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

    def publish_base_pose(self):
        """Measured pose in the same local frame consumed by _move_base."""
        position, orientation = self.base.control_pose
        if not np.isfinite(position).all() or not np.isfinite(orientation).all():
            return
        norm = float(np.linalg.norm(orientation))
        if norm < 1e-8:
            return
        orientation = np.asarray(orientation) / norm
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = map(float, orientation)
        self.base_pose_pub.publish(msg)

    def update_joint_state(self):
        self._update_base_velocity()
        self.publish_base_pose()
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.feedback_joint_names)
        with self.bullet_lock:
            states = p.getJointStates(
                self.robot_id, self.feedback_joint_indices,
                physicsClientId=self.physics_clent_id)
        joint_state.position = [state[0] for state in states]
        joint_state.velocity = [state[1] for state in states]
        self.joint_state_pub.publish(joint_state)
        self.hardware_ready_pub.publish(Bool(data=self.hardware_ready))
        demand_file = getattr(self, 'viewer_demand_file', None)
        viewer_enabled = getattr(self, 'stream_viewer', False) or (
            demand_file is not None and os.path.isfile(demand_file))
        if viewer_enabled and time.monotonic() >= self.next_viewer_frame:
            with self.bullet_lock:
                root_pose = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.physics_clent_id)
                markers = self._viewer_markers(root_pose)
            frame = {'positions': dict(zip(joint_state.name, joint_state.position)),
                     'base_pose': root_pose, 'markers': markers}
            print('HC_SIMULATION_FRAME ' + json.dumps(frame, separators=(',', ':'), allow_nan=False), flush=True)
            self.next_viewer_frame = time.monotonic() + 1.0 / 30.0

    def _viewer_markers(self, root_pose):
        """Read world-space marker poses only when a viewer requests a frame.

        This method runs under ``bullet_lock`` and uses no GUI/debug RPCs.
        Head tracking messages use the root COM frame, while arm targets use
        their configured controller base link frames.
        """
        def serializable(pose):
            return [[float(value) for value in component] for component in pose]

        markers = {'base': serializable(self.base.pose)}
        targets = self.target_ee_poses
        bases = self.configs.get('controller_indices', {}).get('base', [])
        for index, arm in enumerate(self.arms):
            side = ('right', 'left')[index] if index < 2 else str(index)
            markers['ee_actual_' + side] = serializable(arm.abs_ee_pose)
            if targets is not None and index < len(targets) and index < len(bases):
                base_index = bases[index]
                base_pose = (p.getLinkState(
                    self.robot_id, base_index, computeForwardKinematics=True,
                    physicsClientId=self.physics_clent_id)[4:6]
                    if base_index != -1 else self.base.pose)
                markers['ee_target_' + side] = serializable(multiply_transforms(
                    base_pose, pose_msg_to_list(targets[index])))

        teleop = self.teleop_config
        body = teleop.get('body', {})
        camera_name = (body.get('visualization_head_link') or body.get('camera_link')
                       or teleop.get('neck', {}).get('camera_link') or 'camera_head')
        camera_index = self.link_name2id_dict.get(camera_name)
        if camera_index is not None:
            camera_pose = (p.getLinkState(
                self.robot_id, camera_index, computeForwardKinematics=True,
                physicsClientId=self.physics_clent_id)[4:6]
                if camera_index != -1 else root_pose)
            markers['camera_actual'] = serializable(camera_pose)
        if self.head_tracking_poses is not None:
            if time.monotonic() - self.head_tracking_stamp <= 0.5:
                if 'camera_actual' not in markers:
                    markers['camera_actual'] = serializable(multiply_transforms(
                        root_pose, self.head_tracking_poses[0]))
                markers['camera_target'] = serializable(multiply_transforms(
                    root_pose, self.head_tracking_poses[1]))
            else:
                self.head_tracking_poses = None
        return markers

    def _draw_visualizations(self):
        """Optional markers at 5 Hz in the physics worker, outside ROS callbacks."""
        self._draw_head_tracking()
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
        self._draw_target_ee()

    def target_finger_joints_callback(self, msg):
        joint_pos = msg.position
        joint_names = msg.name
        for pos, name in zip(joint_pos, joint_names):
            joint_id = self.joint_name2id_dict.get(name)
            if joint_id is not None:
                self.reset_j([joint_id], [pos])

    def joint_cmd_callback(self, msg):
        joint_pos = msg.position
        joint_names = msg.name
        for pos, name in zip(joint_pos, joint_names):
            joint_id = self.joint_name2id_dict.get(name)
            if joint_id is not None:
                self.reset_j([joint_id], [pos])

    def joint_cmd_from_vr_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position
        for pos, name in zip(joint_positions, joint_names):
            joint_id = self.joint_name2id_dict.get(name)
            if joint_id is not None:
                self.reset_j([joint_id], [pos])

    def gripper_status_callback(self, msg):
        gripper_status = msg.position
        for i, gripper in enumerate(self.grippers):
            gripper.reset(gripper_status[i])

    def target_ee_callback(self, msg):
        # Visualization never competes with incoming control on the ROS executor.
        if len(msg.poses) >= len(self.arms):
            self.target_ee_poses = list(msg.poses)

    def _draw_target_ee(self):
        if self.target_ee_poses is None:
            return
        if "controller_indices" in self.configs:
            for i in range(len(self.arms)):
                pose = self.target_ee_poses[i]
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
            self.base.control_pose,
            [[delta_pos, delta_lateral, 0], p.getQuaternionFromEuler([0, 0, delta_yaw])],
        )
        self.base.reset_control_pose(tar_pose)

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

        if hasattr(self, "folding_waist"):
            self.folding_waist.reset()
        elif hasattr(self, "waist"):
            self.waist.reset()

        if hasattr(self, "initial_joints") and isinstance(self.initial_joints, dict):
            for name, pos in self.initial_joints.items():
                if name in self.joint_name2id_dict:
                    self.reset_j([self.joint_name2id_dict[name]], [float(pos)])

    def init_ground_plane(self, sim_cfg, init_debug=False):
        """Initialize reference ground plane and optional origin coordinate axes."""
        ground_cfg = sim_cfg.get("ground", {}) if isinstance(sim_cfg, dict) else {}
        if not isinstance(ground_cfg, dict):
            ground_cfg = {}
        enable_ground = ground_cfg.get("enabled", True)
        if not enable_ground:
            self.plane_id = None
            return

        ground_z = float(ground_cfg.get("z", 0.0))
        self.plane_id = self.load_ground_plane(z=ground_z)
        if init_debug and self.plane_id is not None:
            # Draw origin reference frame on the ground plane (RGB = XYZ, length 0.4m)
            self.ground_frame_ids = debug_draw_pose(
                [[0.0, 0.0, ground_z], [0.0, 0.0, 0.0, 1.0]],
                line_width=2.5,
                line_length=0.4,
                physics_client_id=self.physics_clent_id,
            )

    def load_ground_plane(self, z=0.0):
        """Load a reference ground plane into the PyBullet world."""
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        candidate_paths = [
            os.path.join(project_root, ".deps", "pybullet_data"),
            os.path.join(os.path.expanduser("~"), ".local", "lib", "python3.10", "site-packages", "pybullet_data"),
        ]
        try:
            import pybullet_data
            candidate_paths.insert(0, pybullet_data.getDataPath())
        except ImportError:
            pass

        for dpath in candidate_paths:
            plane_urdf = os.path.join(dpath, "plane.urdf")
            if os.path.isfile(plane_urdf):
                try:
                    p.setAdditionalSearchPath(dpath, physicsClientId=self.physics_clent_id)
                    plane_id = p.loadURDF(
                        "plane.urdf",
                        basePosition=[0.0, 0.0, z],
                        useFixedBase=True,
                        physicsClientId=self.physics_clent_id,
                    )
                    self.get_logger().info(f"Loaded reference ground plane (plane.urdf) at z={z}")
                    return plane_id
                except Exception as exc:
                    self.get_logger().warn(f"Failed to load plane.urdf from {dpath}: {exc}")

        # Fallback: create procedural ground plane if plane.urdf is not found
        try:
            col_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[50.0, 50.0, 0.05],
                physicsClientId=self.physics_clent_id,
            )
            vis_id = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[50.0, 50.0, 0.05],
                rgbaColor=[0.82, 0.82, 0.85, 1.0],
                physicsClientId=self.physics_clent_id,
            )
            plane_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col_id,
                baseVisualShapeIndex=vis_id,
                basePosition=[0.0, 0.0, z - 0.05],
                physicsClientId=self.physics_clent_id,
            )
            self.get_logger().info(f"Created procedural reference ground box at z={z}")
            return plane_id
        except Exception as exc:
            self.get_logger().warn(f"Failed to create procedural ground plane: {exc}")
            return None


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
    parser.add_argument("--stream-viewer", action="store_true")
    parser.add_argument("--viewer-demand-file", help="Stream frames only while this file exists")

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
    if os.path.isdir(profile):
        arm_teleop_candidate = os.path.join(profile, "arm_teleop.yaml")
        vr_configs_candidate = os.path.join(profile, "vr_configs.yml")
        if os.path.isfile(arm_teleop_candidate):
            config_path = arm_teleop_candidate
        elif os.path.isfile(vr_configs_candidate):
            config_path = vr_configs_candidate
        else:
            config_path = arm_teleop_candidate
    else:
        config_path = profile
    
    robot = None
    try:
        robot = SimRobotController(
            config_path,
            init_debug=arguments.init_debug,
            headless=arguments.headless,
            stream_viewer=arguments.stream_viewer,
            viewer_demand_file=arguments.viewer_demand_file,
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
