"""Independent simulation/VR observation publisher; never an actuator adapter.

Simulation mode mirrors measured PyBullet feedback and submitted actions.
Physical deployments may use --vr-only, which cannot create robot publishers.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import stat
import time

import numpy as np

from .model import UrdfModel, matrix_quaternion, pose_matrix
from .runtime import write_status


class ObservationPublisher:
    def __init__(self, config, state_dir, vr_only=False):
        if config.get("runtime_mode", "sim") != "sim" and not vr_only:
            raise ValueError(
                "physical robot observation mirroring is forbidden; use --vr-only"
            )
        self.config, self.state_dir, self.vr_only = config, Path(state_dir), vr_only
        self.counts = {"feedback": 0, "action": 0, "vr": 0, "invalid": 0, "stale": 0}
        self.q = {}
        self.last_metadata = 0.0
        self.stopped = False
        import rclpy
        from rclpy.context import Context
        from rclpy.qos import (
            QoSProfile,
            ReliabilityPolicy,
            HistoryPolicy,
            DurabilityPolicy,
        )
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        from geometry_msgs.msg import (
            PoseStamped,
            PoseArray,
            Pose,
            TransformStamped,
            TwistStamped,
        )
        from tf2_msgs.msg import TFMessage

        self.types = {
            cls.__name__: cls
            for cls in (
                JointState,
                Bool,
                String,
                PoseStamped,
                PoseArray,
                Pose,
                TransformStamped,
                TwistStamped,
                TFMessage,
            )
        }
        self.context = Context()
        rclpy.init(context=self.context, domain_id=int(config.get("ros_domain_id", 14)))
        self.node = rclpy.create_node(
            "hc_platform_sim_observations"
            if not vr_only
            else "hc_platform_vr_observations",
            context=self.context,
        )
        self.qos = QoSProfile(
            depth=4,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publishers = {}
        self._publisher("/vrdata", "String")
        self._publisher("/observation/vr/input", "String")
        self._publisher("/observation/headset/pose", "PoseStamped")
        self.model = (
            None
            if vr_only
            else UrdfModel(
                config.get("urdf_path", config.get("urdf")), config.get("package_root")
            )
        )
        self.arms = config.get("arms", config.get("teleop", {}).get("arms", {}))
        self.body = config.get("teleop", {}).get("body", {})
        self.camera = config.get("teleop", {}).get("camera_metadata", {})
        if not vr_only:
            for topic, kind in (
                ("/observation/joint_states", "JointState"),
                ("/action/joint_states", "JointState"),
                ("/observation/base/pose", "PoseStamped"),
                ("/status/base", "String"),
                ("/status/hardware/ready", "Bool"),
                ("/observation/ee_poses", "PoseArray"),
                ("/action/ee_poses", "PoseArray"),
                ("/observation/cameras/head/pose", "PoseStamped"),
                ("/observation/head/pose", "PoseStamped"),
                ("/observation/cameras/head/extrinsics", "String"),
                ("/observation/cameras/robot_extrinsics", "String"),
                ("/action/base/velocity", "TwistStamped"),
                ("/tf", "TFMessage"),
            ):
                self._publisher(topic, kind)
            for group in ("left_arm", "right_arm", "waist"):
                for family in ("observation", "action"):
                    self._publisher(f"/{family}/{group}/joint_states", "JointState")
            for side in self.arms:
                self._publisher(f"/observation/{side}_arm/ee_pose", "PoseStamped")
                self._publisher(f"/action/{side}_arm/ee_pose", "PoseStamped")
        path = self.state_dir / "observations.sock"
        if len(os.fsencode(path)) >= 108:
            raise ValueError("observation socket path exceeds Unix limit")
        if path.exists() or path.is_symlink():
            info = path.lstat()
            if not stat.S_ISSOCK(info.st_mode) or info.st_uid != os.getuid():
                raise ValueError("unsafe observation socket")
            path.unlink()
        self.path = path
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.socket.bind(str(path))
        self.socket.settimeout(0.02)
        path.chmod(0o600)

    def _publisher(self, topic, kind):
        self.publishers[topic] = self.node.create_publisher(
            self.types[kind], topic, self.qos
        )

    @staticmethod
    def _stamp(message, nanoseconds):
        message.header.stamp.sec = int(nanoseconds // 1_000_000_000)
        message.header.stamp.nanosec = int(nanoseconds % 1_000_000_000)

    def _string(self, topic, value):
        message = self.types["String"]()
        message.data = json.dumps(value, separators=(",", ":"), allow_nan=False)
        self.publishers[topic].publish(message)

    def _joint(self, topic, names, positions, stamp, velocities=None):
        if not names:
            return
        message = self.types["JointState"]()
        self._stamp(message, stamp)
        message.name = list(names)
        message.position = list(map(float, positions))
        if velocities and len(velocities) == len(names):
            message.velocity = list(map(float, velocities))
        self.publishers[topic].publish(message)

    def _pose(self, transform):
        message = self.types["Pose"]()
        message.position.x, message.position.y, message.position.z = map(
            float, transform[:3, 3]
        )
        quaternion = matrix_quaternion(transform[:3, :3])
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = map(float, quaternion)
        return message

    def _pose_stamped(self, topic, transform, frame, stamp):
        message = self.types["PoseStamped"]()
        self._stamp(message, stamp)
        message.header.frame_id = frame
        message.pose = self._pose(transform)
        self.publishers[topic].publish(message)

    def _groups(self, family, names, positions, stamp):
        values = dict(zip(names, positions))
        groups = {
            "left_arm": self.config.get("groups", {}).get("left_arm", []),
            "right_arm": self.config.get("groups", {}).get("right_arm", []),
            "waist": self.body.get("waist_joint_names", []),
        }
        for group, group_names in groups.items():
            selected = [name for name in group_names if name in values]
            self._joint(
                f"/{family}/{group}/joint_states",
                selected,
                [values[name] for name in selected],
                stamp,
            )

    def _ee(self, family, q, stamp):
        poses = []
        common_base = None
        for side, arm in self.arms.items():
            if not all(name in q for name in arm["joint_names"]):
                continue
            transform = self.model.fk(q, arm["ee_link"], arm.get("base_link"))
            frame = arm.get("base_link", self.model.root)
            self._pose_stamped(f"/{family}/{side}_arm/ee_pose", transform, frame, stamp)
            if common_base is None:
                common_base = frame
            if common_base == frame:
                poses.append(self._pose(transform))
        if poses:
            message = self.types["PoseArray"]()
            self._stamp(message, stamp)
            message.header.frame_id = common_base
            message.poses = poses
            self.publishers[f"/{family}/ee_poses"].publish(message)

    def _feedback(self, data):
        stamp = int(data.get("sample_unix_ns", time.time_ns()))
        names, positions = data["joint_names"], data["positions"]
        self._joint(
            "/observation/joint_states", names, positions, stamp, data.get("velocities")
        )
        self._groups("observation", names, positions, stamp)
        self.q = dict(
            zip(
                data.get("model_joint_names", names),
                data.get("model_positions", positions),
            )
        )
        self._ee("observation", self.q, stamp)
        ready = self.types["Bool"]()
        ready.data = bool(data.get("ready"))
        self.publishers["/status/hardware/ready"].publish(ready)
        if data.get("base_pose"):
            self._pose_stamped(
                "/observation/base/pose",
                pose_matrix(data["base_pose"]),
                "simulation_world",
                stamp,
            )
        now = time.monotonic()
        if now - self.last_metadata < 0.04:
            return
        self.last_metadata = now
        self._string(
            "/status/base",
            {
                "source": "pybullet_state",
                "pose": data.get("base_pose"),
                "chassis_model": "kinematic_velocity",
                "sample_mono_ns": data["sample_mono_ns"],
            },
        )
        camera_frame = self.camera.get(
            "camera_frame", self.body.get("camera_link", "camera_head")
        )
        common = self.camera.get("common_frame", "zhi_Link")
        if camera_frame in self.model.links and common in self.model.links:
            transform = self.model.fk(self.q, camera_frame, common)
            for topic in ("/observation/cameras/head/pose", "/observation/head/pose"):
                self._pose_stamped(topic, transform, common, stamp)
            metadata = {
                "schema": "hc-camera-extrinsics/v1",
                "source": "model_fk",
                "calibrated": False,
                "runtime_mode": "sim",
                "config_hash": self.config.get("config_hash"),
                "camera": "head",
                "parent_frame": common,
                "child_frame": camera_frame,
                "T_parent_child": transform.tolist(),
                "sample_mono_ns": data["sample_mono_ns"],
            }
            for topic in (
                "/observation/cameras/head/extrinsics",
                "/observation/cameras/robot_extrinsics",
            ):
                self._string(topic, metadata)
        frames, _ = self.model.frames(self.q)
        transforms = []
        for name in self.model.order:
            joint = self.model.joints[name]
            local = np.linalg.inv(frames[joint.parent]) @ frames[joint.child]
            transform = self.types["TransformStamped"]()
            self._stamp(transform, stamp)
            transform.header.frame_id = joint.parent
            transform.child_frame_id = joint.child
            pose = self._pose(local)
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            transforms.append(transform)
        if data.get("base_pose"):
            transform = self.types["TransformStamped"]()
            self._stamp(transform, stamp)
            transform.header.frame_id = "simulation_world"
            transform.child_frame_id = self.model.root
            pose = self._pose(pose_matrix(data["base_pose"]))
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation = pose.orientation
            transforms.insert(0, transform)
        message = self.types["TFMessage"]()
        message.transforms = transforms
        self.publishers["/tf"].publish(message)

    def _action(self, data):
        output, ack = data["output"], data["ack"]
        if not ack.get("accepted"):
            return
        stamp = int(data.get("sample_unix_ns", time.time_ns()))
        names, positions = output.get("joint_names", []), output.get("positions", [])
        self._joint(
            "/action/joint_states", names, positions, stamp, output.get("velocities")
        )
        self._groups("action", names, positions, stamp)
        if self.q and names:
            target = dict(self.q)
            target.update(zip(names, positions))
            self._ee("action", target, stamp)
        base = self.types["TwistStamped"]()
        self._stamp(base, stamp)
        base.header.frame_id = "base_link"
        base.twist.linear.x, base.twist.linear.y, base.twist.angular.z = map(
            float, output.get("base_velocity", [0, 0, 0])
        )
        self.publishers["/action/base/velocity"].publish(base)

    def _vr(self, data):
        for topic in ("/vrdata", "/observation/vr/input"):
            self._string(topic, dict(data, schema="hc-vr-input/v1"))
        if data.get("tracked", {}).get("head") and data.get("head_pose"):
            raw = pose_matrix(data["head_pose"])
            mapping = np.asarray(
                self.config.get("teleop", {})
                .get("control", {})
                .get("axis_mapping", [[0, 0, 1], [-1, 0, 0], [0, 1, 0]])
            )
            converted = np.eye(4)
            converted[:3, 3] = mapping @ raw[:3, 3]
            converted[:3, :3] = mapping @ raw[:3, :3] @ mapping.T
            self._pose_stamped(
                "/observation/headset/pose",
                converted,
                "vr_tracking_ros",
                int(data.get("sample_unix_ns", time.time_ns())),
            )

    def handle(self, event):
        kind, data = event.get("kind"), event.get("payload", {})
        if kind not in ("feedback", "action", "vr") or (self.vr_only and kind != "vr"):
            return
        stamp = data.get("sample_mono_ns", data.get("ack", {}).get("sample_mono_ns"))
        if stamp is not None and not 0 <= time.monotonic_ns() - stamp <= 500_000_000:
            self.counts["stale"] += 1
            return
        getattr(self, "_" + kind)(data)
        self.counts[kind] += 1

    def run(self):
        last_status = 0.0
        while not self.stopped:
            try:
                raw = self.socket.recv(65537)
                if len(raw) > 65536:
                    raise ValueError("oversized observation event")
                self.handle(json.loads(raw))
            except socket.timeout:
                pass
            except (ValueError, KeyError, TypeError) as error:
                self.counts["invalid"] += 1
                self.last_error = str(error)
            if time.monotonic() - last_status > 0.25:
                write_status(
                    self.state_dir,
                    "observations",
                    {
                        "ready": True,
                        "mode": "vr_only" if self.vr_only else "simulation",
                        "counts": self.counts,
                        "pose_source": "model_fk",
                        "camera_extrinsics_calibrated": False,
                        "last_error": getattr(self, "last_error", None),
                    },
                )
                last_status = time.monotonic()

    def close(self):
        self.socket.close()
        self.path.unlink(missing_ok=True)
        self.node.destroy_node()
        self.context.shutdown()
        write_status(
            self.state_dir, "observations", {"ready": False, "counts": self.counts}
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--state-dir", required=True)
    parser.add_argument("--vr-only", action="store_true")
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    publisher = ObservationPublisher(config, args.state_dir, args.vr_only)

    def stop(*_):
        publisher.stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        publisher.run()
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
