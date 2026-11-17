"""Off-thread PyBullet camera renderer for local simulation observation streams."""

from __future__ import annotations

import io
import math
import threading
import time

import numpy as np


def _encode_jpeg(rgb: np.ndarray, quality: int = 80) -> bytes:
    try:
        import cv2

        ok, encoded = cv2.imencode(
            ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if ok:
            return encoded.tobytes()
    except ImportError:
        pass
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(rgb, mode="RGB").save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


def _encode_depth_png(depth_buffer: np.ndarray, near: float, far: float) -> bytes:
    depth_m = far * near / (far - (far - near) * np.clip(depth_buffer, 0.0, 0.999999))
    depth_mm = np.clip(depth_m * 1000.0, 0.0, 65535.0).astype(np.uint16)
    try:
        import cv2

        ok, encoded = cv2.imencode(".png", depth_mm)
        if ok:
            return encoded.tobytes()
    except ImportError:
        pass
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(depth_mm).save(buf, format="PNG")
    return buf.getvalue()


def _pinhole_intrinsics(width: int, height: int, fov_deg: float) -> tuple[float, float, float, float]:
    fy = 0.5 * height / math.tan(math.radians(fov_deg) * 0.5)
    return fy, fy, width * 0.5, height * 0.5


class SimCameraRenderer:
    """Renders URDF-aligned camera observations on a background PyBullet DIRECT client."""

    def __init__(self, config: dict, model, topic_config: list | None = None):
        self.config = config
        self.model = model
        self.camera_cfg = config.get("camera", {})
        self.teleop_cfg = config.get("teleop", {})
        self.sim_cfg = config.get("sim", {})
        self.camera_meta = self.teleop_cfg.get("camera_metadata", {})
        self.body_cfg = self.teleop_cfg.get("body", {})
        self.default_frame = self.camera_meta.get(
            "camera_frame", self.body_cfg.get("camera_link", "camera_head")
        )
        self.fov_deg = float(self.sim_cfg.get("camera_fov_deg", 65.0))
        self.near = 0.03
        self.far = 6.0

        self.rgb_streams = []
        depth_topics_enabled = {
            item["topic"]
            for item in (topic_config or [])
            if isinstance(item, dict)
            and item.get("enabled", True)
            and "/depth/" in item.get("topic", "")
            and "record" in item.get("outputs", [])
        }
        self.depth_streams = []
        for stream in self.camera_cfg.get("streams", []):
            topic = stream.get("topic", "")
            if not topic:
                continue
            entry = {
                "id": stream.get("id", "head"),
                "topic": topic,
                "type": stream.get("type", "sensor_msgs/msg/CompressedImage"),
                "width": int(stream.get("width", self.camera_cfg.get("width", 640))),
                "height": int(stream.get("height", self.camera_cfg.get("height", 400))),
                "fps": float(stream.get("fps", self.camera_cfg.get("fps", 15))),
                "frame_id": stream.get("frame_id", self.default_frame),
            }
            if "/depth/" in topic:
                if stream.get("enabled", False) or topic in depth_topics_enabled:
                    self.depth_streams.append(entry)
            elif stream.get("enabled", True):
                self.rgb_streams.append(entry)

        self.depth_topics = {
            topic for topic in depth_topics_enabled if not any(s["topic"] == topic for s in self.depth_streams)
        }
        primary = self.rgb_streams[0] if self.rgb_streams else None
        self.info_topic = (
            f"/observation/cameras/{primary['id']}/camera_info" if primary else None
        )
        self.calibration_topic = (
            f"/observation/cameras/{primary['id']}/calibration" if primary else None
        )
        self.available = bool(self.rgb_streams)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._q = dict(zip(config.get("joint_names", []), config.get("home", [])))
        self._base_pose = None
        self._latest = None

    def required_topics(self) -> list[tuple[str, str]]:
        specs: list[tuple[str, str]] = []
        seen = set()
        for stream in self.rgb_streams:
            kind = "Image" if stream["type"] == "sensor_msgs/msg/Image" else "CompressedImage"
            if stream["topic"] not in seen:
                specs.append((stream["topic"], kind))
                seen.add(stream["topic"])
        for stream in self.depth_streams:
            if stream["topic"] not in seen:
                specs.append((stream["topic"], "CompressedImage"))
                seen.add(stream["topic"])
        for topic in sorted(self.depth_topics):
            if topic not in seen:
                specs.append((topic, "CompressedImage"))
                seen.add(topic)
        if self.info_topic and self.info_topic not in seen:
            specs.append((self.info_topic, "CameraInfo"))
            seen.add(self.info_topic)
        if self.calibration_topic and self.calibration_topic not in seen:
            specs.append((self.calibration_topic, "String"))
            seen.add(self.calibration_topic)
        return specs

    def update_state(self, q: dict[str, float], base_pose=None) -> None:
        with self._lock:
            if q:
                self._q = dict(q)
            if base_pose is not None:
                self._base_pose = list(base_pose)

    def pop_latest(self):
        with self._lock:
            item, self._latest = self._latest, None
            return item

    def start(self) -> None:
        if not self.available or self._thread is not None:
            return
        try:
            import pybullet  # noqa: F401
        except ImportError:
            self.available = False
            return
        self._thread = threading.Thread(
            target=self._run, name="sim-camera-renderer", daemon=True
        )
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _default_base_pose(self) -> tuple[list[float], list[float]]:
        pose = self.sim_cfg.get("base_pose", {})
        pos = list(pose.get("position", self.sim_cfg.get("base_position", [0.0, 0.0, 0.07])))
        orn = list(
            pose.get(
                "orientation",
                self.sim_cfg.get("base_orientation", [0.0, 0.0, 0.70710678, 0.70710678]),
            )
        )
        return pos, orn

    def _setup_scene(self, bullet):
        client = bullet.connect(bullet.DIRECT)
        if client < 0:
            raise RuntimeError("PyBullet render client failed")
        bp, bo = self._default_base_pose()
        robot = bullet.loadURDF(
            str(self.model.path), bp, bo, useFixedBase=True, physicsClientId=client
        )
        joints = {}
        links = {}
        for idx in range(bullet.getNumJoints(robot, physicsClientId=client)):
            info = bullet.getJointInfo(robot, idx, physicsClientId=client)
            joints[info[1].decode()] = idx
            links[info[12].decode()] = idx

        # Visual-only floor and workspace workbench + objects placed in front of robot (+X local).
        floor_vis = bullet.createVisualShape(
            bullet.GEOM_BOX,
            halfExtents=[4.0, 4.0, 0.02],
            rgbaColor=[0.90, 0.91, 0.93, 1.0],
            physicsClientId=client,
        )
        bullet.createMultiBody(0, -1, floor_vis, [0.0, 0.0, -0.02], physicsClientId=client)

        def to_world(local_xyz):
            w_pos, _ = bullet.multiplyTransforms(bp, bo, local_xyz, [0.0, 0.0, 0.0, 1.0])
            return list(w_pos)

        table_vis = bullet.createVisualShape(
            bullet.GEOM_BOX,
            halfExtents=[0.32, 0.45, 0.37],
            rgbaColor=[0.74, 0.60, 0.44, 1.0],
            physicsClientId=client,
        )
        bullet.createMultiBody(0, -1, table_vis, to_world([0.85, 0.0, 0.30]), bo, physicsClientId=client)

        for local_xyz, half, rgba in (
            ([0.70, -0.14, 0.70], [0.03, 0.03, 0.03], [0.90, 0.22, 0.20, 1.0]),
            ([0.72, 0.14, 0.70], [0.03, 0.03, 0.03], [0.18, 0.68, 0.34, 1.0]),
            ([0.78, 0.0, 0.68], [0.08, 0.10, 0.01], [0.22, 0.45, 0.85, 1.0]),
        ):
            vis = bullet.createVisualShape(
                bullet.GEOM_BOX, halfExtents=half, rgbaColor=rgba, physicsClientId=client
            )
            bullet.createMultiBody(0, -1, vis, to_world(local_xyz), bo, physicsClientId=client)
        return client, robot, joints, links, bp, bo

    def _camera_matrices(self, bullet, client, robot, links, bp, bo, width, height, frame_id):
        if frame_id in links:
            ls = bullet.getLinkState(
                robot, links[frame_id], computeForwardKinematics=True, physicsClientId=client
            )
            pos = np.asarray(ls[4], dtype=float)
            mat = np.asarray(bullet.getMatrixFromQuaternion(ls[5]), dtype=float).reshape(3, 3)
            forward = mat[:, 2]
            up = -mat[:, 1]
            eye = pos + 0.05 * forward
            target = eye + forward
        else:
            eye, _ = bullet.multiplyTransforms(bp, bo, [0.25, 0.0, 1.45], [0, 0, 0, 1])
            target, _ = bullet.multiplyTransforms(bp, bo, [0.80, 0.0, 0.78], [0, 0, 0, 1])
            up = [0.0, 0.0, 1.0]
        vm = bullet.computeViewMatrix(list(eye), list(target), list(up))
        pm = bullet.computeProjectionMatrixFOV(
            self.fov_deg, float(width) / float(height), self.near, self.far
        )
        return vm, pm

    def _run(self) -> None:
        import pybullet as bullet

        client = None
        try:
            client, robot, joints, links, default_bp, default_bo = self._setup_scene(bullet)
            target_fps = min(
                15.0,
                max(1.0, float(self.sim_cfg.get("camera_fps", self.rgb_streams[0]["fps"]))),
            )
            period = 1.0 / target_fps
            need_depth = bool(self.depth_streams or self.depth_topics)
            while not self._stop.is_set():
                started = time.monotonic()
                with self._lock:
                    q = dict(self._q)
                    base_pose = list(self._base_pose) if self._base_pose else None
                bp = base_pose[:3] if base_pose and len(base_pose) == 7 else default_bp
                bo = base_pose[3:] if base_pose and len(base_pose) == 7 else default_bo
                bullet.resetBasePositionAndOrientation(robot, bp, bo, physicsClientId=client)
                for name, value in q.items():
                    idx = joints.get(name)
                    if idx is not None:
                        bullet.resetJointState(robot, idx, float(value), physicsClientId=client)

                stamp_ns = time.time_ns()
                mono_ns = time.monotonic_ns()
                rgb_payloads = []
                depth_payloads = []
                primary_meta = None
                for idx, stream in enumerate(self.rgb_streams):
                    width, height = stream["width"], stream["height"]
                    vm, pm = self._camera_matrices(
                        bullet, client, robot, links, bp, bo, width, height, stream["frame_id"]
                    )
                    w, h, rgba, depth_buf, _ = bullet.getCameraImage(
                        width,
                        height,
                        vm,
                        pm,
                        shadow=0,
                        renderer=bullet.ER_TINY_RENDERER,
                        flags=bullet.ER_NO_SEGMENTATION_MASK,
                        physicsClientId=client,
                    )
                    rgb = np.asarray(rgba, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
                    if stream["type"] == "sensor_msgs/msg/Image":
                        data = rgb.tobytes()
                        kind = "Image"
                    else:
                        data = _encode_jpeg(rgb, quality=80)
                        kind = "CompressedImage"
                    rgb_payloads.append((stream["topic"], kind, w, h, stream["frame_id"], data))
                    fx, fy, cx, cy = _pinhole_intrinsics(w, h, self.fov_deg)
                    if idx == 0:
                        primary_meta = (w, h, fx, fy, cx, cy, stream["frame_id"])
                        if need_depth:
                            z_arr = np.asarray(depth_buf, dtype=np.float32).reshape(h, w)
                            png_bytes = _encode_depth_png(z_arr, self.near, self.far)
                            for d_stream in self.depth_streams:
                                depth_payloads.append(
                                    (d_stream["topic"], d_stream["frame_id"], png_bytes)
                                )
                            for d_topic in self.depth_topics:
                                depth_payloads.append((d_topic, stream["frame_id"], png_bytes))
                with self._lock:
                    self._latest = (
                        rgb_payloads,
                        depth_payloads,
                        primary_meta,
                        stamp_ns,
                        mono_ns,
                    )
                remaining = period - (time.monotonic() - started)
                if remaining > 0.001:
                    self._stop.wait(remaining)
        finally:
            if client is not None:
                bullet.disconnect(physicsClientId=client)
