#!/usr/bin/python3
"""ROS 2 Stereo Camera Stitcher Node.

Subscribes to left and right camera topics (Image or CompressedImage),
synchronizes frames with ApproximateTimeSynchronizer, stitches them horizontally
into a single Side-by-Side (SBS) stereo frame, and publishes it for VR teleoperation.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any

# Ensure ROS_DOMAIN_ID matches middleware/config.yaml or 14 by default
if "ROS_DOMAIN_ID" not in os.environ:
    domain_id = "14"
    config_path = os.path.normpath(os.path.join(os.path.dirname(__file__), "../../middleware/config.yaml"))
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                for line in f:
                    if "domain_id:" in line:
                        domain_id = line.split("domain_id:")[1].strip()
                        break
        except Exception:
            pass
    os.environ["ROS_DOMAIN_ID"] = domain_id

# If invoked inside conda or an environment without OpenCV/rclpy, seamlessly re-exec via /usr/bin/python3
try:
    import cv2
    import numpy as np
    import rclpy
except ImportError:
    if os.path.exists("/usr/bin/python3") and sys.executable != "/usr/bin/python3":
        env = dict(os.environ)
        ros_py = "/opt/ros/humble/lib/python3.10/site-packages"
        if os.path.exists(ros_py):
            cur_pp = env.get("PYTHONPATH", "")
            env["PYTHONPATH"] = f"{ros_py}:{cur_pp}" if cur_pp else ros_py
        env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "14")
        os.execve("/usr/bin/python3", ["/usr/bin/python3", os.path.abspath(__file__)] + sys.argv[1:], env)
    raise
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    import message_filters
except ImportError:
    message_filters = None


class StereoSBSStitcher(Node):
    """Synchronizes and stitches left/right cameras into a Side-by-Side image."""

    def __init__(
        self,
        left_topic: str,
        right_topic: str,
        output_topic: str,
        compressed: bool = True,
        slop_seconds: float = 0.04,
        target_fps: float = 30.0,
        jpeg_quality: int = 75,
        sync_mode: str = "latest",
    ):
        super().__init__("stereo_sbs_stitcher")
        self.compressed = compressed
        self.jpeg_quality = jpeg_quality
        self.sync_mode = sync_mode
        self.min_interval = 1.0 / max(target_fps, 1.0)
        self.last_pub_time = 0.0
        self.frame_count = 0
        self.fps_calc_time = time.monotonic()
        self._has_first_frame = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        msg_type = CompressedImage if self.compressed else Image
        self.left_topic = left_topic
        self.right_topic = right_topic
        self.output_topic = output_topic
        self.slop_seconds = slop_seconds
        self.qos = qos
        self.msg_type = CompressedImage if self.compressed else Image

        self.get_logger().info(
            f"Starting Stereo Stitcher:\n"
            f"  Left:   {left_topic}\n"
            f"  Right:  {right_topic}\n"
            f"  Output: {output_topic} ({'CompressedImage' if compressed else 'Image'})\n"
            f"  Slop:   {slop_seconds * 1000:.1f} ms"
        )

        if self.compressed:
            self.publisher = self.create_publisher(CompressedImage, output_topic, qos)
        else:
            self.publisher = self.create_publisher(Image, output_topic, qos)

        self.sub_l = None
        self.sub_r = None
        self.sync = None

        # Probe existing topic type for left_topic immediately
        detected_type = None
        try:
            for name, types in self.get_topic_names_and_types():
                if name == self.left_topic or name.endswith(self.left_topic.lstrip("/")):
                    if "sensor_msgs/msg/Image" in types:
                        detected_type = Image
                    elif "sensor_msgs/msg/CompressedImage" in types:
                        detected_type = CompressedImage
                    break
        except Exception:
            pass

        self.msg_type = detected_type or Image
        self._setup_subscribers(self.msg_type)

        # Dynamic topic type verification timer
        self._type_check_timer = self.create_timer(2.0, self._verify_topic_types)

    def _setup_subscribers(self, msg_type, sync_mode: str | None = None):
        if sync_mode is not None:
            self.sync_mode = sync_mode
        if self.sub_l is not None:
            try:
                if hasattr(self.sub_l, "sub"):
                    self.destroy_subscription(self.sub_l.sub)
            except Exception:
                pass
        if self.sub_r is not None:
            try:
                if hasattr(self.sub_r, "sub"):
                    self.destroy_subscription(self.sub_r.sub)
            except Exception:
                pass
        self.sub_l = None
        self.sub_r = None
        self.sync = None
        self.msg_type = msg_type
        self._latest_left_msg = None
        self._latest_right_msg = None

        type_str = "Image" if msg_type is Image else "CompressedImage"
        self.get_logger().info(f"Subscribing to left/right cameras with msg type: {type_str}, sync_mode: {self.sync_mode}")

        if self.sync_mode == "latest":
            self.sub_l = self.create_subscription(msg_type, self.left_topic, self._on_left_frame, self.qos)
            self.sub_r = self.create_subscription(msg_type, self.right_topic, self._on_right_frame, self.qos)
        elif message_filters is not None:
            self.sub_l = message_filters.Subscriber(self, msg_type, self.left_topic, qos_profile=self.qos)
            self.sub_r = message_filters.Subscriber(self, msg_type, self.right_topic, qos_profile=self.qos)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.sub_l, self.sub_r], queue_size=30, slop=self.slop_seconds
            )
            self.sync.registerCallback(self._synchronized_callback)
        else:
            self.get_logger().warn(
                "message_filters is unavailable, falling back to latest-frame sync."
            )
            self.sub_l = self.create_subscription(msg_type, self.left_topic, self._on_left_frame, self.qos)
            self.sub_r = self.create_subscription(msg_type, self.right_topic, self._on_right_frame, self.qos)

    def _on_left_frame(self, msg: Any) -> None:
        self._latest_left_msg = msg
        self._try_stitch_latest()

    def _on_right_frame(self, msg: Any) -> None:
        self._latest_right_msg = msg
        self._try_stitch_latest()

    def _try_stitch_latest(self) -> None:
        if self._latest_left_msg is None or self._latest_right_msg is None:
            return

        msg_l = self._latest_left_msg
        msg_r = self._latest_right_msg

        pair_key = (
            getattr(msg_l.header.stamp, "sec", 0),
            getattr(msg_l.header.stamp, "nanosec", 0),
            getattr(msg_r.header.stamp, "sec", 0),
            getattr(msg_r.header.stamp, "nanosec", 0),
        )
        if hasattr(self, "_last_stitched_pair") and pair_key == self._last_stitched_pair:
            return

        now = time.monotonic()
        if now - self.last_pub_time < self.min_interval:
            return

        self._last_stitched_pair = pair_key
        img_l = self._decode_frame(msg_l)
        img_r = self._decode_frame(msg_r)
        if img_l is None or img_r is None:
            return

        self._stitch_and_publish(img_l, img_r, msg_l.header)
        self.last_pub_time = now

    def _verify_topic_types(self):
        try:
            for name, types in self.get_topic_names_and_types():
                if name == self.left_topic or name.endswith(self.left_topic.lstrip("/")):
                    detected = None
                    if "sensor_msgs/msg/Image" in types:
                        detected = Image
                    elif "sensor_msgs/msg/CompressedImage" in types:
                        detected = CompressedImage
                    if detected and detected != self.msg_type:
                        self.get_logger().info(
                            f"Auto-detected topic type '{types[0]}' for {self.left_topic}, switching subscribers."
                        )
                        self._setup_subscribers(detected)
                        break
        except Exception:
            pass

    def _decode_frame(self, msg: Any) -> np.ndarray | None:
        try:
            if hasattr(msg, "format") and hasattr(msg, "data"):
                return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if hasattr(msg, "encoding") and hasattr(msg, "data"):
                data = np.frombuffer(msg.data, dtype=np.uint8)
                h, w = msg.height, msg.width
                enc = str(msg.encoding).lower()
                if enc in ("rgb8", "bgr8"):
                    img = data.reshape((h, w, 3))
                    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
                elif enc in ("rgba8", "bgra8"):
                    img = data.reshape((h, w, 4))
                    return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR) if enc == "rgba8" else cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                elif enc in ("mono8", "8uc1"):
                    return cv2.cvtColor(data.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        except Exception as exc:
            self.get_logger().warn(f"Failed to decode frame: {exc}")
        return None

    def _synchronized_callback(self, msg_left: Any, msg_right: Any) -> None:
        now = time.monotonic()
        if now - self.last_pub_time < self.min_interval:
            return

        img_l = self._decode_frame(msg_left)
        img_r = self._decode_frame(msg_right)
        if img_l is None or img_r is None:
            return

        self._stitch_and_publish(img_l, img_r, msg_left.header)
        self.last_pub_time = now

    def _stitch_and_publish(self, img_l: np.ndarray, img_r: np.ndarray, header: Any) -> None:
        # Match vertical resolutions if disparate
        if img_l.shape[0] != img_r.shape[0]:
            target_h = min(img_l.shape[0], img_r.shape[0])
            img_l = cv2.resize(img_l, (int(img_l.shape[1] * target_h / img_l.shape[0]), target_h))
            img_r = cv2.resize(img_r, (int(img_r.shape[1] * target_h / img_r.shape[0]), target_h))

        # Horizontal Side-by-Side concatenation
        sbs = cv2.hconcat([img_l, img_r])

        if self.compressed:
            success, enc = cv2.imencode(".jpg", sbs, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not success:
                return
            out_msg = CompressedImage()
            out_msg.header = header
            out_msg.format = "jpeg"
            out_msg.data = enc.tobytes()
            self.publisher.publish(out_msg)
        else:
            out_msg = Image()
            out_msg.header = header
            out_msg.height, out_msg.width, _ = sbs.shape
            out_msg.encoding = "bgr8"
            out_msg.step = out_msg.width * 3
            out_msg.data = sbs.tobytes()
            self.publisher.publish(out_msg)

        self.frame_count += 1
        now = time.monotonic()
        if not self._has_first_frame:
            self._has_first_frame = True
            self.get_logger().info(f"First stereo frame stitched & published! Size: {sbs.shape[1]}x{sbs.shape[0]}")
        if now - self.fps_calc_time >= 2.0:
            fps = self.frame_count / (now - self.fps_calc_time)
            self.get_logger().info(f"Stereo SBS output active: {fps:.1f} FPS, size={sbs.shape[1]}x{sbs.shape[0]}")
            self.frame_count = 0
            self.fps_calc_time = now

    def _fallback_left_cb(self, msg: Any) -> None:
        self._latest_left = self._decode_frame(msg)

    def _fallback_right_cb(self, msg: Any) -> None:
        self._latest_right = self._decode_frame(msg)

    def _fallback_timer_cb(self) -> None:
        if self._latest_left is not None and self._latest_right is not None:
            header = getattr(self._latest_left, "header", None)
            from std_msgs.msg import Header
            h = Header()
            h.stamp = self.get_clock().now().to_msg()
            self._stitch_and_publish(self._latest_left, self._latest_right, h)


def main():
    parser = argparse.ArgumentParser(description="Stereo SBS Camera Stitcher for ROS 2")
    parser.add_argument("--left", default="/cameras/eye_left/color/compressed", help="Left camera topic")
    parser.add_argument("--right", default="/cameras/eye_right/color/compressed", help="Right camera topic")
    parser.add_argument("--output", default="/cameras/stereo/color/compressed", help="Output SBS topic")
    parser.add_argument("--raw", action="store_true", help="Subscribe and publish raw sensor_msgs/Image")
    parser.add_argument("--fps", type=float, default=30.0, help="Output target FPS")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality (1-100)")
    parser.add_argument("--slop", type=float, default=0.08, help="Approximate time sync window in seconds")
    parser.add_argument("--sync-mode", choices=["latest", "approx"], default="latest", help="Sync mode: 'latest' (highest FPS, zero drops) or 'approx' (timestamp matching)")
    args = parser.parse_args()

    rclpy.init()
    node = StereoSBSStitcher(
        left_topic=args.left,
        right_topic=args.right,
        output_topic=args.output,
        compressed=not args.raw,
        slop_seconds=args.slop,
        target_fps=args.fps,
        jpeg_quality=args.quality,
        sync_mode=args.sync_mode,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

