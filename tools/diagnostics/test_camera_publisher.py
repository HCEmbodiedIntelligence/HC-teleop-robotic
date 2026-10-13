#!/usr/bin/env python3
"""Publish a moving, numbered ROS image for camera transport diagnostics."""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/hc_teleop/test_camera/color')
    parser.add_argument('--fps', type=float, default=20)
    parser.add_argument('--width', type=int, default=320)
    parser.add_argument('--height', type=int, default=240)
    args = parser.parse_args()
    if args.fps <= 0 or min(args.width, args.height) < 100:
        parser.error('fps must be positive and dimensions at least 100')
    rclpy.init()
    node = Node('hc_camera_test_publisher')
    publisher = node.create_publisher(Image, args.topic, qos_profile_sensor_data)
    counter = 0
    start = time.monotonic()

    def publish():
        nonlocal counter
        frame = np.zeros((args.height, args.width, 3), dtype=np.uint8)
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)]
        for i, color in enumerate(colors):
            frame[:, i * args.width // 4:(i + 1) * args.width // 4] = color
        x = (counter * 5) % args.width
        cv2.line(frame, (x, 0), (x, args.height - 1), (255, 255, 255), 4)
        cv2.rectangle(frame, (0, 0), (args.width, 82), (0, 0, 0), -1)
        for row, text in enumerate(['ROS CAMERA TEST', f'frame {counter:06d}', f'{time.monotonic()-start:.1f}s / {args.fps:g} Hz']):
            cv2.putText(frame, text, (8, 23 + row * 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
        msg = Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_test_optical_frame'
        msg.height, msg.width = args.height, args.width
        msg.encoding = 'bgr8'
        msg.step = args.width * 3
        msg.data = frame.tobytes()
        publisher.publish(msg)
        counter += 1

    node.create_timer(1 / args.fps, publish)
    node.get_logger().info(f'{args.topic}: {args.width}x{args.height} bgr8 at {args.fps:g} Hz')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
