#!/usr/bin/env python3
"""订阅 main 的 VLA 状态/动作；--output 可保存原始 ROS 2 CDR MCAP。"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / ".deps")]

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


STATE = "/observation/joint_states"
ACTION = "/action/joint_states"
SOURCE = "/control/source"
DEFAULT_TOPICS = {
    STATE: "sensor_msgs/msg/JointState",
    ACTION: "sensor_msgs/msg/JointState",
    "/observation/hands/joint_states": "sensor_msgs/msg/JointState",
    "/action/left_hand/joint_states": "sensor_msgs/msg/JointState",
    "/action/right_hand/joint_states": "sensor_msgs/msg/JointState",
    "/status/hardware/ready": "std_msgs/msg/Bool",
    "/control/emergency_stop": "std_msgs/msg/Bool",
    SOURCE: "std_msgs/msg/String",
}


class VlaTopicsDemo(Node):
    def __init__(self, topics, output=None, task="", robot_type="unspecified"):
        super().__init__("hc_vla_topics_demo")
        self.counts = dict.fromkeys(topics, 0)
        self.latest = {}
        self.output = Path(output) if output else None
        self.stream = self.writer = None
        self.channels = {}
        self.subscribers = []
        try:
            if self.output:
                from mcap.writer import Writer
                from middleware.core.topic_recorder import _get_msg_def

                self.output.parent.mkdir(parents=True, exist_ok=True)
                self.stream = self.output.open("xb")  # 不覆盖已有采集。
                self.writer = Writer(self.stream)
                self.writer.start(profile="ros2")
                self.writer.add_metadata("hc_teleop.session", {
                    "task": task,
                    "robot_type": robot_type,
                    "interface": "hc-teleop-hardware-interface/v2 (main)",
                    "action_semantics": "selected_joint_target_not_execution_confirmation",
                    "log_time_semantics": "receiver_system_time_ns",
                    "publish_time_semantics": "source_header_stamp_or_receive_time",
                    "topics": json.dumps(topics, ensure_ascii=False),
                })
                schemas = {}
                for topic, kind in topics.items():
                    if kind not in schemas:
                        definition = _get_msg_def(kind)
                        if not definition:
                            raise ValueError(f"找不到消息定义：{kind}，请 source 对应消息包")
                        schemas[kind] = self.writer.register_schema(
                            name=kind, encoding="ros2msg", data=definition)
                    self.channels[topic] = self.writer.register_channel(
                        topic=topic, message_encoding="cdr", schema_id=schemas[kind])

            for topic, kind in topics.items():
                # Best Effort 订阅兼容两种发布可靠性；source 需要读取锁存值。
                latched = topic in (SOURCE, "/tf_static")
                qos = QoSProfile(
                    depth=100 if latched or "Image" not in kind else 10,
                    reliability=(ReliabilityPolicy.RELIABLE if latched
                                 else ReliabilityPolicy.BEST_EFFORT),
                    durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                                else DurabilityPolicy.VOLATILE),
                )
                self.subscribers.append(self.create_subscription(
                    get_message(kind), topic,
                    lambda msg, name=topic: self.on_message(name, msg), qos))
            self.create_timer(1.0, self.show_status)
        except BaseException:
            self.close()
            self.destroy_node()
            raise

    def on_message(self, topic, message):
        received_ns = time.time_ns()
        if self.writer:
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec if stamp else 0
            self.writer.add_message(
                channel_id=self.channels[topic], data=serialize_message(message),
                log_time=received_ns, publish_time=source_ns if source_ns > 0 else received_ns,
                sequence=self.counts[topic],
            )
        self.counts[topic] += 1
        if topic in (STATE, ACTION):
            # 仅展示原始字段；不把两条异步消息拼成已同步的训练帧。
            self.latest[topic] = {
                "name": list(message.name), "position": list(message.position),
                "velocity": list(message.velocity), "effort": list(message.effort),
                "source_stamp_ns": message.header.stamp.sec * 1_000_000_000
                                   + message.header.stamp.nanosec,
                "received_monotonic": time.monotonic(),
            }

    def show_status(self):
        now = time.monotonic()
        latest = {topic: {**{k: v for k, v in value.items() if k != "received_monotonic"},
                          "receive_age_sec": round(now - value["received_monotonic"], 3)}
                  for topic, value in self.latest.items()}
        print(json.dumps({"counts": self.counts, "latest_unsynchronized": latest},
                         ensure_ascii=False), flush=True)

    def close(self):
        try:
            if self.writer:
                self.writer.finish()
        finally:
            self.writer = None
            if self.stream:
                self.stream.close()
                self.stream = None


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("必须是大于零的有限数")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="输出 .mcap；省略则仅订阅显示")
    parser.add_argument("--duration", type=positive, help="秒；省略则 Ctrl+C 结束")
    parser.add_argument("--task", default="", help="本段语言任务描述")
    parser.add_argument("--robot-type", default="unspecified", help="当前机型，如 x1/openarmx")
    parser.add_argument("--camera", action="append", default=[], metavar="TOPIC",
                        help="附加 CompressedImage 相机话题，可重复")
    parser.add_argument("--topic", nargs=2, action="append", default=[],
                        metavar=("TOPIC", "TYPE"), help="附加任意话题和消息类型，可重复")
    args, ros_args = parser.parse_known_args()
    if args.output and args.output.suffix.lower() != ".mcap":
        parser.error("--output 应以 .mcap 结尾")
    topics = dict(DEFAULT_TOPICS)
    for topic, kind in [(t, "sensor_msgs/msg/CompressedImage") for t in args.camera] + args.topic:
        if not topic.startswith("/") or len(kind.split("/")) != 3:
            parser.error("话题需要绝对路径；消息类型格式为 sensor_msgs/msg/JointState")
        if topic in topics and topics[topic] != kind:
            parser.error(f"同一话题指定了不同类型：{topic}")
        topics[topic] = kind

    rclpy.init(args=ros_args)
    node = None
    try:
        node = VlaTopicsDemo(topics, args.output, args.task, args.robot_type)
        deadline = time.monotonic() + args.duration if args.duration else math.inf
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        node.close()
        required = {STATE, ACTION, *args.camera}
        required.update(t for t, kind in args.topic if kind in (
            "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"))
        missing = sorted(t for t in required if not node.counts[t])
        print(json.dumps({"output": str(args.output) if args.output else None,
                          "counts": node.counts, "missing_required_topics": missing},
                         ensure_ascii=False), flush=True)
        if missing:
            print("状态、动作或所选相机缺少消息；请检查运行端、ROS 域和话题。"
                  + ("已有原始数据保留，尚不能组成完整采集。" if args.output else ""),
                  file=sys.stderr)
        return 2 if args.output and missing else 0
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
