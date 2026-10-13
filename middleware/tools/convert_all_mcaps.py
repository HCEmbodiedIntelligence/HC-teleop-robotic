#!/usr/bin/env python3
"""Batch inject 0/1 binary gripper/finger state topics into HC-TJ MCAP recordings."""

import os
import sys
import glob
import tempfile
import shutil
import time
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer as McapWriter
from mcap_ros2.decoder import DecoderFactory
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState


def convert_mcap(in_path: str, out_path: str) -> dict:
    decoder = DecoderFactory()
    stats = {"in_messages": 0, "out_messages": 0, "gripper_events": 0, "transitions": []}
    
    with open(in_path, "rb") as f_in, open(out_path, "wb") as f_out:
        reader = make_reader(f_in)
        writer = McapWriter(f_out)
        writer.start(profile="ros2")

        # 1. Map existing schemas and channels
        old_to_new_schema = {}
        for s_id, s in reader.get_summary().schemas.items():
            new_sid = writer.register_schema(name=s.name, encoding=s.encoding, data=s.data)
            old_to_new_schema[s_id] = new_sid

        old_to_new_channel = {}
        for c_id, c in reader.get_summary().channels.items():
            new_cid = writer.register_channel(
                topic=c.topic,
                message_encoding=c.message_encoding,
                schema_id=old_to_new_schema[c.schema_id],
                metadata=c.metadata,
            )
            old_to_new_channel[c_id] = new_cid

        # Find JointState schema
        joint_state_schema_id = None
        for s_id, s in reader.get_summary().schemas.items():
            if s.name in ("sensor_msgs/JointState", "sensor_msgs/msg/JointState"):
                joint_state_schema_id = old_to_new_schema[s_id]
                break

        # Register new channels
        ch_gripper_state = writer.register_channel(
            topic="/io_teleop/gripper_state",
            message_encoding="cdr",
            schema_id=joint_state_schema_id,
        )
        ch_finger_r = writer.register_channel(
            topic="/io_teleop/joint_cmd_finger_right",
            message_encoding="cdr",
            schema_id=joint_state_schema_id,
        )
        ch_finger_l = writer.register_channel(
            topic="/io_teleop/joint_cmd_finger_left",
            message_encoding="cdr",
            schema_id=joint_state_schema_id,
        )

        prev_r_state = None
        prev_l_state = None
        start_time = None

        for schema, channel, message in reader.iter_messages():
            stats["in_messages"] += 1
            if start_time is None:
                start_time = message.log_time

            # Copy existing message
            writer.add_message(
                channel_id=old_to_new_channel[channel.id],
                log_time=message.log_time,
                data=message.data,
                publish_time=message.publish_time,
                sequence=message.sequence,
            )
            stats["out_messages"] += 1

            # When receiving hand_trigger, map to 0/1 binary states
            if channel.topic == "/io_teleop/hand_trigger":
                ros_msg = decoder.decoder_for(channel.message_encoding, schema)(message.data)
                r_trig = ros_msg.poses[1].position.x if len(ros_msg.poses) > 1 else 0.0
                l_trig = ros_msg.poses[0].position.x if len(ros_msg.poses) > 0 else 0.0

                r_01 = 1.0 if r_trig >= 0.5 else 0.0
                l_01 = 1.0 if l_trig >= 0.5 else 0.0

                if r_01 != prev_r_state:
                    rel_t = (message.log_time - start_time) / 1e9
                    stats["transitions"].append((round(rel_t, 2), "R", r_01))
                    prev_r_state = r_01
                if l_01 != prev_l_state:
                    rel_t = (message.log_time - start_time) / 1e9
                    stats["transitions"].append((round(rel_t, 2), "L", l_01))
                    prev_l_state = l_01

                # 1. /io_teleop/gripper_state
                js = JointState()
                js.header.stamp.sec = int(message.log_time // 1_000_000_000)
                js.header.stamp.nanosec = int(message.log_time % 1_000_000_000)
                js.name = ["R_hand", "L_hand"]
                js.position = [float(r_01), float(l_01)]
                writer.add_message(
                    channel_id=ch_gripper_state,
                    log_time=message.log_time,
                    data=serialize_message(js),
                    publish_time=message.publish_time,
                )

                # 2. /io_teleop/joint_cmd_finger_right
                js_r = JointState()
                js_r.header = js.header
                js_r.name = ["R_ban"]
                js_r.position = [float(r_01)]
                writer.add_message(
                    channel_id=ch_finger_r,
                    log_time=message.log_time,
                    data=serialize_message(js_r),
                    publish_time=message.publish_time,
                )

                # 3. /io_teleop/joint_cmd_finger_left
                js_l = JointState()
                js_l.header = js.header
                js_l.name = ["L_ban"]
                js_l.position = [float(l_01)]
                writer.add_message(
                    channel_id=ch_finger_l,
                    log_time=message.log_time,
                    data=serialize_message(js_l),
                    publish_time=message.publish_time,
                )

                stats["out_messages"] += 3
                stats["gripper_events"] += 1

        writer.finish()

    return stats


def main():
    target_dir = "/home/maple/Downloads/8-18单物体抓放50条/mcaps(1)"
    mcap_files = sorted(glob.glob(os.path.join(target_dir, "**/*.mcap"), recursive=True))
    print(f"Starting batch conversion of {len(mcap_files)} mcap files...")

    t0 = time.time()
    for idx, in_path in enumerate(mcap_files):
        rel_name = os.path.relpath(in_path, target_dir)
        dir_name = os.path.dirname(in_path)
        base_name = os.path.basename(in_path)

        with tempfile.NamedTemporaryFile(suffix=".mcap", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            stats = convert_mcap(in_path, tmp_path)
            shutil.move(tmp_path, in_path)
            print(f"[{idx+1:02d}/{len(mcap_files):02d}] Converted: {rel_name} | Gripper msgs: {stats['gripper_events']*3} | Transitions: {stats['transitions']}")
        except Exception as e:
            print(f"[{idx+1:02d}/{len(mcap_files):02d}] ERROR on {rel_name}: {e}")
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    elapsed = time.time() - t0
    print(f"\nAll {len(mcap_files)} files successfully converted in {elapsed:.2f}s!")


if __name__ == "__main__":
    main()
