# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Private HC ROS process. The inherited Unix socket belongs to one LeRobot instance."""

import json
import math
import socket
import sys
import time
from pathlib import Path
from threading import Event, Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import CompressedImage, JointState

from joint_layout import resolve_axes, select_groups


class Session:
    def __init__(self, node, api, config):
        self.node = node
        self.api = api
        info = node.discover(6)
        self.description = node.latest[api.INFO_TOPIC][0].data
        self.states = resolve_axes(
            config["states"], config.get("state_units"), info["joints"]
        )
        self.actions = resolve_axes(
            config["actions"], config.get("action_units"), info["joints"]
        )
        self.public_actions = [axis.public for axis in self.actions.values()]
        self.binary = {
            j["public_name"]
            for j in info["joints"]
            if j["unit"] == "1" and j["public_name"] in {"left_grip", "right_grip"}
        }
        self.images = config["images"]
        self.epoch = None
        self.age = info["servo"]["feedback_max_age_ms"] / 1000
        self.input_age = info["servo"]["input_stamp_max_age_s"]
        self.groups = []
        covered = set()
        for group in select_groups(
            info["groups"], config["groups"], self.public_actions
        ):
            spec = info["groups"][group]
            endpoint = spec["endpoints"]["servo_j"]
            joints = [
                (n, limit)
                for n, limit in zip(spec["joint_names"], spec["limits"], strict=True)
                if n in self.public_actions
            ]
            if covered.intersection(n for n, _ in joints):
                raise ValueError("Selected servo_j groups overlap")
            covered.update(n for n, _ in joints)
            if joints:
                self.groups.append(
                    (node.create_publisher(JointState, endpoint, 1), joints)
                )
        if covered != set(self.public_actions):
            raise ValueError(
                f"Dataset actions outside selected servo_j groups: {set(self.public_actions) - covered}"
            )
        hand_topics = {
            n: spec["joint_states"]
            for group, spec in info["groups"].items()
            if group.endswith("_hand")
            for n in spec["joint_names"]
        }
        self.feedback = {}
        for name in set(self.public_actions) | {
            axis.public for axis in self.states.values()
        }:
            topic = hand_topics.get(name, info["endpoints"]["joint_states"])
            self.feedback.setdefault(topic, set()).add(name)
            node.subscribe(JointState, topic)
        if node.state_topic not in self.feedback:
            node.state_topic = next(iter(self.feedback))
        self.gated_names = set(self.public_actions) & self.feedback[node.state_topic]
        for topic in set(self.images.values()):
            node.subscribe(CompressedImage, topic)
        node.wait_for(
            lambda: (
                all(node.recent(t, self.age, stamped=True) for t in self.feedback)
                and all(
                    node.recent(t, 0.25, stamped=True) for t in self.images.values()
                )
            ),
            4,
        )

    def measured(self):
        state = {}
        for topic, names in self.feedback.items():
            if not self.node.recent(topic, self.age, stamped=True):
                raise RuntimeError(f"Stale HC joint feedback: {topic}")
            values = self.api.joint_values(self.node.latest[topic][0])
            state.update({key: values[key] for key in names})
        return state

    def dispatch(self, req):
        try:
            return self.handle(req)
        except Exception as exc:
            return {"error": f"{type(exc).__name__}: {exc}"}

    def handle(self, req):
        node = self.node
        if node.latest[self.api.INFO_TOPIC][0].data != self.description:
            raise RuntimeError("HC capabilities changed; reconnect")
        if req["op"] == "observe":
            state = self.measured()
            images = {}
            for name, topic in self.images.items():
                if not node.recent(topic, 0.25, stamped=True):
                    raise RuntimeError(f"Stale HC image: {topic}")
                message = node.latest[topic][0]
                stamp = message.header.stamp
                images[name] = (stamp.sec * 10**9 + stamp.nanosec, message.data)
            # Bind this session to the first authorized observation. A takeover requires reconnect.
            if self.epoch is None and node.gate_problem(self.gated_names) is None:
                self.epoch = node.latest[node.permit_topic][0].frame_id
            return {
                "state": {
                    k: state[axis.public] * axis.scale + axis.offset
                    for k, axis in self.states.items()
                },
                "_images": images,
            }

        if req["op"] != "send":
            raise ValueError("Unknown HC worker operation")
        self.measured()
        problem = node.gate_problem(self.gated_names)
        if problem or self.epoch != node.latest[node.permit_topic][0].frame_id:
            raise RuntimeError(
                problem
                or "No observation under this Motion permit; reconnect after takeover"
            )
        values = req["values"]
        if set(values) != set(self.actions) or not all(
            math.isfinite(v) for v in values.values()
        ):
            raise ValueError("Invalid HC action")
        mask = req.get("valid", [True] * len(self.actions))
        if len(mask) != len(self.actions) or any(
            type(flag) is not bool for flag in mask
        ):
            raise ValueError("Invalid HC action validity mask")
        public = {
            axis.public: (values[key] - axis.offset) / axis.scale
            for (key, axis), valid in zip(self.actions.items(), mask, strict=True)
            if valid
        }
        targets = []
        for publisher, joints in self.groups:
            if not any(name in public for name, _ in joints):
                continue
            if publisher.get_subscription_count() == 0:
                raise RuntimeError("No Motion servo_j subscriber")
            target = JointState()
            for name, limit in joints:
                if name not in public:
                    continue
                value = public[name]
                if name in self.binary and value not in (0.0, 1.0):
                    raise ValueError(f"{name} requires 0 (open) or 1 (closed)")
                if not math.isfinite(value):
                    raise ValueError(f"Nonfinite servo_j coordinate for {name}")
                if (limit["lower"] is not None and value < limit["lower"]) or (
                    limit["upper"] is not None and value > limit["upper"]
                ):
                    raise ValueError(f"{name}={value} violates HC position limits")
                target.name.append(name)
                target.position.append(value)
            targets.append((publisher, target))
        if time.monotonic() - req["time"] > self.input_age:
            raise RuntimeError("Stale HC action request")
        stamp = node.get_clock().now().to_msg()
        for publisher, target in targets:
            target.header.stamp = stamp
            publisher.publish(target)
        return {}


def main():
    # Use the same capability parser, freshness rules and permit checks as the HC client.
    sys.path.insert(0, str(Path(sys.argv[2]) / "examples"))
    import robot_api_demo as api

    rclpy.init(args=[])
    node = api.RobotDemo()
    executor = SingleThreadedExecutor()
    spinner = None
    try:
        with (
            socket.socket(fileno=int(sys.argv[1])) as sock,
            sock.makefile("rwb") as stream,
        ):
            session = None
            for line in stream:
                try:
                    req = json.loads(line)
                    if session is None:
                        if req["op"] != "connect":
                            raise ValueError("Expected HC connection request")
                        if req.get("passive"):
                            from recording import RecordingSession

                            session = RecordingSession(node, api, req)
                        else:
                            session = Session(node, api, req)
                        executor.add_node(node)
                        spinner = Thread(target=executor.spin, daemon=True)
                        spinner.start()
                        reply = getattr(session, "layout", {})
                    else:
                        # Serialize each decision with ROS feedback and permit callbacks.
                        done = Event()
                        future = executor.create_task(session.dispatch, req)
                        future.add_done_callback(
                            lambda _, completed=done: completed.set()
                        )
                        if not done.wait(timeout=2):
                            raise TimeoutError(
                                "HC ROS executor did not service the request"
                            )
                        reply = future.result()
                except Exception as exc:
                    reply = {"error": f"{type(exc).__name__}: {exc}"}
                images = reply.pop("_images", {})
                blocks = []
                if images:
                    reply["image_frames"] = {}
                    for name, (stamp, pixels) in images.items():
                        send = (
                            pixels is not None
                            and req.get("known_images", {}).get(name) != stamp
                        )
                        reply["image_frames"][name] = [
                            stamp,
                            len(pixels) if send else 0,
                        ]
                        if send:
                            blocks.append(pixels)
                stream.write(json.dumps(reply, allow_nan=False).encode() + b"\n")
                for block in blocks:
                    stream.write(block)
                stream.flush()
    finally:
        executor.shutdown()
        rclpy.try_shutdown()
        if spinner is not None:
            spinner.join(timeout=1)
        node.destroy_node()


if __name__ == "__main__":
    main()
