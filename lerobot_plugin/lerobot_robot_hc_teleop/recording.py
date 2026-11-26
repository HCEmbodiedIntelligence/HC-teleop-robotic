"""Read-only ROS session for HC demonstrations. No command publishers are created."""

import io
import json
import math
from functools import partial
from collections import deque

from PIL import Image as PILImage
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState

from joint_layout import resolve_axes


def encoding(message):
    if isinstance(message, CompressedImage):
        return bytes(message.data)
    mode = {
        "rgb8": "RGB",
        "bgr8": "BGR",
        "rgba8": "RGBA",
        "bgra8": "BGRA",
        "mono8": "L",
    }[message.encoding]
    raw = PILImage.frombytes(
        "RGBA" if len(mode) == 4 else "RGB" if len(mode) == 3 else "L",
        (message.width, message.height),
        bytes(message.data),
        "raw",
        mode,
        message.step,
        1,
    )
    stream = io.BytesIO()
    raw.convert("RGB").save(stream, format="PNG")
    return stream.getvalue()


def stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 10**9 + stamp.nanosec


class RecordingSession:
    def __init__(self, node, api, config):
        self.node = node
        self.info_topic = api.INFO_TOPIC
        node.wait_for(lambda: api.INFO_TOPIC in node.latest, 6)
        self.description = node.latest[api.INFO_TOPIC][0].data
        info = json.loads(self.description)
        api.capability_profile(info)
        recording = info["recording"]
        body = recording["body"]
        groups = info["groups"]
        self.names = list(body["model_names"])
        public_names = list(body["joint_names"])
        units = list(body["units"])
        self.fps = config["fps"]
        self.tick = None
        self.semantics = {}
        topics = {groups["whole_body"]["joint_states"]: set(public_names)}
        self.families = {"body": set(public_names)}
        self.presence = {}
        for group in config["record_hands"]:
            if group not in recording.get("hands", []):
                raise ValueError(f"Unknown HC recording hand {group}")
            spec = groups[group]
            if spec.get("command_semantics"):
                self.semantics[group] = spec["command_semantics"]
                hand_public = spec["joint_names"]
                self.families[group] = set(hand_public)
                topics.setdefault(spec["joint_states"], set()).update(hand_public)
                for public, unit in zip(hand_public, spec["units"], strict=True):
                    if public in public_names:
                        continue
                    public_names.append(public)
                    self.names.append(public)
                    units.append(unit)
        self.axes = resolve_axes(self.names, units, info["joints"])
        if [axis.public for axis in self.axes.values()] != public_names:
            raise ValueError(
                "HC recording layout differs from the joint capability map"
            )
        self.family_for = {
            name: family
            for family, members in self.families.items()
            for name in members
        }
        self.samples = {"state": {}, "action": {}}
        for topic, names in topics.items():
            node.create_subscription(
                JointState,
                topic,
                partial(self.update, "state", names),
                qos_profile_sensor_data,
            )
        command = info["endpoints"]["commanded_joint_states"]
        node.create_subscription(
            JointState,
            command,
            partial(self.update, "action", set(public_names)),
            qos_profile_sensor_data,
        )
        declared = {
            camera["feature"].removeprefix("observation.images."): camera
            for camera in recording.get("cameras", [])
        }
        cameras = config["images"] or {
            name: camera["topic"] for name, camera in declared.items()
        }
        self.cameras = cameras
        self.images = {}
        types = {topic: kinds for topic, kinds in node.get_topic_names_and_types()}
        for topic in set(cameras.values()):
            kinds = types.get(topic, []) + [
                c["type"] for c in declared.values() if c["topic"] == topic
            ]
            cls = Image if "sensor_msgs/msg/Image" in kinds else CompressedImage
            node.create_subscription(
                cls, topic, partial(self.image, topic), qos_profile_sensor_data
            )
        node.wait_for(
            lambda: (
                bool(self.samples["state"])
                and set(cameras.values()) <= self.images.keys()
            ),
            6,
        )
        shapes = {}
        for name, topic in cameras.items():
            with PILImage.open(io.BytesIO(self.images[topic][-1][1])) as image:
                shapes[name] = [image.height, image.width, 3]
        self.layout = dict(
            names=self.names,
            units=units,
            cameras=shapes,
            profile=info["profile_id"],
            semantics=self.semantics,
            revision=info.get("revision"),
        )

    def update(self, kind, allowed, msg):
        stamp = self.source_stamp(msg)
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        present = {
            name for name, value in zip(msg.name, msg.position) if math.isfinite(value)
        }
        for family, names in self.families.items():
            if names.intersection(msg.name):
                self.append(
                    self.presence, (kind, family), stamp, present.intersection(names)
                )
        samples = self.samples[kind]
        for name, value in zip(msg.name, msg.position):
            if name in allowed and math.isfinite(value):
                self.append(samples, name, stamp, value)

    def image(self, topic, msg):
        self.append(self.images, topic, self.source_stamp(msg), encoding(msg))

    def source_stamp(self, msg):
        stamp = stamp_ns(msg)
        received = self.node.get_clock().now().nanoseconds
        return (
            stamp
            if stamp > 0 and -20_000_000 <= received - stamp <= 2_000_000_000
            else received
        )

    @staticmethod
    def append(streams, key, stamp, value):
        history = streams.setdefault(key, deque(maxlen=2048))
        if not history or stamp >= history[-1][0]:
            history.append((stamp, value))
            while len(history) > 1 and stamp - history[0][0] > 3_000_000_000:
                history.popleft()

    @staticmethod
    def latest(history, grid):
        return next(
            (sample for sample in reversed(history) if sample[0] <= grid), (-1, None)
        )

    def dispatch(self, req):
        try:
            if req["op"] == "start_episode":
                self.tick = None
                return {}
            if req["op"] != "sample":
                raise ValueError(
                    "HC demonstration session accepts only sample requests"
                )
            if self.node.latest[self.info_topic][0].data != self.description:
                raise RuntimeError("HC capabilities changed during recording")
            now = self.node.get_clock().now().nanoseconds
            if self.tick is None:
                newest_state = max(
                    history[-1][0] for history in self.samples["state"].values()
                )
                earliest = max(
                    [
                        min(h[0][0] for h in self.samples["state"].values()),
                        *(h[0][0] for h in self.images.values()),
                    ]
                )
                self.tick = max(
                    min(now, newest_state) * self.fps // 10**9,
                    (earliest * self.fps + 10**9 - 1) // 10**9,
                )
            grid = (self.tick * 10**9 + self.fps // 2) // self.fps
            ready = grid + min(50_000_000, 10**9 // self.fps)
            if ready > now:
                return {"pending": (ready - now) / 1e9}
            if now - grid > 2_500_000_000:
                raise RuntimeError(
                    "HC live recording fell behind the retained sample history"
                )
            self.tick += 1
            frame = {"observation.timestamp_ns": [grid]}
            for kind, key, validity, clock in (
                (
                    "state",
                    "observation.state",
                    "observation.state_valid",
                    "observation.state_time_ns",
                ),
                ("action", "action", "action.valid", "action.source_time_ns"),
            ):
                values, valid, stamps = [], [], []
                for axis in self.axes.values():
                    stamp, value = self.latest(
                        self.samples[kind].get(axis.public, ()), grid
                    )
                    available = 0 < stamp <= grid
                    values.append(
                        value * axis.scale + axis.offset if available else 0.0
                    )
                    active = available and grid - stamp <= 200_000_000
                    if active:
                        _, present = self.latest(
                            self.presence.get((kind, self.family_for[axis.public]), ()),
                            grid,
                        )
                        active = present is not None and axis.public in present
                    valid.append(active)
                    stamps.append(stamp if available else -1)
                frame.update({key: values, validity: valid, clock: stamps})
            images = {}
            if self.cameras:
                picked = [
                    self.latest(self.images[t], grid) for t in self.cameras.values()
                ]
                frame["observation.camera_time_ns"] = [stamp for stamp, _ in picked]
                frame["observation.camera_valid"] = [
                    stamp > 0 and grid - stamp <= 250_000_000 for stamp, _ in picked
                ]
                images = {
                    name: (stamp, data)
                    for name, (stamp, data) in zip(self.cameras, picked, strict=True)
                }
            source_stamps = {
                "observation.state": max(frame["observation.state_time_ns"]),
                "action": max(frame["action.source_time_ns"]),
            }
            source_stamps.update(
                {
                    "observation.images." + name: stamp
                    for name, (stamp, _) in images.items()
                }
            )
            frame["observation.source_timestamp_ns"] = [
                source_stamps[key] for key in sorted(source_stamps)
            ]
            return {"frame": frame, "_images": images}
        except Exception as exc:
            return {"error": f"{type(exc).__name__}: {exc}"}
