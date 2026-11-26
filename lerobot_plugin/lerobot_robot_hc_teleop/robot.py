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

import io
import json
import socket
import subprocess
import time
from pathlib import Path
from threading import Lock

import numpy as np
from PIL import Image

from lerobot.robots.robot import Robot

from .config import HCTeleopConfig
from .demonstration import readers


class HCTeleop(Robot):
    config_class = HCTeleopConfig
    name = "hc_teleop"

    def __init__(self, config: HCTeleopConfig):
        super().__init__(config)
        self.config = config
        self._process = self._stream = None
        self._lock = Lock()
        self.recording_layout = None
        self._image_cache = {}
        self.states, self.actions, self.images = [], [], {}
        self.state_units = self.action_units = None
        self._demonstrated = None
        self._discovered = None
        if config.dataset_info is None:
            if not 1 <= config.record_fps <= 120:
                raise ValueError("record_fps must be between 1 and 120")
            readers[self.id] = self
            return
        features = json.loads(config.dataset_info.read_text())["features"]

        def joint_names(key: str) -> list[str]:
            feature = features[key]
            names = feature["names"]
            if (
                not isinstance(names, list)
                or not names
                or not all(isinstance(n, str) and n for n in names)
            ):
                raise ValueError(f"Expected named joint positions in {key}")
            if len(set(names)) != len(names) or feature["shape"] != [len(names)]:
                raise ValueError(f"Invalid joint layout in {key}")
            return names

        self.states = joint_names("observation.state")
        self.actions = joint_names("action")
        self.state_units = features["observation.state"].get("units")
        self.action_units = features["action"].get("units")
        if (mask := features.get("action.valid")) and (
            mask["dtype"] != "bool"
            or mask["names"] != self.actions
            or mask["shape"] != [len(self.actions)]
        ):
            raise ValueError("action.valid must follow the training action layout")
        self.images = {
            key.removeprefix("observation.images."): tuple(spec["shape"])
            for key, spec in features.items()
            if key.startswith("observation.images.")
        }
        if set(config.image_topics) != self.images.keys():
            raise ValueError(
                f"Map each RGB feature to its HC CompressedImage topic: {list(self.images)}"
            )
        if self.images.keys() & set(self.states) or any(
            len(s) != 3 or s[-1] != 3 for s in self.images.values()
        ):
            raise ValueError("Expected distinct HWC RGB image features")

    @property
    def action_features(self) -> dict[str, type]:
        self._discover_features()
        return dict.fromkeys(self._action_keys(), float)

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        self._discover_features()
        return dict.fromkeys(self._state_keys(), float) | self.images

    def _state_keys(self):
        return (
            self.states
            if self.config.dataset_info is None
            else [k + ".pos" for k in self.states]
        )

    def _action_keys(self):
        return (
            self.actions
            if self.config.dataset_info is None
            else [k + ".pos" for k in self.actions]
        )

    @property
    def cameras(self):
        self._discover_features()
        return self.images

    def _discover_features(self):
        if self.config.dataset_info is None and self._discovered is None:
            try:
                self.connect_demonstration(self.config.record_fps, True)
            finally:
                self.disconnect()

    @property
    def is_connected(self) -> bool:
        return self._process is not None and self._process.poll() is None

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        if self.config.dataset_info is None:
            self.connect_demonstration(self.config.record_fps, True)
            return
        self._connect()

    def _connect(self, *, passive=False, fps=None):
        if self.is_connected:
            raise RuntimeError("HC robot already connected")
        self.disconnect()
        # Humble loads CPython 3.10 extensions; the policy runs in LeRobot's own interpreter.
        parent, child = socket.socketpair()
        try:
            with parent, child:
                parent.settimeout(12)
                self._process = subprocess.Popen(
                    [
                        self.config.ros_python,
                        str(Path(__file__).with_name("ros_worker.py")),
                        str(child.fileno()),
                        str(self.config.hc_root),
                    ],
                    pass_fds=(child.fileno(),),
                )
                self._stream = parent.makefile("rwb")
            layout = self._request(
                "connect",
                passive=passive,
                fps=fps,
                record_hands=self.config.record_hands,
                states=self.states,
                state_units=self.state_units,
                action_units=self.action_units,
                actions=self.actions,
                groups=self.config.groups,
                images=self.config.image_topics,
            )
            if passive:
                if self._discovered is not None and self._discovered != layout:
                    raise RuntimeError(
                        "HC profile or recording layout changed after feature discovery"
                    )
                self._discovered = layout
                self.recording_layout = layout
                self.robot_type = layout["profile"]
                self.states = self.actions = layout["names"]
                self.images = {k: tuple(v) for k, v in layout["cameras"].items()}
        except BaseException:
            self.disconnect()
            raise

    def connect_demonstration(self, fps, video):
        self._connect(passive=True, fps=fps)
        layout = self.recording_layout
        names = layout["names"]
        features = {}
        for key, dtype in (
            ("action", "float32"),
            ("observation.state", "float32"),
            ("action.valid", "bool"),
            ("observation.state_valid", "bool"),
            ("action.source_time_ns", "int64"),
            ("observation.state_time_ns", "int64"),
        ):
            features[key] = dict(dtype=dtype, shape=(len(names),), names=names)
            if dtype == "float32":
                features[key]["units"] = layout["units"]
                if layout["semantics"]:
                    features[key]["hc.hand_semantics"] = layout["semantics"]
        features["observation.timestamp_ns"] = dict(
            dtype="int64", shape=(1,), names=["ros_epoch"]
        )
        for key, shape in self.images.items():
            features["observation.images." + key] = dict(
                dtype="video" if video else "image",
                shape=shape,
                names=["height", "width", "channels"],
            )
        if self.images:
            for field, dtype in (("camera_valid", "bool"), ("camera_time_ns", "int64")):
                features["observation." + field] = dict(
                    dtype=dtype,
                    shape=(len(self.images),),
                    names=["observation.images." + k for k in self.images],
                )
        sources = sorted(
            [
                "observation.state",
                "action",
                *("observation.images." + key for key in self.images),
            ]
        )
        features["observation.source_timestamp_ns"] = dict(
            dtype="int64", shape=(len(sources),), names=sources
        )
        self.recording_features = features
        return features

    def begin_demonstration_episode(self):
        self._request("start_episode")

    def read_demonstration(self):
        reply = self._request("sample")
        while "pending" in reply:
            time.sleep(min(reply["pending"], 0.1))
            reply = self._request("sample")
        frame = {
            key: np.asarray(v, dtype=self.recording_features[key]["dtype"])
            for key, v in reply["frame"].items()
        }
        for name, pixels in reply.get("images", {}).items():
            frame["observation.images." + name] = pixels
        return frame

    def _request(self, op: str, **fields) -> dict:
        with self._lock:
            if not self.is_connected:
                raise RuntimeError(
                    "HC ROS worker unavailable; check the sourced ROS environment"
                )
            request = {"op": op, "time": time.monotonic(), **fields}
            if op in ("observe", "sample"):
                request["known_images"] = {
                    name: item[0] for name, item in self._image_cache.items()
                }
            self._stream.write(json.dumps(request, allow_nan=False).encode() + b"\n")
            self._stream.flush()
            line = self._stream.readline()
            if not line:
                raise RuntimeError("HC ROS worker exited; see its diagnostic output")
            reply = json.loads(line)
            if "error" in reply:
                raise RuntimeError(reply["error"])
            if "image_frames" in reply:
                reply["images"] = {}
                for name, (stamp, size) in reply.pop("image_frames").items():
                    if size:
                        data = self._stream.read(size)
                        if len(data) != size:
                            raise RuntimeError("HC image payload truncated")
                        with Image.open(io.BytesIO(data)) as image:
                            pixels = np.asarray(image.convert("RGB"))
                        if pixels.shape != self.images[name]:
                            raise ValueError(
                                f"Unexpected image shape for {name}: {pixels.shape}"
                            )
                        self._image_cache[name] = stamp, pixels
                    elif stamp < 0:
                        self._image_cache[name] = (
                            stamp,
                            np.zeros(self.images[name], dtype=np.uint8),
                        )
                    cached, pixels = self._image_cache[name]
                    if cached != stamp:
                        raise RuntimeError("HC image cache differs from the ROS sample")
                    reply["images"][name] = pixels
            return reply

    def get_observation(self) -> dict:
        if self.recording_layout is not None:
            sample = self.read_demonstration()
            self._demonstrated = dict(
                zip(self.actions, sample["action"].tolist(), strict=True)
            )
            observation = dict(
                zip(self.states, sample["observation.state"].tolist(), strict=True)
            )
            observation.update(
                {name: sample["observation.images." + name] for name in self.images}
            )
            return observation
        reply = self._request("observe")
        return {
            **{k + ".pos": reply["state"][k] for k in self.states},
            **reply.get("images", {}),
        }

    def demonstrated_action(self):
        if self._demonstrated is None or self.recording_layout is None:
            raise RuntimeError(
                "Read an HC demonstration observation before requesting its paired action"
            )
        return dict(self._demonstrated)

    def send_action(self, action: dict) -> dict:
        if self.config.dataset_info is None:
            if self._demonstrated is None or action != self._demonstrated:
                raise ValueError(
                    "Read-only HC recording accepts only its paired demonstration"
                )
            return dict(self._demonstrated)
        if set(action) - {"action.valid"} != set(self._action_keys()):
            raise ValueError("Action keys differ from the HC training dataset")
        values = {name: float(action[name + ".pos"]) for name in self.actions}
        if "action.valid" in action:
            valid = np.asarray(action["action.valid"])
            if valid.dtype != np.bool_ or valid.shape != (len(self.actions),):
                raise ValueError("Invalid HC action validity mask")
            values = {
                name: value if enabled else 0.0
                for (name, value), enabled in zip(values.items(), valid, strict=True)
            }
            self._request("send", values=values, valid=valid.tolist())
            return dict(action)
        self._request("send", values=values)
        return dict(action)

    def disconnect(self) -> None:
        self._demonstrated = None
        self._image_cache.clear()
        self.recording_layout = None
        if self._stream is not None:
            self._stream.close()
            self._stream = None
        if self._process is not None:
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            self._process = None
