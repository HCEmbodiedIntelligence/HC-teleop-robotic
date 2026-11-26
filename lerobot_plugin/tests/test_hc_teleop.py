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

import json

import pytest

from lerobot.robots import RobotConfig, make_robot_from_config
from lerobot_robot_hc_teleop import HCTeleopConfig


@pytest.fixture
def training_info(tmp_path):
    path = tmp_path / "info.json"
    features = {
        "observation.state": {"names": ["b", "a", "waist"], "shape": [3]},
        "action": {"names": ["a", "b"], "shape": [2]},
        "observation.images.head": {"shape": [480, 640, 3]},
    }
    path.write_text(json.dumps({"features": features}))
    return path


def test_hc_registry_and_training_order(training_info):
    cls = RobotConfig.get_choice_class("hc_teleop")
    assert cls is HCTeleopConfig
    robot = make_robot_from_config(
        cls(dataset_info=training_info, image_topics={"head": "/head/compressed"})
    )
    assert not robot.is_connected
    assert list(robot.observation_features) == ["b.pos", "a.pos", "waist.pos", "head"]
    assert robot.observation_features["head"] == (480, 640, 3)
    assert list(robot.action_features) == ["a.pos", "b.pos"]
    with pytest.raises(ValueError, match="Action keys"):
        robot.send_action({"wrong_joint": 1.0})


def test_missing_camera_mapping(training_info):
    with pytest.raises(ValueError, match="Map each RGB feature"):
        make_robot_from_config(HCTeleopConfig(dataset_info=training_info))


def test_discovered_body_and_binary_hand_coordinates():
    from lerobot_robot_hc_teleop.joint_layout import resolve_axes, select_groups

    joints = [
        {
            "model_name": "old_waist",
            "public_name": "waist",
            "scale": -2,
            "offset": 0.5,
            "unit": "rad",
        },
        {
            "model_name": None,
            "public_name": "left_grip",
            "scale": None,
            "offset": None,
            "unit": "1",
        },
    ]
    axes = resolve_axes(["left_grip", "old_waist"], ["1", "radians"], joints)
    assert list(axes) == ["left_grip", "old_waist"]
    assert axes["left_grip"].scale == 1
    axis = axes["old_waist"]
    assert axis.public == "waist" and axis.scale * 0.25 + axis.offset == 0
    with pytest.raises(ValueError, match="same servo_j axis"):
        resolve_axes(["waist", "old_waist"], None, joints)
    with pytest.raises(ValueError, match="unit"):
        resolve_axes(["old_waist"], ["m"], joints)
    groups = {
        name: {"joint_names": names, "endpoints": {"servo_j": name}}
        for name, names in {
            "left_arm": ["l"],
            "right_arm": ["r"],
            "whole_body": ["l", "r", "waist"],
            "left_hand": ["left_grip"],
        }.items()
    }
    assert select_groups(groups, (), ["waist", "r"]) == ["whole_body"]
    assert select_groups(groups, (), ["waist", "left_grip"]) == [
        "whole_body",
        "left_hand",
    ]
