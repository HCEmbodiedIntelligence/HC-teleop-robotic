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

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("hc_teleop")
@dataclass(kw_only=True)
class HCTeleopConfig(RobotConfig):
    """Use joint orders and RGB dimensions from the policy's HC training dataset."""

    record_fps: int = 30
    dataset_info: Path | None = None
    image_topics: dict[str, str] = field(default_factory=dict)
    record_hands: tuple[str, ...] = ()
    groups: tuple[str, ...] = ()
    hc_root: Path = Path("/home/hc_op/workspace/HC-teleop-robotic")
    ros_python: str = "/usr/bin/python3"
