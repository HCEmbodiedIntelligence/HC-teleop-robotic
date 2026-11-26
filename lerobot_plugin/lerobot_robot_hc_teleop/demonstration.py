"""Teleoperator reading the HC targets paired with the most recent observation."""

from dataclasses import dataclass
from weakref import WeakValueDictionary

from lerobot.teleoperators import Teleoperator, TeleoperatorConfig

# Native lerobot-record owns both devices in the same process. Index by robot id
# to avoid opening a second ROS subscriber or sampling a different timestep.
readers = WeakValueDictionary()


@TeleoperatorConfig.register_subclass("hc_demonstration")
@dataclass(kw_only=True)
class HCDemonstrationConfig(TeleoperatorConfig):
    robot_id: str | None = None


class HCDemonstration(Teleoperator):
    config_class = HCDemonstrationConfig
    name = "hc_demonstration"

    def __init__(self, config):
        super().__init__(config)
        self.robot_id = config.robot_id
        self.connected = False

    @property
    def reader(self):
        try:
            return readers[self.robot_id]
        except KeyError as exc:
            raise RuntimeError(
                "Select an HC robot in demonstration mode with the same id"
            ) from exc

    @property
    def action_features(self):
        return self.reader.action_features

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self.connected

    @property
    def is_calibrated(self):
        return True

    def calibrate(self):
        pass

    def configure(self):
        pass

    def connect(self, calibrate=True):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_action(self):
        return self.reader.demonstrated_action()

    def send_feedback(self, feedback):
        pass
