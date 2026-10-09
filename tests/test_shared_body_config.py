"""Launch mode must never override body control parameters."""
import os
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('pybullet')
from adapters.core.arm_teleop_node import RobotArmTeleopNode


def test_sim_and_teleop_load_the_same_body_parameters():
    path = Path(__file__).parents[1] / 'adapters/robots/x1/arm_teleop.yaml'
    config = yaml.safe_load(path.read_text())
    assert 'simulation_body' not in config
    rclpy.init(domain_id=169)
    try:
        for mode in ('sim', 'teleop'):
            with patch.dict(os.environ, {'HC_TELEOP_MODE': mode}):
                node = RobotArmTeleopNode(str(path), backend='v23')
            try:
                assert node.body_config == config['body']
                assert node.body.joint_names == config['body']['waist_joint_names']
                assert node.head_base_follower is not None
                assert node.base_pose_sub.topic_name == '/observation/base/pose'
            finally:
                node.destroy_node()
    finally:
        rclpy.shutdown()
