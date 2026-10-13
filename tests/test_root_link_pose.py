"""Root task frames must work for imported fixed-base robot profiles."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

try:
    import pybullet as bullet
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
except ImportError:
    RobotArmTeleopNode = None


@unittest.skipIf(RobotArmTeleopNode is None, 'ROS/PyBullet dependencies unavailable')
class RootLinkPoseTests(unittest.TestCase):
    def test_root_link_pose_removes_inertial_offset(self):
        with tempfile.TemporaryDirectory() as temp:
            urdf = Path(temp) / 'offset.urdf'
            urdf.write_text('''<robot name="offset"><link name="base">
              <inertial><origin xyz="0.2 0.1 0" rpy="0 0 0.5"/><mass value="1"/>
              <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>
              </link></robot>''')
            client = bullet.connect(bullet.DIRECT)
            try:
                body = bullet.loadURDF(str(urdf), basePosition=[1,2,3], useFixedBase=True, physicsClientId=client)
                node = SimpleNamespace(robot_id=body, physics_client=client)
                node._root_pose = lambda: bullet.getBasePositionAndOrientation(body,physicsClientId=client)
                position, orientation = RobotArmTeleopNode._link_pose(node,-1)
                np.testing.assert_allclose(position,[1,2,3],atol=1e-6)
                np.testing.assert_allclose(orientation,[0,0,0,1],atol=1e-6)
            finally:
                bullet.disconnect(client)
