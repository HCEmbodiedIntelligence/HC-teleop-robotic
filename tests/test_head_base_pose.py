"""Measured planar pose following, reference capture and safety stops."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
from adapters.core.head_base_pose import HeadBasePoseFollower, rotate, wrap
from adapters.core.arm_teleop_math import quaternion_from_axis_angle

MAPPING = [[0, 0, 1], [-1, 0, 0], [0, 1, 0]]
LIMITS = (.35, .25, .45)

def head(x=0., z=0., yaw=0., height=0.):
    return (np.array([x, height, z]), quaternion_from_axis_angle([0, 1, 0], yaw))

def test_combined_motion_reaches_measured_target_and_stops():
    f = HeadBasePoseFollower({}, MAPPING)
    base = np.array([1., 2., 1.2])
    assert f.update(head(), base, LIMITS) == (0., 0., 0.)
    target = head(x=-.2, z=.3, yaw=-.4)
    command = f.update(target, base, LIMITS)
    assert all(abs(v) > .01 for v in command)
    original = f.target.copy()
    # A stalled base must keep the same target/error, not integrate commands.
    for _ in range(20):
        assert f.update(target, base, LIMITS) == command
        assert f.target == original
    for _ in range(2000):
        yaw, vx, vy = f.update(target, base, LIMITS)
        base[:2] += rotate((vx, vy), base[2]) * .01
        base[2] = wrap(base[2] + yaw * .01)
    assert np.linalg.norm(base[:2] - original[:2]) < .01001
    assert abs(wrap(base[2] - original[2])) < .03001
    assert max(abs(v) for v in f.update(target, base, LIMITS)) < 1e-6

def test_reference_heading_and_vertical_motion():
    f = HeadBasePoseFollower({}, MAPPING)
    reference = head(yaw=math.pi / 2)
    f.update(reference, (3., 4., math.pi / 2), LIMITS)
    # Initial head forward points along VR +X. Robot forward points world +Y.
    f.update(head(x=.2, yaw=math.pi / 2, height=2.), (3., 4., math.pi / 2), LIMITS)
    assert f.target == pytest.approx([3., 4.2, math.pi / 2])

def test_limits_wrap_and_reference_reset():
    f = HeadBasePoseFollower({'max_displacement': .2, 'max_yaw': .5}, MAPPING)
    f.update(head(), (0., 0., 3.), LIMITS)
    yaw, vx, vy = f.update(head(x=-3., z=4., yaw=-1.), (0., 0., 3.), LIMITS)
    assert np.linalg.norm(f.target[:2]) == pytest.approx(.2)
    assert f.target[2] == pytest.approx(wrap(3.5))
    assert 0 < yaw <= LIMITS[2]
    assert abs(vx) <= LIMITS[0] and abs(vy) <= LIMITS[1]
    f.reset()
    assert f.update(head(x=-3., z=4., yaw=-1.), (1., 2., 3.), LIMITS) == (0., 0., 0.)
    assert f.target == pytest.approx([1., 2., 3.])

def test_invalid_input_and_limits_stop():
    f = HeadBasePoseFollower({}, MAPPING)
    assert f.update(head(z=float('nan')), (0., 0., 0.), LIMITS) == (0., 0., 0.)
    assert f.head_reference is None
    assert f.update(head(), (0., 0., 0.), (0., .1, .1)) == (0., 0., 0.)

@pytest.mark.parametrize('case', ['release', 'head_timeout', 'base_timeout'])
def test_node_stops_and_recaptures_after_interruption(case):
    pytest.importorskip('rclpy')
    pytest.importorskip('pybullet')
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    published = []
    node = SimpleNamespace(
        head_base_follower=HeadBasePoseFollower({}, MAPPING),
        body=SimpleNamespace(head_pose=head(), head_stamp=10.),
        base_pose_feedback=(0., 0., 0.), base_pose_stamp=10.,
        control={'pose_timeout': .5, 'rate_hz': 100.}, body_config={},
        base_pub=SimpleNamespace(publish=lambda msg: published.append(list(msg.data))),
        base_zero_pending=False,
    )
    node._stop_base_if_pending = lambda: RobotArmTeleopNode._stop_base_if_pending(node)
    RobotArmTeleopNode._publish_head_base_pose(node, 10., True)
    node.body.head_pose = head(z=.3, yaw=-.2)
    RobotArmTeleopNode._publish_head_base_pose(node, 10., True)
    assert any(abs(v) > .01 for v in published[-1])
    if case == 'head_timeout': node.body.head_stamp = 8.
    if case == 'base_timeout': node.base_pose_stamp = 8.
    RobotArmTeleopNode._publish_head_base_pose(node, 10., case != 'release')
    assert published[-1] == [0., 0., 0.]
    assert node.head_base_follower.target is None
    node.body.head_stamp = node.base_pose_stamp = 10.
    RobotArmTeleopNode._publish_head_base_pose(node, 10., True)
    assert published[-1] == [0., 0., 0.]

@pytest.mark.parametrize('base_yaw', [0., .8, -1.2])
@pytest.mark.parametrize('head_yaw', [0., .9])
def test_x1_forward_matches_model_front(base_yaw, head_yaw):
    """Use actual X1 FK, not an assumed command-X forward direction."""
    from pathlib import Path
    import yaml
    p = pytest.importorskip('pybullet')
    root = Path(__file__).parents[1]
    cfg = yaml.safe_load((root / 'adapters/robots/x1/arm_teleop.yaml').read_text())
    client = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(str(root / 'adapters/robots/x1/urdf/hc_tj_robot.urdf'),
                           baseOrientation=p.getQuaternionFromEuler([0, 0, math.pi / 2 + base_yaw]),
                           useFixedBase=True, physicsClientId=client)
        for i in range(p.getNumJoints(robot, physicsClientId=client)):
            info = p.getJointInfo(robot, i, physicsClientId=client)
            if info[1].decode() in cfg['robot']['initial_joints']:
                p.resetJointState(robot, i, cfg['robot']['initial_joints'][info[1].decode()], physicsClientId=client)
            if info[12].decode() == 'camera_head': camera = i
        quat = p.getLinkState(robot, camera, computeForwardKinematics=True, physicsClientId=client)[5]
        front = (np.asarray(p.getMatrixFromQuaternion(quat)).reshape(3, 3) @ [0, 0, 1])[:2]
        front /= np.linalg.norm(front)
        f = HeadBasePoseFollower(cfg['body']['head_pose_base'], cfg['control']['head_axis_mapping'])
        f.update(head(yaw=head_yaw), (0., 0., base_yaw + math.pi / 2), LIMITS)
        for distance in [.2, -.2]:
            # Move along the captured headset's actual horizontal forward.
            yaw, vx, vy = f.update(head(x=distance * math.sin(head_yaw),
                                       z=distance * math.cos(head_yaw), yaw=head_yaw),
                                   (0., 0., base_yaw + math.pi / 2), LIMITS)
            world_velocity = rotate((vx, vy), base_yaw + math.pi / 2)
            assert np.dot(world_velocity, front) * distance > 0
            assert abs(np.cross(world_velocity, front)) < 1e-5
            assert yaw == pytest.approx(0.)
    finally:
        p.disconnect(client)

@pytest.mark.parametrize('grip', [False, True])
@pytest.mark.parametrize('velocity', [(.2, 0.), (-.2, 0.), (0., .2), (0., -.2)])
def test_manual_stick_overrides_pose_and_recaptures(grip, velocity):
    pytest.importorskip('rclpy')
    pytest.importorskip('pybullet')
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from unittest.mock import Mock
    follower = HeadBasePoseFollower({}, MAPPING)
    follower.update(head(), (0,0,0), LIMITS)
    node = SimpleNamespace(head_base_follower=follower, body_config={},
        _compute_user_base_velocity=Mock(return_value=(*velocity,0.)),
        _publish_head_base_pose=Mock(), base_pub=Mock())
    RobotArmTeleopNode._publish_base_command(node, 10., left_clutch_active=grip)
    assert list(node.base_pub.publish.call_args.args[0].data) == pytest.approx([0., *velocity])
    assert follower.target is None
    node._publish_head_base_pose.assert_not_called()
    node._compute_user_base_velocity.return_value=(0.,0.,0.)
    RobotArmTeleopNode._publish_base_command(node, 10., left_clutch_active=grip)
    node._publish_head_base_pose.assert_called_once_with(10.,grip)

@pytest.mark.parametrize('xy, expected', [((0.,1.),(1,0)),((0.,-1.),(-1,0)),
                                         ((-1.,0.),(0,1)),((1.,0.),(0,-1))])
def test_real_joystick_axes_to_command_frame(xy, expected):
    pytest.importorskip('rclpy')
    pytest.importorskip('pybullet')
    import yaml
    from pathlib import Path
    from sensor_msgs.msg import Joy
    from unittest.mock import Mock
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    cfg=yaml.safe_load((Path(__file__).parents[1]/'adapters/robots/x1/arm_teleop.yaml').read_text())
    node=SimpleNamespace(body_config=cfg['body'], control=cfg['control'],
        arms={'left':SimpleNamespace(joy=Joy(axes=[0.,0.,*xy])),
              'right':SimpleNamespace(joy=Joy(axes=[0.,0.,0.,0.]))},
        _input_fresh=lambda arm,now:True,
        head_base_follower=HeadBasePoseFollower(cfg['body']['head_pose_base'],MAPPING),
        base_pub=Mock(), _publish_head_base_pose=Mock())
    node._compute_user_base_velocity=lambda *a,**k:RobotArmTeleopNode._compute_user_base_velocity(node,*a,**k)
    RobotArmTeleopNode._publish_base_command(node,10.,left_clutch_active=False)
    yaw,forward,lateral=node.base_pub.publish.call_args.args[0].data
    assert yaw == 0.
    assert np.sign([forward,lateral]).tolist() == list(expected)
