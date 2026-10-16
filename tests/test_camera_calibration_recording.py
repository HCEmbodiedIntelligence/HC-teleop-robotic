from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from middleware.core.camera_calibration import CameraCalibrations


def info(frame='color_optical', fx=400.0):
    return SimpleNamespace(k=[fx, 0., 320., 0., 401., 240., 0., 0., 1.],
                           d=[0.] * 5, width=640, height=480,
                           distortion_model='plumb_bob',
                           header=SimpleNamespace(frame_id=frame))


def test_rgb_only_then_depth_and_changed_calibration():
    cache = CameraCalibrations()
    topic, value, token = cache.observe('/camera/left/color/camera_info', info(), 10)
    assert topic == '/camera/left/calibration'
    assert value['rgb']['fx'] == 400
    assert value['rgb']['frame_id'] == 'color_optical'
    assert 'depth' not in value
    cache.committed(token)
    assert cache.observe('/camera/left/color/camera_info', info(), 20) is None
    _, value, token = cache.observe('/camera/left/depth/camera_info', info('depth_optical', 390.), 30)
    assert value['rgb']['fx'] == 400 and value['depth']['fx'] == 390
    cache.committed(token)
    assert cache.observe('/camera/left/color/camera_info', info(fx=402.), 40)


def test_invalid_intrinsics_and_queue_retry():
    cache = CameraCalibrations()
    topic = '/head/camera_info'
    assert cache.observe(topic, info(fx=0.), 1) is None
    assert cache.observe(topic, info(fx=float('nan')), 1) is None
    assert cache.observe(topic, info(), 2)
    # An uncommitted candidate must be retried, even if its values are unchanged.
    _, _, token = cache.observe(topic, info(), 3)
    cache.committed(token)
    assert cache.observe(topic, info(), 4) is None
    assert cache.observe(topic, info(fx=0.), 5) is None
    assert cache.observe(topic, info(), 6)


def test_raw_calibration_keeps_rgb_only_head():
    from middleware.core.capture.holobrain_mcap import calibration_document
    camera_info = info()
    value = calibration_document('camera_hand', {'rgb': {
        'k': camera_info.k, 'd': camera_info.d, 'width': 640, 'height': 480,
        'frame_id': 'camera_hand_camera_color_optical_frame'}}, 1)
    assert value['camera'] == 'head'
    assert value['rgb']['frame_id'] == 'camera_hand_camera_color_optical_frame'
    assert 'depth' not in value and set(value['resolution']) == {'color'}
    assert calibration_document('camera_hand', {}, 1) is None
    intrinsics = {'k': camera_info.k, 'd': camera_info.d, 'width': 640, 'height': 480}
    rgbd = calibration_document('camera_left', {'rgb': intrinsics, 'depth': intrinsics}, 2)
    assert rgbd['rgb']['fx'] == rgbd['depth']['fx'] == 400.
    assert 'rgb_depth_extrinsics' not in rgbd


def test_camera_mounts_connected_and_moving_with_correct_arm():
    root = Path(__file__).resolve().parents[1]
    model = ET.parse(root / 'adapters/robots/openarmx/urdf/openarmx_robot.urdf').getroot()
    joints = {j.find('child').get('link'): j for j in model.findall('joint')}
    links = {link.get('name') for link in model.findall('link')}
    assert len(joints) == len(links) - 1
    for camera, arm in [('L_Link7_XJ_Link', 'left'), ('R_LINK7_XJ_Link', 'right'), ('XJ_Link', None)]:
        chain = []
        frame = camera
        while frame != 'world':
            assert frame not in chain
            chain.append(frame)
            frame = joints[frame].find('parent').get('link')
        moving = [joints[f].get('name') for f in chain if joints[f].get('type') != 'fixed']
        assert set(moving) == ({f'openarmx_{arm}_joint{i}' for i in range(1, 8)} if arm else set())


def test_static_tf_qos_retains_prior_broadcasts():
    import pytest
    pytest.importorskip('rclpy')
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from middleware.core.ros_qos import subscription_qos
    qos = subscription_qos({'topic': '/tf_static'})
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert qos.reliability == ReliabilityPolicy.RELIABLE


def test_openarmx_profile_selects_real_topics_and_rgb_only_head():
    import copy
    from middleware.core.profile_cameras import apply_profile_cameras
    root = Path(__file__).resolve().parents[1]
    config = {'camera': {'enabled': True}, 'ros': {'subscriptions': [
        {'topic': '/tf', 'type': 'tf2_msgs/msg/TFMessage'},
        {'topic': '/observation/cameras/wrist_left/camera_info', 'type': 'sensor_msgs/msg/CameraInfo'}]}}
    original = copy.deepcopy(config)
    apply_profile_cameras(config, root / 'adapters/robots/openarmx', simulation=True)
    assert config == original
    apply_profile_cameras(config, root / 'adapters/robots/openarmx')
    topics = {s['topic'] for s in config['ros']['subscriptions']}
    assert '/tf' in topics
    assert '/observation/cameras/wrist_left/camera_info' not in topics
    assert len(topics) == 11
    assert '/camera_hand/camera/color/camera_info' in topics
    assert '/camera_hand/camera/depth/camera_info' not in topics
    assert '/camera_left/camera/depth/camera_info' in topics
    assert '/camera_right/camera/depth/camera_info' in topics
    assert len(config['camera']['streams']) == 3
