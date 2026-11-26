"""The live export selects source frames on their ROS stamps before accessing RGB pixels."""
from io import BytesIO
import json
from pathlib import Path

import pytest

pytest.importorskip('rclpy.serialization')
pytest.importorskip('mcap.writer')
from mcap.writer import Writer
from PIL import Image
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CompressedImage, JointState

from middleware.core.lerobot_export import prepare_mcap
from middleware.core.robot_api import RobotApiProfile


def recording(path, sizes, frame_ids=('rgb_optical',) * 3):
    api = RobotApiProfile(Path(__file__).resolve().parents[1] / 'adapters/robots/openarmx')
    base_ns = 1_700_000_000_000_000_000
    payloads = []
    camera_topic = '/observation/cameras/head/color/compressed'
    config = {'subscriptions': [dict(topic=camera_topic, type='sensor_msgs/msg/CompressedImage',
                                     outputs=['record'])]}
    with path.open('wb') as file:
        writer = Writer(file)
        writer.start(profile='ros2')
        writer.add_metadata('hc_teleop.session', {'profile_id': 'openarmx',
                            'robot_api_info': json.dumps(api.describe_servo_j(config))})
        joints = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        rgb = writer.register_schema('sensor_msgs/msg/CompressedImage', 'ros2msg', b'')
        state = writer.register_channel('/observation/joint_states', 'cdr', joints)
        action = writer.register_channel('/action/joint_states', 'cdr', joints)
        camera = writer.register_channel(camera_topic, 'cdr', rgb)
        names = api.groups['left_arm'] + api.groups['right_arm']
        for offset in (0, 100, 200):
            stamp = base_ns + offset * 1_000_000
            joint = JointState(name=names, position=[.2] * len(names))
            joint.header.stamp.sec, joint.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            for channel in (state, action):
                writer.add_message(channel, stamp, serialize_message(joint), stamp)
        for i, (offset, size) in enumerate(zip((10, 25, 90), sizes)):
            stamp = base_ns + offset * 1_000_000
            # MCAP write time intentionally trails the source header by 40 ms.
            receipt = stamp + 40_000_000
            file = BytesIO()
            Image.new('RGB', size, (60*i, 20, 21)).save(file, format='PNG')
            payloads.append(file.getvalue())
            frame = CompressedImage(format='png', data=payloads[-1])
            frame.header.frame_id = frame_ids[i]
            frame.header.stamp.sec, frame.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            writer.add_message(camera, receipt, serialize_message(frame), receipt)
        writer.finish()
    return payloads


def test_live_selection_reuses_original_compressed_frames_and_header(tmp_path):
    source = tmp_path / 'video.mcap'
    original = recording(source, [(16, 12)] * 3)
    stage = tmp_path / 'stage'
    stage.mkdir()
    manifest = json.loads(prepare_mcap(source, stage, 20, 'task', 'openarmx').read_text())
    key, = manifest['sync_audit']['staged_compressed_frames']
    assert manifest['sync_audit']['staged_compressed_frames'][key] == 2
    assert manifest['source_frame_ids'][manifest['source_topics'][key]] == 'rgb_optical'
    rows, = manifest['episodes']
    values = [Path(row['values'][key]) for row in rows]
    assert len(values) == 5 and values[0].exists()
    assert values[1].read_bytes() == original[1]
    assert values[2] == values[3] == values[4] and values[2].read_bytes() == original[2]
    assert {p for p in (stage / key).glob('*.image')} == {values[1], values[2]}
    assert rows[1]['observation.camera_time_ns'] == [1_700_000_000_025_000_000]


def test_resolution_change_is_detected_on_a_skipped_source_frame(tmp_path):
    source = tmp_path / 'video.mcap'
    recording(source, [(18, 10), (16, 12), (16, 12)])
    with pytest.raises(ValueError, match='Camera resolution changed'):
        prepare_mcap(source, tmp_path, 20, 'task', 'openarmx')


def test_compressed_image_empty_frame_remains_significant(tmp_path):
    source = tmp_path / 'video.mcap'
    recording(source, [(16, 12)] * 3, ('rgb_optical', '', 'rgb_optical'))
    with pytest.raises(ValueError, match='Reference frame changed'):
        prepare_mcap(source, tmp_path, 20, 'task', 'openarmx')
