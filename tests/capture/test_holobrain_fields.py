"""Independent checks of the agreed public ROS and LeRobot recording fields."""
import io
import json
from pathlib import Path

import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import pytest
from mcap.reader import make_reader
from PIL import Image
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, CompressedImage

from middleware.core.capture.holobrain_fields import description
from middleware.core.capture.holobrain_mcap import calibration_document
from test_openarm_capture import produced  # Shared actual recorder/exporter fixture.

NAMES = [*[f'Joint{i}_R' for i in range(1,8)], 'R_ban',
         *[f'Joint{i}_L' for i in range(1,8)], 'L_ban']


def test_new_mcap_contains_standard_ros_fields_and_exact_depth(produced):
    required = {'io_teleop/joint_cmd', 'io_teleop/joint_states',
                '/io_teleop/joint_cmd_finger_right', '/io_teleop/joint_cmd_finger_left',
                '/io_teleop/gripper_state', '/io_teleop/camera_head/color', '/io_teleop/camera_head/depth'}
    seen = set()
    with (Path(produced['session'])/'raw.mcap').open('rb') as stream:
        for schema, channel, message in make_reader(stream).iter_messages():
            if channel.message_encoding != 'cdr':
                continue
            assert schema.encoding == 'ros2msg'
            seen.add(channel.topic)
            if schema.name.endswith('/JointState'):
                joint = deserialize_message(message.data, JointState)
                assert set(joint.name) <= set(NAMES)
                assert len(joint.name) == len(joint.position)
                assert list(joint.velocity) == list(joint.effort) == []
            elif channel.topic == '/io_teleop/camera_head/depth':
                depth = deserialize_message(message.data, CompressedImage)
                assert depth.format == '16UC1; png'
                array = np.asarray(Image.open(io.BytesIO(depth.data)))
                stamp = depth.header.stamp.sec*10**9+depth.header.stamp.nanosec
                source_frame = round((stamp-produced['input_base_ns'])*30/10**9)
                np.testing.assert_array_equal(array, np.full((480,640),1000+source_frame))
    assert required <= seen


def test_public_dataset_preserves_source_ns_and_single_channel_depth(produced):
    root = Path(produced['path'])
    info = json.loads((root/'meta/info.json').read_text())
    for key in ('action','observation.state'):
        assert info['features'][key] == {'dtype':'float32','shape':[16],'names':NAMES,
                                       'units':['rad']*7+['m']+['rad']*7+['m']}
    assert info['features']['observation.images.head']['shape'] == [480,640,3]
    assert info['features']['observation.images.head_depth']['shape'] == [480,640,1]
    table = pq.read_table(root/'data')
    assert table.schema.field('source.timestamp_ns').type == pa.int64()
    source_frames = [*range(6), *range(15,21)]
    with (Path(produced['session'])/'raw.mcap').open('rb') as stream:
        stamps = []
        for _,channel,message in make_reader(stream).iter_messages(topics=['/io_teleop/camera_head/color']):
            rgb = deserialize_message(message.data,CompressedImage)
            stamps.append(rgb.header.stamp.sec*10**9+rgb.header.stamp.nanosec)
    expected = [sorted(stamps)[frame] for frame in source_frames]
    assert table['source.timestamp_ns'].to_pylist() == expected
    with av.open(str(root/'videos/observation.images.head_depth/chunk-000/file-000.mp4')) as container:
        frames=list(container.decode(video=0))
    assert len(frames)==6 and frames[0].format.name=='gray12le'
    assert not any(key.startswith('observation.holobrain.') for key in info['features'])
    public=json.loads((root/'meta/recording-fields.json').read_text())
    assert [a['unit'] for a in public['axes']] == ['rad']*7+['m']+['rad']*7+['m']


def test_camera_alias_collision_rejected():
    cfg={'capture':None,'sources':{'camera_hand':{'kind':'rgbd'},'camera_front':{'kind':'rgbd'}}}
    with pytest.raises(ValueError,match='collide'):
        description(cfg)


def test_calibration_uses_measured_intrinsics_without_invented_extrinsics():
    info={'k':[400.,0.,319.,0.,401.,239.,0.,0.,1.], 'd':[0.]*5,
          'distortion_model':'plumb_bob','width':640,'height':480}
    value=calibration_document('camera_hand',{'rgb':info,'depth':info},123)
    assert value['schema']=='hc_camera_calibration/v1'
    assert value['camera']=='head'
    assert value['rgb']['fx']==400. and value['rgb']['cy']==239.
    assert 'rgb_depth_extrinsics' not in value
    assert calibration_document('camera_hand',{},123) is None


def test_aligned_upload_bundle_uses_same_public_names_and_depth(produced, tmp_path):
    from middleware.core.capture.native_bundle import prepare
    output=tmp_path/'recording'
    prepare(produced['session'],output)
    config=json.loads((output/'recording-config.json').read_text())
    assert len(config['cameras'])==6
    assert {'observation.images.head','observation.images.head_depth'} <= {c['topic'] for c in config['cameras']}
    with (output/'sensors/robot.mcap').open('rb') as stream:
        for _,channel,message in make_reader(stream).iter_messages(topics=['/action']):
            value=json.loads(message.data)
            assert value['names']==NAMES
            assert value['units']==['rad']*7+['m']+['rad']*7+['m']
