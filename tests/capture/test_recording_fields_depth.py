import io

import numpy as np
import pytest
from PIL import Image
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

from middleware.core.capture.holobrain_fields import description
from middleware.core.capture.holobrain_mcap import PublicWriter


@pytest.mark.parametrize('source,expected', [({}, None), ({'depth_scale': .00025}, .00025),
    ({'depth_scale': True}, None), ({'depth_scale': float('nan')}, None),
    ({'depth_source': {}, 'depth_scale': .001}, None)])
def test_public_depth_scale_never_defaults_to_mm(source, expected):
    result = description({'sources': {'head': {'kind': 'rgbd', **source}}})
    assert result['cameras'][0]['depth_scale_m_per_unit'] == expected


def test_public_depth_scale_comes_from_versioned_source_profile():
    profile = dict(schema_version='hc-depth-source/1', encoding='16UC1', unit='source_unit',
        scale_m_per_unit=.00025, invalid_values=[0], invalid_nonfinite=True,
        registered_to_rgb=None, registration_evidence='',
        evidence=dict(kind='synthetic_fixture', reference=__name__, version='1'))
    result = description({'sources': {'head': {'kind': 'rgbd', 'depth_source': profile}}})
    assert result['cameras'][0]['depth_scale_m_per_unit'] == .00025


def test_live_depth_png_preserves_full_uint16_range_and_source_timestamp():
    class Sink:
        def register_schema(self, *args): return 1
        def register_channel(self, *args): return 1
        def add_message(self, channel, **kwargs): self.message = kwargs

    sink = Sink()
    writer = PublicWriter(sink, {'sources': {'camera_hand': {'kind': 'rgbd'}}})
    pixels = np.arange(65536, dtype=np.uint16).reshape(256, 256)
    stamp = 1791594940529731085
    writer.append('depth', {'source_id': 'camera_hand', 'source_seq': 17,
        'source_timestamp_ns': stamp, 'receive_time_ns': stamp + 100000,
        'depth': {'dtype': '<u2', 'shape': [256, 256], 'header_timestamp_ns': stamp,
                  'source_seq': 17, 'coordinate_frame': 'depth_optical_frame'}},
        pixels.tobytes(), stamp + 200000)
    message = deserialize_message(sink.message['data'], CompressedImage)
    assert message.format == '16UC1; png'
    assert message.header.stamp.sec * 10**9 + message.header.stamp.nanosec == stamp
    assert message.header.frame_id == 'depth_optical_frame'
    np.testing.assert_array_equal(np.asarray(Image.open(io.BytesIO(message.data))), pixels)
