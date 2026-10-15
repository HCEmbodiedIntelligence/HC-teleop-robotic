"""R2 source contracts, original bytes and ROS layouts; no device or node starts."""
import json
import struct
from pathlib import Path

import numpy as np
import pytest
from mcap.reader import make_reader
from sensor_msgs.msg import Image

from humanoid_camera.depth_source import realsense_ros_depth_profile
from middleware.core.capture.clocks import Clocks
from middleware.core.capture.depth_source import inspect_depth, profile_hash, require_depth_source
from middleware.core.capture.models import normalize
from middleware.core.capture.ros_bridge import decode_image
from middleware.core.capture.ros_images import HeaderPairs
from middleware.core.capture.synthetic import generate, header


def declaration(encoding='16UC1', unit='mm', scale=.001):
    return dict(schema_version='hc-depth-source/1', encoding=encoding, unit=unit,
                scale_m_per_unit=scale, invalid_values=[0], invalid_nonfinite=True,
                registered_to_rgb=False, registration_evidence='independent test grids',
                evidence=dict(kind='synthetic_fixture', reference=__name__, version='1'))


def document(depth, profile):
    source = {'kind': 'rgbd', 'depth_source': profile}
    rgb = np.zeros((4, 6, 3), np.uint8)
    pairs = HeaderPairs()
    for name, pixels, encoding in [('rgb', rgb, 'rgb8'), ('depth', depth, '32FC1' if depth.dtype.kind == 'f' else '16UC1')]:
        msg = Image(header=header(10**9), height=pixels.shape[0], width=pixels.shape[1],
                    encoding=encoding, step=pixels.strides[0], data=pixels.tobytes())
        pair, _ = pairs.add(name, msg, 10**9 + 1, 100)
    return pairs.document('camera', source, pair), source, rgb


@pytest.mark.parametrize('dtype,encoding,unit,scale', [
    ('<u2', '16UC1', 'mm', .001), ('>u2', '16UC1', 'source_unit', .00025),
    ('<f4', '32FC1', 'm', 1.), ('>f4', '32FC1', 'source_unit', .5),
])
def test_source_contract_keeps_independent_grid_and_original_codes(dtype, encoding, unit, scale):
    depth = np.array([[0, 1, 4000], [2, 7, 12]], dtype=dtype)
    doc, source, rgb = document(depth, declaration(encoding, unit, scale))
    sample = normalize(doc, source, Clocks(), 's', 101, rgb, depth)
    meta = sample.data()['depth']
    assert sample.depth_values == depth.tobytes()
    assert meta['source_contract']['status'] == 'DECLARED'
    assert meta['shape'] == [2, 3] and meta['depth_scale'] == scale
    assert require_depth_source(meta, depth)['unit'] == unit
    assert meta['source_profile_sha256'] == profile_hash(meta['source_profile'])


@pytest.mark.parametrize('change', [
    {'source_profile': None, 'depth_scale': None}, {'depth_scale': 0},
    {'depth_scale': -1}, {'depth_scale': float('nan')}, {'depth_scale': float('inf')},
    {'depth_scale': True}, {'encoding': '32FC1'}, {'shape': [3, 2]},
    {'source_profile_sha256': '0' * 64}, {'invalid_value': False},
])
def test_raw_keeps_bad_unit_evidence_and_bytes_but_conversion_rejects(change):
    depth = np.array([[0, 1000, 65535], [1, 20, 30]], dtype='<u2')
    doc, source, rgb = document(depth, declaration())
    doc['depth'].update(change)
    sample = normalize(doc, source, Clocks(), 's', 101, rgb, depth, allow_unknown_depth=True)
    meta = sample.data()['depth']
    assert sample.depth_values == depth.tobytes()
    assert meta['source_contract']['status'] in {'UNKNOWN', 'INVALID'}
    with pytest.raises(ValueError, match='DEPTH_SOURCE_CONTRACT_REQUIRED'):
        require_depth_source(meta, depth)
    with pytest.raises(ValueError):
        normalize(doc, source, Clocks(), 's', 101, rgb, depth)
    if isinstance(change.get('depth_scale'), float) and not np.isfinite(change['depth_scale']):
        assert sample.data()['raw_nonfinite_values'][0]['path'] == ['depth', 'depth_scale']


def test_float_nan_inf_and_zero_pixels_preserved_and_registration_not_invented():
    depth = np.array([[np.nan, np.inf, -np.inf], [0, 1.5, 10]], dtype='<f4')
    doc, source, rgb = document(depth, declaration('32FC1', 'm', 1.))
    sample = normalize(doc, source, Clocks(), 's', 101, rgb, depth)
    assert sample.depth_values == depth.tobytes()
    assert inspect_depth(sample.data()['depth'], depth)['status'] == 'DECLARED'
    doc['depth']['source_profile']['registered_to_rgb'] = True
    with pytest.raises(ValueError, match='registration_shape'):
        normalize(doc, source, Clocks(), 's', 101, rgb, depth)


@pytest.mark.parametrize('encoding,dtype', [('16UC1', '>u2'), ('32FC1', '>f4')])
def test_ros_big_endian_and_row_padding(encoding, dtype):
    pixels = np.array([[0, 1, 123], [2, 42, 200]], dtype=dtype)
    payload = b''.join(row.tobytes() + b'PAD!' for row in pixels)
    msg = Image(height=2, width=3, encoding=encoding, is_bigendian=1,
                step=3 * pixels.dtype.itemsize + 4, data=payload)
    decoded = decode_image(msg)
    np.testing.assert_array_equal(decoded, pixels)
    assert decoded.dtype.str == dtype and decoded.tobytes() == pixels.tobytes()


def test_driver_output_contract_is_exact_version_and_format():
    profile = realsense_ros_depth_profile('4.57.7', 'Z16')
    assert profile['unit'] == 'mm' and profile['scale_m_per_unit'] == .001
    assert profile['registered_to_rgb'] is False
    assert '4.57.7' in profile['evidence']['reference']
    for version, fmt in [('4.57.8', 'Z16'), ('4.57.7', 'Y16'), (None, 'Z16')]:
        assert realsense_ros_depth_profile(version, fmt) is None


def test_unknown_depth_survives_real_raw_mcap_and_seal(tmp_path, monkeypatch):
    original = HeaderPairs.document
    def unknown(self, *args):
        doc = original(self, *args)
        doc['depth'].update(source_profile=None, depth_scale=None, invalid_value=None)
        return doc
    monkeypatch.setattr(HeaderPairs, 'document', unknown)
    result = generate(tmp_path / 'unknown', raw_only=True)
    count = 0
    with (Path(result['path']) / 'raw.mcap').open('rb') as stream:
        for _, channel, msg in make_reader(stream, validate_crcs=True).iter_messages():
            if channel.message_encoding != 'humanoid-depth':
                continue
            length = struct.unpack('<I', msg.data[:4])[0]
            metadata = json.loads(msg.data[4:4+length])['depth']
            pixels = np.frombuffer(msg.data[4+length:], dtype=metadata['dtype'])
            assert metadata['source_contract']['status'] == 'UNKNOWN'
            assert metadata['depth_scale'] is None
            assert np.all((pixels >= 1000) & (pixels <= 1024))
            count += 1
    assert count > 0
    assert (Path(result['path']) / 'raw.mcap').read_bytes() == (Path(result['session']) / 'raw.mcap').read_bytes()
