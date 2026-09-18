import pytest
from middleware.core.config import ConfigError, validate_config


def stream(topic='/new/color', **kw):
    return {'id': 'head', 'topic': topic, **kw}


def test_projection_creates_subscription_without_recording_or_json():
    cfg = validate_config({'camera': {'enabled': True, 'streams': [stream()]}})
    assert cfg['ros']['subscriptions'] == [{'topic': '/new/color', 'type': 'sensor_msgs/msg/Image',
        'enabled': True, 'outputs': [], 'max_hz': 0., 'event_max_hz': 0.}]
    assert validate_config(cfg) == cfg


def test_projection_preserves_disabled_recording_configuration():
    cfg = validate_config({'camera': {'enabled': True, 'streams': [stream('/new/compressed')]},
        'ros': {'subscriptions': [{'topic': '/new/compressed', 'type': 'sensor_msgs/msg/CompressedImage',
            'outputs': ['record'], 'enabled': False, 'max_hz': 12}]}})
    item = cfg['ros']['subscriptions'][0]
    assert not item['enabled'] and item['outputs'] == ['record'] and item['max_hz'] == 12


def test_disabled_projection_does_not_add_subscription():
    cfg = validate_config({'camera': {'enabled': True, 'streams': [stream(enabled=False)]}})
    assert cfg['ros']['subscriptions'] == []


def test_four_streams_keep_types_and_output_settings():
    streams=[{'id': name, 'topic': '/'+name, 'type': 'sensor_msgs/msg/CompressedImage',
              'width': 800, 'height': 600, 'fps': i+10} for i,name in enumerate(['head','overhead','d405_left','d405_right'])]
    cfg=validate_config({'camera': {'enabled': True, 'streams': streams}})
    assert len(cfg['ros']['subscriptions']) == 4
    assert [s['fps'] for s in cfg['camera']['streams']] == [10,11,12,13]
    assert all(s['type']=='sensor_msgs/msg/CompressedImage' for s in cfg['ros']['subscriptions'])


def test_non_image_subscription_cannot_be_used_as_camera():
    with pytest.raises(ConfigError):
        validate_config({'camera': {'enabled': True, 'streams': [stream()]},
            'ros': {'subscriptions': [{'topic': '/new/color', 'type': 'std_msgs/msg/String'}]}})
