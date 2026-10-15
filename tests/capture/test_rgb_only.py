"""Mixed RGB-D/RGB recordings retain real pixels and omit disabled depth artifacts."""
import json
from pathlib import Path

import pytest
from mcap.reader import make_reader

from middleware.core.capture import synthetic
from middleware.core.capture.session import Session


@pytest.mark.parametrize('raw_only', [True, False])
def test_mixed_streams_survive_recording_encoding_and_export(tmp_path, monkeypatch, raw_only):
    original_config, submit = synthetic.synthetic_config, Session.submit

    def config(profile=None):
        cfg = original_config(profile)
        cfg['capture']['profile']['profile_version'] = 'robot-config-v1'
        cfg['sources']['camera_hand'].update(depth_enabled=False, transport='envelope')
        return cfg

    def mixed_submit(self, metadata, rgb=None, depth=None, **kwargs):
        if metadata['source_id'] == 'camera_hand':
            metadata = {k:v for k,v in metadata.items() if k!='depth'}
            depth = None
        return submit(self, metadata, rgb, depth, **kwargs)

    monkeypatch.setattr(synthetic, 'synthetic_config', config)
    monkeypatch.setattr(Session, 'submit', mixed_submit)
    result = synthetic.generate(tmp_path/'synthetic', raw_only=raw_only)
    root = Path(result['path'])
    if raw_only:
        with (root/'raw.mcap').open('rb') as stream:
            messages = list(make_reader(stream, validate_crcs=True).iter_messages())
        depths = {c.topic.rsplit('/',1)[-1] for _,c,_ in messages if c.message_encoding=='humanoid-depth'}
        assert depths == {'camera_left','camera_right'}
        rgb = [json.loads(m.data) for _,c,m in messages if c.topic=='/humanoid/session/raw/camera_hand']
        assert rgb and all('rgb' in d and 'depth' not in d for d in rgb)
        # Current raw policy stores RGB in finalized segmented MP4, with frame references.
        import av
        videos = list((root/'videos'/'camera_hand').glob('*.mp4'))
        assert videos
        frames = []
        for video in videos:
            with av.open(str(video)) as container:
                frames.extend(frame.to_ndarray(format='rgb24') for frame in container.decode(video=0))
        assert frames and all(frame.shape[2] == 3 and frame.max() > frame.min() for frame in frames)
        assert not list((root/'videos').glob('camera_hand_depth/*'))
    else:
        features = json.loads((root/'meta/info.json').read_text())['features']
        assert 'observation.images.head' in features
        assert 'observation.images.head_depth' not in features
        assert 'observation.images.camera_left_depth' in features
        assert 'observation.images.camera_right_depth' in features


def test_disabled_depth_is_not_silently_accepted_as_rgb(tmp_path, monkeypatch):
    original = synthetic.synthetic_config
    def config(profile=None):
        cfg = original(profile)
        cfg['sources']['camera_hand'].update(depth_enabled=False, transport='envelope')
        return cfg
    monkeypatch.setattr(synthetic, 'synthetic_config', config)
    with pytest.raises((AssertionError, ValueError), match='unexpectedly supplied depth'):
        synthetic.generate(tmp_path/'invalid', raw_only=True)
