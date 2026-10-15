"""An actual A capture must work with SQLite and robot-side processing disabled."""
import hashlib
import json
import sqlite3

from mcap.reader import make_reader

from middleware.core.capture import alignment, exporter
from middleware.core.capture.synthetic import generate


def test_raw_depth_upload_preserves_source_bytes_without_depth_encoder(tmp_path, monkeypatch):
    import struct
    from pathlib import Path
    from middleware.core.capture import compact_depth
    from middleware.core.capture.session import Session
    expected = {}
    original = Session._persist_raw
    def persist(self, sample, data, source):
        if getattr(sample, 'depth_values', None):
            expected[(sample.source_id, data['depth']['source_seq'])] = hashlib.sha256(sample.depth_values).hexdigest()
        return original(self, sample, data, source)
    def forbidden(*args, **kwargs):
        raise AssertionError('raw depth must never start a depth video encoder')
    monkeypatch.setattr(compact_depth, 'DepthEncoder', forbidden)
    monkeypatch.setattr(Session, '_persist_raw', persist)
    result = generate(tmp_path/'raw-depth', raw_only=True)
    assert expected
    for root in (Path(result['session']), Path(result['path'])):
        actual = {}
        with (root/'raw.mcap').open('rb') as file:
            for _, channel, message in make_reader(file, validate_crcs=True).iter_messages():
                assert channel.message_encoding != 'humanoid-rgb'
                assert '/depth_video_frame/' not in channel.topic
                if channel.message_encoding == 'humanoid-depth':
                    size = struct.unpack('<I', message.data[:4])[0]
                    metadata = json.loads(message.data[4:4+size])
                    actual[(metadata['source_id'], metadata['depth']['source_seq'])] = hashlib.sha256(message.data[4+size:]).hexdigest()
        assert actual == expected
        assert list((root/'videos').rglob('*.mp4'))
        assert not list((root/'videos').glob('*_depth'))
        assert not (root/'originals.mcap').exists()
        fields = json.loads((root/'recording-fields.json').read_text())
        assert fields['storage_contract']['depth_quantized'] is False
        assert json.loads((root/'session.json').read_text())['raw_storage_format'] == 'rgb-mp4-raw-depth/v1'


def test_disk_spool_retains_backlog_and_detects_corruption(tmp_path):
    import pytest
    from middleware.core.capture.media_spool import MediaSpool
    queue = MediaSpool(tmp_path/'depth.frames', reserve_bytes=0)
    for i in range(100):
        assert queue.offer(({'source_seq': i, 'capture_time_ns': i*33333333}, bytes([i])*2048), 2048)
    assert queue.status()['items'] == 100
    assert queue.status()['rejected'] == 0
    queue.close()
    for i in range(100):
        metadata, raw = queue.get()
        assert metadata['source_seq'] == i
        assert raw == bytes([i])*2048
    queue.release()  # failed/unconfirmed encode must retain the entire source
    assert queue.path.exists()
    queue.release(success=True)
    assert not queue.path.exists()
    broken = MediaSpool(tmp_path/'broken.frames', reserve_bytes=0)
    assert broken.offer(({'source_seq': 1}, b'original'), 8)
    broken.close()
    with broken.path.open('r+b') as file:
        file.seek(-1, 2)
        file.write(b'!')
    with pytest.raises(ValueError, match='checksum'):
        broken.get()
    broken.release()
    assert broken.path.exists()


def test_compact_rgb_encoder_drains_disk_backlog_larger_than_frame_budget(tmp_path, monkeypatch):
    import threading
    from types import SimpleNamespace
    from middleware.core.capture.storage import VideoEncoder
    ready = threading.Event()
    original_run = VideoEncoder.run
    def delayed_run(self):
        assert ready.wait(5)
        original_run(self)
    monkeypatch.setattr(VideoEncoder, 'run', delayed_run)
    records, errors = [], []
    storage = SimpleNamespace(path=tmp_path, compact_media=True, offline=False,
                              required=lambda kind, data: records.append((kind, data)) or True)
    cfg = {'sources': {'camera': {'fps': 30, 'depth_enabled': False}},
           'buffers': {'video_frames_per_camera': 2, 'video_bytes_per_camera': 1024},
           'storage': {'fsync_on_commit': True, 'minimum_free_bytes': 0,
                       'video_segment_seconds': 60, 'encoder_threads': 1}}
    encoder = VideoEncoder('camera', cfg, storage, errors.append)
    for i in range(65):
        data = {'source_seq': i, 'capture_time_ns': i*33333333, 'pair_seq': i,
                'rgb': {'shape': [16, 16, 3], 'source_seq': i, 'exposure_midpoint_ns': i*33333333}}
        sample = SimpleNamespace(data=lambda d=data: d, document=b'', rgb_pixels=bytes([i])*768)
        assert encoder.submit(sample)
    assert encoder.queue.status()['items'] == 65
    ready.set()
    encoder.stop()
    assert not errors and encoder.encoded == 65
    frames = [data for kind, data in records if kind == 'video_frame']
    assert [frame['source_seq'] for frame in frames] == list(range(65))
    assert [frame['source_capture_time_ns'] for frame in frames] == [i*33333333 for i in range(65)]
    encoder.queue.release(success=True)


def test_raw_recording_does_not_retain_duplicate_ros_archive(tmp_path, monkeypatch):
    from pathlib import Path
    from middleware.core.capture import synthetic
    configure = synthetic.synthetic_config
    def config(profile=None):
        cfg = configure(profile)
        cfg['sources']['camera_left'].update({
            'transport': 'ros_rgbd', 'topic': '/test/rgbd',
            'metadata_topic': '/test/metadata',
            'original_topics': {'rgb_frame': '/test/color', 'depth_frame': '/test/depth'}})
        return cfg
    monkeypatch.setattr(synthetic, 'synthetic_config', config)
    result = synthetic.generate(tmp_path/'capture', raw_only=True)
    for root in (Path(result['session']), Path(result['path'])):
        assert not (root/'originals.mcap').exists()
        assert (root/'raw.mcap').is_file()
    sealed = Path(result['path'])
    marker = json.loads((sealed/'recording-complete.json').read_text())
    assert all(a['path'] != 'originals.mcap' for a in marker['recording_upload']['assets'])
    assert list((sealed/'videos').rglob('*.mp4'))


def test_raw_seal_rejects_over_limit_before_copy_and_retains_source(tmp_path):
    import pytest
    from middleware.core.capture import raw_bundle
    source=tmp_path/'recording'
    source.mkdir()
    (source/'raw.mcap').write_bytes(b'preserved-original')
    (source/'session.json').write_text(json.dumps({
        'recording_mode':'raw','session_id':'capacity','config_hash':'frozen',
        'metadata':{'capture_context':{'robot_id':'robot','collection_task_id':'task',
                                       'local_robot_id':'local'}}}))
    (source/'manifest.json').write_text(json.dumps({
        'state':'completed','t0_ns':1,'end_ns':2,
        'committed':[{'path':'raw.mcap'}]+[{'path':f'video/{i:04d}.mp4'} for i in range(4093)]}))
    before={p.name:p.read_bytes() for p in source.iterdir()}
    with pytest.raises(ValueError,match='4096'):
        raw_bundle.prepare(source,tmp_path/'sealed')
    assert not (tmp_path/'sealed').exists() and not list(tmp_path.glob('.export-*'))
    assert before=={p.name:p.read_bytes() for p in source.iterdir()}


def test_raw_capture_retries_transient_writer_contention(tmp_path,monkeypatch):
    from middleware.core.capture.storage import Storage
    submit=Storage.submit
    rejected=[]
    def temporarily_busy(self,kind,data,raw=b''):
        if kind=='raw' and not rejected:
            rejected.append(data['source_id'])
            return False
        return submit(self,kind,data,raw)
    monkeypatch.setattr(Storage,'submit',temporarily_busy)
    result=generate(tmp_path/'contention',raw_only=True)
    from pathlib import Path
    manifest=json.loads((Path(result['session'])/'manifest.json').read_text())
    assert rejected and manifest['state']=='completed' and not manifest['errors']
    assert (Path(result['path'])/'recording-complete.json').is_file()


def test_raw_capture_encodes_during_recording_and_seals_only_compact_media(tmp_path,monkeypatch):
    from middleware.core.capture import synthetic
    original_config=synthetic.synthetic_config
    def config(profile=None):
        cfg=original_config(profile)
        cfg['storage']['depth_video_encoding']=True
        return cfg
    monkeypatch.setattr(synthetic,'synthetic_config',config)
    from pathlib import Path
    import av
    from middleware.core.capture.session import Session
    observed=[]
    persist=Session._persist_raw
    def persist_with_evidence(self,*args,**kwargs):
        result=persist(self,*args,**kwargs)
        observed.append({k:e.encoded for k,e in self.encoders.items()})
        return result
    monkeypatch.setattr(Session,'_persist_raw',persist_with_evidence)
    def forbidden(*args,**kwargs):
        raise AssertionError('robot raw path invoked platform alignment')
    monkeypatch.setattr(sqlite3,'connect',forbidden)
    monkeypatch.setattr(alignment.Aligner,'__init__',forbidden)
    result=generate(tmp_path/'capture',raw_only=True)
    root,source=Path(result['path']),Path(result['session'])
    assert any(values and all(n>0 for n in values.values()) for values in observed)
    assert not list((tmp_path/'capture').rglob('*.sqlite3'))
    assert not list((tmp_path/'capture').rglob('quality.json'))
    for directory in (source,root):
        assert not (directory/'originals.mcap').exists()
        with (directory/'raw.mcap').open('rb') as stream:
            messages=list(make_reader(stream,validate_crcs=True).iter_messages())
        assert all(c.message_encoding in {'json','humanoid-pointcloud'} for _,c,_ in messages)
        assert any('/depth_video_frame/' in c.topic for _,c,_ in messages)
        assert not any('/humanoid/session/rgb/' in c.topic or '/humanoid/session/depth/' in c.topic for _,c,_ in messages)
        for file in (directory/'videos').rglob('*.mp4'):
            with av.open(str(file)) as video:
                if '_depth' in str(file):
                    assert video.streams.video[0].codec_context.name=='hevc'
                    assert video.streams.video[0].codec_context.format.name=='gray12le'
                assert sum(1 for _ in video.decode(video=0))>0
    marker=json.loads((root/'recording-complete.json').read_text())
    for asset in marker['recording_upload']['assets']:
        assert hashlib.sha256((root/asset['path']).read_bytes()).hexdigest()==asset['sha256']
    assert (source/'raw.mcap').read_bytes()==(root/'raw.mcap').read_bytes()


def test_raw_seal_failure_never_leaves_upload_marker(tmp_path,monkeypatch):
    from pathlib import Path
    import pytest
    from middleware.core.capture import raw_bundle
    result=generate(tmp_path/'capture',raw_only=True)
    source=Path(result['session'])
    output=tmp_path/'failed-seal'
    sync=raw_bundle.sync_directory
    def fail_parent(path):
        if Path(path)==tmp_path:
            raise OSError('injected directory durability failure')
        sync(path)
    monkeypatch.setattr(raw_bundle,'sync_directory',fail_parent)
    with pytest.raises(OSError):raw_bundle.prepare(source,output)
    assert not (output/'recording-complete.json').exists()
    assert (source/'raw.mcap').exists()
    # A completed immutable package cannot be reused under another task binding.
    (source/'capture-binding.json').write_text(json.dumps({'collection_task_id':'changed-task'}))
    with pytest.raises(ValueError,match='身份冲突'):
        raw_bundle.prepare(source,Path(result['path']))
