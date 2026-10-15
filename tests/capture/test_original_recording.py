"""Unpaired/invalid inputs remain original evidence, independent of derived media."""
from array import array
import json
from pathlib import Path
import time

from mcap.reader import make_reader
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, CameraInfo
from realsense2_camera_msgs.msg import Metadata

from middleware.core.capture import synthetic
from middleware.core.capture.config import validate
from middleware.core.capture.originals import OriginalArchive, camera_topics, compress_archive
from middleware.core.capture.session import Session


def camera(depth=True):
    return {'namespace':'unpaired','camera_name':'camera','rgb_topic':'/unpaired/rgb',
            'depth_topic':'/unpaired/depth','depth_enabled':depth}


def source(depth=True):
    return {'kind':'rgbd','transport':'ros_rgbd','topic':'/unpaired/normalized/rgbd',
            'metadata_topic':'/unpaired/normalized/metadata','depth_enabled':depth,
            'original_topics':camera_topics(camera(depth))}


def image(stamp,depth=False):
    msg=Image(height=2,width=2,encoding='16UC1' if depth else 'rgb8',step=6 if depth else 8)
    msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,10**9)
    msg.header.frame_id='original_optical_frame'
    # Include row padding: original ROS bytes must survive, not only decoded pixels.
    msg.data=array('B',range(msg.height*msg.step))
    return msg


def messages(path):
    with path.open('rb') as stream:
        return [(s.name,c.topic,c.metadata,m.log_time,m.publish_time,m.sequence,m.data)
                for s,c,m in make_reader(stream,validate_crcs=True).iter_messages(log_time_order=False)]


def test_independent_originals_keep_unmatched_images_and_invalid_metadata(tmp_path):
    cfg=validate({'sources':{'left':source()},'storage':{'minimum_free_bytes':0}})
    errors=[]
    archive=OriginalArchive(tmp_path,cfg,errors.append)
    archive.start()
    expected=[]
    try:
        for number in range(80): # Exceeds both the old 32-group cache and frame window.
            msg=image(10**9+number*33333333)
            data=serialize_message(msg)
            assert archive.submit('/unpaired/rgb',data,2*10**9+number)
            expected.append(data)
        orphan=image(999999999,True)
        assert archive.submit('/unpaired/depth',serialize_message(orphan),3*10**9)
        invalid=Metadata(json_data='not JSON; original diagnostic evidence')
        assert archive.submit('/unpaired/camera/color/metadata',serialize_message(invalid),4*10**9)
        info=CameraInfo(width=2,height=2)
        assert archive.submit('/unpaired/camera/color/camera_info',serialize_message(info),5*10**9)
    finally:
        archive.stop()
    assert archive.closed and not errors
    result=messages(tmp_path/'originals.mcap')
    assert [row[-1] for row in result if row[1]=='/unpaired/rgb']==expected
    assert len(result)==83
    assert deserialize_message(result[0][-1],Image)==image(10**9)
    compress_archive(tmp_path/'originals.mcap',tmp_path/'compressed.mcap')
    assert messages(tmp_path/'compressed.mcap')==result


def test_originals_sealed_and_hashed_even_when_absent_from_paired_video(tmp_path,monkeypatch):
    configure=synthetic.synthetic_config
    start=Session.start
    expected=[]
    def config(profile=None):
        cfg=configure(profile)
        cfg['storage']['retain_original_ros']=True
        cfg['sources']['camera_left'].update(source())
        return cfg
    def start_with_unmatched(self):
        result=start(self)
        for offset in range(4):
            data=serialize_message(image(time.time_ns()+offset))
            assert self.record_original('/unpaired/rgb',data,time.time_ns())
            expected.append(data)
        assert self.record_original('/unpaired/depth',serialize_message(image(time.time_ns(),True)),time.time_ns())
        return result
    monkeypatch.setattr(synthetic,'synthetic_config',config)
    monkeypatch.setattr(Session,'start',start_with_unmatched)
    result=synthetic.generate(tmp_path/'capture',raw_only=True)
    original=Path(result['session'])/'originals.mcap'
    sealed=Path(result['path'])/'originals.mcap'
    assert [row[-1] for row in messages(original) if row[1]=='/unpaired/rgb']==expected
    assert messages(original)==messages(sealed)
    manifest=json.loads((sealed.parent/'manifest.json').read_text())
    assert any(c['path']=='originals.mcap' for c in manifest['committed'])
    marker=json.loads((sealed.parent/'recording-complete.json').read_text())
    import hashlib
    asset=next(a for a in marker['recording_upload']['assets'] if a['path']=='originals.mcap')
    assert asset['sha256']==hashlib.sha256(sealed.read_bytes()).hexdigest()
    assert asset['media_type']=='application/mcap'


def test_archive_capacity_failure_is_recording_failure_not_silent_image_drop(tmp_path,monkeypatch):
    from middleware.core.capture.clocks import Clocks
    cfg=validate({'alignment':{'strict':False},'sources':{'left':source()},
                  'storage':{'minimum_free_bytes':0,'retain_original_ros':True}})
    session=Session(tmp_path,cfg,Clocks(),{'left':{'registered_monotonic_ns':time.monotonic_ns()}},raw_only=True)
    session.start()
    monkeypatch.setattr(session.originals.queue,'offer',lambda *args:False)
    assert not session.record_original('/unpaired/rgb',serialize_message(image(time.time_ns())),time.time_ns())
    assert session.closed.wait(3)
    assert session.state=='failed'
    assert any(e['reason'].startswith('original_writer_queue_overflow:') for e in session.errors)
    assert session.originals.closed


def test_rgb_only_original_topics_never_request_depth():
    cfg=validate({'sources':{'head':source(False)}})
    assert set(cfg['sources']['head']['original_topics'])=={'rgb','rgb_meta','rgb_info'}
    broken=source(False)
    broken['original_topics']['depth']='/unpaired/depth'
    with pytest.raises(ValueError,match='原始相机话题'):
        validate({'sources':{'head':broken}})


def test_actual_ros_input_survives_adapter_eviction():
    import subprocess
    import sys
    result=subprocess.run([sys.executable,str(Path(__file__).with_name('ros_original_recording_case.py'))],
                          text=True,capture_output=True,timeout=30)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert '"unpaired_images_preserved": true' in result.stdout
