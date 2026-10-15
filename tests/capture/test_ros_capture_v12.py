"""ROS timestamp contract tests; no camera SDK or physical timing certification."""
import importlib.util
import json
from pathlib import Path
import time

import numpy as np
import pytest
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Header

from middleware.core.capture.clocks import Clocks, RosSteadyMapping, RosClockJump
from middleware.core.capture.config import validate
from middleware.core.capture.models import normalize
from middleware.core.capture.ros_bridge import joint_document
from middleware.core.capture.ros_images import HeaderPairs
from middleware.core.capture.alignment import Aligner
from middleware.core.capture.buffers import TimeCache
from middleware.core.capture.session import Session

SCRIPT=Path(__import__('os').environ['HC_CAMERA_SOURCE'])/'scripts/timestamp_adapter.py'
spec=importlib.util.spec_from_file_location('timestamp_adapter',SCRIPT)
adapter=importlib.util.module_from_spec(spec);spec.loader.exec_module(adapter)


def header(stamp):
    h=Header();h.stamp.sec,h.stamp.nanosec=divmod(stamp,10**9);h.frame_id='optical_frame';return h


def test_global_sdk_anchor_converts_sensor_midpoint_without_half_exposure():
    data={'clock_domain':'global_time','frame_timestamp':1700000000104.,'hw_timestamp':104000,
        'sensor_timestamp':102000,'actual_exposure':4000,'gain_level':16}
    value,model=adapter.midpoint(data)
    assert value==1700000000102000000
    # AE metadata may change; the already-midpoint timestamp must not move twice.
    assert adapter.midpoint({**data,'actual_exposure':8000,'gain_level':100})[0]==value
    assert model['uncertainty_ns'] is None
    with pytest.raises(ValueError,match='GLOBAL_TIME'):
        adapter.midpoint({**data,'clock_domain':'system_time'})


def test_raw_uvc_microsecond_wrap_preserves_local_midpoint_delta():
    raw={'clock_domain':'global_time','frame_timestamp':'1700000000000.000001',
        'hw_timestamp':100,'sensor_timestamp':2**32-1900}
    assert adapter.midpoint(raw)[0]==1700000000000000001-2_000_000


def test_joint_ros_header_is_preserved_without_robot_clock_registration():
    cfg=validate({'sources':{'joints':{'kind':'state','transport':'ros_joint_state','topic':'/joint_states','timestamp_semantics':'read'}}})
    source=cfg['sources']['joints']
    capture=1700000000123456789
    msg=JointState(header=header(capture),name=['j1'],position=[.3])
    d=joint_document('joints',source,msg,7,capture+3_000_000,777)
    sample=normalize(d,source,Clocks(),'s',888)
    assert sample.capture_time_ns==capture
    assert sample.data()['source_timestamp_ns']==capture
    assert sample.data()['receive_steady_ns']==888
    assert sample.data()['clock_model_id']=='ros_header_direct'


def test_ros_epoch_is_converted_before_steady_wait_and_jumps_end_epoch():
    m=RosSteadyMapping()
    ros,steady=1700000000000000000,5000000000
    m.observe(ros,steady)
    assert m.to_steady(ros+80_000_000)==steady+80_000_000
    m.observe(ros+10_000_000,steady+10_000_000)
    with pytest.raises(RosClockJump):m.observe(ros-1_000_000_000,steady+20_000_000)
    assert m.epoch==1 and not m.valid
    with pytest.raises(RosClockJump):m.to_steady(ros)


def test_header_pairing_is_bounded_and_never_joins_neighbor_stamp():
    pairs=HeaderPairs(item_limit=2,byte_limit=100)
    def img(t):return Image(header=header(t),height=2,width=2,encoding='rgb8',step=6,data=bytes(12))
    assert pairs.add('rgb',img(10),20,30)[0] is None
    assert pairs.add('depth',img(11),20,30)[0] is None
    result,_=pairs.add('depth',img(10),20,30)
    assert result['key']==10
    assert pairs.add('rgb',img(10),20,30)[0] is None
    for k in range(20,100):pairs.add('rgb',img(k),k,k)
    assert pairs.bytes<=100 and len(pairs.items)<=2
    assert pairs.dropped>0


def test_ros_session_ends_on_clock_jump_without_waiting_epoch_seconds(tmp_path):
    cfg=validate({'alignment':{'strict':False},'sources':{'joints':{'kind':'state','transport':'envelope'}},
        'storage':{'minimum_free_bytes':0}})
    s=Session(tmp_path,cfg,Clocks(),{})
    s.start()
    source=cfg['sources']['joints']
    now=time.time_ns()
    msg=JointState(header=header(now),name=['j'],position=[1.])
    s.submit(joint_document('joints',source,msg,1,now,time.monotonic_ns()))
    until=time.monotonic()+2
    while s.t0 is None and time.monotonic()<until:time.sleep(.001)
    assert s.t0 is not None
    s.observe_ros_clock(time.time_ns()-1_000_000_000,time.monotonic_ns())
    assert s.closed.wait(3)
    assert s.status()['state']=='failed'
    assert s.status()['clock_epoch']==1


def test_actual_ros_messages_pass_through_independent_adapter_recorder_and_export():
    pytest.importorskip("av")
    pytest.importorskip("pyarrow")
    pytest.importorskip("mcap")
    import os
    import subprocess
    import sys
    result=subprocess.run([sys.executable,str(Path(__file__).with_name('ros_timestamp_pipeline_case.py'))],
        env={**os.environ,'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID','232')},text=True,capture_output=True,timeout=30)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert 'native_pointcloud_bytes_preserved' in result.stdout
