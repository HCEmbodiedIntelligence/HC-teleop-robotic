"""Synthetic evidence only: these tests do not assert real camera timing performance."""
import copy
from dataclasses import replace
import json
import math
import time

import numpy as np
import pytest
pytest.importorskip("av")
pytest.importorskip("pyarrow")
from mcap.reader import make_reader

from middleware.core.capture.adapters import preflight
from middleware.core.capture.alignment import Aligner, image_quality, target_time, target_count
from middleware.core.capture.buffers import ByteQueue, TimeCache
from middleware.core.capture.clocks import Clocks, exposure_midpoint
from middleware.core.capture.config import validate
from middleware.core.capture.models import normalize
from middleware.core.capture.session import Session
from middleware.core.capture.catalog import connection, row_at, edits, save_edits
from middleware.core.capture.exporter import export, DepthReader
from middleware.core.capture.storage import decode_depth
from middleware.core.capture.offline import realign


def config(camera=True):
    sources={'joints':{'kind':'state','transport':'envelope','fields':{'position':{'semantics':'continuous','units':'rad','coordinate_frame':'joint'}}},
        'commands':{'kind':'action','transport':'envelope','fields':{'position':{'semantics':'continuous','units':'rad','coordinate_frame':'joint'}}}}
    if camera:
        sources['front']={'kind':'rgbd','transport':'envelope','fps':30}
    return validate({'clock':{'domain':'host_monotonic'},'alignment':{'camera_validation':'strict'},'sources':sources,'storage':{'minimum_free_bytes':0,'numeric_flush_seconds':.02,'video_segment_seconds':.07}})


def clock():
    clocks=Clocks()
    clocks.add({'id':'synthetic','clock_id':'test','epoch':0,'device_origin_ns':0,'common_origin_ns':0,
        'uncertainty_ns':10000,'evidence':'synthetic test fixture, not a device measurement'})
    return clocks


def metadata(ident,seq,stamp,arrival=None):
    return {'source_id':ident,'source_seq':seq,'source_timestamp_ns':stamp,'receive_time_ns':arrival if arrival is not None else stamp,
        'clock_id':'test','clock_epoch':0,'clock_model_id':'synthetic','timestamp_quality':'mapped'}


def numeric(cfg,clocks,ident,seq,stamp,value=None,arrival=None,**extra):
    d=metadata(ident,seq,stamp,arrival)
    d.update(payload={'position':[stamp/1e9 if value is None else value]},**extra)
    if ident=='commands':
        d.update(action_stage='sent',send_time_ns=stamp)
    return normalize(d,cfg['sources'][ident],clocks,'test',arrival if arrival is not None else stamp)


def rgbd_data(stamp,seq=1,arrival=None):
    d=metadata('front',seq,stamp,arrival)
    d.update(pair_seq=seq,trigger_cycle_id=seq,pairing_method='verified_trigger',sync_evidence_version='synthetic-sync',
        calibration_id='synthetic-calibration',camera_config_id='synthetic-config',rgbd_relative_uncertainty_ns=10000,
        relative_uncertainty_evidence='synthetic relative bound')
    for name in ('rgb','depth'):
        d[name]={**metadata('front_'+name,seq,stamp),
            'trigger_cycle_id':seq,'timestamp_semantics':'exposure_midpoint','actual_exposure_us':4000,
            'exposure_uncertainty_us':10,'grid_uncertainty_ns':10000,'uncertainty_evidence':'synthetic conservative bound',
            'temporal_support':{'model':'global_single_exposure','cross_frame_fusion':False}}
    d['depth'].update(depth_scale=.001,invalid_value=0)
    return d


def camera_sample(cfg,clocks,stamp,seq=1,arrival=None,edit=None):
    d=rgbd_data(stamp,seq,arrival)
    if edit:
        edit(d)
    rgb=np.full((16,20,3),seq*20%255,np.uint8)
    depth=np.arange(320,dtype=np.uint16).reshape(16,20)+seq
    return normalize(d,cfg['sources']['front'],clocks,'test',arrival or stamp,rgb,depth)


def reports(cfg):
    result={ident:{'timestamp_quality':'mapped','clock_model_ids':['synthetic']} for ident in cfg['sources']}
    if 'front' in result:
        result['front'].update(exposure_readback={name:{'auto':False,'requested_us':4000,'actual_us':4000,'uncertainty_us':10} for name in ('rgb','depth')},
            sync_evidence_version='synthetic-sync',camera_config_id='synthetic-config',calibration_id='synthetic-calibration',
            uncertainty_evidence='synthetic fixture',temporal_support='global_single_exposure',sync_verified=True,
            sync_mode='hardware_trigger',measured_new_frame_fps=30)
    return result


def test_integer_grid_has_no_hour_drift_and_half_open_count():
    assert target_count(123,123+3600*10**9,30)==108000
    assert target_time(123,108000,30)==123+3600*10**9
    assert target_time(0,1,30)==33333333
    assert target_time(0,2,30)==66666667
    for fps in (1,30,60,144,240):
        assert target_count(0,target_time(0,1000,fps),fps)==1000


def test_deadline_interpolation_and_hold_action_never_select_future():
    cfg,clocks=config(False),clock()
    cache=TimeCache(cfg['buffers'])
    for s in [numeric(cfg,clocks,'joints',3,30_000_000,3),numeric(cfg,clocks,'joints',4,40_000_000,4),
              numeric(cfg,clocks,'commands',1,20_000_000,10),numeric(cfg,clocks,'commands',2,35_000_000,20)]:
        cache.add(s,40_000_000)
    row=Aligner(cfg,cache).row(1,0,120_000_000)
    assert row['valid']
    field=row['state']['joints']['position']
    assert field['value'][0]==pytest.approx(10/3,abs=1e-7)
    assert field['left_seq']==3 and field['right_seq']==4
    assert row['action']['commands']['source_seq']==1
    assert row['action']['commands']['time_field']=='send_time_ns'
    late=replace(numeric(cfg,clocks,'joints',4,40_000_000,4),arrival_time_ns=120_000_000)
    cache=TimeCache(cfg['buffers'])
    cache.add(numeric(cfg,clocks,'joints',3,30_000_000,3),40_000_000)
    cache.add(late,40_000_000)
    row=Aligner(cfg,cache).row(1,0,130_000_000)
    assert 'joints:position:state_missing_bracket' in row['invalid_reasons']


def test_common_exposure_end_has_different_midpoints_and_uncertainty_is_added():
    cfg,clocks=config(),clock()
    model=clocks.models['synthetic']
    assert exposure_midpoint(104_000_000,'exposure_end',4000,model)==102_000_000
    assert exposure_midpoint(104_000_000,'exposure_end',2000,model)==103_000_000
    sample=camera_sample(cfg,clocks,104_000_000,edit=lambda d:(d['rgb'].update(timestamp_semantics='exposure_end'),d['depth'].update(timestamp_semantics='exposure_end',actual_exposure_us=2000)))
    image,reasons=image_quality(sample.data(),102_000_000,cfg['alignment'],cfg['sources']['front'])
    assert image['rgbd_midpoint_skew_ns']==1_000_000
    assert 'rgbd_midpoint_skew_exceeded' in reasons  # 1ms + bound, not 1ms alone.


@pytest.mark.parametrize('edit,reason',[
    (lambda d:d['depth'].update(trigger_cycle_id=99),'rgbd_cycle_mismatch'),
    (lambda d:d.update(pairing_method='sdk_frameset'),'synchronization_evidence_missing'),
    (lambda d:d['rgb'].update(exposure_uncertainty_us=None),'rgb:exposure_evidence_missing'),
    (lambda d:d['depth'].update(actual_exposure_us=5000),'depth:exposure_limit_exceeded'),
    (lambda d:d['rgb'].update(timestamp_quality='host_receive'),'rgb:timestamp_quality_insufficient'),
    (lambda d:d['rgb'].update(grid_uncertainty_ns=None),'rgb:grid_timing_evidence_missing'),
    (lambda d:d['rgb'].update(temporal_support={'model':'rolling_shutter'}),'rgb:temporal_support_unverified'),
])
def test_strict_camera_failures_are_not_silently_accepted(edit,reason):
    cfg,clocks=config(),clock()
    sample=camera_sample(cfg,clocks,100_000_000,edit=edit)
    _,reasons=image_quality(sample.data(),100_000_000,cfg['alignment'],cfg['sources']['front'])
    assert reason in reasons


def test_trigger_mapping_never_substitutes_neighbor_and_camera_reuse_is_explicit():
    cfg,clocks=config(),clock()
    cfg['sources']={k:v for k,v in cfg['sources'].items() if k=='front'}
    cfg['sources']['front']['trigger_origin']=2
    cache=TimeCache(cfg['buffers'])
    cache.add(camera_sample(cfg,clocks,100_000_000,1),100_000_000)
    row=Aligner(cfg,cache).row(0,100_000_000,200_000_000)
    assert row['invalid_reasons']==['front:camera_missing']
    cfg['sources']['front']['trigger_origin']=None
    aligner=Aligner(cfg,cache)
    assert aligner.row(0,100_000_000,200_000_000)['valid']
    row=aligner.row(0,100_000_000,200_000_000)
    assert row['images']['front']['rgb']['repeated']
    assert 'front:rgb:repeated_source_frame' in row['invalid_reasons']


def test_gap_epoch_quaternion_and_angle_semantics():
    from middleware.core.capture.alignment import interpolate
    cfg,clocks=config(False),clock()
    cache=TimeCache(cfg['buffers'])
    cache.add(numeric(cfg,clocks,'joints',0,1_000_000_000,0),1_000_000_000)
    cache.add(numeric(cfg,clocks,'joints',1,1_100_000_000,1),1_100_000_000)
    row=Aligner(cfg,cache).row(0,1_050_000_000,1_200_000_000)
    assert 'joints:position:bracket_gap_exceeded' in row['invalid_reasons']
    assert interpolate([0,0,0,1],[0,0,0,-1],.5,{'semantics':'quaternion'},10_000_000)==[0,0,0,1]
    angle=interpolate([math.pi-.01],[-math.pi+.01],.5,{'semantics':'angle','max_speed':3},10_000_000)
    assert abs(abs(angle[0])-math.pi)<1e-8
    clocks.add({'id':'reset','clock_id':'test','epoch':1,'device_origin_ns':0,'common_origin_ns':0,'uncertainty_ns':1000,'evidence':'synthetic'})
    changed=metadata('joints',5,1_020_000_000)
    changed.update(clock_epoch=1,clock_model_id='reset',payload={'position':[2]})
    cache=TimeCache(cfg['buffers'])
    cache.add(numeric(cfg,clocks,'joints',4,1_000_000_000,0),1_020_000_000)
    cache.add(normalize(changed,cfg['sources']['joints'],clocks,'test',1_020_000_000),1_020_000_000)
    row=Aligner(cfg,cache).row(0,1_010_000_000,1_100_000_000)
    assert 'joints:position:clock_or_trajectory_discontinuity' in row['invalid_reasons']


def test_startup_config_and_evidence_rejections():
    with pytest.raises(ValueError,match='输出 FPS'):
        validate({'dataset':{'fps':60},'sources':{'front':{'kind':'rgbd','fps':30}}})
    with pytest.raises(ValueError,match='增量'):
        validate({'sources':{'commands':{'kind':'action','semantics':'incremental'}}})
    cfg=config()
    assert not preflight(cfg,{},clock())['ready']
    assert preflight(cfg,reports(cfg),clock())['ready']
    invalid=reports(cfg)
    invalid['front']['exposure_readback']['depth']['uncertainty_us']=None
    assert not preflight(cfg,invalid,clock())['ready']


def test_byte_queue_and_image_buffer_ownership():
    q=ByteQueue(4,10)
    assert q.offer('a',8)
    assert not q.offer('b',3)
    assert q.get()=='a'
    assert q.status()['peak_bytes']==8
    cfg,clocks=config(),clock()
    rgb=np.zeros((2,2,3),np.uint8)
    depth=np.ones((2,2),np.uint16)
    sample=normalize(rgbd_data(100_000_000),cfg['sources']['front'],clocks,'test',100_000_000,rgb,depth)
    rgb[:]=255
    depth[:]=99
    assert set(sample.rgb_pixels)=={0}
    assert np.frombuffer(sample.depth_values,np.uint16).tolist()==[1]*4


@pytest.fixture
def captured(tmp_path):
    cfg,clocks=config(),clock()
    session=Session(tmp_path,cfg,clocks,reports(cfg))
    session.start()
    base=time.monotonic_ns()+20_000_000
    try:
        schedule=[]
        schedule.append((base-10_000_000,'commands',1))
        for seq in range(23):
            schedule.append((base-10_000_000+seq*10_000_000,'joints',seq))
        for seq in range(6):
            schedule.append((target_time(base,seq,30),'front',seq+1))
        for stamp,ident,seq in sorted(schedule):
            time.sleep(max(0,(stamp-time.monotonic_ns())/1e9))
            arrival=time.monotonic_ns()
            if ident=='front':
                d=rgbd_data(stamp,seq,arrival)
                session.submit(d,np.full((16,20,3),seq*20,np.uint8),np.arange(320,dtype=np.uint16).reshape(16,20)+seq)
            else:
                sample=numeric(cfg,clocks,ident,seq,stamp,seq,arrival)
                session.submit(sample.data())
        session.end=base+200_000_000
        result=session.stop()
        assert result['state']=='completed',result['errors']
        assert result['counters']['rows']==6
        assert result['counters']['valid_rows']==6, list(session.latest_rows)
        yield session
    finally:
        if not session.closed.is_set():
            session.stop()


def test_full_rate_raw_depth_video_commits_and_streaming_export(captured,tmp_path):
    session=captured
    with connection(session.path) as db:
        assert db.execute("SELECT count(*) FROM raw_samples WHERE source='joints'").fetchone()[0]==23
        assert db.execute('SELECT count(*) FROM rows').fetchone()[0]==6
        assert db.execute('SELECT count(*) FROM frames').fetchone()[0]==6
    with (session.path/'raw.mcap').open('rb') as f:
        depths=[]
        for _,channel,message in make_reader(f).iter_messages():
            if channel.message_encoding=='humanoid-depth':
                d,values=decode_depth(message.data)
                depths.append(d['source_seq'])
                assert values.dtype==np.uint16
                np.testing.assert_array_equal(values,np.arange(320,dtype=np.uint16).reshape(16,20)+d['source_seq'])
        assert len(depths)==6
    assert row_at(session.path,0)['exportable']
    e=edits(session.path)
    e['data']['episodes']=[{'start_frame':1,'end_frame':6,'task':'synthetic reach'}]
    save_edits(session.path,e['data'],e['etag'])
    result=export(session.path,tmp_path/'training')
    assert result['frames']==5
    reader=DepthReader(tmp_path/'training')
    for frame in range(5):
        d,values=reader.get('front',0,frame)
        assert d['source_seq']==frame+2
        np.testing.assert_array_equal(values,np.arange(320,dtype=np.uint16).reshape(16,20)+frame+2)
    assert len(list((session.path/'videos/front').glob('*.mp4')))>=2


def test_manual_invalid_export_refused_and_offline_is_new_version(captured,tmp_path):
    e=edits(captured.path)
    e['data']['episodes']=[{'start_frame':0,'end_frame':6,'task':'reach'}]
    e['data']['invalid_intervals']=[{'start_frame':2,'end_frame':3}]
    save_edits(captured.path,e['data'],e['etag'])
    with pytest.raises(ValueError,match='无效区间'):
        export(captured.path,tmp_path/'rejected')
    old=(captured.path/'raw.mcap').stat().st_size
    result=realign(captured.path)
    assert result['rows']==6 and result['valid_rows']==6
    assert row_at(captured.path,0,result['version'])['alignment_version'].endswith(result['version'])
    assert (captured.path/'raw.mcap').stat().st_size==old


def test_nonfinite_numeric_samples_preserve_original_bits_and_are_invalid():
    cfg,clocks=config(False),clock()
    sample=numeric(cfg,clocks,'joints',1,100_000_000,float('nan'))
    data=sample.data()
    assert data['valid'] is False
    assert data['payload']['position']==[None]
    assert data['raw_nonfinite_values'][0]['path']==['payload','position',0]
    import struct
    assert math.isnan(struct.unpack('<d',bytes.fromhex(data['raw_nonfinite_values'][0]['float64_le_hex']))[0])


def test_future_effective_command_cannot_evict_current_hold():
    cfg,clocks=config(False),clock()
    cfg['sources']={'commands':cfg['sources']['commands']}
    cache=TimeCache(cfg['buffers'])
    cache.add(numeric(cfg,clocks,'commands',0,1_000_000_000,1,effective_time_ns=1_000_000_000),1_000_000_000)
    cache.add(numeric(cfg,clocks,'commands',1,1_050_000_000,2,effective_time_ns=2_000_000_000),1_500_000_000)
    row=Aligner(cfg,cache).row(0,1_500_000_000,1_600_000_000)
    assert row['valid'] and row['action']['commands']['source_seq']==0


def test_early_stop_shortens_scheduled_session_and_finishes(tmp_path):
    cfg,clocks=config(False),clock()
    cfg['dataset']['duration_seconds']=3600
    cfg['sources']={'joints':cfg['sources']['joints']}
    session=Session(tmp_path,cfg,clocks,reports(cfg))
    session.start()
    stamp=time.monotonic_ns()
    session.submit(numeric(cfg,clocks,'joints',0,stamp,1).data())
    until=time.monotonic()+1
    while session.t0 is None and time.monotonic()<until:
        time.sleep(.001)
    assert session.end-session.t0==3600*10**9
    before=time.monotonic()
    result=session.stop()
    assert time.monotonic()-before<2
    assert result['end_ns']-result['t0_ns']<2*10**9


def test_encoder_failure_cannot_make_a_row_exportable(tmp_path,monkeypatch):
    import av
    cfg,clocks=config(),clock()
    session=Session(tmp_path,cfg,clocks,reports(cfg))
    def fail_open(*args,**kwargs):
        raise OSError('injected disk failure')
    monkeypatch.setattr(av,'open',fail_open)
    session.start()
    stamp=time.monotonic_ns()
    for ident in ('joints','commands'):
        session.submit(numeric(cfg,clocks,ident,0,stamp,0).data())
    session.submit(rgbd_data(stamp,1,time.monotonic_ns()),np.zeros((16,20,3),np.uint8),np.zeros((16,20),np.uint16))
    result=session.stop()
    assert result['state']=='failed'
    assert any('encoder_failure' in e['reason'] for e in result['errors'])
    if result['counters'].get('rows'):
        assert not row_at(session.path,0)['exportable']


def test_startup_disk_failure_does_not_leave_session_busy(tmp_path,monkeypatch):
    cfg,clocks=config(False),clock()
    session=Session(tmp_path,cfg,clocks,reports(cfg))
    def failed_start():
        raise OSError('injected disk unavailable')
    monkeypatch.setattr(session.storage,'start',failed_start)
    with pytest.raises(OSError,match='disk unavailable'):
        session.start()
    assert session.closed.is_set()
    assert session.stop()['state']=='failed'
