import pytest
pytest.importorskip("aiohttp")
pytest.importorskip("mcap")
import asyncio
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest
from aiohttp import web
from mcap.reader import make_reader
from mcap.writer import Writer

from middleware.core.data_tools import Datasets, DatasetPlayer
from middleware.core.data_quality import DataQuality


@pytest.fixture
def datasets(tmp_path):
    path=tmp_path/'source.mcap'
    with path.open('wb') as stream:
        writer=Writer(stream);writer.start()
        schema=writer.register_schema('sample','jsonschema',b'{}')
        channel=writer.register_channel('/joints','json',schema)
        for i in range(11):
            writer.add_message(channel,1_000_000_000+i*100_000_000,json.dumps({'position':[i*.1]}).encode(),1_000_000_000+i*100_000_000)
        writer.finish()
    return Datasets(SimpleNamespace(directory=tmp_path,is_recording=lambda:False,path=None))


def test_edit_trim_export_is_lossless_outside_excluded_segment_and_preserves_source(datasets):
    path=datasets.path('source.mcap');original=hashlib.sha256(path.read_bytes()).hexdigest()
    info=datasets.overview('source.mcap')
    updated=datasets.save('source.mcap',{'etag':info['etag'],'notes':'检查','trim':{'start':.1,'end':.9},
        'annotations':[{'id':'one','start':.3,'end':.5,'label':'invalid','note':'输入异常'}]})
    with pytest.raises(web.HTTPConflict):
        datasets.save('source.mcap',{'etag':info['etag']})
    exported=datasets.export('source.mcap',{'etag':updated['etag'],'filename':'clean.mcap','exclude_invalid':True})
    assert exported['message_count']==6
    with datasets.path('clean.mcap').open('rb') as stream:
        values=[json.loads(m.data)['position'][0] for _,_,m in make_reader(stream).iter_messages()]
    assert values==pytest.approx([.1,.2,.6,.7,.8,.9])
    assert hashlib.sha256(path.read_bytes()).hexdigest()==original
    with pytest.raises(web.HTTPConflict):
        datasets.export('source.mcap',{'etag':updated['etag'],'filename':'source.mcap'})
    with pytest.raises(ValueError):
        datasets.path('../source.mcap')
    assert datasets.audit('source.mcap')['ok']
    assert len(datasets.frames('source.mcap',.2,.4)['frames'])==3


def test_audit_detects_nonfinite_and_timestamp_regression(datasets):
    path=datasets.recorder.directory/'bad.mcap'
    with path.open('wb') as stream:
        writer=Writer(stream);writer.start();channel=writer.register_channel('/values','json',0)
        writer.add_message(channel,3_000_000_000,b'{"x":NaN}',3_000_000_000)
        writer.add_message(channel,2_000_000_000,b'{"x":1}',2_000_000_000)
        writer.finish()
    report=datasets.audit('bad.mcap')
    assert report['counts']=={'non_finite':1,'timestamp_regression':1}


def test_player_pause_seek_stop_and_error_visibility(datasets):
    events=[];player=DatasetPlayer(datasets,events.append)
    player.play('source.mcap',0,1)
    time.sleep(.08);player.pause();position=player.status()['position'];time.sleep(.13)
    assert player.status()['position']==position
    player.play('source.mcap',.8,4);time.sleep(.15)
    assert not player.status()['is_active']
    assert events[-1]['position']==pytest.approx(1)
    assert all(x['kind']=='data_preview_message' for x in events)
    player.stop()


def test_buttons_only_mark_pressed_edges_while_recording(tmp_path):
    recorded=[]
    recorder=SimpleNamespace(path=tmp_path/'active.mcap',is_recording=lambda:True,record=lambda event:recorded.append(event) or True)
    runtime=SimpleNamespace(recorder=recorder,config={'data_quality':{'scope':'segment','button_enabled':True,'controller':'right','button':'secondary'}},emit=lambda x:None)
    quality=DataQuality(runtime)
    def sample(action):
        quality.buttons({'payload':{'data':json.dumps({'edges':[{'controller':'right','button':'secondary','action':action}]})}})
    sample('pressed');assert quality.active_segment
    sample('released');assert len(recorded)==1
    sample('pressed');assert not quality.active_segment
    assert [x['payload']['action'] for x in recorded]==['start','end']
    assert recorded[0]['payload']['id']==recorded[1]['payload']['id']
    runtime.recorder.is_recording=lambda:False
    sample('pressed');assert len(recorded)==2
    assert json.loads((tmp_path/'active.mcap.session.json').read_text())['events']==quality.events


def test_recovery_of_truncated_footer_keeps_only_intact_messages(datasets):
    source=datasets.path('source.mcap').read_bytes()
    path=datasets.recorder.directory/'interrupted.mcap'
    path.write_bytes(source[:-80])
    result=datasets.recover('interrupted.mcap',{'filename':'recovered.mcap'})
    assert result['recovered_messages']==11
    assert result['stopped_at']
    assert path.read_bytes()==source[:-80]
    assert datasets.audit('recovered.mcap')['ok']


def test_point_marker_binds_to_nearest_sample_and_does_not_invalidate_next_sample_after_export(datasets):
    path=datasets.recorder.directory/'point.mcap'
    with path.open('wb') as stream:
        writer=Writer(stream);writer.start()
        data=writer.register_channel('/data','json',0)
        marker=writer.register_channel('/humanoid/annotations','json',0)
        writer.add_message(data,1_000_000_000,b'{"x":0}',1_000_000_000)
        writer.add_message(marker,1_010_000_000,json.dumps({'id':'point','action':'point','stamp_ns':1_010_000_000}).encode(),1_010_000_000)
        writer.add_message(data,1_100_000_000,b'{"x":1}',1_100_000_000)
        writer.finish()
    info=datasets.overview('point.mcap')
    assert info['edit']['annotations'][0]['start']==0
    result=datasets.export('point.mcap',{'etag':info['etag'],'filename':'point_clean.mcap'})
    assert result['message_count']==2
    assert datasets.overview('point_clean.mcap')['edit']['annotations']==[]
