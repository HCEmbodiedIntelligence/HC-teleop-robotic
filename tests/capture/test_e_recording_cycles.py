"""Explicit recording boundaries, using real aligner and disk writer threads."""
import json
import subprocess
import sys
import threading
import time

import pytest

from middleware.core.capture.catalog import connection
from mcap.reader import make_reader
from middleware.core.capture.hub import CaptureHub
from test_alignment import clock, config, metadata, reports


def feed(session, value, seconds=.22):
    deadline=time.monotonic()+seconds
    seq=0
    while time.monotonic()<deadline:
        seq+=1
        stamp=time.monotonic_ns()
        for source in ('joints','commands'):
            sample=metadata(source,seq,stamp)
            sample['payload']={'position':[value]}
            if source=='commands':sample.update(action_stage='sent',send_time_ns=stamp)
            assert session.submit(sample)['accepted']
        time.sleep(.01)


def hub_at(tmp_path):
    hub=CaptureHub(tmp_path,{'enabled':False})
    hub.cfg=config(False)
    hub.clocks=clock()
    hub.reports=reports(hub.cfg)
    hub.directory.mkdir(parents=True,exist_ok=True)
    return hub


def test_next_recording_starts_while_previous_writer_is_draining(tmp_path):
    hub=hub_at(tmp_path)
    release=threading.Event()
    draining=threading.Event()
    try:
        hub.start()
        first=hub.session
        close=first.storage.stop
        def slow_disk():
            draining.set()
            assert release.wait(5)
            close()
        first.storage.stop=slow_disk
        feed(first,11)
        started=time.monotonic()
        hub.request_stop()
        assert time.monotonic()-started<.1
        cutoff=first.end
        assert not hub.recording()
        assert draining.wait(2)
        assert not first.closed.is_set()
        hub.start()
        second=hub.session
        assert second.id!=first.id and second.ingress is not first.ingress
        assert first.cache is None and second.cache is None
        assert first in hub.input_sessions() and second in hub.input_sessions()
        assert hub.recording()
        feed(second,22)
        hub.request_stop()
        assert second.closed.wait(3)
        release.set()
        assert first.closed.wait(3)
        assert first.end==cutoff
        for session,value in ((first,11),(second,22)):
            assert session.state=='completed',list(session.errors)
            assert not (session.path/'index.sqlite3').exists()
            assert not (session.path/'quality.json').exists()
            with (session.path/'raw.mcap').open('rb') as stream:
                messages=[(c.topic,json.loads(m.data)) for _,c,m in make_reader(stream).iter_messages()]
            assert not any('/aligned' in topic for topic,_ in messages)
            rows=[body for topic,body in messages if topic.endswith('/raw/joints')]
            assert rows and all(row['payload']['position']==pytest.approx([value]) for row in rows)
        counts=(first.k,second.k)
        time.sleep(.08)
        assert (first.k,second.k)==counts
        assert not hub.input_sessions() and not hub.recording()
    finally:
        release.set()
        hub.shutdown()


def test_scheduled_duration_is_not_a_user_stop(tmp_path):
    hub=hub_at(tmp_path)
    try:
        hub.cfg['dataset']['duration_seconds']=10
        hub.start()
        feed(hub.session,1,.06)
        assert hub.session.end is not None
        assert hub.recording()
        with pytest.raises(ValueError,match='已经在录制'):
            hub.start()
        hub.request_stop()
        first_end=hub.session.end
        hub.request_stop()
        assert hub.session.end==first_end
    finally:hub.shutdown()


def test_stop_before_sources_ready_does_not_keep_recording(tmp_path):
    hub=hub_at(tmp_path)
    try:
        hub.start()
        first=hub.session
        hub.request_stop()
        assert first.closed.wait(2)
        assert first.k==0 and first.state=='failed'
        assert not hub.recording()
        hub.start()
        assert hub.session.id!=first.id
    finally:hub.shutdown()


def test_stop_during_input_copy_is_not_reported_as_queue_overflow(tmp_path,monkeypatch):
    from middleware.core.capture import session as module
    hub=hub_at(tmp_path)
    try:
        hub.start()
        feed(hub.session,1,.06)
        normalize=module.normalize
        def stop_during_copy(*args,**kwargs):
            sample=normalize(*args,**kwargs)
            hub.session.request_stop()
            return sample
        monkeypatch.setattr(module,'normalize',stop_during_copy)
        document=metadata('joints',100,time.monotonic_ns())
        document['payload']={'position':[1]}
        assert hub.session.submit(document)=={'accepted':False,'reason':'session_not_accepting'}
        assert hub.session.closed.wait(3)
        assert hub.session.state=='completed' and not hub.session.errors
        assert not hub.session.counters['ingress_rejected']
    finally:hub.shutdown()


def test_encoding_runs_alongside_capture_and_yields_only_under_pressure(tmp_path,monkeypatch):
    hub=hub_at(tmp_path)
    progress=tmp_path/'encoding-progress'
    worker=subprocess.Popen([sys.executable,'-c',
        'import pathlib,time,sys\np=pathlib.Path(sys.argv[1])\n'
        'while True:\n with p.open("ab") as f: f.write(b".")\n time.sleep(.01)',str(progress)])
    try:
        deadline=time.monotonic()+3
        while not progress.exists() and time.monotonic()<deadline:time.sleep(.01)
        assert progress.exists()
        with hub.lock:hub.process=worker
        hub.start()
        # Brief startup protection, followed by real concurrent progress.
        assert hub.job_paused
        before=progress.stat().st_size
        feed(hub.session,1,1.3)
        assert not hub.job_paused and progress.stat().st_size>before
        pressure=[.7]
        monkeypatch.setattr(hub,'_capture_pressure',lambda:pressure[0])
        deadline=time.monotonic()+3
        while not hub.job_paused and time.monotonic()<deadline:time.sleep(.02)
        assert hub.job_paused
        time.sleep(.05)
        paused=progress.stat().st_size
        feed(hub.session,1)
        assert progress.stat().st_size==paused
        # Hysteresis resumes encoding while the new recording is still active.
        pressure[0]=0.
        feed(hub.session,1,1.3)
        assert hub.recording() and not hub.job_paused and progress.stat().st_size>paused
        pressure[0]=.7
        deadline=time.monotonic()+3
        while not hub.job_paused and time.monotonic()<deadline:time.sleep(.02)
        assert hub.job_paused
        hub.request_stop()
        assert hub.session.closed.wait(3)
        deadline=time.monotonic()+3
        while hub.job_paused and time.monotonic()<deadline:time.sleep(.02)
        assert not hub.job_paused
    finally:
        hub.shutdown()
        worker.wait(timeout=3)
