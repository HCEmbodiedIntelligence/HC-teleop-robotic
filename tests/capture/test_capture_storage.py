"""Changing disks preserves old data access and does not silently reset policy."""
import copy
import json
from pathlib import Path

import pytest

from middleware.core.capture.hub import CaptureHub
from middleware.core.capture.storage import atomic_json


def fixture_session(hub, ident):
    path=hub.directory/ident
    path.mkdir()
    atomic_json(path/'session.json',{'recording_mode':'raw','session_id':ident,'config':hub.cfg})
    atomic_json(path/'manifest.json',{'recording_mode':'raw','state':'completed','committed':[]})
    return path


def test_switch_directory_keeps_old_sessions_after_restart(tmp_path):
    hub=CaptureHub(tmp_path/'manager',{'enabled':False})
    old=fixture_session(hub,'previous')
    try:
        cfg=copy.deepcopy(hub.cfg)
        cfg['storage'].update(directory=str(tmp_path/'disk'/'raw'),export_directory=str(tmp_path/'disk'/'bundles'))
        state=hub.save_config(cfg,hub.state()['etag'])
        new=fixture_session(hub,'next')
        assert state['directory']==str(new.parent)
        assert state['export_directory']==str(tmp_path/'disk'/'bundles')
        assert {s['directory'] for s in hub.sessions()}=={str(old),str(new)}
        assert hub.session_path('previous')==old
        hub.jobs_stop.set();hub.job_thread.join(2)
        job=hub.start_job('recording','previous',{})
        assert job['source']==str(old) and Path(job['output']).parent==hub.export_directory
    finally:hub.shutdown()
    again=CaptureHub(tmp_path/'manager',{'enabled':False})
    try:
        assert again.session_path('previous')==old
        assert again.session_path('next')==new
        with pytest.raises(ValueError):again.session_path('../manager')
        (again.directory/'escape').symlink_to(tmp_path/'manager',target_is_directory=True)
        with pytest.raises(ValueError):again.session_path('escape')
    finally:again.shutdown()


def test_invalid_directory_does_not_replace_saved_config(tmp_path):
    hub=CaptureHub(tmp_path/'manager',{'enabled':False})
    try:
        hub.save_config(hub.cfg,hub.state()['etag'])
        before=hub.config_path.read_bytes()
        blocked=tmp_path/'file';blocked.write_text('not a directory')
        cfg=copy.deepcopy(hub.cfg);cfg['storage']['directory']=str(blocked/'raw')
        with pytest.raises(ValueError,match='不可写'):hub.save_config(cfg,hub.state()['etag'])
        assert hub.config_path.read_bytes()==before
        assert hub.cfg==json.loads(before)
    finally:hub.shutdown()
