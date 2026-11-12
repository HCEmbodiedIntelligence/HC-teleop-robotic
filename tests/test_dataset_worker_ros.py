"""The isolated projection child must follow parent lifetime and surface errors."""
import json
import os
import subprocess
import sys

import pytest

pytestmark=pytest.mark.skipif(os.environ.get('HC_RUN_LEGACY_ROS_TESTS')!='1',
                             reason='explicit isolated ROS test only')


def assert_isolated():
    assert os.environ.get('ROS_DOMAIN_ID')=='231'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    assert not os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE')


def test_projection_process_stops_on_parent_pipe_eof():
    assert_isolated()
    process=subprocess.Popen([sys.executable,'-u','-m','middleware.core.dataset_worker'],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    try:
        # communicate closes the parent pipe immediately after the one config
        # frame. The child must finish even if EOF arrives during ROS startup.
        out,err=process.communicate(json.dumps(dict(config={'joint_groups':{},'camera_metadata':{}},
            domain_id=231,node_name='dataset_eof_test'))+'\n',timeout=8)
        assert process.returncode==0,(out,err)
        assert any(json.loads(line).get('state')=='ready' for line in out.splitlines())
    finally:
        if process.poll() is None:process.terminate();process.wait(timeout=3)


def test_projection_startup_error_is_reported_and_owned_child_is_reaped(monkeypatch):
    assert_isolated()
    from middleware.core.dataset_worker import DatasetTopicProcess
    original=subprocess.Popen
    children=[]
    def spawn(*args,**kwargs):
        child=original(*args,**kwargs);children.append(child);return child
    monkeypatch.setattr(subprocess,'Popen',spawn)
    worker=DatasetTopicProcess({},231,'dataset_bad_config_test')
    with pytest.raises(RuntimeError,match='joint_groups'):
        worker.start()
    assert children and all(child.poll() is not None for child in children)
    assert worker.process is None
