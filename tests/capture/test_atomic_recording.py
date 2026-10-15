import subprocess
import sys
from pathlib import Path
import pytest
from middleware.core.capture.config import validate

@pytest.mark.parametrize('mode',['rgbd','rgb'])
def test_atomic_dds_recording_without_separate_metadata(mode):
    result=subprocess.run([sys.executable,str(Path(__file__).with_name('ros_atomic_recording_case.py')),mode],
                          text=True,capture_output=True,timeout=30)
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    assert '"atomic_transport": true' in result.stdout

@pytest.mark.parametrize('packet',[None,'relative', '/bad path',42])
def test_bad_packet_topic_rejected(packet):
    with pytest.raises(ValueError,match='原子相机消息话题'):
        validate({'sources':{'front':{'kind':'rgbd','transport':'ros_rgbd','topic':'/legacy',
                                    'packet_topic':packet}}})
