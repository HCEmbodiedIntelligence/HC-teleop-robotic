import pytest
from humanoid_upload import cli
from humanoid_upload.common import UploadError


def test_connection_probe_is_read_only_and_does_not_open_queue(monkeypatch,capsys):
    cfg={'robot_id':'robot','timeout_seconds':30}
    class Http:
        def __init__(self,config):assert config['timeout_seconds']==5
        def api(self,method,path):
            assert (method,path)==('GET',':capabilities')
            return {'schema_version':'robot-ingest-capabilities/v1','robot_id':'robot','max_recording_assets':4095}
    monkeypatch.setattr(cli,'Http',Http)
    monkeypatch.setattr(cli,'load_config',lambda _:cfg)
    monkeypatch.setattr(cli,'Queue',lambda *_:pytest.fail('Read-only probe opened a queue'))
    assert cli.main(['--config','unused','check-connection'])==0
    assert '"connected": true' in capsys.readouterr().out


@pytest.mark.parametrize('value,code',[
    ({},'CAPABILITIES_INVALID'),
    ({'schema_version':'robot-ingest-capabilities/v1','robot_id':'other'},'ROBOT_ID_MISMATCH'),
    ({'schema_version':'robot-ingest-capabilities/v1','robot_id':'robot','max_recording_assets':0},'RECORDING_UPLOAD_DISABLED'),
])
def test_probe_does_not_claim_connected_for_wrong_platform_or_identity(monkeypatch,value,code):
    class Http:
        def __init__(self,_):pass
        def api(self,*_):return value
    monkeypatch.setattr(cli,'Http',Http)
    with pytest.raises(UploadError) as error:cli.check_connection({'robot_id':'robot','timeout_seconds':30})
    assert error.value.code==code
