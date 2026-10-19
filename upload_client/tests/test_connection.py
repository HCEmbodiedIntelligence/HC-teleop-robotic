import pytest
from humanoid_upload import cli
from humanoid_upload.common import UploadError


def test_missing_credential_file_reports_missing_not_permissions(tmp_path):
    from humanoid_upload.common import credential
    with pytest.raises(UploadError) as error:
        credential('file:' + str(tmp_path / 'absent.token'))
    assert error.value.code == 'CREDENTIAL_MISSING'


def test_loopback_upload_ignores_environment_proxy(monkeypatch):
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from humanoid_upload.http import Http

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            assert self.headers['Authorization'] == 'Bearer synthetic-local-token'
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"connected":true}')

        def log_message(self, *_):
            pass

    monkeypatch.setenv('http_proxy', 'http://127.0.0.1:1')
    monkeypatch.setenv('HTTP_PROXY', 'http://127.0.0.1:1')
    monkeypatch.setenv('no_proxy', '')
    monkeypatch.setenv('NO_PROXY', '')
    monkeypatch.setenv('HC_TEST_LOCAL_TOKEN', 'synthetic-local-token')
    with ThreadingHTTPServer(('127.0.0.1', 0), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            cfg = dict(platform_base_url=f'http://127.0.0.1:{server.server_port}',
                       credential_ref='env:HC_TEST_LOCAL_TOKEN', timeout_seconds=2)
            assert Http(cfg).api('GET', ':capabilities') == {'connected': True}
        finally:
            server.shutdown()
            thread.join()


def test_external_upload_keeps_proxy_support(monkeypatch):
    from urllib.request import ProxyHandler, Request
    from humanoid_upload.http import UploadProxyHandler
    called = []
    monkeypatch.setattr(ProxyHandler, 'proxy_open', lambda self, *args: called.append(args))
    UploadProxyHandler({}).proxy_open(Request('https://platform.example/api'), 'http://proxy:8080', 'https')
    assert len(called) == 1


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
