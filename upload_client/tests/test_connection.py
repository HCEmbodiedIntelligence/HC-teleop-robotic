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


@pytest.mark.parametrize('host', ['127.0.0.1', '10.20.30.40', '192.168.2.23', '172.19.0.5',
                                 '169.254.1.2', '[::1]', '[fd00::1]', 'localhost.'])
def test_lan_platform_does_not_enter_internet_proxy(monkeypatch, host):
    from urllib.request import ProxyHandler, Request
    from humanoid_upload.http import UploadProxyHandler
    monkeypatch.setattr(ProxyHandler, 'proxy_open', lambda *args: pytest.fail('LAN request entered proxy'))
    assert UploadProxyHandler({}).proxy_open(Request(f'http://{host}:8088/api'), 'http://proxy:8080', 'http') is None


@pytest.mark.parametrize('assigned', [True, False])
def test_public_range_address_on_local_interface_bypasses_proxy(monkeypatch, assigned):
    import errno
    from urllib.request import ProxyHandler, Request
    from humanoid_upload import http
    class Probe:
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def bind(self, address):
            assert address == ('172.168.200.30', 0)
            if not assigned: raise OSError(errno.EADDRNOTAVAIL, 'not a local interface')
    monkeypatch.setattr(http.socket, 'socket', lambda *args: Probe())
    proxied = []
    monkeypatch.setattr(ProxyHandler, 'proxy_open', lambda *args: proxied.append(True))
    http.UploadProxyHandler({}).proxy_open(Request('http://172.168.200.30:8088/api'), 'http://proxy:8080', 'http')
    assert bool(proxied) is not assigned


@pytest.mark.parametrize('kind,code,reason', [
    ('timeout', 'NETWORK_TIMEOUT', '超时'),
    ('dns', 'NETWORK_DNS_FAILED', '域名'),
    ('refused', 'NETWORK_CONNECTION_REFUSED', '拒绝连接'),
    ('unreachable', 'NETWORK_UNREACHABLE', '无法到达'),
    ('certificate', 'TLS_CERTIFICATE_INVALID', '证书验证失败'),
    ('reset', 'NETWORK_UNCERTAIN', '连接中断'),
])
@pytest.mark.parametrize('wrapped', [True, False])
def test_transport_failures_have_safe_specific_messages(monkeypatch, kind, code, reason, wrapped):
    import errno
    import socket
    import ssl
    from urllib.error import URLError
    from humanoid_upload.http import Http
    errors = {
        'timeout': TimeoutError('secret-ticket'),
        'dns': socket.gaierror(socket.EAI_NONAME, 'secret-ticket'),
        'refused': ConnectionRefusedError(errno.ECONNREFUSED, 'secret-ticket'),
        'unreachable': OSError(errno.ENETUNREACH, 'secret-ticket'),
        'certificate': ssl.SSLCertVerificationError(1, 'secret-ticket'),
        'reset': ConnectionResetError(errno.ECONNRESET, 'secret-ticket'),
    }
    failure = URLError(errors[kind]) if wrapped else errors[kind]
    client = Http({'platform_base_url': 'https://platform.example', 'timeout_seconds': 30})
    def fail(*args, **kwargs): raise failure
    monkeypatch.setattr(client.opener, 'open', fail)
    with pytest.raises(UploadError) as error:
        client.request('POST', 'https://platform.example/api', payload={'ticket': 'secret-ticket'}, anonymous=True)
    assert error.value.code == code
    assert reason in error.value.action
    assert 'secret-ticket' not in error.value.action
    assert error.value.retry == (kind != 'certificate')


@pytest.mark.parametrize('status,payload,reason,retryable', [
    (502, b'<html>secret-ticket</html>', '平台服务或网络代理暂时不可用', True),
    (504, b'', '平台请求超时', True),
    (404, b'', '平台接口不存在', False),
    (410, b'{"code":"CONNECTION_TICKET_EXPIRED"}', '连接链接已过期', False),
    (409, b'{"code":"CONNECTION_TICKET_USED"}', '已被其他请求使用', False),
    (401, b'{"code":"ROBOT_CREDENTIAL_REVOKED"}', '上传凭据已撤销', False),
])
def test_http_failures_explain_cause_and_recovery(monkeypatch, status, payload, reason, retryable):
    import io
    from urllib.error import HTTPError
    from humanoid_upload.http import Http
    client = Http({'platform_base_url': 'https://platform.example', 'timeout_seconds': 30})
    def fail(*args, **kwargs):
        raise HTTPError('https://platform.example', status, 'secret-ticket', {}, io.BytesIO(payload))
    monkeypatch.setattr(client.opener, 'open', fail)
    with pytest.raises(UploadError) as error:
        client.request('POST', 'https://platform.example/api', anonymous=True)
    assert reason in error.value.action
    assert 'secret-ticket' not in error.value.action
    assert error.value.retry is retryable


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
