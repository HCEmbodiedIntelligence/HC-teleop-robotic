import socket
from types import SimpleNamespace as NS

import pytest
from middleware.core.media_qos import mark_media_transport
from middleware.core.config import validate_config, ConfigError


def sender_for(sock):
    transport = NS(get_extra_info=lambda name: sock)
    return NS(transport=NS(transport=NS(_connection=NS(_protocols=[NS(transport=transport)]))))


def test_ipv4_actual_socket_mark_and_disable():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 2)
        sender = sender_for(sock)
        assert mark_media_transport(sender)['state'] == 'disabled'
        assert sock.getsockopt(socket.IPPROTO_IP, socket.IP_TOS) == 2
        result = mark_media_transport(sender, 34)
        assert result['state'] == 'marked' and result['marked_sockets'] == 1
        assert not result['wmm_verified']
        assert sock.getsockopt(socket.IPPROTO_IP, socket.IP_TOS) == 0x8A


def test_ipv6_actual_socket():
    if not socket.has_ipv6:
        pytest.skip('IPv6 unavailable')
    with socket.socket(socket.AF_INET6, socket.SOCK_DGRAM) as sock:
        assert mark_media_transport(sender_for(sock), 34)['state'] == 'marked'
        assert sock.getsockopt(socket.IPPROTO_IPV6, socket.IPV6_TCLASS) == 0x88


def test_unsupported_dependency_layout_is_visible():
    assert mark_media_transport(NS(), 34)['state'] == 'unsupported'


@pytest.mark.parametrize('value', [True, -1, 64, '34', 34.0])
def test_bad_dscp_rejected_for_global_and_stream(value):
    with pytest.raises(ConfigError):
        validate_config({'camera': {'dscp': value}})
    with pytest.raises(ConfigError):
        validate_config({'camera': {'streams': [{'id':'head','topic':'/camera','dscp':value}]}})


def test_media_options_inheritance_and_override():
    camera = validate_config({'camera': {'encoder_backend':'auto', 'dscp':34,
        'streams':[{'id':'head','topic':'/camera'}, {'id':'wrist','topic':'/wrist','dscp':None,'encoder_backend':'x264'}]}})['camera']
    assert 'dscp' not in camera['streams'][0]
    assert 'encoder_backend' not in camera['streams'][0]
    assert {**camera, **camera['streams'][0]}['encoder_backend'] == 'auto'
    camera['encoder_backend'] = 'nvenc'
    camera = validate_config({'camera': camera})['camera']
    assert {**camera, **camera['streams'][0]}['encoder_backend'] == 'nvenc'
    assert camera['streams'][1]['dscp'] is None


def test_relay_without_exposed_socket_is_not_reported_as_fully_marked():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sender = sender_for(sock)
        sender.transport.transport._connection._protocols.append(NS(transport=NS(get_extra_info=lambda name: None)))
        result = mark_media_transport(sender, 34)
        assert result['state'] == 'partial'
        assert result['unmarked_protocols'] == 1
        assert not result['selected_path_verified']


def test_process_stale_status_invalidates_receiver_measurements():
    from middleware.core.camera_process import CameraProcessService
    service = CameraProcessService({'enabled': False})
    service._status.update(receiver_stale=False, receiver_decoded_fps=30, receiver_render_gap_p95_ms=35)
    status = service.status()
    assert status['receiver_stale']
    assert status['receiver_decoded_fps'] is None
    assert status['receiver_render_gap_p95_ms'] is None


@pytest.mark.parametrize('options', [
    {'width':1920,'height':1080}, {'width':1280,'height':720,'fps':60}, {'width':641},
])
def test_h264_rejects_output_beyond_advertised_level(options):
    with pytest.raises(ConfigError, match='level 3.1'):
        validate_config({'camera':options})
    with pytest.raises(ConfigError, match='level 3.1'):
        validate_config({'camera':{'streams':[{'id':'head','topic':'/camera',**options}]}})


def test_h264_level_31_accepts_720p30():
    assert validate_config({'camera':{'width':1280,'height':720,'fps':30}})['camera']['width'] == 1280
