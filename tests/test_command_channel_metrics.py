"""Arm frequency must never be inferred from mixed arm/finger packet totals."""
from collections import deque
from types import SimpleNamespace
import time

import pytest

from middleware.core.ros_bridge import RosBridge


def rig(monkeypatch):
    clock = [10.]
    monkeypatch.setattr('middleware.core.ros_bridge.time.monotonic', lambda: clock[0])
    bridge = RosBridge({}, lambda *args: None)
    bridge._boundary = SimpleNamespace(arms=('arm',), hands={'right': ('right_finger',), 'left': ('left_finger',)},
        prepare=lambda m: m, commit=lambda *_args, **_kwargs: True)
    bridge.receive_hardware_ready(True)
    bridge._command_output_enabled = True
    sent = []
    def send(name, when, publish=None):
        clock[0] = when
        sec, nanosec = divmod(time.time_ns(), 10**9)
        msg = SimpleNamespace(name=[name], position=[.1],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))
        assert bridge.forward_live_command('vr', msg, publish or sent.append)
        return msg
    return bridge, clock, sent, send


def test_combined_rate_above_200_does_not_hide_50hz_arm(monkeypatch):
    bridge, clock, sent, send = rig(monkeypatch)
    for tick in range(20):
        when = 10. + tick*.01
        if tick % 2 == 0:
            send('arm', when)
        send('right_finger', when)
        send('left_finger', when)
    mux = bridge.status()['command_mux']
    assert mux['forwarded'] == mux['received']['vr'] == len(sent) == 50
    assert mux['forwarded_hz'] > 200
    assert mux['rate_scope']['forwarded_hz'] == 'all_sources_combined_joint_channels'
    channels = mux['channels']
    assert channels['arms']['forwarded_hz'] == channels['arms']['received_hz'] == 50.
    assert channels['right_hand']['forwarded_hz'] == 100.
    assert channels['left_hand']['forwarded_hz'] == 100.
    assert channels['arms']['received'] == channels['arms']['forwarded'] == 10
    assert channels['arms']['max_publication_gap_ms'] == 20.
    assert channels['other']['received'] == channels['other']['forwarded'] == 0
    assert channels['other']['max_publication_gap_ms'] is None
    send('arm', 10.28)
    assert bridge.status()['command_mux']['channels']['arms']['max_publication_gap_ms'] == 100.


def test_rejected_or_failed_send_is_received_but_never_counted_as_publication(monkeypatch):
    bridge, clock, sent, send = rig(monkeypatch)
    duplicate = send('arm', 10.)
    clock[0] = 10.01
    assert not bridge.forward_live_command('vr', duplicate, sent.append)
    def failed(_):
        raise RuntimeError('fake DDS failure')
    with pytest.raises(RuntimeError, match='fake DDS failure'):
        send('right_finger', 10.02, failed)
    channels = bridge.status()['command_mux']['channels']
    assert channels['arms']['received'] == 2
    assert channels['arms']['forwarded'] == 1
    assert channels['right_hand']['received'] == 1
    assert channels['right_hand']['forwarded'] == 0
    assert channels['right_hand']['max_publication_gap_ms'] is None
    clock[0] = 13.
    channels = bridge.status()['command_mux']['channels']
    assert channels['arms']['forwarded_hz'] == 0.
    assert channels['arms']['max_publication_gap_ms'] is None


def test_channel_gap_scans_are_outside_authority_lock(monkeypatch):
    bridge, clock, sent, send = rig(monkeypatch)
    class OutsideFloat(float):
        def __sub__(self, other):
            assert not bridge._lock._is_owned()
            return float(self)-float(other)
        def __rsub__(self, other):
            assert not bridge._lock._is_owned()
            return float(other)-float(self)
    clock[0] = 10.1
    samples = deque([OutsideFloat(10.), OutsideFloat(10.05)], maxlen=300)
    bridge._mux_channel_forwarded_times['arms'] = samples
    assert bridge.status()['command_mux']['channels']['arms']['max_publication_gap_ms'] == 50.


def test_marker_fields_do_not_obscure_physical_hand_and_unknown_cache_is_bounded(monkeypatch):
    bridge, clock, sent, send = rig(monkeypatch)
    assert bridge._command_channel(('R_ban', 'right_finger')) == 'right_hand'
    assert bridge._command_channel(('L_ban', 'left_finger')) == 'left_hand'
    assert bridge._command_channel(('arm', 'right_finger')) == 'arms'
    assert bridge._command_channel(('right_finger', 'left_finger')) == 'other'
    for index in range(100):
        assert bridge._command_channel(('unknown_'+str(index),)) == 'other'
    assert len(bridge._command_channel_cache) == 64
