#!/usr/bin/env python3
"""反复准备录制资源，检查有效 VR 输入下是否出现启动阶段超时。"""
import time
from concurrent.futures import ThreadPoolExecutor
from dagger_sim_common import run


def repeated_start(s):
    s.program_start()
    prepared = []
    for index in range(12):
        started = time.monotonic()
        mode = 'full_session' if index % 2 else 'corrections_only'
        s.session(auto=False, capture_mode=mode)
        prepared.append(time.monotonic() - started)
        until = time.monotonic() + 3.2
        while time.monotonic() < until:
            assert s.owner['state'] == 'HOLD', s.owner
            time.sleep(.02)
        print('startup cycle', index + 1, 'PASS', flush=True)
    assert not s.send_gaps, s.send_gaps
    return dict(cycles=12, preparation_seconds=prepared, vr_send_gaps_over_50ms=0)


def held_during_preparation(s):
    s.neutral()
    # Standalone cold-start runs begin with the configured startup safety stop.
    s.request('/api/safety/resume', {})
    s.wait(lambda: s.request('/api/status')['ros']['command_mux']['hardware_ready'], 5, 'simulation driver ready')
    s.program_start()
    time.sleep(.35)
    with ThreadPoolExecutor(max_workers=1) as executor:
        enabling = executor.submit(s.action, 'enable')
        s.wait(lambda: s.request('/api/control/ownership').get('preparing_capture'),
               10, 'capture preparation begins')
        before = dict(s.joints)
        started = time.monotonic()
        s.set_vr(grip=1.)
        result = enabling.result(timeout=30)
    s.sessions.append(result['ownership']['session_id'])
    def ready():
        owner = s.request('/api/control/ownership')
        assert owner['state'] == 'HOLD', owner
        assert not owner['input_armed'], owner
        return owner['capture']['ready'] and not owner['preparing_capture']
    s.wait(ready, 25, 'capture ready with Grip held')
    time.sleep(.4)
    s.state('HOLD')
    s.static(started, before)
    s.set_vr(grip=0.)
    s.wait(lambda: s.request('/api/control/ownership')['input_armed'], 5, 'fresh release')
    s.action('return')
    s.state('AUTO')
    s.manual()
    s.set_vr(grip=0.)
    s.state('AUTO')
    return dict(held_during_preparation_remained_hold=True, fresh_release_then_takeover=True)


if __name__ == '__main__':
    run([('held_grip_during_preparation', held_during_preparation),
         ('repeated_recording_preparation', repeated_start)], 'startup')
