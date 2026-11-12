#!/usr/bin/env python3
"""模拟 VR UDP 位姿和右 Grip，自动测试 AUTO → MANUAL → AUTO；仅用于隔离仿真。"""
import argparse
import copy
import json
import math
from pathlib import Path
import signal
import socket
import struct
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://172.168.200.30:7876')
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--auto-seconds', type=float, default=4)
    parser.add_argument('--manual-seconds', type=float, default=3)
    parser.add_argument('--displacement', type=float, default=.015, help='接管时双手相对位移，m')
    parser.add_argument('--joint-recording', action='store_true', help='临时只录关节 MCAP，结束后恢复原配置')
    parser.add_argument('--report', type=Path, default=Path('runtime/dagger_two_scripts/result.json'))
    args = parser.parse_args()
    if (args.cycles < 1 or not all(math.isfinite(v) for v in
            (args.auto_seconds, args.manual_seconds, args.displacement))
            or min(args.auto_seconds, args.manual_seconds) < .5 or not 0 < args.displacement <= .05):
        parser.error('cycles >= 1，两阶段时长 >= 0.5 s，0 < displacement <= 0.05 m')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    if args.report.exists():
        parser.error('--report 已存在，请换新路径以保留之前的结果')
    base = args.url.rstrip('/')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, data=None, method=None):
        body = None if data is None else json.dumps(data).encode()
        req = urllib.request.Request(base+path, data=body, method=method,
                                     headers={'Content-Type': 'application/json'})
        try:
            with opener.open(req, timeout=30) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            raise RuntimeError(f'{path}: HTTP {exc.code}: {exc.read().decode()}') from exc

    simulation = request('/api/simulation')
    if (simulation['runtime_mode'] != 'simulation' or simulation['state'] != 'running'
            or int(simulation['ros_domain_id']) == int(simulation['hardware_domain_id'])):
        raise RuntimeError('仅支持正在运行、与真机 ROS 域隔离的仿真')
    initial = request('/api/control/ownership')
    live = request('/api/status')
    if initial['state'] != 'OFF' or live['recording']['recording']:
        raise RuntimeError('请先退出已有 DAgger 会话和普通录制')
    if live['vr']['last_packet_at'] and time.time() - live['vr']['last_packet_at'] < 2:
        raise RuntimeError('已有 VR 输入正在发送，请先断开它')
    original = request('/api/config')
    if live['recording']['recording_selection'].get('mode') != 'local':
        raise RuntimeError('本测试需要在上传管理中选择本地保存')

    def action(name):
        state = request('/api/control/ownership')
        return request('/api/control/intervention/'+name, dict(request_id=uuid.uuid4().hex,
            session_id=state['session_id'], expected_epoch=state['epoch']))

    stop = threading.Event()
    lock = threading.Lock()
    grip, manual_start, sending_error = False, None, []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host = urllib.parse.urlparse(base).hostname
    port = int(original['vr']['pose_port'])
    thread = None
    changed, enabled = False, False
    result = dict(result='FAIL', simulation={k: simulation[k] for k in
        ('profile_id', 'ros_domain_id', 'hardware_domain_id', 'runtime_mode')},
        input=f'PICO v2 UDP {host}:{port} -> /vrdata', cycles=[], started_at=time.time())

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, interrupted)

    def send_vr():
        sequence, previous, tick = 0, False, time.monotonic()
        try:
            while not stop.is_set():
                with lock:
                    held, began = grip, manual_start
                elapsed = 0 if began is None else time.monotonic() - began
                delta = args.displacement * math.sin(math.pi*min(1., elapsed/args.manual_seconds)) if held else 0.
                poses = (0., 1.5, 0., 0., 0., 0., 1.,
                         -.2, 1.3, -.4-delta, 0., 0., 0., 1.,
                         .2, 1.3, -.4-delta, 0., 0., 0., 1.)
                bit = 4  # grip_button; v2 also carries the analog grip value.
                packet = struct.pack('<4sBIdB21f3H6f3H6f', b'PICO', 2, sequence,
                    time.monotonic(), 7, *poses, 0, 0, 0, .5, 0., 0., 0., 0., 0.,
                    bit if held else 0, bit if held and not previous else 0,
                    bit if previous and not held else 0, .5, float(held), 0., 0., 0., 0.)
                sock.sendto(packet, (host, port))
                sequence = (sequence + 1) & 0xffffffff
                previous = held
                tick = max(tick + .01, time.monotonic())
                stop.wait(max(0, tick-time.monotonic()))
        except Exception as exc:
            sending_error.append(str(exc))

    def wait_for(predicate, timeout=15):
        deadline, last = time.monotonic()+timeout, None
        while time.monotonic() < deadline:
            if sending_error:
                raise RuntimeError(sending_error[0])
            last = request('/api/control/ownership')
            if last['state'] in ('FAULT', 'ESTOP'):
                raise RuntimeError(f"{last['state']}: {last['reason']}")
            if predicate(last):
                return last
            time.sleep(.025)
        raise RuntimeError('等待超时：'+json.dumps(last, ensure_ascii=False))

    def dwell(seconds, state):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            current = wait_for(lambda s: s['state'] == state, timeout=1)
            if current.get('recording_error'):
                raise RuntimeError(current['recording_error'])
            time.sleep(min(.1, max(0, deadline-time.monotonic())))
        return request('/api/control/ownership')

    def mux():
        return request('/api/status')['ros']['command_mux']

    def wait_program(timeout=30):
        before = mux()
        deadline = time.monotonic()+timeout
        print('等待外部程序的 ServoJ 最终输出；请在另一终端运行 dagger_program.py', flush=True)
        while time.monotonic() < deadline:
            dwell(.2, 'AUTO')
            after = mux()
            if (after['received']['motion'] > before['received']['motion']
                    and after['forwarded'] > before['forwarded']):
                return
        raise RuntimeError('未收到程序最终输出；请先启动 dagger_program.py')

    try:
        if args.joint_recording:
            config = copy.deepcopy(original)
            for item in config['ros']['subscriptions']:
                outputs = [value for value in item.get('outputs', []) if value != 'record']
                if item['topic'] in ('/observation/joint_states', '/action/joint_states'):
                    item['enabled'] = True
                    outputs.append('record')
                item['outputs'] = outputs
            config['ros']['recording']['format'] = 'mcap'
            config['ros']['recording']['action_sources'] = []
            args.report.with_suffix('.original-config.json').write_text(json.dumps(original, ensure_ascii=False, indent=2))
            changed = True
            request('/api/config', config, 'PUT')
        wait_for(lambda s: {'motion', 'teleop'} <= s['peers'].keys())
        thread = threading.Thread(target=send_vr, name='simulated-vr', daemon=True)
        thread.start()
        time.sleep(.3)
        enabled = True  # Ensure partial enable failures are also cleaned up.
        opened = action('enable')
        result['session_id'] = opened['ownership']['session_id']
        wait_for(lambda s: s['input_valid'] and s['input_armed'] and s['capture'] and s['capture']['ready'], 25)
        action('return')  # Only the initial start uses HTTP; every handover uses Grip packets.
        wait_for(lambda s: s['state'] == 'AUTO')
        wait_program()
        for index in range(args.cycles):
            print(f"[{index+1}/{args.cycles}] AUTO：程序运行 {args.auto_seconds:g}s", flush=True)
            auto_before = mux()
            dwell(args.auto_seconds, 'AUTO')
            auto_after = mux()
            if (auto_after['received']['motion'] <= auto_before['received']['motion']
                    or auto_after['forwarded'] <= auto_before['forwarded']):
                raise RuntimeError('AUTO 阶段程序没有产生最终输出')
            sent = time.monotonic()
            with lock:
                grip, manual_start = True, None
            manual = wait_for(lambda s: s['state'] == 'MANUAL', 5)
            takeover_ms = (time.monotonic()-sent)*1000
            with lock:
                manual_start = time.monotonic()
            print(f"[{index+1}/{args.cycles}] MANUAL：VR 已抢占，手柄移动 {args.manual_seconds:g}s", flush=True)
            end = dwell(args.manual_seconds, 'MANUAL')
            if end['command_seq'] <= 0:
                raise RuntimeError('进入 MANUAL，但没有人工动作通过最终输出门')
            sent = time.monotonic()
            with lock:
                grip, manual_start = False, None
            resumed = wait_for(lambda s: s['state'] == 'AUTO', 5)
            cycle = dict(cycle=index+1, intervention_id=manual['intervention_id'],
                manual_epoch=manual['epoch'], resumed_epoch=resumed['epoch'],
                human_commands=end['command_seq'], takeover_observed_ms=takeover_ms,
                return_observed_ms=(time.monotonic()-sent)*1000,
                auto_motion_commands=auto_after['received']['motion']-auto_before['received']['motion'])
            result['cycles'].append(cycle)
            print(f"[{index+1}/{args.cycles}] AUTO：松开 Grip 后已自动恢复", flush=True)
        resume_before = mux()
        dwell(args.auto_seconds, 'AUTO')
        resume_after = mux()
        if resume_after['forwarded'] <= resume_before['forwarded']:
            raise RuntimeError('最后一次松手后没有恢复程序输出')
        final = request('/api/control/ownership')
        result['metrics'] = final['metrics']
        result['capture'] = final['capture']
        result['result'] = 'PASS'
    except BaseException as exc:
        result['error'] = str(exc) or type(exc).__name__
        raise
    finally:
        # Disable before stopping valid VR samples; stopping samples in AUTO would latch FAULT.
        try:
            if enabled and request('/api/control/ownership')['state'] != 'OFF':
                action('disable')
            result['final_ownership'] = request('/api/control/ownership')
            result['recordings'] = request('/api/recordings')
        except Exception as exc:
            result['result'] = 'FAIL'
            result['cleanup_error'] = str(exc)
        finally:
            stop.set()
            if thread:
                thread.join(timeout=2)
            sock.close()
            if changed:
                try:
                    request('/api/config', original, 'PUT')
                    result['configuration_restored'] = True
                except Exception as exc:
                    result['result'] = 'FAIL'
                    result['restore_error'] = str(exc)
            result['finished_at'] = time.time()
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
            print(f"{result['result']}：{args.report}", flush=True)
    if result['result'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
