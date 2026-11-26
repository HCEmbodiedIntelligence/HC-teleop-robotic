#!/usr/bin/env python3
"""仿真中的外部程序：持续发 ServoJ，即使 VR 已经接管也不停发。"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import signal
import time
import urllib.request


def subscribe_robot_state(node, info, feedback):
    """Follow the profile's measured joint stream with one active DDS reader."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    topic = None
    subscription = None

    def discover(message):
        nonlocal topic, subscription
        profile = json.loads(message.data)
        selected = profile['endpoints']['joint_states']
        if selected != topic:
            if subscription is not None:
                node.destroy_subscription(subscription)
            subscription = node.create_subscription(JointState, selected, feedback, qos_profile_sensor_data)
            topic = selected
        info.update(profile)

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/status/robot/info', discover, qos)


def arm_commands(node, info, measured, amplitude):
    from sensor_msgs.msg import JointState

    arms = {}
    groups = info['groups']
    for side in ('left_arm', 'right_arm'):
        spec = groups[side]
        names = spec['joint_names']
        if len(names) < 4 or 'servo_j' not in spec['endpoints']:
            raise RuntimeError(f'{side} 不支持本示例')
        baseline = [measured[n] for n in names]
        limit = spec['limits'][3]
        if not limit['lower'] <= baseline[3]-amplitude < baseline[3]+amplitude <= limit['upper']:
            raise RuntimeError(f'{side} 第 4 关节初始位置离限位太近，请减小 --amplitude')
        arms[side] = baseline

    whole_body = groups.get('whole_body', {}).get('endpoints', {}).get('servo_j')
    channels = [(whole_body, tuple(arms))] if whole_body else [
        (groups[side]['endpoints']['servo_j'], (side,)) for side in arms]
    commands = []
    for endpoint, sides in channels:
        names, positions, moving = [], [], []
        for side in sides:
            moving.append((len(names) + 3, arms[side][3]))
            names.extend(groups[side]['joint_names'])
            positions.extend(arms[side])
        commands.append((endpoint, node.create_publisher(JointState, endpoint, 1),
                         JointState(name=names, position=positions), moving))
    return commands, arms


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://172.168.200.30:7876')
    parser.add_argument('--duration', type=float, default=0, help='秒；0 表示一直运行，Ctrl+C 停止')
    parser.add_argument('--amplitude', type=float, default=.12, help='双臂第 4 关节的正弦振幅，rad')
    parser.add_argument('--period', type=float, default=8)
    parser.add_argument('--rate', type=float, default=50)
    parser.add_argument('--log', type=Path, help='可选 JSONL，记录输入、反馈和最终输出证据')
    args = parser.parse_args()
    if (not all(math.isfinite(x) for x in (args.duration, args.amplitude, args.period, args.rate))
            or args.duration < 0 or not 0 < args.amplitude <= .3 or args.period < 4 or not 20 <= args.rate <= 100):
        parser.error('duration >= 0，0 < amplitude <= 0.3，period >= 4，20 <= rate <= 100')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(args.url.rstrip('/') + '/api/simulation', timeout=5) as response:
        simulation = json.load(response)
    domain = int(simulation['ros_domain_id'])
    if (simulation['state'] != 'running' or simulation['runtime_mode'] != 'simulation'
            or domain == int(simulation['hardware_domain_id'])):
        raise RuntimeError('仅支持正在运行、与真机 ROS 域隔离的仿真')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('dagger_external_program_demo')
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop)
    stream = None
    info, measured, owner = {}, {}, {}
    counters = collections.Counter()
    measured_at, logged_feedback = 0., 0.
    started = time.monotonic()

    def log(kind, **value):
        if stream:
            stream.write(json.dumps(dict(kind=kind, monotonic_ns=time.monotonic_ns(),
                time_ns=time.time_ns(), **value), ensure_ascii=False) + '\n')

    def sample(message):
        return dict(stamp_ns=message.header.stamp.sec*10**9 + message.header.stamp.nanosec,
                    frame_id=message.header.frame_id, names=list(message.name), positions=list(message.position))

    def feedback(message):
        nonlocal measured_at, logged_feedback
        measured.update(zip(message.name, message.position))
        measured_at = time.monotonic()
        if stream and measured_at - logged_feedback >= .05:
            log('feedback', state=owner.get('state'), **sample(message))
            logged_feedback = measured_at

    def ownership(message):
        value = json.loads(message.data)
        if (value.get('state'), value.get('epoch')) != (owner.get('state'), owner.get('epoch')):
            print(f"控制权 {value['state']} / {value['source'] or '-'} / epoch={value['epoch']}", flush=True)
            log('ownership', **value)
        owner.update(value)

    def output(message):
        counters['output_' + owner.get('state', 'UNKNOWN')] += 1
        if stream:
            log('output', state=owner.get('state'), epoch=owner.get('epoch'), **sample(message))

    def evidence(message):
        counters['human_evidence'] += 1
        log('evidence', **json.loads(message.data))

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            stream = args.log.open('x', buffering=1)
        subscribe_robot_state(node, info, feedback)
        node.create_subscription(String, '/status/control/ownership', ownership, latched)
        node.create_subscription(JointState, '/action/joint_states', output, 100)
        if stream:
            node.create_subscription(JointState, '/internal/control/joint_targets',
                                     lambda m: log('candidate', **sample(m)), 100)
        node.create_subscription(String, '/internal/control/action_evidence', evidence, 100)
        deadline = time.monotonic() + 15
        while running and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if info and all(name in measured for side in ('left_arm', 'right_arm')
                            for name in info['groups'][side]['joint_names']):
                break
        if not running:
            return
        if not info or time.monotonic() - measured_at > .3:
            raise RuntimeError('未收到机器人能力或新鲜反馈，请先 source /opt/ros/humble/setup.bash')
        commands, baselines = arm_commands(node, info, measured, args.amplitude)
        started = time.monotonic()
        log('start', domain=domain, profile=info['profile_id'], amplitude=args.amplitude,
            period=args.period, rate=args.rate, baselines=baselines)
        print(f"外部程序已启动：{info['profile_id']} / ROS {domain} / 双臂 ServoJ {args.rate:g} Hz", flush=True)
        next_send = next_print = started
        while running and (args.duration == 0 or time.monotonic() - started < args.duration):
            now = time.monotonic()
            if now >= next_send:
                if now - measured_at > .3:
                    if owner.get('state', 'OFF') != 'OFF' or now - measured_at > 15:
                        raise RuntimeError('仿真反馈已过期，停止发送')
                    # The VR demo may restart the middleware to select joint-only recording.
                    # No control session is open during this configuration change.
                    rclpy.spin_once(node, timeout_sec=.05)
                    continue
                # Deliberately independent of ownership: HC must reject this input during MANUAL.
                offset = args.amplitude * math.sin(2*math.pi*(now-started)/args.period)
                for endpoint, publisher, message, moving in commands:
                    for index, baseline in moving:
                        message.position[index] = baseline + offset
                    message.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(message)
                    counters['sent_' + owner.get('state', 'UNKNOWN')] += 1
                    if stream:
                        log('program', endpoint=endpoint, state=owner.get('state'), **sample(message))
                next_send = now + 1/args.rate
            if now >= next_print:
                print(json.dumps(dict(seconds=round(now-started, 1), state=owner.get('state'),
                                      counts=dict(counters)), ensure_ascii=False), flush=True)
                next_print = now + 2
            rclpy.spin_once(node, timeout_sec=max(0., min(.01, next_send-time.monotonic())))
    finally:
        log('summary', seconds=time.monotonic()-started, counts=dict(counters))
        if stream:
            stream.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
