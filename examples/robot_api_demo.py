#!/usr/bin/env python3
"""HC 通用在线客户端：能力发现、状态、ServoJ、ServoP 和 MoveJ。

名称、DOF、限位和 frame 来自 /status/robot/info；不自动切源或使能。
"""

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String


STATE_TOPIC = '/observation/joint_states'
COMMAND_TOPIC = '/action/joint_states'
READY_TOPIC = '/status/hardware/ready'
SOURCE_TOPIC = '/control/source'
PERMIT_TOPIC = '/control/motion/permit'
INFO_TOPIC = '/status/robot/info'


def emit(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError('必须是有限数值')
    return number


def positive(value):
    number = finite(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('必须大于 0')
    return number


def load_profile(path):
    # Optional offline describe only; online commands never load local YAML.
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from middleware.core.robot_api import RobotApiProfile
    return capability_profile(RobotApiProfile(path).describe())


def capability_profile(info):
    if info.get('schema') != 'hc-robot-api/v1':
        raise ValueError('不支持的机器人能力版本')
    profile = dict(info=info, state_topic=info['endpoints']['joint_states'],
                   permit_topic=info['endpoints']['permit'], groups={}, channels=[],
                   params=dict(info['servo']))
    for group, spec in info['groups'].items():
        names = spec['joint_names']
        if not names or len(names)!=spec['count'] or len(set(names))!=len(names):
            raise ValueError(f'{group}: 能力描述的关节数量/名称无效')
        profile['groups'][group] = names
        limits = dict(zip(names, spec['limits']))
        for side in ('lower', 'upper'):
            profile['params'][f'group_{side}_limits.{group}'] = [limits[n][side] for n in names]
        for kind, endpoint in spec['endpoints'].items():
            profile['channels'].append(dict(kind=kind, endpoint=endpoint, group=group,
                fk_pose_topic=spec['fk_pose_topic'], base_frame=spec['base_frame']))
    return profile


def joint_values(message, require_velocity=False):
    names = list(message.name)
    if not names or not all(names) or len(set(names)) != len(names):
        raise ValueError('JointState.name 为空或重复')
    if len(message.position) != len(names):
        raise ValueError('JointState.position 与 name 长度不同')
    for field in ('position', 'velocity', 'effort'):
        data = getattr(message, field)
        if len(data) not in (0, len(names)) or not all(math.isfinite(v) for v in data):
            raise ValueError(f'JointState.{field} 长度或数值无效')
    if require_velocity and len(message.velocity) != len(names):
        raise ValueError('Motion 需要与 name 等长的实测 velocity')
    return dict(zip(names, message.position))


def make_target(names, measured, args, params):
    if args.positions is not None:
        target = list(args.positions)
    else:
        if args.joint not in names:
            raise ValueError(f'--joint 必须是所选组中的关节：{names}')
        target = [measured[name] + (args.delta if name == args.joint else 0.0)
                  for name in names]
    if len(target) != len(names) or not all(math.isfinite(v) for v in target):
        raise ValueError(f'需要 {len(names)} 个有限关节目标')
    lower = params.get('group_lower_limits.' + args.group)
    upper = params.get('group_upper_limits.' + args.group)
    if lower is not None or upper is not None:
        if lower is None or upper is None or len(lower) != len(names) or len(upper) != len(names):
            raise ValueError('profile 关节限位配置不完整')
        for name, value, lo, hi in zip(names, target, lower, upper):
            if (lo is not None and value < lo) or (hi is not None and value > hi):
                raise ValueError(f'{name}={value} 超出 profile 限位 [{lo}, {hi}]')
    return target


class RobotDemo(Node):
    def __init__(self, profile=None):
        super().__init__('hc_robot_api_demo')
        self.profile = profile or {}
        self.state_topic = self.profile.get('state_topic', STATE_TOPIC)
        self.permit_topic = self.profile.get('permit_topic', PERMIT_TOPIC)
        self.latest = {}
        self.subscribe(JointState, self.state_topic)
        self.subscribe(JointState, COMMAND_TOPIC)
        self.subscribe(Bool, READY_TOPIC)
        self.subscribe(Header, self.permit_topic)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(String, SOURCE_TOPIC, latched)
        self.subscribe(String, INFO_TOPIC, latched)
        if profile:
            from geometry_msgs.msg import PoseStamped
            topics = {c['fk_pose_topic'] for c in profile['channels'] if c.get('fk_pose_topic')}
            for topic in sorted(topics):
                self.subscribe(PoseStamped, topic)

    def discover(self, timeout):
        self.wait_for(lambda: INFO_TOPIC in self.latest, timeout)
        info = json.loads(self.latest[INFO_TOPIC][0].data)
        self.profile = capability_profile(info)
        if self.profile['state_topic'] != STATE_TOPIC or self.profile['permit_topic'] != PERMIT_TOPIC:
            raise ValueError('机器人未使用标准公共端点')
        from geometry_msgs.msg import PoseStamped
        for topic in {c['fk_pose_topic'] for c in self.profile['channels'] if c.get('fk_pose_topic')}:
            self.subscribe(PoseStamped, topic)
        return info

    def subscribe(self, message_type, topic, qos=qos_profile_sensor_data):
        self.create_subscription(message_type, topic,
                                 lambda msg: self.latest.__setitem__(topic, (msg, time.monotonic())), qos)

    def recent(self, topic, max_age, stamped=False):
        sample = self.latest.get(topic)
        if sample is None or time.monotonic() - sample[1] > max_age:
            return False
        if not stamped:
            return True
        message = sample[0]
        stamp = message.stamp if isinstance(message, Header) else message.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return stamp_ns > 0 and -0.05 <= age <= max_age

    def gate_problem(self, names):
        age = self.profile['params'].get('feedback_max_age_ms', 200) / 1000.0
        if not self.recent(self.state_topic, age, stamped=True):
            return '关节反馈缺失或过期；检查时间戳及两端时钟'
        try:
            measured = joint_values(self.latest[self.state_topic][0], require_velocity=True)
        except ValueError as error:
            return str(error)
        if not set(names).issubset(measured):
            return '反馈缺少在线能力描述中的关节'
        if not self.recent(READY_TOPIC, 0.5) or not self.latest[READY_TOPIC][0].data:
            return 'hardware ready 缺失、为 false 或过期'
        source = self.latest.get(SOURCE_TOPIC)
        if source is None or source[0].data != 'motion':
            return '当前控制源不是 motion；由机器人操作方选择程序控制'
        if not self.recent(self.permit_topic, 0.3, stamped=True):
            return 'Motion 许可缺失或过期；由机器人操作方检查使能及联锁'
        if not self.latest[self.permit_topic][0].frame_id.startswith('motion:'):
            return '当前许可不允许普通 Move 动作'
        return None

    def wait_for(self, condition, timeout, guard=None):
        deadline = time.monotonic() + timeout
        while not condition():
            if not rclpy.ok() or time.monotonic() >= deadline:
                raise RuntimeError('等待 ROS 接口、反馈或结果超时')
            rclpy.spin_once(self, timeout_sec=0.01)
            if guard:
                guard()

    def snapshot(self):
        result = {}
        for topic, (message, received) in self.latest.items():
            row = {'receive_age_sec': round(time.monotonic() - received, 3)}
            if isinstance(message, JointState):
                try:
                    row['position_by_name'] = joint_values(message)
                    for field in ('velocity', 'effort'):
                        row[field + '_by_name'] = dict(zip(message.name, getattr(message, field)))
                    row['stamp'] = [message.header.stamp.sec, message.header.stamp.nanosec]
                except ValueError as error:
                    row['invalid'] = str(error)
            elif isinstance(message, (Bool, String)):
                row['data'] = message.data
            elif isinstance(message, Header):
                row['frame_id'] = message.frame_id
            else:
                p, q = message.pose.position, message.pose.orientation
                numbers = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if all(math.isfinite(v) for v in numbers):
                    row.update(frame_id=message.header.frame_id, xyz=numbers[:3], xyzw=numbers[3:])
                else:
                    row['invalid'] = '位姿含非有限数值'
            result[topic] = row
        return result or {'waiting': self.state_topic}

    def move_j(self, args):
        # Only control clients require the generated custom interface package.
        from action_msgs.msg import GoalStatus
        from humanoid_motion_interfaces.action import MoveJ
        from rclpy.action import ActionClient

        names = self.profile['groups'].get(args.group)
        channels = [c for c in self.profile['channels']
                    if c['kind'] == 'move_j' and c['group'] == args.group]
        if not names or len(channels) != 1:
            raise ValueError('所选组需要唯一的 MoveJ 通道；先运行 describe 查看配置')
        client = ActionClient(self, MoveJ, channels[0]['endpoint'])
        pending = handle = result = None
        try:
            if not client.wait_for_server(timeout_sec=args.wait):
                raise RuntimeError('MoveJ 服务不存在；检查 motion 后端、ROS Domain 和网络')
            try:
                self.wait_for(lambda: self.gate_problem(names) is None, args.wait)
            except RuntimeError as error:
                raise RuntimeError(self.gate_problem(names) or str(error)) from error
            measured = joint_values(self.latest[self.state_topic][0], require_velocity=True)
            goal = MoveJ.Goal()
            goal.group_name = args.group
            goal.target.name = names
            if args.command == 'home':
                goal.target.position = [self.profile['info']['poses']['home'][args.group][n] for n in names]
            else:
                goal.target.position = make_target(names, measured, args, self.profile['params'])
            goal.target.header.stamp = self.get_clock().now().to_msg()
            goal.options.velocity_scale = args.scale
            goal.options.acceleration_scale = args.scale
            goal.options.jerk_scale = args.scale
            goal.options.timeout_sec = args.timeout
            permit = self.latest[self.permit_topic][0].frame_id

            def guard():
                problem = self.gate_problem(names)
                if problem:
                    raise RuntimeError(problem)
                if self.latest[self.permit_topic][0].frame_id != permit:
                    raise RuntimeError('控制许可代际已改变，取消本次动作')

            guard()
            emit({'endpoint': channels[0]['endpoint'], 'target': dict(zip(names, goal.target.position))})
            pending = client.send_goal_async(goal)
            self.wait_for(pending.done, args.wait, guard)
            handle = pending.result()
            if not handle.accepted:
                raise RuntimeError('MoveJ Goal 被服务端拒绝')
            result = handle.get_result_async()
            self.wait_for(result.done, args.timeout + 5, guard)
            wrapped = result.result()
            status = wrapped.result.status
            emit({'action_status': wrapped.status, 'code': status.code, 'message': status.message,
                  'final_position_by_name': joint_values(wrapped.result.final_joint_state)
                  if wrapped.result.final_joint_state.name else {}})
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or status.code != 0:
                raise RuntimeError('运动未成功完成')
        finally:
            # Preserve ROS on Ctrl+C and cancel this client's goal, including a late acceptance.
            if pending is not None and (result is None or not result.done()):
                try:
                    self.wait_for(pending.done, args.wait)
                    handle = handle or pending.result()
                    if handle.accepted:
                        canceled = handle.cancel_goal_async()
                        self.wait_for(canceled.done, 3.0)
                        result = result or handle.get_result_async()
                        self.wait_for(result.done, 3.0)
                        terminal = result.result().status
                        if terminal not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                            GoalStatus.STATUS_ABORTED):
                            raise RuntimeError(f'Action 尚未终止：{terminal}')
                        emit({'confirmed_action_terminal_status': terminal})
                except Exception as error:
                    print(f'未确认 Action 已停止，请检查机器人端：{error}', file=sys.stderr)
            client.destroy()

    def servo(self, args):
        from geometry_msgs.msg import PoseStamped
        names = self.profile['groups'][args.group]
        spec = self.profile['info']['groups'][args.group]
        kind = args.command.replace('-', '_')
        if kind not in spec['endpoints']:
            raise ValueError('所选组不支持该 Servo 接口')
        self.wait_for(lambda: self.gate_problem(names) is None, args.wait)
        epoch = self.latest[self.permit_topic][0].frame_id
        hz = args.hz or min(50., self.profile['params']['control_frequency_hz'])
        lease = self.profile['params']['servo_lease_ms']/1000.
        if hz*lease < 2 or hz > self.profile['params']['control_frequency_hz']:
            raise ValueError('发布频率必须高于 lease 所需频率且不超过服务端控制频率')
        if kind == 'servo_j':
            measured = joint_values(self.latest[self.state_topic][0])
            target = JointState(name=names, position=make_target(names, measured, args, self.profile['params']))
            cls = JointState
        else:
            topic = spec['fk_pose_topic']
            self.wait_for(lambda: self.recent(topic, .2, stamped=True), args.wait)
            target = copy.deepcopy(self.latest[topic][0])
            if target.header.frame_id != spec['base_frame']:
                raise ValueError('实测 FK frame 与能力描述不符')
            p, q = target.pose.position, target.pose.orientation
            if args.pose is not None:
                p.x,p.y,p.z,q.x,q.y,q.z,q.w = args.pose
            else:
                delta = args.delta_xyz if args.delta_xyz is not None else [args.dx,args.dy,args.dz]
                p.x += delta[0]; p.y += delta[1]; p.z += delta[2]
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if norm < 1e-9:
                raise ValueError('姿态四元数不能为零')
            q.x,q.y,q.z,q.w = [v/norm for v in (q.x,q.y,q.z,q.w)]
            cls = PoseStamped
        publisher = self.create_publisher(cls, spec['endpoints'][kind], 1)
        deadline, next_send, sent = time.monotonic()+args.duration, 0., 0
        try:
            while time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=min(.01, 1./hz))
                problem = self.gate_problem(names)
                if problem or self.latest[self.permit_topic][0].frame_id != epoch:
                    raise RuntimeError(problem or '控制许可代际已改变，停止 Servo')
                if time.monotonic() >= next_send:
                    target.header.stamp = self.get_clock().now().to_msg()
                    publisher.publish(target)
                    sent += 1
                    next_send = time.monotonic()+1./hz
        finally:
            self.destroy_publisher(publisher)
            # Stop means no more targets; server lease expires without resending.
            emit(dict(servo_stopped=True, sent=sent, server_lease_sec=lease))


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest='command', required=True)
    sub = commands.add_parser('subscribe', help='只订阅；默认每秒输出一次最新消息')
    sub.add_argument('--wait', type=positive, default=5.0)
    sub.add_argument('--duration', type=positive, help='运行秒数；省略则持续运行')
    sub.add_argument('--period', type=positive, default=1.0, help='打印周期，秒')
    describe = commands.add_parser('describe', help='订阅在线能力描述；--profile 可离线检查配置')
    describe.add_argument('--profile', type=Path)
    describe.add_argument('--wait', type=positive, default=5.0)
    for kind in ('move-j', 'home', 'servo-j', 'servo-p'):
        command = commands.add_parser(kind)
        command.add_argument('--group', default='left_arm')
        command.add_argument('--wait', type=positive, default=5.0)
        if kind in ('move-j', 'servo-j'):
            target = command.add_mutually_exclusive_group(required=True)
            target.add_argument('--joint', help='公共关节名称；与 --delta 一起使用')
            target.add_argument('--positions', nargs='+', type=finite)
            command.add_argument('--delta', type=finite, help='公共 SI 单位的相对目标')
        if kind in ('move-j', 'home'):
            command.add_argument('--scale', type=positive, default=0.1)
            command.add_argument('--timeout', type=positive, default=30.0)
        else:
            command.add_argument('--duration', type=positive, default=3.)
            command.add_argument('--hz', type=positive)
        if kind == 'servo-p':
            target = command.add_mutually_exclusive_group(required=True)
            target.add_argument('--delta-xyz', nargs=3, type=finite, metavar=('DX','DY','DZ'), help='在在线 base_frame 中相对当前 FK 平移，单位 m')
            target.add_argument('--dx', type=finite, help='沿在线 base_frame 的 X 方向平移，单位 m')
            target.add_argument('--pose', nargs=7, type=finite, metavar=('X','Y','Z','QX','QY','QZ','QW'))
            command.add_argument('--dy', type=finite, default=0.)
            command.add_argument('--dz', type=finite, default=0.)
    return root


def main(argv=None):
    cli = parser()
    args = cli.parse_args((sys.argv[1:] if argv is None else argv) or ['subscribe'])
    if args.command in ('move-j', 'home') and args.scale > 1:
        cli.error('--scale 必须在 (0, 1] 内')
    if args.command in ('move-j', 'servo-j'):
        if (args.joint is not None) != (args.delta is not None):
            cli.error('--joint 和 --delta 必须一起使用；--positions 不能使用 --delta')
    if args.command == 'servo-p' and args.dx is None and (args.dy or args.dz):
        cli.error('--dy/--dz 需要与 --dx 一起使用，或改用 --delta-xyz')
    node = None
    initialized = False
    try:
        if args.command == 'describe' and args.profile:
            emit(load_profile(args.profile)['info'])
            return 0
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = RobotDemo()
        info = node.discover(args.wait)
        if args.command == 'describe':
            emit(info)
        elif args.command in ('move-j', 'home'):
            node.move_j(args)
        elif args.command in ('servo-j', 'servo-p'):
            node.servo(args)
        else:
            end = time.monotonic() + args.duration if args.duration else math.inf
            next_print = 0.0
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() >= next_print:
                    emit(node.snapshot())
                    next_print = time.monotonic() + args.period
        return 0
    except KeyboardInterrupt:
        return 130
    except (RuntimeError, ValueError, KeyError, OSError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
