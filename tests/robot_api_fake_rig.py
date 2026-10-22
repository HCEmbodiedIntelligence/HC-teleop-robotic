"""Isolated real Motion/driver/HC boundary with a deterministic fake plant.

Only the hardware controller is simulated. No IK, RTC, public conversion,
driver conversion, permission or command-selector code is replaced.
"""
import fcntl
import itertools
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped

from middleware.core.config import validate_config
from middleware.core.robot_api import RobotApiProfile
from middleware.core.ros_bridge import RosBridge


class FakeRig:
    # Reusing a domain across consecutive fixtures leaves stale DDS discovery
    # for a lease interval. Keep every rig in this suite isolated from it.
    domains = itertools.cycle(range(200, 216))

    def __init__(self, profile, report, with_teleop=False):
        self.domain = next(self.domains)
        self.api = RobotApiProfile(profile)
        self.report = Path(report)
        self.report.mkdir(parents=True, exist_ok=True)
        self.runtime = self.api.write_runtime(self.report / 'runtime')
        self.processes, self.logs = [], []
        self.clients, self.extra_nodes = [], []
        self.with_teleop = with_teleop
        self.bridge = self.adapter = self.node = self.executor = None
        self.feedback_enabled = True
        self.received, self.controller_commands = {}, []
        self.positions = {n: self.api.teleop['robot']['initial_joints'].get(n, 0.) for n in self.api.joints}
        self.lock = open(f'/tmp/hc-unified-robot-api-domain-{self.domain}.lock', 'w')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def __enter__(self):
        try:
            os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(self.domain),
                              ROS_LOG_DIR=str(self.report / 'ros-log'))
            args = []
            if self.api.driver['kind'] == 'openarmx_hc_robot_adapter':
                args = ['--ros-args', '--params-file', str(self.runtime / 'driver.yaml'),
                        '-p', 'hardware_mode:=fake', '-p', 'resource_id:=unified-test-' + uuid.uuid4().hex]
            rclpy.init(args=args, domain_id=self.domain)
            self.node = Node('unified_api_fake_plant', use_global_arguments=False)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.pump(.5)
            # DDS may briefly retain discovery of the preceding fixture.
            self.until(lambda: self.node.get_node_names() == [self.node.get_name()], timeout=5.)
            if self.with_teleop:
                from adapters.core.arm_teleop_node import RobotArmTeleopNode
                from adapters.nodes.unified_motion_teleop import UnifiedMotionTeleopBridge
                self.teleop = RobotArmTeleopNode(str(self.api.directory/'arm_teleop.yaml'), backend='motion')
                self.vr = UnifiedMotionTeleopBridge(self.api.directory)
                for peer in (self.teleop,self.vr):
                    self.extra_nodes.append(peer); self.executor.add_node(peer)
            self._start_driver()
            self.node.create_timer(.01, self._feedback)
            for topic, cls in [('/observation/joint_states', JointState), ('/action/joint_states', JointState),
                               ('/internal/robot/joint_commands', JointState), ('/status/hardware/ready', Bool)]:
                self.node.create_subscription(cls, topic, lambda msg, t=topic: self.received.setdefault(t, []).append(msg), qos_profile_sensor_data)
            for group in ('left_arm', 'right_arm'):
                topic = self.api.describe()['groups'][group]['fk_pose_topic']
                self.node.create_subscription(PoseStamped, topic, lambda msg,t=topic: self.received.setdefault(t, []).append(msg), qos_profile_sensor_data)
            config = validate_config({'ros': {'domain_id':self.domain,
                                              'robot_api_profile':str(self.api.directory),
                                              'command_mux':{'source':'motion'}}})['ros']
            config['dataset_topics'] = dict(enabled=True, joint_groups={
                group: [self.api.joints[n]['public_name'] for n in names]
                for group,names in self.api.groups.items() if group!='whole_body'})
            for category in ('observation', 'action'):
                for group in config['dataset_topics']['joint_groups']:
                    topic=f'/{category}/{group}/joint_states'
                    self.node.create_subscription(JointState, topic,
                        lambda msg,t=topic: self.received.setdefault(t,[]).append(msg), qos_profile_sensor_data)
            self.bridge = RosBridge(config, lambda *_: None)
            self.bridge.start()
            self.until(lambda: self.bridge.status()['state'] == 'running')
            self.until(lambda: self.received.get('/status/hardware/ready') and self.received['/status/hardware/ready'][-1].data)
            self.bridge.set_control_source('motion')
            self.pump(.1)
            self.bridge.set_command_output_enabled(True)
            self.until(lambda: self.bridge.status()['command_mux']['hardware_ready'])
            self.spawn('motion', ['ros2','run','humanoid_motion_server','humanoid_motion_control_node',
                                 '--ros-args','--params-file',str(self.runtime/'motion.yaml')])
            self.until(lambda: self.received.get('/observation/motion/left_arm/ee_pose'), timeout=15.)
            self.select('motion')
            return self
        except BaseException:
            self.close()
            raise

    def spawn(self, name, command):
        log = (self.report/(name+'.log')).open('w')
        self.logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def _start_driver(self):
        params = self.api.driver['parameters']
        if self.api.driver['kind'] == 'openarmx_hc_robot_adapter':
            from openarmx_hc_robot_adapter.node import OpenArmXHcRobotAdapter
            from controller_manager_msgs.msg import ControllerState
            from controller_manager_msgs.srv import ListControllers
            self.adapter = OpenArmXHcRobotAdapter()
            self.executor.add_node(self.adapter)
            def controllers(_, response):
                for side in ('right','left'):
                    state = ControllerState(name=side+'_forward_position_controller', state='active')
                    state.claimed_interfaces = [n+'/position' for n in
                        self.adapter.gate.joint_names[side] + (self.adapter.gate.gripper_joint_names[side],)]
                    response.controller.append(state)
                return response
            self.node.create_service(ListControllers,'/controller_manager/list_controllers',controllers)
            state_topic = params['robot_feedback_topic']
            for side in ('right','left'):
                names = list(self.adapter.gate.joint_names[side]) + [self.adapter.gate.gripper_joint_names[side]]
                self._controller(params[side+'_controller_topic'], names)
            self.vendor = {n:(n,1.,0.) for n in self.positions}
        else:
            plugin = dict(entry.split('=',1) for entry in params['plugin_parameters'])
            state_topic = plugin['state_topic']
            self.vendor = dict(zip(params['joint_names'], zip(params['vendor_joint_names'],
                params['vendor_to_logical_scales'],params['vendor_to_logical_offsets_rad'])))
            for group in dict.fromkeys(params['vendor_joint_groups']):
                names = [n for n,g in zip(params['joint_names'],params['vendor_joint_groups']) if g == group]
                self._controller(plugin['command_topic.'+group],names, vendor=True)
            self.spawn('driver',['ros2','run','humanoid_driver_runtime','humanoid_driver_runtime_node',
                                 '--ros-args','--params-file',str(self.runtime/'driver.yaml')])
        self.feedback_pub = self.node.create_publisher(JointState,state_topic,10)

    def _controller(self, topic, names, vendor=False):
        def apply(msg):
            assert len(msg.data) == len(names)
            self.controller_commands.append((topic,list(msg.data)))
            for n,value in zip(names,msg.data):
                if vendor:
                    _,scale,offset = self.vendor[n]
                    value = scale*value + offset
                self.positions[n] = value
        self.node.create_subscription(Float64MultiArray,topic,apply,10)

    def _feedback(self):
        if not self.feedback_enabled:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        # Deliberately reverse hardware order; all consumers must use names.
        for model,(name,scale,offset) in reversed(list(self.vendor.items())):
            msg.name.append(name)
            msg.position.append((self.positions[model]-offset)/scale)
            msg.velocity.append(0.)
        self.feedback_pub.publish(msg)

    def pump(self, seconds, publish=None):
        deadline = time.monotonic()+seconds
        next_publish = 0.
        while time.monotonic()<deadline:
            if publish and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic()+.02
            self.executor.spin_once(timeout_sec=.005)
            for process in self.processes:
                assert process.poll() is None, 'runtime exited; inspect '+str(self.report)

    def until(self, predicate, timeout=5.):
        deadline=time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<deadline, 'timed out; ROS nodes='+str(self.node.get_node_names())+'; inspect '+str(self.report)
            self.pump(.01)

    def future(self, future, timeout=10.):
        self.until(future.done, timeout)
        return future.result()

    def select(self, source):
        self.bridge.set_control_source(source)
        self.pump(.15)
        self.bridge.set_command_output_enabled(True)
        self.bridge.publish('/control/teleop/enabled','std_msgs/msg/Bool',dict(data=source=='vr'))
        self.pump(.2)

    def run_demo(self, args, timeout=12., interrupt=False):
        log_path=self.report/f'demo-{len(self.clients)}.log'
        log=log_path.open('w')
        self.logs.append(log)
        client=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'examples/robot_api_demo.py'),*args],
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        self.clients.append(client)
        if interrupt:
            self.until(lambda:'"target"' in log_path.read_text(),timeout)
            self.pump(.2)
            os.killpg(client.pid,signal.SIGINT)
        self.until(lambda:client.poll() is not None,timeout)
        log.flush()
        assert client.returncode==(130 if interrupt else 0), log_path.read_text()
        if interrupt:
            assert 'confirmed_action_terminal_status' in log_path.read_text()
        return log_path.read_text()

    def close(self):
        for process in reversed(self.processes+self.clients):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=3.)
        for node in self.extra_nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        if self.bridge:
            self.bridge.stop()
        if self.executor:
            self.executor.shutdown()
        if self.adapter:
            self.adapter.destroy_node()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for log in self.logs:
            log.close()
        self.lock.close()

    def __exit__(self,*_):
        self.close()
