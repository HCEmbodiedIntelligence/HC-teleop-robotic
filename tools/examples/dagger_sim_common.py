"""Shared live-simulation fixture for the DAgger boundary scripts (not hardware)."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import traceback
import urllib.error
import urllib.parse
import urllib.request
import uuid

ROOT = Path(__file__).resolve().parents[2]


class Sim:
    def __init__(self, url, report):
        self.url, self.report = url.rstrip('/'), Path(report)
        self.report.mkdir(parents=True, exist_ok=False)
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self.original = None
        self.node = self.executor = self.ros_thread = self.vr_thread = self.program = None
        self.program_log = self.log = None
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.rows, self.results = [], []
        self.info, self.joints, self.owner, self.teleop = {}, {}, {}, {}
        self.base = None
        self.publishers = {}
        self.sessions = []
        self.send_gaps = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.neutral()

    def request(self, path, data=None, method=None, expected=None):
        req = urllib.request.Request(self.url+path, data=None if data is None else json.dumps(data).encode(),
            method=method, headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(req, timeout=30) as response:
                code, raw = response.status, response.read().decode()
        except urllib.error.HTTPError as exc:
            code, raw = exc.code, exc.read().decode()
        try:
            value = json.loads(raw)
        except ValueError:
            value = {'reason': raw}
        if expected is not None:
            assert code == expected, (path, code, value)
        elif code >= 400:
            raise RuntimeError(f'{path}: {code}: {value}')
        return value

    def neutral(self):
        with self.lock:
            self.vr = dict(grip=0., left_grip=0., left_axis=[0.,0.], right_axis=[0.,0.],
                head=[0.,1.5,0.,0.,0.,0.,1.], left=[-.2,1.3,-.4,0.,0.,0.,1.],
                right=[.2,1.3,-.4,0.,0.,0.,1.], flags=7, paused=False,
                freeze_sequence=False, freeze_timestamp=False)

    def set_vr(self, **values):
        with self.lock:
            self.vr.update(copy.deepcopy(values))

    def add(self, kind, **value):
        row = dict(kind=kind, mono=time.monotonic(), time_ns=time.time_ns(), **value)
        with self.lock:
            self.rows.append(row)
            if self.log:
                self.log.write(json.dumps(row, ensure_ascii=False)+'\n')

    def wait(self, predicate, timeout=10, reason='condition'):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError(f'timeout: {reason}; ownership={self.owner}')

    def state(self, name, timeout=10):
        self.wait(lambda: self.owner.get('state') == name, timeout, 'state '+name)

    def open(self):
        simulation = self.request('/api/simulation')
        assert simulation['state']=='running' and simulation['runtime_mode']=='simulation'
        assert int(simulation['ros_domain_id'])!=int(simulation['hardware_domain_id'])
        live = self.request('/api/status')
        assert live['ros']['ownership']['state']=='OFF' and not live['recording']['recording']
        assert not live['vr']['last_packet_at'] or time.time()-live['vr']['last_packet_at']>2, 'VR already connected'
        assert live['recording']['recording_selection']=={'mode':'local'}, 'Select local recording first'
        self.original = self.request('/api/config')
        (self.report/'original-config.json').write_text(json.dumps(self.original,ensure_ascii=False,indent=2))
        config = copy.deepcopy(self.original)
        for item in config['ros']['subscriptions']:
            item['outputs'] = [v for v in item.get('outputs',[]) if v!='record']
            if item['topic'] in ('/observation/joint_states','/action/joint_states','/observation/base/pose'):
                item['enabled']=True
                item['outputs'].append('record')
        config['ros']['recording']['format']='mcap'
        self.request('/api/config',config,'PUT')
        os.environ['ROS_DOMAIN_ID']=str(simulation['ros_domain_id'])
        os.environ['ROS_LOCALHOST_ONLY']='1'
        os.environ['ROS_LOG_DIR']=str(self.report/'ros-log')
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String, Float64MultiArray
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node('dagger_boundary_observer')
        self.log=(self.report/'trace.jsonl').open('x',buffering=1)
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String,'/status/robot/info',lambda m:self.info.update(json.loads(m.data)),latched)
        def ownership(m):
            value=json.loads(m.data)
            if (self.owner.get('state'),self.owner.get('epoch'))!=(value['state'],value['epoch']):
                self.add('ownership',**value)
            self.owner=value
        def joints(m):
            self.joints=dict(zip(m.name,m.position))
            self.add('feedback',stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,positions=self.joints)
        def base(m):
            p,q=m.pose.position,m.pose.orientation
            self.base=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
            self.add('base',positions=self.base)
        def joint_message(kind,m):
            self.add(kind,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                frame_id=m.header.frame_id,names=list(m.name),positions=list(m.position))
        self.node.create_subscription(String,'/status/control/ownership',ownership,latched)
        self.node.create_subscription(String,'/status/teleop',lambda m:self.teleop.update(json.loads(m.data)),10)
        self.node.create_subscription(JointState,'/observation/joint_states',joints,qos_profile_sensor_data)
        self.node.create_subscription(PoseStamped,'/observation/base/pose',base,qos_profile_sensor_data)
        for kind,topic in [('output','/action/joint_states'),('candidate','/internal/control/joint_targets')]:
            self.node.create_subscription(JointState,topic,lambda m,k=kind:joint_message(k,m),100)
        self.node.create_subscription(Float64MultiArray,'/action/base/command',lambda m:self.add('base_command',values=list(m.data)),100)
        self.node.create_subscription(String,'/internal/control/action_evidence',lambda m:self.add('evidence',**json.loads(m.data)),100)
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.ros_thread=threading.Thread(target=self.executor.spin,daemon=True)
        self.ros_thread.start()
        self.destination=(urllib.parse.urlparse(self.url).hostname,int(config['vr']['pose_port']))
        self.vr_thread=threading.Thread(target=self._send_vr,daemon=True)
        self.vr_thread.start()
        self.wait(lambda:self.info and self.joints and self.base and {'motion','teleop'}<=self.owner.get('peers',{}).keys(),15,'ROS discovery')
        self.add('setup',simulation=simulation)

    def _send_vr(self):
        sequence=0
        stamp=time.monotonic()
        previous=stamp
        while not self.stop.wait(.01):
            with self.lock:
                v=copy.deepcopy(self.vr)
            if v['paused']:
                continue
            if not v['freeze_sequence']:
                sequence=(sequence+1)&0xffffffff
            if not v['freeze_timestamp']:
                stamp=time.monotonic()
            packet=struct.pack('<4sBIdB21f3H6f3H6f',b'PICO',2,sequence,stamp,v['flags'],
                *(v['head']+v['left']+v['right']),4 if v['left_grip']>=.55 else 0,0,0,
                .5,v['left_grip'],*v['left_axis'],0.,0.,4 if v['grip']>=.55 else 0,0,0,
                .5,v['grip'],*v['right_axis'],0.,0.)
            packet=self.transform_packet(packet,v)
            self.sock.sendto(packet,self.destination)
            now=time.monotonic()
            self.packet_sent(packet,v,now)
            if now-previous>.05:
                self.send_gaps.append(dict(seconds=now-previous,sequence=sequence,paused=v['paused']))
            previous=now

    def transform_packet(self, packet, _values):
        return packet

    def packet_sent(self, _packet, _values, _now):
        pass

    def action(self, name, **fields):
        current=self.request('/api/control/ownership')
        return self.request('/api/control/intervention/'+name,dict(request_id=uuid.uuid4().hex,
            session_id=current['session_id'],expected_epoch=current['epoch'],**fields))

    def session(self, auto=True, capture_mode='corrections_only'):
        if self.request('/api/control/ownership')['state']!='OFF':
            self.action('disable')
        self.neutral()
        time.sleep(.35)
        opened=self.action('enable',capture_mode=capture_mode)
        self.sessions.append(opened['ownership']['session_id'])
        def ready():
            s=self.request('/api/control/ownership')
            return s['input_armed'] and s['input_valid'] and s['capture'] and s['capture']['ready']
        self.wait(ready,25,'recording ready')
        if auto:
            self.action('return')
            self.state('AUTO')
        else:
            self.state('HOLD')

    def manual(self):
        self.set_vr(grip=1.)
        self.state('MANUAL')
        time.sleep(.15)

    def program_start(self, amplitude=.25):
        if self.program:
            return
        self.program_log=(self.report/'program.log').open('w')
        self.program=subprocess.Popen(['/usr/bin/python3','-u',str(ROOT/'tools/examples/dagger_program.py'),
            '--url',self.url,'--amplitude',str(amplitude),'--period','4','--log',str(self.report/'program.jsonl')],
            cwd=ROOT,stdout=self.program_log,stderr=subprocess.STDOUT)
        self.wait(lambda:'外部程序已启动' in (self.report/'program.log').read_text(),15,'program startup')

    def publish(self, topic, message):
        if topic not in self.publishers:
            self.publishers[topic]=self.node.create_publisher(type(message),topic,10)
            self.wait(lambda:self.publishers[topic].get_subscription_count()>0,5,'subscriber '+topic)
        self.publishers[topic].publish(message)

    def window(self, start, kind):
        with self.lock:
            return [r for r in self.rows if r['mono']>=start and r['kind']==kind]

    def static(self, start, before, *, names=None, tolerance=1e-5, base=None):
        feedback=self.window(start,'feedback')
        assert len(feedback)>10,'missing feedback'
        delta=max(abs(r['positions'][n]-before[n]) for r in feedback for n in (names or before))
        assert delta<=tolerance, f'joint moved {delta} > {tolerance}'
        if base is not None:
            frames=self.window(start,'base')
            assert frames,'missing base feedback'
            drift=max(abs(r['positions'][i]-base[i]) for r in frames for i in range(7))
            assert drift<=tolerance,f'base moved {drift} > {tolerance}'
        return delta

    def case(self, name, function):
        self.add('case_start',name=name)
        started=time.monotonic()
        try:
            detail=function(self) or {}
            value=dict(name=name,result='PASS',seconds=time.monotonic()-started,detail=detail)
        except Exception as exc:
            value=dict(name=name,result='FAIL',seconds=time.monotonic()-started,error=str(exc),
                traceback=traceback.format_exc(),ownership=self.owner,teleop=self.teleop,
                gateway=self.request('/api/status')['vr'],send_gaps=self.send_gaps[-10:])
        self.results.append(value)
        (self.report/'results.json').write_text(json.dumps(dict(tests=self.results,sessions=self.sessions,
            send_gaps=self.send_gaps),ensure_ascii=False,indent=2)+'\n')
        print(name,value['result'],value.get('error',''),flush=True)
        if self.request('/api/control/ownership')['state']!='OFF':
            self.action('disable')
        self.neutral()
        time.sleep(.25)

    def close(self):
        try:
            if self.original:
                if self.request('/api/control/ownership')['state']!='OFF':
                    self.action('disable')
                self.neutral()
                self.request('/api/safety/stop',{'reason':'simulation boundary test completed'})
        finally:
            if self.program and self.program.poll() is None:
                self.program.send_signal(signal.SIGINT)
                self.program.wait(timeout=10)
            if self.program_log:
                self.program_log.close()
            self.stop.set()
            if self.vr_thread:
                self.vr_thread.join(timeout=2)
            self.sock.close()
            if self.executor:
                self.executor.shutdown(timeout_sec=3)
            if self.ros_thread:
                self.ros_thread.join(timeout=3)
            if self.node:
                import rclpy
                self.node.destroy_node()
                rclpy.try_shutdown()
            if self.log:
                self.log.close()
            if self.original:
                self.request('/api/config',self.original,'PUT')


def run(cases, default, *, sim_class=Sim):
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://172.168.200.30:7876')
    parser.add_argument('--report',type=Path,default=ROOT/'runtime/dagger_boundary'/default)
    parser.add_argument('--case',choices=[name for name,_ in cases],nargs='+')
    args=parser.parse_args()
    sim=sim_class(args.url,args.report)
    try:
        sim.open()
        for name,case in cases:
            if not args.case or name in args.case:
                sim.case(name,case)
    finally:
        sim.close()
    if any(r['result']!='PASS' for r in sim.results):
        raise SystemExit(1)
