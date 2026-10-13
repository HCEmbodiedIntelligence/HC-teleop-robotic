"""Manual isolated ROS regression: fake 13 Hz feedback, dual 30 mm ServoP targets.
Run after building motion, with ROS Humble sourced. Uses domain 201 only.
"""
import os,time,signal,subprocess,json
from pathlib import Path
import rclpy,yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Bool
from geometry_msgs.msg import PoseArray
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
root=Path(__file__).resolve().parents[1];log=Path('/tmp/x1_motion_follow_probe.log')
env=os.environ.copy();env['ROS_DOMAIN_ID']='201';os.environ['ROS_DOMAIN_ID']='201'
rclpy.init();node=rclpy.create_node('motion_servo_isolated_test')
# Discover the domain BEFORE starting publishers or launching fake hardware.
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
assert node.get_node_names()==['motion_servo_isolated_test'], 'ROS domain 201 is already in use'
f=log.open('w');p=subprocess.Popen([str(root/'tools/run_motion.sh'),'--profile',str(root/'adapters/robots/x1')],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
bridge_log=open('/tmp/motion_full_bridge.log','w');bridge=subprocess.Popen(['/usr/bin/python3',str(root/'adapters/nodes/motion_teleop_bridge.py')],env=env,stdout=bridge_log,stderr=subprocess.STDOUT,start_new_session=True)
cfg=yaml.safe_load((root/'adapters/robots/x1/arm_teleop.yaml').read_text());initial=cfg['robot']['initial_joints'];values=dict(initial);fk={};cmds=[]
pub=node.create_publisher(JointState,'/observation/joint_states',10);posepub=node.create_publisher(PoseArray,'/action/ee_poses',10);statuspub=node.create_publisher(String,'/status/teleop',10);readypub=node.create_publisher(Bool,'/status/hardware/ready',10);native=[];native_values={};differences=[]
def command(m):
 cmds.append(m);values.update(dict(zip(m.name,m.position)));differences.append(max((abs(values[k]-v) for k,v in native_values.items()),default=0))
subs=[node.create_subscription(PoseStamped,'/observation/motion/left_arm/ee_pose',lambda m:fk.update(left=m),qos_profile_sensor_data),node.create_subscription(JointState,'/control/solver/joint_targets',command,10),node.create_subscription(JointState,'/control/motion/joint_targets',lambda m:(native.append(m),native_values.update(dict(zip(m.name,m.position)))),10),node.create_subscription(PoseStamped,'/observation/motion/right_arm/ee_pose',lambda m:fk.update(right=m),qos_profile_sensor_data)]
def feedback():
 m=JointState();m.header.stamp=node.get_clock().now().to_msg();m.name=list(values);m.position=list(values.values());m.velocity=[0.]*len(values);pub.publish(m)
node.create_timer(1/13,feedback)
node.create_timer(.1,lambda:readypub.publish(Bool(data=True)))
node.create_timer(.2,lambda:statuspub.publish(String(data=json.dumps({'enabled':True,'mode':'arms_grippers','arms':{'left':{'active':True},'right':{'active':True}}}))))
try:
 end=time.monotonic()+45
 while time.monotonic()<end and len(fk)<2:rclpy.spin_once(node,timeout_sec=.01)
 assert len(fk)==2,'No X1 measured FK'
 assert node.count_publishers('/observation/joint_states')==1, 'Test domain is already in use'
 target=fk['left'];target.pose.position.x+=.03;right=fk['right'];right.pose.position.x+=.03
 end=time.monotonic()+5;last=0.
 while time.monotonic()<end:
  if time.monotonic()-last>.025:m=PoseArray();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='zhi_Link';m.poses=[right.pose,target.pose];posepub.publish(m);last=time.monotonic()
  rclpy.spin_once(node,timeout_sec=.005)
 print('Motion native commands:',len(native),'Bridge commands:',len(cmds))
 print('native last',native[-1] if native else None)
 print('Max SDK to bridge difference:',max(differences,default=-1))
 print('Controlled joints:',cmds[-1].name if cmds else [])
 print('Max joint movement:',max(abs(values[k]-initial[k]) for k in initial))
 print(log.read_text()[-1400:])
 assert len(cmds)>10
 assert native
 assert max(abs(values[k]-initial[k]) for k in initial)>.02, 'Robot barely moved'
 for side,goal in [('left',target),('right',right)]:
  actual=fk[side].pose.position;desired=goal.pose.position
  error=((actual.x-desired.x)**2+(actual.y-desired.y)**2+(actual.z-desired.z)**2)**.5
  print(side,'TCP error (mm):',error*1000)
  assert error<.001,(side,error)
 assert 'motion session aborted' not in log.read_text(), 'Trajectory unexpectedly aborted'
 print('PASS: both arms reached 30 mm targets through motion and the bridge.')
finally:
 node.destroy_node();rclpy.shutdown()
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=10)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=8)
 if bridge.poll() is None:os.killpg(bridge.pid,signal.SIGINT)
 bridge.wait(timeout=5);bridge_log.close()
 f.close()
