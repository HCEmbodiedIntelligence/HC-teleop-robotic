"""VR input authorization and auxiliary routing around the shared Motion core.

No joint interpolation or IK lives here. Program/VR poses use separate
authorized channels of the same Motion node and the same final joint RTC.
"""
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from humanoid_motion_interfaces.action import MoveJ

from middleware.core.robot_api import RobotApiProfile, INTERNAL_STATE
from adapters.nodes.motion_teleop_bridge import MotionCache, valid_stamp


class UnifiedMotionTeleopBridge(Node):
    def __init__(self, profile):
        super().__init__('motion_teleop_bridge')
        self.api = RobotApiProfile(profile)
        # PoseArray order is the teleop adapter's established right,left order.
        self.sides = tuple(side for side in ('right','left') if side in self.api.teleop['arms'])
        self.cache = MotionCache({s:tuple(self.api.groups[s+'_arm']) for s in self.sides})
        bases = {self.api.teleop['arms'][s]['base_link'] for s in self.sides}
        if len(bases) != 1:
            raise ValueError('VR PoseArray requires one common arm base frame')
        self.base_frame = bases.pop()
        self.pose_pubs = {s:self.create_publisher(PoseStamped,f'/internal/vr/{s}_arm/servo_p',1) for s in self.sides}
        self.move_clients = {s:ActionClient(self,MoveJ,f'/internal/vr/{s}_arm/move_j') for s in self.sides}
        self.vr_pub = self.create_publisher(JointState,'/control/vr/joint_states',1)
        self.owned_pub = self.create_publisher(JointState, '/internal/control/joint_targets', 1)
        self.state_pub = self.create_publisher(JointState,'/status/solver/joint_states',1)
        self.ack_pub = self.create_publisher(Header,'/status/solver/reset_ack',1)
        self.home_pub = self.create_publisher(String,'/internal/vr/home_result',1)
        self.permit = ''
        self.permit_time = float('-inf')
        self.cutoff = 0.
        self.home_batch = None
        self.intervention = False
        self.ownership_key = ''
        self.ownership_phase = ''
        self.permit_stamp = 0.
        self.create_subscription(JointState,INTERNAL_STATE,self.feedback,qos_profile_sensor_data)
        self.create_subscription(PoseArray,self.api.teleop['control']['target_pose_topic'],self.pose,1)
        self.create_subscription(PoseArray, '/internal/vr/owned_poses', self.owned_pose, 1)
        self.create_subscription(JointState,'/internal/motion/vr_joint_targets',self.result,1)
        self.create_subscription(JointState,'/internal/vr/aux_joint_targets',self.aux,1)
        self.create_subscription(Header,'/internal/vr/home',self.home,1)
        self.create_subscription(Bool,'/internal/vr/home_cancel',lambda m:self.cancel_home() if m.data else None,1)
        self.create_subscription(Header,'/control/motion/permit',self.permit_received,1)
        self.create_subscription(String,'/status/teleop',self.status,1)
        self.create_subscription(Bool,'/status/hardware/ready',self.ready,1)
        self.create_subscription(Bool,'/control/emergency_stop',self.stop,1)
        self.create_subscription(Bool,'/control/teleop/enabled',self.enable,1)
        self.create_subscription(Bool,'/control/solver/reset',self.reset,1)

    def times(self):
        return time.monotonic(), self.get_clock().now().nanoseconds*1e-9

    @staticmethod
    def stamp(header):
        return header.stamp.sec+header.stamp.nanosec*1e-9

    def authorized(self):
        if self.intervention:
            from middleware.core.control_ownership import ControlToken
            return (self.ownership_phase == 'grant' and
                    ControlToken.from_key(self.ownership_key).source == 'vr' and
                    time.monotonic()-self.permit_time <= .3)
        return self.permit.startswith('vr:') and time.monotonic()-self.permit_time<=.3 and not self.cache.estopped

    def permit_received(self,msg):
        now,wall=self.times()
        if not valid_stamp(self.stamp(msg),wall,.3) or self.stamp(msg) <= self.permit_stamp:
            return
        self.permit_stamp = self.stamp(msg)
        if msg.frame_id.startswith('hc1:'):
            try:
                from middleware.core.control_ownership import ControlToken
                key, phase, _, _ = msg.frame_id.split('#')
                ControlToken.from_key(key)
                if phase not in {'revoke', 'grant', 'off'}:
                    return
                # This stateless router may miss the brief revoke on a
                # keep-last topic. A fresh grant replaces its routing key;
                # teleop and Motion acknowledge the actual anchor/IK fences.
                # owned_pose still rejects every pose carrying an older key.
                self.intervention = phase != 'off'
                self.ownership_key, self.ownership_phase = key, phase
            except ValueError:
                return
        elif self.intervention:
            return
        if msg.frame_id != self.permit:
            self.cancel_home()
            self.cache.clear(wall)
            self.cutoff=self.stamp(msg)
        self.permit,self.permit_time=msg.frame_id,now

    def feedback(self,msg):
        now,wall=self.times()
        try:
            ack=self.cache.feedback_received(msg.name,msg.position,self.stamp(msg.header),now,wall)
        except ValueError:
            return
        if ack:
            self.ack_pub.publish(Header(stamp=self.get_clock().now().to_msg()))
        # Only measured, fresh model state is used to re-anchor the VR adapter.
        self.state_pub.publish(msg)

    def status(self,msg):
        now,wall=self.times()
        try:
            self.cache.update_status(json.loads(msg.data),now,wall)
        except (ValueError,TypeError,AttributeError):
            self.cache.status={}
            self.cache.clear(wall)

    def ready(self,msg):
        now,wall=self.times()
        self.cache.hardware,self.cache.hardware_time=bool(msg.data),now
        if not msg.data:
            self.cache.clear(wall)
            self.cancel_home()

    def stop(self,msg):
        if msg.data:
            self.cache.estopped=True
            self.cache.clear(self.times()[1])
            self.cancel_home()

    def enable(self,msg):
        self.cache.estopped=not bool(msg.data)
        self.cache.clear(self.times()[1])
        if not msg.data:
            self.cancel_home()

    def reset(self,msg):
        if msg.data:
            self.cache.reset(self.times()[1])

    def pose(self,msg):
        if self.intervention:
            return
        now,wall=self.times()
        if (not self.authorized() or self.home_batch is not None or
                msg.header.frame_id != self.base_frame or len(msg.poses)!=len(self.sides) or
                self.stamp(msg.header)<=self.cutoff or not valid_stamp(self.stamp(msg.header),wall,.2)):
            return
        for side,pose in zip(self.sides,msg.poses):
            q=pose.orientation
            numbers=[pose.position.x,pose.position.y,pose.position.z,q.x,q.y,q.z,q.w]
            if not all(math.isfinite(v) for v in numbers) or sum(v*v for v in numbers[3:])<1e-12:
                continue
            if self.cache.grant(side,now,wall):
                self.pose_pubs[side].publish(PoseStamped(header=msg.header,pose=pose))

    def owned_pose(self, msg):
        _, wall = self.times()
        if not self.intervention or not self.authorized():
            return
        if (msg.header.frame_id != self.ownership_key+'#'+self.base_frame or
                len(msg.poses) != len(self.sides) or self.stamp(msg.header) <= self.cutoff or
                not valid_stamp(self.stamp(msg.header), wall, .2)):
            return
        for side, pose in zip(self.sides, msg.poses):
            self.pose_pubs[side].publish(PoseStamped(header=msg.header, pose=pose))

    def result(self,msg):
        if self.intervention:
            return
        now,wall=self.times()
        active = self.home_batch is not None or any(
            self.cache.allowed(side, now) and now-self.cache.lease.get(side,(-math.inf,False))[0] <= .2
            for side in self.sides)
        if active and self.authorized() and self.stamp(msg.header)>self.cutoff and valid_stamp(self.stamp(msg.header),wall,.1):
            # Already public and fully processed by the shared final RTC.
            # No post-RTC arm filtering, interpolation or held-target replay.
            self.vr_pub.publish(msg)

    def aux(self,msg):
        _,wall=self.times()
        if not self.authorized() or not valid_stamp(self.stamp(msg.header),wall,.1) or self.stamp(msg.header)<=self.cutoff:
            return
        arm_names={n for side in self.sides for n in self.api.groups[side+'_arm']}
        if arm_names.intersection(msg.name):
            self.get_logger().error('refused arm command on auxiliary input')
            return
        try:
            if self.intervention:
                if msg.header.frame_id != self.ownership_key+'#':
                    return
                self.owned_pub.publish(self.api.convert_message(msg))
            else:
                self.vr_pub.publish(self.api.convert_message(msg))
        except ValueError as exc:
            self.get_logger().warning(str(exc),throttle_duration_sec=5.)

    def home(self,msg):
        if self.intervention:
            return
        now,wall=self.times()
        request_ns=msg.stamp.sec*1_000_000_000+msg.stamp.nanosec
        if (self.home_batch is not None or not self.authorized() or self.stamp(msg)<=self.cutoff or
                not valid_stamp(self.stamp(msg),wall,.2) or now-self.cache.feedback_time>.2 or
                not self.cache.hardware or now-self.cache.hardware_time>.5 or
                self.cache.status.get('enabled') is not True or now-self.cache.status_time>.6 or
                not all(client.server_is_ready() for client in self.move_clients.values())):
            self.home_pub.publish(String(data=json.dumps(dict(request_ns=request_ns,success=False,reason='home not authorized/ready'))))
            return
        batch=dict(request_ns=request_ns,epoch=self.permit,pending=set(self.sides),handles={},success=True,canceled=False)
        self.home_batch=batch
        self.cache.clear(wall)
        for side in self.sides:
            group=side+'_arm'
            target=self.api.poses['home'][group]
            goal=MoveJ.Goal(group_name=group)
            goal.target=JointState(name=list(target),position=list(target.values()))
            goal.target.header.stamp=self.get_clock().now().to_msg()
            future=self.move_clients[side].send_goal_async(goal)
            future.add_done_callback(lambda f,s=side,b=batch:self.home_accepted(b,s,f))

    def home_accepted(self,batch,side,future):
        handle=future.result()
        if not handle.accepted:
            self.home_finished(batch,side,False)
            return
        batch['handles'][side]=handle
        if batch is not self.home_batch or batch['canceled'] or batch['epoch']!=self.permit:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f:self.home_finished(
            batch,side,f.result().status==4 and f.result().result.status.code==0))

    def home_finished(self,batch,side,success):
        batch['pending'].discard(side)
        batch['success'] &= success
        if batch['pending']:
            return
        if batch is self.home_batch:
            self.home_batch=None
            self.cutoff=self.times()[1]
            self.cache.clear(self.cutoff)
            self.home_pub.publish(String(data=json.dumps(dict(request_ns=batch['request_ns'],
                success=batch['success'] and not batch['canceled'],reason='MoveJ terminal'))))

    def cancel_home(self):
        if self.home_batch:
            self.home_batch['canceled']=True
            for handle in self.home_batch['handles'].values():
                handle.cancel_goal_async()

    def destroy_node(self):
        # Humble Node.destroy_node does not destroy ActionClient waitables;
        # their native handles otherwise keep this node alive after restart.
        for client in self.move_clients.values():
            client.destroy()
        self.move_clients.clear()
        return super().destroy_node()


def main(profile):
    rclpy.init()
    node=UnifiedMotionTeleopBridge(profile)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_home()
        deadline=time.monotonic()+2.
        while node.home_batch and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
