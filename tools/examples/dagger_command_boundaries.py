#!/usr/bin/env python3
"""ServoJ 非法输入、旁路请求、旧控制身份的仿真边界测试。"""
import math
import time
import uuid
from dagger_sim_common import run


def invalid(kind):
    def test(s):
        from sensor_msgs.msg import JointState
        s.session()
        spec=s.info['groups']['left_arm']
        before=s.joints.copy();base=s.base.copy();start=time.monotonic()
        for _ in range(45):
            names=spec['joint_names'].copy();positions=[before[n] for n in names]
            positions[3]+=.25
            message=JointState(name=names,position=positions)
            message.header.stamp=s.node.get_clock().now().to_msg()
            if kind=='stale': message.header.stamp.sec-=2
            if kind=='future': message.header.stamp.sec+=2
            if kind=='zero_stamp': message.header.stamp.sec=message.header.stamp.nanosec=0
            if kind=='duplicate_name': message.name[-1]=message.name[0]
            if kind=='missing_name': message.name=[]
            if kind=='wrong_length': message.position=positions[:-1]
            if kind=='wrong_group': message.name=s.info['groups']['right_arm']['joint_names']
            if kind=='model_names': message.name=['Joint'+str(i)+'_L' for i in range(1,8)]
            if kind=='passive_axis': message.name.append('waist_joint_1');message.position.append(before['waist_joint_1']+.2)
            if kind=='nan': message.position[3]=math.nan
            if kind=='infinity': message.position[3]=math.inf
            if kind=='over_limit': message.position[3]=spec['limits'][3]['upper']+.5
            s.publish(spec['endpoints']['servo_j'],message)
            time.sleep(.01)
        time.sleep(.1)
        s.static(start,before,base=base)
        return dict(invalid_messages=45,final_outputs=len(s.window(start,'output')))
    return test


def api_bypass(s):
    s.session()
    s.request('/api/teleop/source',{'source':'exoskeleton'},expected=409)
    s.request('/api/teleop/home',{},expected=409)
    for topic,typ,data in [('/action/base/command','std_msgs/msg/Float64MultiArray',{'data':[1.,1.,1.]}),
        ('/control/hardware/enabled','std_msgs/msg/Bool',{'data':True})]:
        answer=s.request('/api/ros/publish',dict(topic=topic,type=typ,data=data),expected=429)
        assert answer['accepted'] is False
    current=s.request('/api/control/ownership')
    s.request('/api/control/intervention/hold',dict(request_id=uuid.uuid4().hex,
        session_id=current['session_id'],expected_epoch=current['epoch']-1),expected=409)
    assert s.owner['state']=='AUTO'


def stale_generation(s):
    from sensor_msgs.msg import JointState
    s.session();s.manual()
    candidate=next(r for r in reversed(s.rows) if r['kind']=='candidate')
    old=candidate['frame_id']
    s.set_vr(grip=0.);s.state('AUTO');time.sleep(.2)
    before=s.joints.copy();start=time.monotonic()
    for _ in range(40):
        message=JointState(name=candidate['names'],position=[v+.2 for v in candidate['positions']])
        message.header.stamp=s.node.get_clock().now().to_msg();message.header.frame_id=old
        s.publish('/internal/control/joint_targets',message);time.sleep(.01)
    s.static(start,before)
    assert not s.window(start,'output'),'old authority leaked to final output'


if __name__=='__main__':
    run([(name,invalid(name)) for name in ('stale','future','zero_stamp','duplicate_name','missing_name',
        'wrong_length','wrong_group','model_names','passive_axis','nan','infinity','over_limit')]+
        [('api_bypass_and_stale_request',api_bypass),('old_control_generation',stale_generation)], 'commands')
