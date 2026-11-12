#!/usr/bin/env python3
"""普通 VR 身体/底盘运动，以及 DAgger 中辅助轴保持的真实仿真测试。"""
import math
import time
from dagger_sim_common import run


def ordinary(s):
    s.neutral()
    s.request('/api/teleop/source',{'source':'vr'})
    s.request('/api/safety/resume',{})
    time.sleep(.5)


def translation(axis):
    def test(s):
        ordinary(s)
        before=s.base.copy()
        s.set_vr(left_axis=axis)
        time.sleep(.7)
        moved=math.dist(before[:2],s.base[:2])
        assert moved>.04,f'base did not translate: {moved}'
        s.set_vr(left_axis=[0.,0.])
        time.sleep(.25)
        start=time.monotonic(); held=s.joints.copy(); base=s.base.copy()
        time.sleep(.4)
        s.static(start,held,names=[n for n in held if 'waist' in n or 'head' in n],base=base)
        return dict(displacement_m=moved,stopped_on_release=True)
    return test


def head_body(s):
    ordinary(s)
    before=s.joints.copy()
    s.set_vr(left_grip=1.)
    time.sleep(.2)
    s.set_vr(head=[0.,1.34,0.,math.sin(.15/2),0.,0.,math.cos(.15/2)])
    time.sleep(1.6)
    delta=max(abs(s.joints[n]-before[n]) for n in before if n.startswith('waist_'))
    assert delta>.015,f'waist did not follow head: {delta}'
    assert max(abs(s.joints[n]-before[n]) for n in before if n.startswith('head_'))<1e-6, 'disabled neck tracking moved'
    s.set_vr(left_grip=0.)
    time.sleep(.3)
    start=time.monotonic(); held=s.joints.copy(); base=s.base.copy()
    s.set_vr(head=[.2,1.7,.2,0.,.4,0.,math.sqrt(.84)])
    time.sleep(.6)
    s.static(start,held,names=[n for n in held if n.startswith(('waist_','head_'))],base=base)
    return dict(waist_max_delta_rad=delta,release_holds=True,neck_tracking_disabled=True)


def head_base(s):
    ordinary(s)
    before=s.base.copy()
    s.set_vr(left_grip=1.)
    time.sleep(.2)
    s.set_vr(head=[.15,1.5,.15,0.,math.sin(.3/2),0.,math.cos(.3/2)])
    time.sleep(1.4)
    translation=math.dist(before[:2],s.base[:2])
    rotation=max(abs(a-b) for a,b in zip(before[3:],s.base[3:]))
    assert translation>.04 and rotation>.03,(translation,rotation)
    s.set_vr(left_grip=0.)
    time.sleep(.2)
    start=time.monotonic(); base=s.base.copy(); joints=s.joints.copy()
    time.sleep(.4)
    s.static(start,joints,names=[n for n in joints if n.startswith(('waist_','head_'))],base=base)
    return dict(translation_m=translation,quaternion_delta=rotation)


def dagger_holds(state):
    def test(s):
        s.program_start()
        s.session(auto=state!='HOLD')
        if state=='MANUAL':
            s.manual()
        time.sleep(.2)
        before=s.joints.copy(); base=s.base.copy(); start=time.monotonic()
        s.set_vr(left_grip=1.,left_axis=[1.,1.],right_axis=[1.,1.],
            head=[.3,1.2,.3,.2,.3,0.,math.sqrt(.87)])
        time.sleep(1.)
        s.state(state)
        passive=[n for n in before if not n.startswith(('left_joint_','right_joint_'))]
        s.static(start,before,names=passive,base=base)
        commands=s.window(start,'base_command')
        assert all(max(map(abs,r['values']),default=0)<1e-8 for r in commands),'nonzero base command leaked'
        return dict(passive_joints=len(passive),base_held=True,nonzero_base_commands=0)
    return test


if __name__=='__main__':
    run([(f'ordinary_base_{name}',translation(axis)) for name,axis in
         [('forward',[0.,.8]),('backward',[0.,-.8]),('left',[-.8,0.]),('right',[.8,0.]),('diagonal',[1.,1.])]]+
        [('ordinary_waist_and_neck',head_body),('ordinary_head_base_pose',head_base)]+
        [('dagger_aux_held_'+state,dagger_holds(state)) for state in ('AUTO','MANUAL','HOLD')], 'body_base')
