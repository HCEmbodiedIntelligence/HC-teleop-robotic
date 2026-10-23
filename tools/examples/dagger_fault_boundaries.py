#!/usr/bin/env python3
"""Grip 边界、输入故障、暂停、急停与恢复；使用真实 UDP VR 接收链。"""
import time
from dagger_sim_common import run


def recover(s):
    s.neutral()
    s.wait(lambda:s.owner.get('input_valid') and not s.owner.get('gripped'),5,'fresh release')
    s.action('recover')
    s.state('HOLD')
    time.sleep(.4)
    assert s.owner['state']=='HOLD'
    s.action('return')
    s.state('AUTO')
    time.sleep(.3)


def pause_release(s):
    s.program_start();s.session()
    s.action('hold');s.state('HOLD')
    s.manual()
    s.set_vr(grip=0.);s.state('HOLD')
    time.sleep(.15)
    start=time.monotonic();before=s.joints.copy();base=s.base.copy()
    time.sleep(.5)
    s.static(start,before,base=base)


def input_fault(change):
    def test(s):
        s.program_start();s.session();s.manual()
        start=time.monotonic()
        s.set_vr(**change)
        s.wait(lambda:s.owner.get('state') in ('FAULT','ESTOP'),3,'input fault')
        delay=(time.monotonic()-start)*1000
        state=s.owner['state'];reason=s.owner['reason']
        s.neutral()
        time.sleep(.35)
        assert s.owner['state']==state,'input recovery silently restarted AUTO'
        start=time.monotonic();before=s.joints.copy();base=s.base.copy()
        time.sleep(.35)
        s.static(start,before,base=base)
        recover(s)
        return dict(fault=state,reason=reason,observed_ms=delay,explicit_recovery_stays_hold=True)
    return test


def estop(s):
    s.program_start();s.session();s.manual()
    s.request('/api/safety/stop',{'reason':'boundary test'})
    s.state('ESTOP')
    s.set_vr(grip=0.)
    time.sleep(.35)
    assert s.owner['state']=='ESTOP'
    recover(s)


def hysteresis(s):
    s.program_start();s.session()
    for value in (.1,.49,.54):
        s.set_vr(grip=value);time.sleep(.15);assert s.owner['state']=='AUTO'
    s.set_vr(grip=.56);s.state('MANUAL')
    epoch=s.owner['epoch']
    for value in (.54,.52,.501,.54):
        s.set_vr(grip=value);time.sleep(.1)
        assert s.owner['state']=='MANUAL' and s.owner['epoch']==epoch,'Grip chatter changed epoch'
    s.set_vr(grip=.49);s.state('AUTO')


def short_cycles(s):
    s.program_start();s.session()
    epochs=[]
    for _ in range(10):
        s.set_vr(grip=1.);s.state('MANUAL');time.sleep(.10)
        epochs.append(s.owner['epoch'])
        s.set_vr(grip=0.);s.state('AUTO');time.sleep(.15)
    assert len(set(epochs))==10
    return dict(cycles=10,manual_seconds=.1)


if __name__=='__main__':
    run([('pause_then_release_stays_hold',pause_release),('grip_hysteresis',hysteresis),
        ('ten_short_interventions',short_cycles),('right_tracking_loss',input_fault({'flags':3,'grip':0.})),
        ('left_tracking_loss',input_fault({'flags':5})),('head_tracking_loss',input_fault({'flags':6})),
        ('vr_transport_timeout',input_fault({'paused':True})),
        ('vr_repeated_sequence',input_fault({'freeze_sequence':True})),
        ('vr_frozen_source_time',input_fault({'freeze_timestamp':True})),
        ('vr_zero_quaternion',input_fault({'right':[.2,1.3,-.4,0.,0.,0.,0.]})),
        ('explicit_emergency_stop',estop)], 'faults')
