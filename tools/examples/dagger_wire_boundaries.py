#!/usr/bin/env python3
"""Malformed UDP, nonfinite poses, invalid grip values and unreachable VR poses."""
import math
import struct
import time

from dagger_fault_boundaries import input_fault
from dagger_sim_common import Sim, run


class WireSim(Sim):
    def transform_packet(self, packet, values):
        mode = values.get('wire')
        if mode == 'magic': return b'FAIL' + packet[4:]
        if mode == 'version': return packet[:4] + b'\x7f' + packet[5:]
        if mode == 'short': return packet[:-1]
        if mode == 'long': return packet + b'\0'
        if mode == 'legacy':
            return packet[:4] + b'\x01' + packet[5:struct.calcsize('<4sBIdB21f')]
        return packet


def invalid_packet(mode):
    def test(s):
        before=s.request('/api/status')['vr']['invalid']
        result=input_fault({'wire':mode})(s)
        after=s.request('/api/status')['vr']['invalid']
        assert after>before, 'invalid wire packet was not rejected'
        return dict(result, invalid_packets=after-before)
    return test


def extremes(s):
    s.program_start(); s.session(); s.manual()
    started=time.monotonic(); initial=s.joints.copy(); base=s.base.copy()
    passive=[n for n in initial if not n.startswith(('left_joint_','right_joint_'))]
    s.set_vr(left=[-100.,100.,-100.,0.,0.,0.,1.],right=[100.,-100.,100.,0.,0.,0.,1.],
             left_grip=1.,left_axis=[100.,-100.],right_axis=[-100.,100.])
    time.sleep(.8)
    assert s.owner['state'] in ('MANUAL','FAULT','ESTOP'), s.owner
    s.static(started,initial,names=passive,base=base)
    frames=s.window(started,'output')
    for f in frames:
        for n,q in zip(f['names'],f['positions']):
            limits=next(j for j in s.info['joints'] if j['public_name']==n)['limits']
            assert math.isfinite(q) and limits['lower']-1e-6<=q<=limits['upper']+1e-6,(n,q)
    return dict(extreme_translation_m=100,joystick_magnitude=100,finite_bounded_outputs=len(frames),
                passive_axes_held=True,state=s.owner['state'])


if __name__=='__main__':
    run([(f'wire_{mode}',invalid_packet(mode)) for mode in ('magic','version','short','long')]+[
        ('legacy_vr_cannot_control_dagger',input_fault({'wire':'legacy'})),
        ('nan_position',input_fault({'right':[float('nan'),1.3,-.4,0.,0.,0.,1.]})),
        ('infinite_quaternion',input_fault({'left':[-.2,1.3,-.4,0.,0.,float('inf'),1.]})),
        ('negative_grip',input_fault({'grip':-.01})),('grip_over_one',input_fault({'grip':1.01})),
        ('unreachable_pose_and_extreme_joysticks',extremes)],'wire',sim_class=WireSim)
