#!/usr/bin/env python3
"""高频、大幅 VR 位姿与 0.3 rad 程序动作竞争，并产生可检查的纠正录制。"""
import math
import time
from dagger_sim_common import run


def stress(s):
    s.program_start(amplitude=.3)
    s.session()
    initial=s.joints.copy();base=s.base.copy();started=time.monotonic()
    passive=[n for n in initial if not n.startswith(('left_joint_','right_joint_'))]
    epochs=[]
    for cycle in range(12):
        time.sleep(.4)
        s.manual();epoch=s.owner['epoch'];epochs.append(epoch)
        origin=time.monotonic()
        for index in range(80):
            angle=2*math.pi*index/20
            # A 24 cm peak-to-peak hand translation and 70 degree orientation range.
            delta=.12*math.sin(angle)
            rotation=.6*math.sin(angle+.5)
            s.set_vr(left=[-.2+.04*math.cos(angle),1.3,-.4+delta,math.sin(rotation/2),0.,0.,math.cos(rotation/2)],
                     right=[.2-.04*math.cos(angle),1.3,-.4-delta,0.,math.sin(rotation/2),0.,math.cos(rotation/2)],
                     left_grip=1.,left_axis=[1.,1.],right_axis=[1.,1.])
            time.sleep(.01)
            assert s.owner['state']=='MANUAL',s.owner
        assert s.owner['command_seq']>20,'no human output under stress'
        s.set_vr(grip=0.,left_grip=0.,left_axis=[0.,0.],right_axis=[0.,0.])
        s.state('AUTO')
        print('stress cycle',cycle+1,'manual epoch',epoch,'seconds',round(time.monotonic()-origin,3),flush=True)
    time.sleep(.5)
    s.static(started,initial,names=passive,base=base)
    frames=s.window(started,'output')
    for f in frames:
        assert all(math.isfinite(q) for q in f['positions'])
        for n,q in zip(f['names'],f['positions']):
            spec=next(j for j in s.info['joints'] if j['public_name']==n)
            assert spec['limits']['lower']-1e-6<=q<=spec['limits']['upper']+1e-6,(n,q)
    return dict(cycles=12,hand_peak_to_peak_m=.24,program_amplitude_rad=.3,
        final_outputs=len(frames),passive_axes_held=True,finite_and_within_limits=True,
        metrics=s.request('/api/control/ownership')['metrics'])


if __name__=='__main__':
    run([('large_fast_vr_and_continuing_program',stress)], 'aggressive')
