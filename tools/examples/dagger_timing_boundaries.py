#!/usr/bin/env python3
"""Measure VR release send, authority return, first program output and feedback."""
import math
import statistics
import struct
import time

from dagger_sim_common import Sim, run


def distribution(values):
    values=sorted(values)
    return dict(count=len(values),median=statistics.median(values),
                p95=values[math.ceil(len(values)*.95)-1],max=max(values)) if values else None


class TimingSim(Sim):
    wire_pressed=False

    def packet_sent(self, packet, values, now):
        pressed=values['grip']>=.55
        if pressed!=self.wire_pressed:
            self.add('wire_grip',pressed=pressed,send_mono=now,sequence=struct.unpack_from('<I',packet,5)[0])
            self.wire_pressed=pressed


def timing(s):
    s.program_start(amplitude=.3); s.session()
    cycles=[]
    for index in range(30):
        time.sleep(.15); s.manual()
        for step in range(30):
            d=.12*math.sin(2*math.pi*step/29)
            s.set_vr(right=[.2,1.3,-.4+d,0.,0.,0.,1.])
            time.sleep(.01)
        time.sleep(.2)
        before=s.joints.copy(); started=time.monotonic()
        s.set_vr(grip=0.)
        s.state('AUTO')
        epoch=s.owner['epoch']
        metrics=[]
        def first_output():
            current=s.request('/api/control/ownership')
            metrics[:]=[m for m in current['metrics'] if m['epoch']==epoch and 'open_to_first_final_command_ns' in m]
            return bool(metrics)
        s.wait(first_output,5,'first final program output')
        time.sleep(.18)
        released=next(r for r in s.window(started,'wire_grip') if not r['pressed'])
        metric=metrics[0]
        prefix=f"hc1:{s.owner['controller_instance_id']}:{s.owner['session_id']}:{epoch}:motion:"
        candidates={r['stamp_ns']:r for r in s.window(started,'candidate') if r['frame_id'].startswith(prefix)}
        outputs=[r for r in s.window(started,'output') if r['stamp_ns'] in candidates]
        assert outputs, 'no actual program output with the returned authority'
        first=outputs[0]; candidate=candidates[first['stamp_ns']]
        assert first['names']==candidate['names'] and first['positions']==candidate['positions']
        first_final=metric['stamp_ns']+metric['open_to_first_final_command_ns']
        feedback=[r for r in s.window(started,'feedback') if r['stamp_ns']>=first_final]
        changed=[r for r in feedback if any(abs(r['positions'][n]-before[n])>1e-5 for n in before)]
        equal=[r for r in feedback if all(abs(r['positions'][n]-q)<1e-5 for n,q in zip(first['names'],first['positions']))]
        result=dict(cycle=index+1,epoch=epoch,
            semantic_release_to_permission_ms=metric['release_to_open_ns']/1e6,
            permission_to_final_program_ms=metric['open_to_first_final_command_ns']/1e6,
            semantic_release_to_final_program_ms=(metric['release_to_open_ns']+metric['open_to_first_final_command_ns'])/1e6,
            udp_send_to_final_program_ms=(first_final-released['time_ns'])/1e6,
            udp_send_to_observed_program_ms=(first['mono']-released['send_mono'])*1000,
            udp_send_to_feedback_change_ms=(changed[0]['stamp_ns']-released['time_ns'])/1e6 if changed else None,
            udp_send_to_exact_first_target_feedback_ms=(equal[0]['stamp_ns']-released['time_ns'])/1e6 if equal else None)
        assert result['udp_send_to_final_program_ms']>=0
        cycles.append(result); s.add('timing_cycle',**result)
        print('release timing',index+1,round(result['udp_send_to_final_program_ms'],2),'ms',flush=True)
    keys=[k for k in cycles[0] if k.endswith('_ms')]
    return dict(cycles=cycles,summary={k:distribution([c[k] for c in cycles if c[k] is not None]) for k in keys},
                same_host_clock=True,hardware_latency_verified=False,
                exact_first_target_unmatched=sum(c['udp_send_to_exact_first_target_feedback_ms'] is None for c in cycles))


if __name__=='__main__':
    run([('thirty_release_to_program_measurements',timing)],'timing',sim_class=TimingSim)
