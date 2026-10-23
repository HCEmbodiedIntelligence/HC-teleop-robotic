#!/usr/bin/env python3
"""One real continuous MCAP containing AUTO, MANUAL and HOLD intervals."""
import json
import math
from pathlib import Path
import sys
import time

from dagger_sim_common import ROOT, run


def full_session(s):
    s.program_start(amplitude=.3)
    s.session(capture_mode='full_session')
    session=s.owner['session_id']
    for cycle in range(3):
        time.sleep(.4); s.manual()
        for i in range(50):
            angle=2*math.pi*i/25
            s.set_vr(right=[.2,1.3+.12*math.sin(angle),-.4,0.,math.sin(.4*math.sin(angle)),0.,math.cos(.4*math.sin(angle))])
            time.sleep(.01)
        if cycle==1:
            s.action('hold'); s.state('HOLD'); s.neutral(); time.sleep(.4)
            s.action('return'); s.state('AUTO')
        else:
            s.neutral(); s.state('AUTO')
    time.sleep(.4)
    s.action('disable'); s.state('OFF')
    directory=Path(s.original['ros']['recording']['directory'])
    if not directory.is_absolute(): directory=ROOT/directory
    found=[]

    def exported():
        for p in directory.glob('*.session.json'):
            metadata=json.loads(p.read_text())
            if metadata.get('session_id')==session:
                found[:] = [(p,metadata)]
                return True
        return False

    s.wait(exported,45,'full-session sidecar')
    path,metadata=found[0]
    mcap=path.with_suffix('').with_suffix('.mcap')
    sys.path.insert(0,str(ROOT/'.deps'))
    from mcap.reader import make_reader
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    events=[]; control_events=[]; evidence=[]; actions={}; observations=[]; base_observations=[]; ages=[]
    with mcap.open('rb') as stream:
        for _,channel,message in make_reader(stream,validate_crcs=True).iter_messages(log_time_order=False):
            if channel.topic=='/events/control/intervention':
                if channel.message_encoding=='json': events.append(json.loads(message.data))
                else: control_events.append(json.loads(deserialize_message(message.data,String).data))
            elif channel.topic=='/events/control/action_evidence': evidence.append(json.loads(message.data))
            elif channel.topic in ('/action/joint_states','/observation/joint_states'):
                value=deserialize_message(message.data,JointState)
                stamp=value.header.stamp.sec*10**9+value.header.stamp.nanosec
                assert len(value.name)==len(value.position) and all(math.isfinite(q) for q in value.position)
                assert -50_000_000<=message.log_time-stamp<=250_000_000, 'stale full-session joint sample'
                ages.append((message.log_time-stamp)/1e6)
                if channel.topic.startswith('/action'):
                    assert stamp not in actions, 'duplicate output stamp'
                    actions[stamp]=dict(zip(value.name,value.position))
                else: observations.append(stamp)
            elif channel.topic == '/observation/base/pose':
                value = deserialize_message(message.data, PoseStamped)
                stamp = value.header.stamp.sec*10**9+value.header.stamp.nanosec
                p, q = value.pose.position, value.pose.orientation
                assert all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                assert -50_000_000 <= message.log_time-stamp <= 250_000_000, 'stale full-session base sample'
                ages.append((message.log_time-stamp)/1e6)
                base_observations.append(stamp)
    assert events[0]['kind']=='session_open' and events[-1]['kind']=='session_close'
    intervals=metadata['interventions']
    assert len(intervals)==3 and len([e for e in events if e['kind']=='manual_open'])==3
    assert len([e for e in events if e['kind']=='manual_close'])==3
    assert len([e for e in control_events if e['kind']=='manual_open'])==3
    assert len([e for e in control_events if e['kind']=='manual_close'])==3
    assert metadata['quality']['grade']=='ok' and metadata['quality']['rejected']==0
    for interval in intervals:
        samples=[e for e in evidence if e['key']==interval['key']]
        assert [e['command_seq'] for e in samples]==list(range(1,interval['last_command_seq']+1))
        assert samples
        for e in samples:
            assert e['source']=='vr' and interval['start_ns']<=e['stamp_ns']<interval['end_ns']
            assert 0<=e['stamp_ns']-e['command_stamp_ns']<=300_000_000
            assert actions[e['command_stamp_ns']]==dict(zip(e['names'],e['positions']))
    human={e['command_stamp_ns'] for e in evidence}
    program=[stamp for stamp in actions if stamp not in human]
    assert program and evidence and observations and base_observations
    for stamps in (observations, base_observations):
        assert all(b > a for a, b in zip(stamps, stamps[1:])), 'source timestamps not monotonic'
    assert any(stamp<intervals[0]['start_ns'] for stamp in program)
    assert any(stamp>=intervals[-1]['end_ns'] for stamp in program)
    assert any(intervals[0]['end_ns']<=stamp<intervals[1]['start_ns'] for stamp in program)
    return dict(path=str(mcap),session_id=session,manual_intervals=3,human_actions=len(human),
                program_actions=len(program),observations=len(observations),crc_verified=True,
                action_evidence_exact=True,quality=metadata['quality']['grade'],
                receive_age_max_ms=max(ages),control_events=len(control_events),
                command_to_final_max_ms=max(e['stamp_ns']-e['command_stamp_ns'] for e in evidence)/1e6,
                feedback_max_source_gap_ms=max(b-a for a,b in zip(observations,observations[1:]))/1e6,
                base_observations=len(base_observations),
                base_max_source_gap_ms=max(b-a for a,b in zip(base_observations,base_observations[1:]))/1e6)


if __name__=='__main__':
    run([('continuous_auto_manual_hold_capture',full_session)],'full_session')
