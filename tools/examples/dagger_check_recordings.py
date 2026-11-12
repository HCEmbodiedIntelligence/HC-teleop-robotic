#!/usr/bin/env python3
"""回读纠正 MCAP：CRC、来源、时间间隔、接收年龄、片段边界与动作反馈匹配。"""
import argparse
import bisect
import collections
import json
import math
from pathlib import Path
import statistics
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'.deps'))
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def stats(values):
    values=sorted(values)
    if not values:return None
    return dict(count=len(values),min=min(values),median=statistics.median(values),
        p95=values[math.ceil(.95*len(values))-1],p99=values[math.ceil(.99*len(values))-1],max=max(values))


def inspect(path):
    metadata=json.loads(path.read_text())
    mcap=path.with_suffix('').with_suffix('.mcap')
    provenance=collections.defaultdict(list)
    times=collections.defaultdict(list)
    actions=[];feedback=[];evidence=[]
    with mcap.open('rb') as stream:
        for schema,channel,message in make_reader(stream,validate_crcs=True).iter_messages(log_time_order=False):
            times[channel.topic].append(message.log_time)
            if channel.topic=='/events/recording/sample_provenance':
                event=json.loads(message.data)
                if 'topic' in event:
                    provenance[event['topic']].append(event)
                else:
                    assert event.get('action_boundary')=='hc_final' and 'command_seq' in event
            if channel.topic=='/events/control/action_evidence':
                evidence.append(json.loads(message.data))
            if channel.topic in ('/action/joint_states','/observation/joint_states'):
                data=deserialize_message(message.data,JointState)
                assert len(data.name)==len(data.position) and len(set(data.name))==len(data.name)
                assert all(math.isfinite(v) for v in data.position)
                stamp=data.header.stamp.sec*10**9+data.header.stamp.nanosec
                assert stamp==message.log_time, 'recorded source time changed'
                (actions if channel.topic.startswith('/action') else feedback).append((stamp,dict(zip(data.name,data.position))))
            if channel.topic == '/observation/base/pose':
                data = deserialize_message(message.data, PoseStamped)
                p, q = data.pose.position, data.pose.orientation
                assert all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
                assert data.header.stamp.sec*10**9+data.header.stamp.nanosec == message.log_time, 'base source time changed'
    assert actions and feedback and evidence,'missing core streams'
    assert len(evidence)==len(actions)==metadata['quality']['action_received']
    assert all(e['source']=='vr' and e['intervention'] and e['action_boundary']=='hc_final' for e in evidence)
    assert len({e['intervention_id'] for e in evidence})==1
    assert [e['command_seq'] for e in evidence]==list(range(1,len(evidence)+1)),'missing or duplicate action sequence'
    assert all(metadata['start_ns']<=stamp<metadata['end_ns'] for stamp,_ in actions)
    command_ages=[(e['stamp_ns']-e['command_stamp_ns'])/1e6 for e in evidence if 'command_stamp_ns' in e]
    assert all(0<=age<=300 for age in command_ages), 'command source/final time mismatch'
    for (stamp,positions),e in zip(actions,evidence):
        assert stamp==e['stamp_ns'] and positions==dict(zip(e['names'],e['positions']))
    streams={}
    topics = ['/observation/joint_states','/action/joint_states']
    if times['/observation/base/pose']:
        topics.append('/observation/base/pose')
    for topic in topics:
        stamps=times[topic]
        assert all(b>a for a,b in zip(stamps,stamps[1:])),topic+' is not strictly monotonic'
        gaps=[(b-a)/1e6 for a,b in zip(stamps,stamps[1:])]
        ages=[(p['received_ns']-p['source_stamp_ns'])/1e6 for p in provenance[topic]]
        # 250 ms is the configured freshness limit, not a physical motor latency guarantee.
        assert all(-50<=age<=250 for age in ages),topic+' stale/future source'
        streams[topic]=dict(messages=len(stamps),interval_ms=stats(gaps),receive_age_ms=stats(ages),
            gaps_over_50ms=sum(x>50 for x in gaps),gaps_over_200ms=sum(x>200 for x in gaps))
    feedback.sort();stamps=[p[0] for p in feedback]
    matched=[];unmatched=0;without_following_feedback=0;nearest_error=[]
    for stamp,target in actions:
        index=bisect.bisect_left(stamps,stamp)
        if index<len(feedback):
            nearest_error.append(max(abs(feedback[index][1][name]-q) for name,q in target.items()))
        found=None
        for t,actual in feedback[index:index+16]:
            if t-stamp>150_000_000:break
            if all(abs(actual[name]-q)<1e-5 for name,q in target.items()):
                found=(t-stamp)/1e6;break
        if found is None:
            unmatched+=1
            if index==len(feedback):without_following_feedback+=1
        else:matched.append(found)
    quality=metadata['quality']
    assert quality['missing_actions']==0 and not quality['missing_topics']
    return dict(path=str(mcap),key=metadata['key'],duration_sec=(metadata['end_ns']-metadata['start_ns'])/1e9,
        end_reason=metadata['end_reason'],quality=quality['grade'],streams=streams,
        command_to_final_ms=stats(command_ages),
        action_to_first_equal_feedback_ms=stats(matched),actions_without_equal_feedback_within_150ms=unmatched,
        actions_at_end_without_following_feedback=without_following_feedback,
        next_feedback_max_joint_error_rad=stats(nearest_error),crc_and_action_evidence_verified=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--session',action='append',default=[])
    parser.add_argument('--sessions-from',type=Path,action='append',default=[])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    sessions=set(args.session)
    for source in args.sessions_from:
        data=json.loads(source.read_text());sessions.update(data.get('sessions',[]))
        if data.get('session_id'):sessions.add(data['session_id'])
    results=[]
    for path in sorted(args.directory.glob('*.correction.json')):
        if sessions and json.loads(path.read_text())['key'].split(':')[2] not in sessions:continue
        try:results.append(dict(result='PASS',**inspect(path)))
        except Exception as exc:results.append(dict(result='FAIL',path=str(path),error=str(exc)))
    assert results,'No matching recordings'
    report=dict(result='PASS' if all(r['result']=='PASS' for r in results) else 'FAIL',recordings=results,
        quality_grades=dict(collections.Counter(r.get('quality','audit_failed') for r in results)),
        quality_warning_files=[r['path'] for r in results if r.get('quality') in ('warning','review')],
        scope='CRC, action identity and configured freshness checks; PASS does not erase quality warnings')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(result=report['result'],files=len(results),quality_grades=report['quality_grades'],
        failures=[r for r in results if r['result']=='FAIL']),ensure_ascii=False))
    if report['result']!='PASS':raise SystemExit(1)


if __name__=='__main__':main()
