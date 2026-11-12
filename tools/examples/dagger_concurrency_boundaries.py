#!/usr/bin/env python3
"""Actual HTTP concurrency, delayed retries and stop/release races in simulation."""
from concurrent.futures import ThreadPoolExecutor
import json
import time
import urllib.error
import urllib.request
import uuid

from dagger_sim_common import run


def payload(s, **changes):
    state = s.request('/api/control/ownership')
    return dict(dict(request_id=uuid.uuid4().hex, session_id=state['session_id'], expected_epoch=state['epoch']), **changes)


def send(s, name, value):
    body = value if isinstance(value, bytes) else json.dumps(value).encode()
    request = urllib.request.Request(s.url+'/api/control/intervention/'+name, data=body,
                                    headers={'Content-Type':'application/json'})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(request, timeout=40) as response:
            code, text = response.status, response.read().decode()
    except urllib.error.HTTPError as exc:
        code, text = exc.code, exc.read().decode()
    try:
        value = json.loads(text)
    except ValueError:
        value = {'error':text}
    s.add('http_result', operation=name, code=code, response=value)
    return code, value


def ready(s):
    s.wait(lambda: bool((s.request('/api/control/ownership').get('capture') or {}).get('ready')), 25, 'capture ready')


def concurrent_enable(s, identical):
    p = payload(s)
    bodies = [p if identical else dict(p, request_id=uuid.uuid4().hex) for _ in range(8)]
    with ThreadPoolExecutor(max_workers=8) as pool:
        responses = list(pool.map(lambda body: send(s, 'enable', body), bodies))
    assert sorted(code for code,_ in responses) == ([202]*8 if identical else [202]+[409]*7), responses
    sessions = {r['ownership']['session_id'] for code,r in responses if code == 202}
    assert len(sessions) == 1
    s.sessions.extend(sessions)
    ready(s)
    return dict(requests=8, accepted=sum(code==202 for code,_ in responses), sessions=list(sessions))


def old_stop(s):
    s.session(auto=False)
    stop = payload(s)
    assert send(s, 'disable', stop)[0] == 202
    s.session(auto=False)
    before = s.request('/api/control/ownership')
    assert send(s, 'disable', stop)[0] == 202
    after = s.request('/api/control/ownership')
    assert after['session_id'] == before['session_id'] and after['epoch'] == before['epoch']
    assert after['capture'] and after['capture']['ready'], 'old stop request stopped the new recording'
    return dict(old_session=stop['session_id'], protected_session=after['session_id'])


def old_enable(s):
    p = payload(s)
    try:
        code, response = send(s, 'enable', p)
        assert code == 202
        s.sessions.append(response['ownership']['session_id'])
        ready(s)
        s.action('disable')
        assert send(s, 'enable', p)[0] == 202
        state = s.request('/api/control/ownership')
        assert state['state']=='OFF' and state['capture'] is None, 'old enable created an orphan recording'
    finally:
        # Also clean up an orphan if testing an unfixed server.
        if s.request('/api/control/ownership').get('capture'):
            s.action('disable')


def stale_epoch(s):
    s.session(auto=False)
    p = payload(s)
    s.action('hold')
    before = s.request('/api/control/ownership')
    for operation in ('return','disable','takeover','recover','enable'):
        assert send(s, operation, dict(p,request_id=uuid.uuid4().hex))[0] == 409
    after = s.request('/api/control/ownership')
    assert after['epoch']==before['epoch'] and after['capture']['ready']


def hold_return(s):
    s.program_start(); s.session(auto=False)
    for _ in range(5):
        if s.owner['state']!='HOLD':
            s.action('hold'); s.state('HOLD')
        p = payload(s)
        with ThreadPoolExecutor(max_workers=2) as pool:
            responses = list(pool.map(lambda op:send(s,op,dict(p,request_id=uuid.uuid4().hex)), ('hold','return')))
        assert sorted(code for code,_ in responses)==[202,409], responses
        s.wait(lambda:s.owner['state'] in ('HOLD','AUTO'),5,'race settles')
    return dict(rounds=5)


def stop_release(s):
    s.program_start()
    conflicts = 0
    for _ in range(5):
        s.session(); s.manual()
        request = payload(s)
        with ThreadPoolExecutor(max_workers=2) as pool:
            stop = pool.submit(send,s,'disable',request)
            release = pool.submit(s.set_vr,grip=0.)
            code, response = stop.result(); release.result()
        assert code in (202, 409), (code, response)
        if code == 409:
            # A real release may win the epoch transaction. The old request
            # must stay rejected; only a new request for this same session can
            # express the still-current intent to stop.
            assert response['error'] == 'stale session or epoch', response
            conflicts += 1
            s.state('AUTO')
            current = payload(s)
            assert current['session_id'] == request['session_id']
            assert current['expected_epoch'] > request['expected_epoch']
            assert send(s,'disable',current)[0] == 202
        s.state('OFF'); time.sleep(.2)
        start=time.monotonic(); before=s.joints.copy(); base=s.base.copy()
        time.sleep(.3); s.static(start,before,base=base)
        assert s.request('/api/control/ownership')['capture'] is None
    return dict(rounds=5, release_won_conflicts=conflicts, stale_request_remained_rejected=True)


def estop_return(s):
    s.program_start(); s.session(auto=False)
    p = payload(s)
    with ThreadPoolExecutor(max_workers=2) as pool:
        resume=pool.submit(send,s,'return',p)
        stop=pool.submit(s.request,'/api/safety/stop',{'reason':'simulation concurrent stop/return'})
        resume.result(); stop.result()
    s.state('ESTOP'); time.sleep(.2)
    start=time.monotonic(); before=s.joints.copy(); base=s.base.copy()
    time.sleep(.35); s.static(start,before,base=base)
    assert s.owner['state']=='ESTOP'


def invalid_json(s):
    before=s.request('/api/control/ownership')
    for body in (b'{',b'',b'\xff',b'null',b'[]',b'42'):
        assert send(s,'enable',body)[0]==400
    after=s.request('/api/control/ownership')
    assert after['state']=='OFF' and after['epoch']==before['epoch'] and after['capture'] is None


if __name__=='__main__':
    run([('eight_identical_enable',lambda s:concurrent_enable(s,True)),
         ('eight_distinct_enable',lambda s:concurrent_enable(s,False)),
         ('old_stop_new_session',old_stop),('old_enable_after_exit',old_enable),
         ('stale_epoch_all_operations',stale_epoch),('hold_return_race',hold_return),
         ('disable_release_race',stop_release),('estop_return_race',estop_return),
         ('malformed_json',invalid_json)], 'concurrency')
