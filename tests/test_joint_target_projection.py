"""Field projection preserves messages and authority while avoiding full clones."""
import copy
import time

from test_legacy_control import Rig, message


def test_projection_preserves_stamp_frame_order_and_optional_arrays_without_aliasing():
    rig=Rig();b=rig.boundary
    names=list(reversed(b.arms))
    original=message({n:b.positions[n] for n in names})
    original.header.frame_id='base#reference'
    original.velocity=[float(i) for i in range(len(names))]
    original.effort=[float(i+20) for i in range(len(names))]
    before=copy.deepcopy(original)
    selected=b.select(original,[b.arms[0],b.arms[3],'missing'])
    assert selected.name==[b.arms[0],b.arms[3]]
    assert selected.header==original.header
    assert list(selected.position)==[b.positions[n] for n in selected.name]
    assert list(selected.velocity)==[original.velocity[names.index(n)] for n in selected.name]
    assert list(selected.effort)==[original.effort[names.index(n)] for n in selected.name]
    selected.header.frame_id='changed';selected.header.stamp.sec+=1
    selected.position[0]+=1;selected.velocity[0]+=1;selected.effort[0]+=1
    assert original==before


def test_owned_projection_removes_only_private_frame_and_does_not_mutate_input():
    rig=Rig();b=rig.boundary
    rig.control.input(device_instance='vr',sequence=2,valid=True,gripped=True,stamp_ns=time.time_ns())
    rig.advance();rig.advance();rig.feed()
    original=message({n:b.positions[n] for n in b.arms})
    original.header.frame_id=rig.control.token.key()+'#base#reference'
    before=copy.deepcopy(original)
    b.receive_owned(original,rig.control.token,frame_id='base#reference')
    sent=[m for t,m in rig.wire if t=='/action/joint_states']
    assert len(sent)==1
    assert sent[0].header.frame_id=='base#reference'
    assert sent[0].header.stamp==original.header.stamp
    assert sent[0].name==original.name and sent[0].position==original.position
    assert not sent[0].velocity and not sent[0].effort
    assert original==before
