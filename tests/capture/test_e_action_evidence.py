"""Only actual ordered producer messages may make an OpenArm action valid."""
import pytest

from middleware.core.capture.ros_bridge import action_evidence_document


SOURCE={'kind':'action','timestamp_semantics':'send','fields':{
    'position':{'semantics':'continuous','units':['rad','rad'],'coordinate_frame':'joint'}},
    'named_axes':[{'name':'left_joint','unit':'rad'}, {'name':'right_joint','unit':'rad'}]}


def message(kind,sequence,source_epoch,**values):
    return {'schema':'openarm-action-evidence/v1','kind':kind,'sequence':sequence,
            'source_epoch':source_epoch,'stamp_ns':1_700_000_000_000_000_000,
            'names':['left_joint','right_joint'],**values}


def test_real_command_then_termination_fences_old_epoch():
    command,state=action_evidence_document('arm_action',SOURCE,
        message('command',12,'control-a',positions=[0.1,0.2]),None,
        1_700_000_000_001_000_000,123,0)
    assert command['validity_known'] is True
    assert command['source_epoch']=='control-a'
    assert command['payload']['position']==[0.1,0.2]
    event,state=action_evidence_document('arm_action',SOURCE,
        message('event',13,'control-b',reason='pause'),state,
        1_700_000_000_002_000_000,124,0)
    assert event['validity_event']=='pause'
    assert event['command_valid'] is False
    with pytest.raises(ValueError,match='epoch'):
        action_evidence_document('arm_action',SOURCE,
            message('command',14,'control-a',positions=[0.2,0.3]),state,
            1_700_000_000_003_000_000,125,0)
    resumed,_=action_evidence_document('arm_action',SOURCE,
        message('command',14,'control-b',positions=[0.2,0.3]),state,
        1_700_000_000_003_000_000,125,0)
    assert resumed['source_epoch']=='control-b'


@pytest.mark.parametrize('invalid',[
    message('command',14,'control-a',positions=[0.2,0.3]),
    message('command',13,'control-b',positions=[0.2,0.3]),
    message('command',13,'control-a',positions=[float('nan'),0.3]),
    {**message('command',13,'control-a',positions=[0.2,0.3]),'names':['wrong_joint']},
])
def test_gap_epoch_or_unnamed_target_cannot_create_valid_action(invalid):
    with pytest.raises((ValueError,KeyError)):
        action_evidence_document('arm_action',SOURCE,invalid,(12,'control-a',123,True),
                                 1_700_000_000_003_000_000,125,0)
