"""Actual SDK/RTC/HC/driver closed-loop software acceptance, no real hardware."""
import copy
import json
import math
import os
import shutil
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool, Header, String
from humanoid_motion_interfaces.action import MoveJ

from tests.robot_api_fake_rig import FakeRig
from middleware.core.app import MiddlewareRuntime
from adapters.nodes.unified_motion_teleop import UnifiedMotionTeleopBridge

ROOT = Path(__file__).resolve().parents[1]
PROFILES = [ROOT/'adapters/robots'/n for n in ('openarmx','x1')]+[ROOT/'tests/fixtures/robot_api_six_dof']


def pose_error(first, second):
    a,b=first.pose,second.pose
    distance=math.sqrt(sum((getattr(a.position,k)-getattr(b.position,k))**2 for k in ('x','y','z')))
    qa=[getattr(a.orientation,k) for k in ('x','y','z','w')]
    qb=[getattr(b.orientation,k) for k in ('x','y','z','w')]
    angle=2*math.acos(min(1.,abs(sum(x*y for x,y in zip(qa,qb)))))
    return distance,angle


def home_move(rig, info):
    names=info['groups']['left_arm']['joint_names']
    client=ActionClient(rig.node,MoveJ,info['groups']['left_arm']['endpoints']['move_j'])
    try:
        rig.until(client.server_is_ready)
        goal=MoveJ.Goal(group_name='left_arm')
        goal.target=JointState(name=names,position=[info['poses']['home']['left_arm'][n] for n in names])
        goal.target.header.stamp=rig.node.get_clock().now().to_msg()
        handle=rig.future(client.send_goal_async(goal))
        assert handle.accepted
        wrapped=rig.future(handle.get_result_async(),20.)
        assert wrapped.status==4 and wrapped.result.status.code==0, wrapped.result.status
    finally:
        client.destroy()
    rig.pump(.3)


def check_shared_vr(rig, info, reachable):
    """A8/A9/A10/A11: real IK+RTC with identical mapped pose and feedback."""
    group=info['groups']['left_arm']
    fk=group['fk_pose_topic']
    names=group['joint_names']
    public_pub=rig.node.create_publisher(PoseStamped,group['endpoints']['servo_p'],1)
    def program_pose():
        reachable.header.stamp=rig.node.get_clock().now().to_msg()
        public_pub.publish(reachable)
    initial_error=pose_error(rig.received[fk][-1],reachable)
    rig.pump(.15)
    rig.pump(2.5,program_pose)
    program_error=pose_error(rig.received[fk][-1],reachable)
    program_end={n:rig.positions[n] for n in rig.api.groups['left_arm']}
    assert program_error[0]<.003 and program_error[1]<.02, program_error
    assert sum(program_error)<sum(initial_error)*.65, (initial_error,program_error)
    rig.pump(.3)
    home_move(rig,info)
    vr=UnifiedMotionTeleopBridge(rig.api.directory)
    rig.extra_nodes.append(vr); rig.executor.add_node(vr)
    status=rig.node.create_publisher(String,'/status/teleop',1)
    poses=rig.node.create_publisher(PoseArray,rig.api.teleop['control']['target_pose_topic'],1)
    status_msg=String(data=json.dumps(dict(enabled=True,mode='teleop',arms=dict(left=dict(active=True),right=dict(active=False)))))
    pair=PoseArray()
    pair.header.frame_id=group['base_frame']
    pair.poses=[rig.received[info['groups']['right_arm']['fk_pose_topic']][-1].pose, reachable.pose]
    def vr_pose():
        pair.header.stamp=rig.node.get_clock().now().to_msg()
        status.publish(status_msg); poses.publish(pair)
    rig.pump(.2)
    count=len(rig.received['/internal/robot/joint_commands'])
    rig.pump(.4,vr_pose)
    assert len(rig.received['/internal/robot/joint_commands'])==count, 'VR input escaped motion source'
    rig.select('vr')
    count=len(rig.received['/internal/robot/joint_commands'])
    rig.pump(.4,program_pose)
    assert len(rig.received['/internal/robot/joint_commands'])==count, 'public input escaped VR source'
    rig.pump(2.5,vr_pose)
    vr_error=pose_error(rig.received[fk][-1],reachable)
    assert vr_error[0]<.003 and vr_error[1]<.02, vr_error
    difference=max(abs(rig.positions[n]-q) for n,q in program_end.items())
    assert difference<.03, difference
    rig.pump(.3)

    # Public MoveJ is forbidden under VR; only the private home channel works.
    client=ActionClient(rig.node,MoveJ,group['endpoints']['move_j'])
    rig.until(client.server_is_ready)
    goal=MoveJ.Goal(group_name='left_arm',target=JointState(name=names,position=[info['poses']['home']['left_arm'][n] for n in names]))
    goal.target.header.stamp=rig.node.get_clock().now().to_msg()
    handle=rig.future(client.send_goal_async(goal))
    assert not handle.accepted
    client.destroy()
    home_results=[]
    rig.node.create_subscription(String,'/internal/vr/home_result',lambda m:home_results.append(json.loads(m.data)),1)
    home_pub=rig.node.create_publisher(Header,'/internal/vr/home',1)
    internal=rig.node.create_publisher(PoseStamped,'/internal/vr/left_arm/servo_p',1)
    rig.pump(.2,lambda:status.publish(status_msg))
    request=Header(stamp=rig.node.get_clock().now().to_msg())
    home_pub.publish(request)
    rig.until(lambda:vr.home_batch is not None or home_results)
    # Inject ServoP only while the group is occupied by MoveJ.
    deadline=time.monotonic()+20.
    while not home_results:
        assert time.monotonic()<deadline, 'VR MoveJ home did not terminate'
        status.publish(status_msg)
        reachable.header.stamp=rig.node.get_clock().now().to_msg()
        internal.publish(reachable)
        rig.pump(.01)
    assert home_results[-1]['success'], home_results
    rig.pump(.3)
    count=len(rig.received['/internal/robot/joint_commands'])
    rig.pump(.3,lambda:internal.publish(reachable))  # stamp predates release
    assert len(rig.received['/internal/robot/joint_commands'])==count
    for model in rig.api.groups['left_arm']:
        assert abs(rig.positions[model]-rig.api.teleop['robot']['initial_joints'][model])<.012
    rig.select('motion')
    return dict(initial_error=initial_error,program_error=program_error,vr_error=vr_error,
                max_joint_difference=difference,vr_home=home_results[-1])


def check_recovery_and_invalid(rig, info):
    spec=info['groups']['left_arm']; names=spec['joint_names']
    public_home=info['poses']['home']['left_arm']
    target=JointState(name=names,position=[public_home[n] for n in names])
    target.position[3]+=.04
    publisher=rig.node.create_publisher(JointState,spec['endpoints']['servo_j'],1)
    def publish():
        target.header.stamp=rig.node.get_clock().now().to_msg(); publisher.publish(target)
    rig.pump(.2)
    invalid=[JointState(name=['unknown'],position=[0.]), JointState(name=[names[0]]*2,position=[0.,0.]),
             JointState(name=names,position=[0.]),JointState(name=[names[0]],position=[float('nan')]),
             JointState(name=['right_joint_1'],position=[0.]),
             JointState(name=[names[0]],position=[spec['limits'][0]['upper']+1.])]
    for bad in invalid:
        count=len(rig.received['/internal/robot/joint_commands'])
        def send_bad():
            bad.header.stamp=rig.node.get_clock().now().to_msg(); publisher.publish(bad)
        rig.pump(.15,send_bad)
        assert len(rig.received['/internal/robot/joint_commands'])==count

    # A disconnected VR input must not revoke selected program control.
    runtime=SimpleNamespace(ros=rig.bridge,config={'safety':{'enabled':True}},emit=lambda *_:None,
                            _on_safety_event=lambda reason:pytest.fail('VR timeout stopped motion: '+reason))
    MiddlewareRuntime._on_vr_timeout(runtime,'simulated headset timeout')
    rig.pump(.8,publish)
    assert rig.bridge.status()['command_mux']['output_enabled']
    old=copy.deepcopy(target)
    rig.bridge.emergency_stop('/control/emergency_stop','isolated acceptance')
    rig.pump(.4,publish)
    count=len(rig.received['/internal/robot/joint_commands'])
    rig.pump(.2,publish)
    assert len(rig.received['/internal/robot/joint_commands'])==count
    rig.select('motion')
    rig.pump(.3,lambda:publisher.publish(old))
    assert len(rig.received['/internal/robot/joint_commands'])==count, 'old target survived stop epoch'
    rig.pump(.5,publish)
    assert len(rig.received['/internal/robot/joint_commands'])>count
    rig.feedback_enabled=False
    rig.pump(.6,publish)
    assert not rig.bridge.status()['command_mux']['output_enabled']
    rig.feedback_enabled=True
    rig.pump(.4,publish)
    count=len(rig.received['/internal/robot/joint_commands'])
    rig.pump(.2,publish)
    assert len(rig.received['/internal/robot/joint_commands'])==count, 'feedback recovery implicitly enabled output'
    rig.select('motion')
    rig.pump(.6,publish)
    assert len(rig.received['/internal/robot/joint_commands'])>count
    rig.pump(.3)

    client=ActionClient(rig.node,MoveJ,spec['endpoints']['move_j'])
    rig.until(client.server_is_ready)
    goal=MoveJ.Goal(group_name='left_arm',target=copy.deepcopy(target))
    goal.target.position[3]+=.35
    goal.target.header.stamp=rig.node.get_clock().now().to_msg()
    handle=rig.future(client.send_goal_async(goal)); assert handle.accepted
    rig.pump(.2,publish)  # Servo attempts cannot preempt occupied MoveJ.
    cancel=rig.future(handle.cancel_goal_async()); assert cancel.goals_canceling
    terminal=rig.future(handle.get_result_async(),5.)
    assert terminal.status==5, terminal.result.status
    rig.pump(.3)
    count=len(rig.received['/internal/robot/joint_commands'])
    rig.pump(.3,lambda:publisher.publish(target))
    assert len(rig.received['/internal/robot/joint_commands'])==count, 'canceled Move resumed old Servo'
    client.destroy()
    home_move(rig,info)

    # Exoskeleton stays a selected public-named source; historical names fail.
    rig.select('exoskeleton')
    exo=rig.node.create_publisher(JointState,'/control/exoskeleton/joint_states',1)
    rig.pump(.2)
    def exo_target():
        target.header.stamp=rig.node.get_clock().now().to_msg(); exo.publish(target)
    rig.pump(.3,exo_target)
    model=rig.api.public[names[3]]
    assert abs(rig.positions[model['model_name']]-(model['scale']*target.position[3]+model['offset']))<.005
    rig.pump(.1)  # Drain public feedback from the last valid exoskeleton sample.
    before=len(rig.received['/action/joint_states'])
    bad=JointState(name=[model['model_name']],position=[0.])
    def native_exo():
        bad.header.stamp=rig.node.get_clock().now().to_msg(); exo.publish(bad)
    rig.pump(.2,native_exo)
    assert len(rig.received['/action/joint_states'])==before
    rig.select('motion')
    home_move(rig,info)
    return dict(stop_epoch=True,feedback_expiry=True,explicit_recovery=True,
                invalid_targets=len(invalid),cancel_status=terminal.status,exoskeleton_public=True)


@pytest.mark.parametrize('profile',PROFILES,ids=['openarmx','x1','six_dof'])
def test_public_closed_loop(profile,tmp_path):
    report = Path(os.environ.get('HC_UNIFIED_REPORT_DIR',tmp_path))/profile.name
    with FakeRig(profile,report) as rig:
        # Subscribe late using durability; no client needs a profile or brand.
        descriptions=[]
        rig.node.create_subscription(String,'/status/robot/info',
            lambda msg: descriptions.append(json.loads(msg.data)), QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        rig.until(lambda: descriptions)
        info=descriptions[-1]; group=info['groups']['left_arm']; names=group['joint_names']
        assert info['schema']=='hc-robot-api/v1'
        assert len(names)==group['count']
        assert names==[f'left_joint_{i}' for i in range(1,len(names)+1)]
        measured=rig.received['/observation/joint_states'][-1]
        initial=dict(zip(measured.name,measured.position))
        target=JointState(name=list(names),position=[initial[n] for n in names])
        target.position[3]+=.06
        pub=rig.node.create_publisher(JointState,group['endpoints']['servo_j'],1)
        def servo():
            target.header.stamp=rig.node.get_clock().now().to_msg()
            pub.publish(target)
        rig.pump(.15)
        rig.pump(1.5,servo)
        model=rig.api.public[names[3]]
        expected=model['scale']*target.position[3]+model['offset']
        assert abs(rig.positions[model['model_name']]-expected)<.012, rig.bridge.status()['command_mux']
        assert rig.controller_commands
        rig.pump(.3)
        stopped=len(rig.received['/internal/robot/joint_commands'])
        held=dict(rig.positions)
        rig.pump(.2)
        assert len(rig.received['/internal/robot/joint_commands'])==stopped, 'Servo lease continued after input stopped'
        assert rig.positions==held  # Drivers may continue publishing measured hold.
        reachable=copy.deepcopy(rig.received[group['fk_pose_topic']][-1])

        # Same executable processes a pose derived from actual FK, using IK+RTC.
        pose=copy.deepcopy(rig.received[group['fk_pose_topic']][-1])
        pose.pose.position.x+=.002
        poses=rig.node.create_publisher(PoseStamped,group['endpoints']['servo_p'],1)
        def servo_p():
            pose.header.stamp=rig.node.get_clock().now().to_msg()
            poses.publish(pose)
        rig.pump(.15)
        before=len(rig.controller_commands)
        rig.pump(1.8,servo_p)
        assert len(rig.controller_commands)>before, 'valid ServoP produced no hardware commands'
        achieved=rig.received[group['fk_pose_topic']][-1]
        assert abs(achieved.pose.position.x-pose.pose.position.x)<.006
        rig.pump(.2)

        # MoveJ observes measured completion; feedback/results use public names.
        client=ActionClient(rig.node,MoveJ,group['endpoints']['move_j'])
        rig.until(lambda: client.server_is_ready())
        goal=MoveJ.Goal(group_name='left_arm')
        goal.target.name=list(names)
        goal.target.position=[info['poses']['home']['left_arm'][n] for n in names]
        goal.target.header.stamp=rig.node.get_clock().now().to_msg()
        feedback=[]
        handle=rig.future(client.send_goal_async(goal,feedback_callback=lambda f:feedback.append(f.feedback)))
        assert handle.accepted
        result=rig.future(handle.get_result_async(),20.)
        assert result.status==4 and result.result.status.code==0, result.result.status
        public=set(j['public_name'] for j in info['joints'])
        assert feedback and set(feedback[-1].actual_joint_state.name)<=public
        assert set(result.result.final_joint_state.name)<=public
        assert all(set(msg.name)<=public for msg in rig.received['/action/joint_states'])
        assert all(set(msg.name)<=public for msg in rig.received['/observation/joint_states'])
        assert all(set(msg.name)<=set(rig.api.joints) for msg in rig.received['/internal/robot/joint_commands'])
        client.destroy()
        rig.pump(.3)
        shared=check_shared_vr(rig,info,reachable)
        safety=check_recovery_and_invalid(rig,info)
        description=json.loads(rig.run_demo(['describe']))
        assert description==info
        assert 'left_joint_1' in rig.run_demo(['subscribe','--duration','.3','--period','.1'])
        rig.run_demo(['servo-j','--joint','left_joint_4','--delta','.02','--duration','.8'])
        rig.pump(.3)
        rig.run_demo(['servo-p','--dx','0.001','--duration','.8'])
        rig.pump(.3)
        rig.run_demo(['home','--scale','.25','--timeout','15'])
        rig.run_demo(['move-j','--joint','left_joint_4','--delta','.01','--scale','.25'])
        rig.run_demo(['move-j','--joint','left_joint_4','--delta','.3','--scale','.01'],interrupt=True)
        rig.pump(.3)
        home_move(rig,info)
        assert rig.received['/observation/left_arm/joint_states'][-1].name==names
        assert rig.received['/action/left_arm/joint_states'][-1].name==names
        (report/'basic-result.json').write_text(json.dumps(dict(
            info=info, real_motion=True, real_driver=True, fake_plant=True,
            domain=rig.domain, controller_command_count=len(rig.controller_commands),
            action_status=result.status,business_status=result.result.status.code,shared=shared,safety=safety),indent=2))


def test_live_mapping_and_connection_changes(tmp_path):
    profile=tmp_path/'mapped_six_dof'
    shutil.copytree(PROFILES[-1],profile)
    path=profile/'robot_api.yaml'
    api=yaml.safe_load(path.read_text())
    api['joint_overrides']={'fixture_left_axis_4':{'scale':-.5,'offset_rad':.2},
                            'fixture_right_axis_2':{'scale':2.,'offset_rad':-.1}}
    params=api['driver']['parameters']
    params['plugin_parameters']=[
        'state_topic=/remapped/measured','command_topic.left_arm=/remapped/left',
        'command_topic.right_arm=/remapped/right','state_timeout_s=0.25','startup_grace_s=3.0']
    params['vendor_to_logical_scales']=[-1.5,.75]+[1.]*10
    params['vendor_to_logical_offsets_rad']=[.1,-.2]+[0.]*10
    path.write_text(yaml.safe_dump(api))
    test_public_closed_loop(profile,tmp_path)


@pytest.mark.parametrize('profile',PROFILES,ids=['openarmx','x1','six_dof'])
def test_invalid_and_unreachable_servo_p(profile,tmp_path):
    report=Path(os.environ.get('HC_UNIFIED_REPORT_DIR',tmp_path))/('invalid-pose-'+profile.name)
    with FakeRig(profile,report) as rig:
        spec=rig.api.describe()['groups']['left_arm']
        publisher=rig.node.create_publisher(PoseStamped,spec['endpoints']['servo_p'],1)
        pose=copy.deepcopy(rig.received[spec['fk_pose_topic']][-1])
        invalid=[]
        bad=copy.deepcopy(pose); bad.header.frame_id='unconfigured_frame'; invalid.append(bad)
        bad=copy.deepcopy(pose); bad.pose.position.x=float('nan'); invalid.append(bad)
        bad=copy.deepcopy(pose)
        bad.pose.orientation.x=bad.pose.orientation.y=bad.pose.orientation.z=bad.pose.orientation.w=0.
        invalid.append(bad)
        rig.pump(.2)
        for target in invalid:
            count=len(rig.received.get('/internal/robot/joint_commands',[]))
            def send():
                target.header.stamp=rig.node.get_clock().now().to_msg(); publisher.publish(target)
            rig.pump(.25,send)
            assert len(rig.received.get('/internal/robot/joint_commands',[]))==count
        log_path=report/'motion.log'
        previous_log=len(log_path.read_text())
        target=copy.deepcopy(pose); target.pose.position.x=100.
        def unreachable():
            target.header.stamp=rig.node.get_clock().now().to_msg(); publisher.publish(target)
        rig.pump(3.,unreachable)
        diagnostics=log_path.read_text()[previous_log:]
        assert any(reason in diagnostics for reason in ('returned false','violates','exceeded')), diagnostics
        for n in rig.api.groups['left_arm']:
            joint=rig.api.joints[n]
            assert math.isfinite(rig.positions[n])
            assert joint['model_lower']-1e-9<=rig.positions[n]<=joint['model_upper']+1e-9
        rig.pump(.3)
        count=len(rig.received.get('/internal/robot/joint_commands',[]))
        held=dict(rig.positions)
        rig.pump(.3)
        assert len(rig.received.get('/internal/robot/joint_commands',[]))==count
        assert rig.positions==held
        (report/'result.json').write_text(json.dumps(dict(invalid_rejected=len(invalid),
            unreachable_diagnostics=diagnostics,limits_preserved=True,stopped_after_lease=True),indent=2))
