"""Actual teleop auxiliaries and an enabled shared collision boundary."""
import copy
import json
import math
import os
from pathlib import Path
import shutil
from xml.etree import ElementTree as ET

import pytest
import yaml
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from mcap.writer import Writer
from rclpy.serialization import serialize_message
from middleware.core.topic_player import TopicPlayer
from middleware.core.topic_recorder import _get_msg_def
from middleware.core.app import MiddlewareRuntime
from middleware.core.config import validate_config

from adapters.nodes.unified_motion_teleop import UnifiedMotionTeleopBridge
from tests.robot_api_fake_rig import FakeRig
from tests.test_unified_robot_api_ros import home_move

ROOT=Path(__file__).resolve().parents[1]


def test_actual_vr_auxiliary_and_clutch_paths(tmp_path):
    report=Path(os.environ.get('HC_UNIFIED_REPORT_DIR',tmp_path))/'vr-auxiliary'
    # The shipped X1 profile deliberately disables neck tracking. Exercise its
    # optional enabled path in a local profile without changing that default.
    source=ROOT/'adapters/robots/x1'
    profile=tmp_path/'x1_neck_enabled'
    shutil.copytree(source,profile)
    config=yaml.safe_load((profile/'arm_teleop.yaml').read_text())
    assert config['body']['neck_tracking_enabled'] is False
    config['body']['neck_tracking_enabled']=True
    config['control']['backend']='motion'
    (profile/'arm_teleop.yaml').write_text(yaml.safe_dump(config))
    with FakeRig(profile,report,with_teleop=True) as rig:
        rig.select('vr')
        hands=[]; aux=[]; mapped=[]
        rig.node.create_subscription(JointState,'/action/right_hand/joint_states',hands.append,10)
        rig.node.create_subscription(JointState,'/internal/vr/aux_joint_targets',aux.append,10)
        rig.node.create_subscription(PoseStamped,'/internal/vr/left_arm/servo_p',mapped.append,10)
        publisher=rig.node.create_publisher(String,'/vrdata',1)
        frame=dict(tracking=dict(head=True,left=True,right=True),poses={
            'head':dict(position=[0.,1.5,0.],quaternion=[0.,0.,0.,1.]),
            'left':dict(position=[-.2,1.3,-.4],quaternion=[0.,0.,0.,1.]),
            'right':dict(position=[.2,1.3,-.4],quaternion=[0.,0.,0.,1.])},
            inputs=dict(left=dict(grip=1.,trigger=0.),right=dict(grip=0.,trigger=0.)))
        def send(): publisher.publish(String(data=json.dumps(frame)))
        rig.pump(.6,send)
        initial={n:rig.positions[n] for n in rig.api.groups['waist']}
        frame['poses']['head']['position'][1]+=.04
        rig.pump(1.,send)
        assert any(abs(rig.positions[n]-q)>1e-5 for n,q in initial.items()), (initial,rig.positions)
        assert aux and hands
        arm_names=set(rig.api.groups['whole_body'])
        assert all(not arm_names.intersection(m.name) for m in aux)
        assert any(set(rig.api.groups['head']).issubset(m.name) for m in aux)
        initial_head={n:rig.positions[n] for n in rig.api.groups['head']}
        frame['poses']['head']['quaternion']=[0.,math.sin(.1),0.,math.cos(.1)]
        rig.pump(.5,send)
        assert any(abs(rig.positions[n]-q)>1e-4 for n,q in initial_head.items())
        frame['inputs']['right']['trigger']=1.
        rig.pump(.3,send)
        assert hands[-1].position[-1]==1.  # Existing binary grasp indicator.
        assert hands[-1].position[:-1]==pytest.approx(rig.api.teleop['grippers']['right']['finger_closed'])
        # Reset acknowledgement uses measured feedback; physical release/press
        # is still required before mapped arm targets are forwarded.
        assert not mapped
        frame['inputs']['right']['grip']=1.
        rig.pump(.5,send)
        assert mapped, rig.teleop.solver_rearm.state
        before_arm={n:rig.positions[n] for n in rig.api.groups['left_arm']}
        before_waist={n:rig.positions[n] for n in rig.api.groups['waist']}
        frame['poses']['left']['position'][2]-=.01
        frame['poses']['head']['position'][1]+=.01
        rig.pump(1.2,send)
        assert any(abs(rig.positions[n]-q)>1e-4 for n,q in before_arm.items())
        assert any(abs(rig.positions[n]-q)>1e-5 for n,q in before_waist.items())
        frame['inputs']['right']['grip']=0.
        rig.pump(.4,send)
        count=len(mapped)
        rig.pump(.25,send)
        assert len(mapped)==count
        (report/'result.json').write_text(json.dumps(dict(waist_moved=True,head_retained=True,
            finger_closed=True,arm_aux_separated=True,simultaneous_arm_waist=True,clutch_gated=True),indent=2))


def collision_profile(tmp_path):
    import pybullet as bullet

    profile=tmp_path/'collision_fixture'
    shutil.copytree(ROOT/'tests/fixtures/robot_api_six_dof',profile)
    config_path=profile/'arm_teleop.yaml'
    config=yaml.safe_load(config_path.read_text())
    config['control']['collision_avoidance_enabled']=True
    for side in ('left','right'):
        config['arms'][side]['generic_task_base_link']=side+'_link_1'
    config_path.write_text(yaml.safe_dump(config))
    urdf=profile/'urdf/six_dof.urdf'
    client=bullet.connect(bullet.DIRECT)
    try:
        robot=bullet.loadURDF(str(urdf),useFixedBase=True,physicsClientId=client)
        indices={bullet.getJointInfo(robot,i,physicsClientId=client)[1].decode():i for i in range(bullet.getNumJoints(robot,physicsClientId=client))}
        for name,q in config['robot']['initial_joints'].items():
            bullet.resetJointState(robot,indices[name],q,physicsClientId=client)
        goal=config['robot']['initial_joints']['fixture_left_axis_4']+.5
        bullet.resetJointState(robot,indices['fixture_left_axis_4'],goal,physicsClientId=client)
        state=bullet.getLinkState(robot,indices['left_tool_mount'],computeForwardKinematics=True,physicsClientId=client)
        xyz,quat=state[4:6]
    finally:
        bullet.disconnect(client)
    tree=ET.parse(urdf); root=tree.getroot()
    collision=ET.SubElement(root.find("link[@name='left_tcp']"),'collision')
    ET.SubElement(ET.SubElement(collision,'geometry'),'sphere',radius='.01')
    body=ET.SubElement(root,'link',name='fixture_obstacle')
    collision=ET.SubElement(body,'collision')
    ET.SubElement(ET.SubElement(collision,'geometry'),'sphere',radius='.025')
    mount=ET.SubElement(root,'joint',name='fixture_obstacle_mount',type='fixed')
    ET.SubElement(mount,'parent',link='fixture_base'); ET.SubElement(mount,'child',link='fixture_obstacle')
    ET.SubElement(mount,'origin',xyz=' '.join(map(str,xyz)),rpy='0 0 0')
    tree.write(urdf)
    target=PoseStamped(); target.header.frame_id='fixture_base'
    target.pose.position.x,target.pose.position.y,target.pose.position.z=xyz
    target.pose.orientation.x,target.pose.orientation.y,target.pose.orientation.z,target.pose.orientation.w=quat
    return profile,goal,target


def test_enabled_collision_check_is_shared(tmp_path):
    profile,goal,pose=collision_profile(tmp_path)
    report=Path(os.environ.get('HC_UNIFIED_REPORT_DIR',tmp_path))/'collision'
    with FakeRig(profile,report) as rig:
        info=rig.api.describe(); spec=info['groups']['left_arm']
        collisions=[]
        rig.node.create_subscription(String,'/status/robot/collision',lambda m:collisions.append(json.loads(m.data)),10)
        publisher=rig.node.create_publisher(JointState,spec['endpoints']['servo_j'],1)
        target=JointState(name=spec['joint_names'],position=list(info['poses']['home']['left_arm'].values()))
        target.position[3]=goal
        def send():
            target.header.stamp=rig.node.get_clock().now().to_msg(); publisher.publish(target)
        rig.pump(.2); rig.pump(2.5,send)
        assert any(c['blocked'] for c in collisions)
        program_q=rig.positions['fixture_left_axis_4']
        assert program_q<goal-.03
        rig.pump(.3); home_move(rig,info)
        vr=UnifiedMotionTeleopBridge(profile)
        rig.extra_nodes.append(vr); rig.executor.add_node(vr)
        rig.select('vr')
        status=rig.node.create_publisher(String,'/status/teleop',1)
        targets=rig.node.create_publisher(PoseArray,rig.api.teleop['control']['target_pose_topic'],1)
        pair=PoseArray(); pair.header.frame_id='fixture_base'
        pair.poses=[rig.received[info['groups']['right_arm']['fk_pose_topic']][-1].pose,pose.pose]
        def send_vr():
            status.publish(String(data=json.dumps(dict(enabled=True,mode='teleop',arms=dict(left=dict(active=True),right=dict(active=False))))))
            pair.header.stamp=rig.node.get_clock().now().to_msg(); targets.publish(pair)
        collisions.clear(); rig.pump(.2); rig.pump(2.5,send_vr)
        assert any(c['blocked'] for c in collisions)
        vr_q=rig.positions['fixture_left_axis_4']
        assert vr_q<goal-.03
        (report/'result.json').write_text(json.dumps(dict(program_blocked=True,vr_blocked=True,
            goal=goal,program_actual=program_q,vr_actual=vr_q),indent=2))


def test_public_recording_replay_and_historical_rejection(tmp_path):
    report=Path(os.environ.get('HC_UNIFIED_REPORT_DIR',tmp_path))/'replay'
    with FakeRig(ROOT/'tests/fixtures/robot_api_six_dof',report) as rig:
        api=rig.api; info=api.describe(); names=info['groups']['left_arm']['joint_names']
        runtime=MiddlewareRuntime(validate_config({'robot_profiles':{
            'active':api.directory.name,'root':str(api.directory.parent)},'ros':{'domain_id':rig.domain}}),ROOT/'middleware')
        metadata=runtime._load_session_profile_metadata()
        assert metadata['robot_api_schema']==info['schema']
        assert metadata['robot_api_revision']==api.revision
        assert json.loads(metadata['free_joints'])==list(api.public)
        assert json.loads(metadata['robot_api_info'])==info
        target=JointState(name=names,position=list(info['poses']['home']['left_arm'].values()))
        target.position[3]+=.02; target.header.stamp.sec=1
        for name,meta in [('public',metadata),('historical',{'profile_id':api.profile_id})]:
            with (report/f'{name}.mcap').open('wb') as stream:
                writer=Writer(stream); writer.start(profile='ros2')
                writer.add_metadata('hc_teleop.session',meta)
                schema=writer.register_schema('sensor_msgs/msg/JointState','ros2msg',_get_msg_def('sensor_msgs/msg/JointState'))
                channel=writer.register_channel('/action/joint_states','cdr',schema)
                for index in range(12):
                    stamp=1_000_000_000+index*20_000_000
                    writer.add_message(channel,stamp,bytes(serialize_message(target)),stamp)
                writer.finish()
        player=TopicPlayer(report,lambda:rig.bridge,active_profile_provider=lambda:api.profile_id,
                           active_api_metadata_provider=lambda:metadata)
        try:
            try:
                player.play('historical.mcap',mode='drive')
                raise AssertionError('historical native-name recording was accepted')
            except ValueError as exc:
                assert 'historical' in str(exc)
            player.play('public.mcap',mode='drive')
            rig.until(lambda:player.status()['state'] in ('completed','error'),5.)
            assert player.status()['state']=='completed', player.status()
            rig.pump(.2)
            assert abs(rig.positions['fixture_left_axis_4']-target.position[3])<.005
            assert rig.received['/action/joint_states'][-1].header.stamp.sec>1
            assert not rig.bridge.status()['command_mux']['output_enabled']
            (report/'result.json').write_text(json.dumps(dict(public_replay=True,historical_rejected=True,
                names=names,metadata=metadata),indent=2))
        finally:
            player.stop()
