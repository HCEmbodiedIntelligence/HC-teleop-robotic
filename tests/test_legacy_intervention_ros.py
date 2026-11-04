"""Actual X1 v23 IK + VR adapter + middleware, with only fake actuators.

Opt in with HC_RUN_LEGACY_ROS_TESTS=1 on a localhost-only ROS domain >= 200.
No Motion node, action server, recorder or real hardware is used.
"""
import copy
import math
import importlib.util
import json
import os
from pathlib import Path
import time

import pytest

pytestmark=pytest.mark.skipif(os.environ.get('HC_RUN_LEGACY_ROS_TESTS')!='1', reason='isolated ROS opt-in required')


def test_v23_own_topic_takeover_and_return_without_recording(monkeypatch):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, CompressedImage
    from std_msgs.msg import Bool, String
    import yaml
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from middleware.core.ros_bridge import RosBridge

    domain=int(os.environ['ROS_DOMAIN_ID'])
    assert 200<=domain<=232 and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    # Do not leave ROS file logs outside the authorized project. Test output
    # is captured by the caller on the development workstation.
    original_init=Node.__init__
    def init(node,*args,**kwargs):
        kwargs['cli_args']=[*(kwargs.get('cli_args') or []),'--ros-args','--disable-external-lib-logs']
        original_init(node,*args,**kwargs)
    monkeypatch.setattr(Node,'__init__',init)
    root=Path(__file__).resolve().parents[1]
    profile=root/'adapters/robots/x1'
    cfg=yaml.safe_load((profile/'arm_teleop.yaml').read_text())
    spec=importlib.util.spec_from_file_location('hc_test_v23',root/'adapters/v23/script/control_v2_3_ros2.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init()
    probe=Node('legacy_test_fake_hardware')
    executor=SingleThreadedExecutor();executor.add_node(probe)
    teleop=solver=bridge=None
    frame=dict(protocol_version=3,sequence=0,vr_timestamp=0.,
        tracking=dict(head=True,left=True,right=True),poses={
            'head':dict(position=[0.,1.5,0.],quaternion=[0.,0.,0.,1.]),
            'left':dict(position=[-.2,1.3,-.4],quaternion=[0.,0.,0.,1.]),
            'right':dict(position=[.2,1.3,-.4],quaternion=[0.,0.,0.,1.])},
        inputs=dict(left=dict(grip=0.,trigger=0.),right=dict(grip=0.,trigger=0.)))
    values={n:float(q) for n,q in cfg['robot']['initial_joints'].items()}
    hand_values={n:float(q) for side in ('right','left') for n,q in zip(
        cfg['grippers'][side]['finger_joint_names'],cfg['grippers'][side]['finger_closed'])}
    arm_names=[n for side in ('right','left') for n in cfg['arms'][side]['joint_names']]
    outputs=[];states=[];images=[]
    program_active=True
    def command(msg,channel):
        outputs.append((time.monotonic(),channel,copy.deepcopy(msg),bridge.ownership.state if bridge else 'OFF'))
        (values if channel=='arms' else hand_values).update(
            (n,q) for n,q in zip(msg.name,msg.position) if math.isfinite(q))
    probe.create_subscription(JointState,'/action/joint_states',lambda m:command(m,'arms'),10)
    for side in ('right','left'):
        probe.create_subscription(JointState,f'/action/{side}_hand/joint_states',lambda m,s=side:command(m,s),10)
    probe.create_subscription(JointState,'/observation/joint_states',lambda m:states.append(m),10)
    probe.create_subscription(CompressedImage,'/observation/cameras/head/depth/compressed',lambda m:images.append(m),qos_profile_sensor_data)
    state_pub=probe.create_publisher(JointState,'/observation/joint_states',10)
    hands_pub=probe.create_publisher(JointState,'/observation/hands/joint_states',10)
    ready_pub=probe.create_publisher(Bool,'/status/hardware/ready',10)
    vr_pub=probe.create_publisher(String,'/vrdata',10)
    program=probe.create_publisher(JointState,'/hc_teleop/joint_cmd',10)
    program_hands={s:probe.create_publisher(JointState,'/hc_teleop/joint_cmd_finger_'+s,qos_profile_sensor_data) for s in ('right','left')}
    image_pub=probe.create_publisher(CompressedImage,'/observation/cameras/head/depth/compressed',qos_profile_sensor_data)
    def msg(data,stamp):
        m=JointState(name=list(data),position=list(data.values()))
        m.header.stamp=stamp
        return m
    def feed():
        stamp=probe.get_clock().now().to_msg()
        state=msg(values,stamp);state.velocity=[0.]*len(state.name)
        state_pub.publish(state)
        # Real X1 feedback can be outside the calibrated command interval.
        # The recent final command still provides a legal continuity reference.
        hands_pub.publish(msg({**hand_values, 'L_thumb_roll_joint':.0252},stamp))
        ready_pub.publish(Bool(data=True))
        frame.update(sequence=frame['sequence']+1,vr_timestamp=time.monotonic(),hc_received_ns=time.time_ns())
        vr_pub.publish(String(data=json.dumps(frame)))
        if program_active:
            program.publish(msg({n:float(cfg['robot']['initial_joints'][n])+.005 for n in arm_names},stamp))
            for side,pub in program_hands.items():
                pub.publish(msg({n:hand_values[n] for n in cfg['grippers'][side]['finger_joint_names']},stamp))
        image=CompressedImage(format='png',data=b'unchanged-depth-bytes');image.header.stamp=stamp
        image_pub.publish(image)
    timer=probe.create_timer(.02,feed)
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.002)
    def until(condition,timeout=6):
        end=time.monotonic()+timeout
        while not condition() and time.monotonic()<end:executor.spin_once(timeout_sec=.002)
        if not condition():
            print(json.dumps(bridge.status(),ensure_ascii=False,default=str) if bridge else 'no bridge')
        assert condition()
    try:
        pump(.4)
        assert not any(n in ('hc_teleop_middleware','hc_robot_vr_teleop','controller_v2_3') for n,ns in probe.get_node_names_and_namespaces())
        teleop=RobotArmTeleopNode(str(profile/'arm_teleop.yaml'),backend='v23')
        # Preserve production home policy; this fake rig already starts at home.
        executor.add_node(teleop)
        solver=module.ControllerNode(str(profile/'controller_v23.yml'));executor.add_node(solver)
        bridge=RosBridge(dict(enabled=True,domain_id=domain,legacy_control_profile=str(profile),
            intervention=dict(automatic_recording=False),command_mux=dict(enabled=True,source='motion')),
            lambda *args:None)
        bridge.start()
        control=bridge.ownership
        opened=[];control.open_segment=lambda *a:opened.append(a)
        until(lambda:bridge._boundary is not None and control.required_peers<=control.peers.keys())
        control.enable()
        until(lambda:control.input_valid and control.input_armed and bridge.control_ready())
        control.start_auto(recording_ready=False,control_ready=bridge.control_ready())
        until(lambda:control.state=='AUTO')
        pump(.3)
        assert states and set(arm_names+['leg_1','leg_2','zhi']) <= set(states[-1].name)
        assert images and bytes(images[-1].data)==b'unchanged-depth-bytes'
        assert {channel for _,channel,_,state in outputs if state=='AUTO'}=={'arms','left','right'}
        assert not solver.intervention.authorized()
        for cycle in range(3):
            frame['inputs']['right']['grip']=1.
            until(lambda:control.state=='MANUAL')
            token=control.token
            count=len(outputs)
            if cycle == 0:
                def at_pose(side, pose):
                    grip = cfg['grippers'][side]
                    return all(abs(hand_values[name]-q)<1e-5 for name,q in
                               zip(grip['finger_joint_names'],grip[pose]))
                # Closed model pose at takeover. With no trigger edge at all,
                # both hands must slew to open on their final hardware topics.
                until(lambda:at_pose('left','finger_open') and at_pose('right','finger_open'))
                frame['inputs']['right']['trigger']=1.
                until(lambda:at_pose('right','finger_closed'))
                assert at_pose('left','finger_open') and control.state=='MANUAL'
                frame['inputs']['right']['trigger']=0.
                until(lambda:at_pose('right','finger_open'))
                assert control.token==token
                assert not bridge._boundary.diagnostics['rejected:joint limit']
            frame['poses']['right']['position'][2]-=.001
            frame['inputs']['right']['trigger']+=.01
            pump(.3)
            assert any(channel=='arms' and state=='MANUAL' for _,channel,_,state in outputs[count:])
            assert solver.intervention.token==token and solver.intervention.authorized()
            assert len(teleop.intervention.grippers)==20
            frame['inputs']['right']['grip']=0.
            until(lambda:control.state=='AUTO')
            pump(.15)
        assert not opened and not control.segment_open and not control.recording_error
        assert len(control.metrics)==3
        # No Motion command endpoint or competing own publisher was introduced.
        assert len(probe.get_publishers_info_by_topic('/hc_teleop/joint_cmd'))==1
        assert not any('/control/motion/' in topic and topic.endswith(('servo_j','move_j','servo_p'))
                       for topic,_ in probe.get_topic_names_and_types())
        frame['inputs']['right']['grip']=1.
        until(lambda:control.state=='MANUAL')
        frame['tracking']['right']=False
        until(lambda:control.state=='FAULT')
        pump(.1);count=len(outputs);pump(.15)
        assert len(outputs)==count
        print('legacy ROS: three AUTO/MANUAL/AUTO cycles, arms + hands, preserved depth, no recorder/Motion, fault gate passed')
    finally:
        if bridge:
            bridge.ownership.disable();bridge.stop()
        for node in (solver,teleop,probe):
            if node:
                executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()
