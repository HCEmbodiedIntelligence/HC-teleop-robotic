"""Actual profile v23 IK + VR adapter + middleware, with only fake actuators.

Opt in with HC_RUN_LEGACY_ROS_TESTS=1 on a localhost-only ROS domain >= 200.
Public ServoJ and gateway commands share the v23 actuator and ownership boundary.
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


@pytest.mark.parametrize(('profile_id', 'bridge_kind'), [
    ('x1', 'legacy_thread'), ('x1', 'isolated_process'),
    ('openarmx', 'isolated_process'), ('HC_REALMAN', 'isolated_process'),
])
def test_v23_own_topic_takeover_and_return_without_recording(monkeypatch, profile_id, bridge_kind):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState, CompressedImage
    from std_msgs.msg import Bool, Header, String
    import yaml
    from middleware.core.control_ownership import ControlToken
    from adapters.core.arm_teleop_node import RobotArmTeleopNode
    from middleware.core.ros_bridge import RosBridge
    from middleware.core.control_process import IsolatedRosBridge

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
    profile=root/'adapters/robots'/profile_id
    cfg=yaml.safe_load((profile/'arm_teleop.yaml').read_text())
    ros_settings = yaml.safe_load((profile/'middleware.yaml').read_text())['ros']
    legacy_inputs = ros_settings.get('intervention', {}).get('legacy_topics', False)
    spec=importlib.util.spec_from_file_location('hc_test_v23',root/'adapters/v23/script/control_v2_3_ros2.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    rclpy.init()
    probe=Node('legacy_test_fake_hardware')
    executor=SingleThreadedExecutor();executor.add_node(probe)
    # Check discovery before creating any of the fake actuator/input publishers.
    discovery_deadline = time.monotonic() + .4
    while time.monotonic() < discovery_deadline:
        executor.spin_once(timeout_sec=.02)
    if set(probe.get_node_names()) != {probe.get_name()}:
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()
        pytest.fail('isolated ROS domain is occupied')
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
    public_program = True
    sources, grants = [], []
    def on_permit(message):
        fields = message.frame_id.split('#')
        if len(fields) == 4 and fields[1] == 'grant':
            grants.append(ControlToken.from_key(fields[0]))
    probe.create_subscription(Header, '/control/motion/permit', on_permit, 10)
    probe.create_subscription(String, '/control/source', lambda m: sources.append(m.data),
                              QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    public_status, robot_info = [], []
    feedback_by_topic = {'/status/motion/joint_states': public_status}
    probe.create_subscription(JointState, '/status/motion/joint_states', public_status.append, 10)
    probe.create_subscription(String, '/status/robot/info', lambda m: robot_info.append(json.loads(m.data)),
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    servo = probe.create_publisher(JointState, '/control/motion/whole_body/servo_j', 1)
    servo_hands, public_hand_names, hand_goals = {}, {}, {}
    public_names = {model: f'{side}_joint_{i}' for side in ('left', 'right')
                    for i, model in enumerate(cfg['arms'][side]['joint_names'], 1)}
    public_names.update({model: f'waist_joint_{i}' for i, model in enumerate(cfg['body']['waist_joint_names'], 1)})
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
        in_state = cfg['grippers'].get('feedback_in_joint_state', False)
        state=msg({**values, **hand_values} if in_state else values,stamp);state.velocity=[0.]*len(state.name)
        state_pub.publish(state)
        # Real X1 feedback can be outside the calibrated command interval.
        # The recent final command still provides a legal continuity reference.
        if profile_id == 'x1':
            hands_pub.publish(msg({**hand_values, 'L_thumb_roll_joint':.0252},stamp))
        elif not in_state:
            names = [n for side in ('left', 'right') for n in cfg['grippers'][side]['feedback_joint_names']]
            hands_pub.publish(msg({n: hand_values[n] for n in names}, stamp))
        ready_pub.publish(Bool(data=True))
        frame.update(sequence=frame['sequence']+1,vr_timestamp=time.monotonic(),hc_received_ns=time.time_ns())
        vr_pub.publish(String(data=json.dumps(frame)))
        arm_goal = {n: float(cfg['robot']['initial_joints'][n]) + .005 for n in arm_names}
        if public_program:
            target = msg({public: arm_goal.get(model, values[model]) for model, public in public_names.items()}, stamp)
            target.header.frame_id = 'test_public_servoj'
            servo.publish(target)
        else:
            target = msg(arm_goal, stamp)
            target.header.frame_id = 'test_gateway'
            program.publish(target)
        if public_program:
            for side, publisher in servo_hands.items():
                target = msg(dict(zip(public_hand_names[side], hand_goals[side].values())), stamp)
                target.header.frame_id = 'test_public_hand_servoj'
                publisher.publish(target)
        elif legacy_inputs:
            for side, pub in program_hands.items():
                pub.publish(msg({n: hand_values[n] for n in cfg['grippers'][side]['finger_joint_names']}, stamp))
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
        teleop=RobotArmTeleopNode(str(profile/'arm_teleop.yaml'),backend='v23')
        # Preserve production home policy; this fake rig already starts at home.
        executor.add_node(teleop)
        solver=module.ControllerNode(str(profile/'controller_v23.yml'));executor.add_node(solver)
        bridge_class=IsolatedRosBridge if bridge_kind=='isolated_process' else RosBridge
        bridge=bridge_class(dict(enabled=True,domain_id=domain,legacy_control_profile=str(profile),servo_j_profile=str(profile),
            intervention=dict(automatic_recording=False,legacy_topics=legacy_inputs),command_mux=dict(enabled=True,source='motion')),
            lambda *args:None)
        bridge.start()
        control=bridge.ownership
        opened=[];control.open_segment=lambda *a:opened.append(a)
        until(lambda:bridge._boundary is not None and control.required_peers<=control.peers.keys())
        until(lambda: robot_info)
        info = robot_info[-1]
        unit = {'HC_REALMAN': 'fraction', 'openarmx': 'm', 'x1': 'rad'}[profile_id]
        for side in ('left', 'right'):
            group = info['groups'][side+'_hand']
            grip = cfg['grippers'][side]
            models = grip.get('feedback_joint_names', grip['finger_joint_names'])
            names = group['joint_names']
            assert len(names) == len(models) and group['units'] == [unit] * len(names)
            hand_status_topic = group['joint_states']
            expected_topic = ('/status/motion/joint_states' if cfg['grippers'].get('feedback_in_joint_state')
                              else '/status/motion/hand_joint_states')
            assert hand_status_topic == expected_topic
            if hand_status_topic not in feedback_by_topic:
                feedback_by_topic[hand_status_topic] = []
                probe.create_subscription(JointState, hand_status_topic, feedback_by_topic[hand_status_topic].append, 10)
            public_hand_names[side] = names
            hand_goals[side] = {model: .75*closed + .25*opened
                                for model, closed, opened in zip(grip['finger_joint_names'],
                                                               grip['finger_closed'], grip['finger_open'])
                                if model in models}
            servo_hands[side] = probe.create_publisher(JointState, group['endpoints']['servo_j'], 1)
        until(lambda: all(pub.get_subscription_count() == 1 for pub in servo_hands.values()))
        control.enable()
        until(lambda:control.input_valid and control.input_armed and bridge.control_ready())
        control.start_auto(recording_ready=False,control_ready=bridge.control_ready())
        until(lambda:control.state=='AUTO')
        pump(.3)
        assert states and set(arm_names+cfg['body']['waist_joint_names']) <= set(states[-1].name)
        until(lambda: robot_info and public_status)
        whole_body = robot_info[-1]['groups']['whole_body']
        assert set(whole_body['joint_names']) == set(public_names.values())
        assert whole_body['endpoints']['servo_j'] == servo.topic_name
        assert robot_info[-1]['endpoints']['joint_states'] == '/status/motion/joint_states'
        # DDS callbacks on these two readers may arrive in either order.
        matched_feedback = []
        def find_matching_feedback():
            observed = {(m.header.stamp.sec, m.header.stamp.nanosec): m for m in states}
            for projected in reversed(public_status):
                stamp = projected.header.stamp
                sample = observed.get((stamp.sec, stamp.nanosec))
                if sample is not None and public_names[arm_names[0]] in projected.name:
                    matched_feedback.append((projected, sample))
                    return True
            return False
        until(find_matching_feedback)
        status, sample = matched_feedback[-1]
        measured = dict(zip(sample.name, sample.position))
        public = dict(zip(status.name, status.position))
        assert all(public[name] == measured[model] for model, name in public_names.items())
        assert not set(public_names).intersection(status.name)
        assert servo.get_subscription_count() == 1
        def arm_output(marker, since=0):
            return [m for _, channel, m, _ in outputs[since:] if channel == 'arms' and m.header.frame_id == marker]
        until(lambda: arm_output('test_public_servoj'))
        command_values = dict(zip(arm_output('test_public_servoj')[-1].name, arm_output('test_public_servoj')[-1].position))
        assert set(public_names) <= command_values.keys()
        assert all(math.isfinite(command_values[model]) for model in public_names)
        assert images and bytes(images[-1].data)==b'unchanged-depth-bytes'
        def reached_hand_goal(side):
            return all(abs(hand_values[n]-v)<1e-5 for n,v in hand_goals[side].items())
        until(lambda: all(reached_hand_goal(side) for side in ('left', 'right')))
        for side, names in public_hand_names.items():
            expected = dict(zip(names, hand_goals[side].values()))
            if profile_id == 'x1' and side == 'left':
                expected[names[0]] = .0252  # actual sensor sample injected above
            def measured_hand():
                topic = info['groups'][side+'_hand']['joint_states']
                for state in reversed(feedback_by_topic[topic]):
                    positions = dict(zip(state.name, state.position))
                    if all(abs(positions.get(n, math.inf)-v) < 1e-5 for n,v in expected.items()):
                        return True
                return False
            until(measured_hand)
        assert {channel for _,channel,_,state in outputs if state=='AUTO'} == {'arms','left','right'}
        assert not solver.intervention.authorized()
        def assert_grant(source, offset, previous_epoch):
            until(lambda: sources[offset:] and sources[-1] == source and control.token in grants)
            assert control.token.source == source and control.token.epoch > previous_epoch
            return control.token.epoch
        epoch = assert_grant('motion', 0, 0)
        for cycle in range(3):
            source_offset = len(sources)
            frame['inputs']['right']['grip']=1.
            until(lambda:control.state=='MANUAL')
            epoch = assert_grant('vr', source_offset, epoch)
            token=control.token
            count=len(outputs)
            if cycle == 0:
                def at_pose(side, pose):
                    grip = cfg['grippers'][side]
                    physical = grip.get('feedback_joint_names', grip['finger_joint_names'])
                    return all(abs(hand_values[name]-q)<1e-5 for name,q in
                               zip(grip['finger_joint_names'],grip[pose]) if name in physical)
                # Closed model pose at takeover. With no trigger edge at all,
                # both hands must slew to open on their final hardware topics.
                until(lambda:at_pose('left','finger_open') and at_pose('right','finger_open'))
                frame['inputs']['right']['trigger']=1.
                until(lambda:at_pose('right','finger_closed'))
                assert at_pose('left','finger_open') and control.state=='MANUAL'
                frame['inputs']['right']['trigger']=0.
                until(lambda:at_pose('right','finger_open'))
                assert control.token==token
                assert not bridge._boundary.status().get('diagnostics', {}).get('rejected:joint limit', 0)
            frame['poses']['right']['position'][2]-=.001
            frame['inputs']['right']['trigger']+=.01
            pump(.3)
            assert any(channel=='arms' and state=='MANUAL' for _,channel,_,state in outputs[count:])
            assert not arm_output('test_public_servoj', count) and not arm_output('test_gateway', count)
            assert not any(m.header.frame_id == 'test_public_hand_servoj' for _, _, m, _ in outputs[count:])
            assert solver.intervention.token==token and solver.intervention.authorized()
            measured_hand_names = {n for side in ('left', 'right') for n in
                                  cfg['grippers'][side].get('feedback_joint_names', cfg['grippers'][side]['finger_joint_names'])}
            assert set(teleop.intervention.grippers) == measured_hand_names
            public_program = not public_program if legacy_inputs else True
            source_offset = len(sources)
            frame['inputs']['right']['grip']=0.
            until(lambda:control.state=='AUTO')
            epoch = assert_grant('motion', source_offset, epoch)
            until(lambda: arm_output('test_public_servoj' if public_program else 'test_gateway', count))
        # Independent hand feedback must never replace the arm snapshot on the client.
        expected_arms = {public_names[n] for n in arm_names}
        assert all(expected_arms.issubset(state.name) for state in public_status)
        assert not opened and not control.recording_error
        if bridge_kind=='legacy_thread':
            assert not control.segment_open
        assert len(control.metrics)==3
        # The gateway and the public API converge on one arbiter.
        assert len(probe.get_publishers_info_by_topic('/hc_teleop/joint_cmd'))==1
        assert len(probe.get_publishers_info_by_topic('/action/joint_states')) == 1
        frame['inputs']['right']['grip']=1.
        until(lambda:control.state=='MANUAL')
        frame['tracking']['right']=False
        until(lambda:control.state=='FAULT')
        pump(.1);count=len(outputs);pump(.15)
        assert len(outputs)==count
        print(f'{profile_id} {bridge_kind}: three AUTO/MANUAL/AUTO cycles with public ServoJ, hand takeover, and fault stop')
    finally:
        if bridge:
            bridge.ownership.disable();bridge.stop()
        for node in (solver,teleop,probe):
            if node:
                executor.remove_node(node);node.destroy_node()
        executor.shutdown();rclpy.shutdown()


@pytest.mark.parametrize('profile_id', ['x1', 'openarmx', 'HC_REALMAN'])
def test_public_arm_servoj_and_measured_status(profile_id):
    """An external client discovers the endpoint and uses only public joint names."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    import yaml
    from middleware.core.ros_bridge import RosBridge
    from humanoid_motion_interfaces.msg import Status

    domain = int(os.environ['ROS_DOMAIN_ID'])
    assert 200 <= domain <= 232 and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    profile = Path(__file__).resolve().parents[1] / 'adapters/robots' / profile_id
    cfg = yaml.safe_load((profile / 'arm_teleop.yaml').read_text())
    home = cfg['robot']['initial_joints']
    left, right = (cfg['arms'][side]['joint_names'] for side in ('left', 'right'))
    rclpy.init()
    probe = Node('v23_public_servoj_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    bridge = None

    def wait_for(predicate, send=None, timeout=6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if send:
                send()
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail(str(bridge.status() if bridge else probe.get_node_names()))

    try:
        deadline = time.monotonic() + .4
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert set(probe.get_node_names()) == {probe.get_name()}
        infos, feedback, actions, health = [], [], [], []
        probe.create_subscription(String, '/status/robot/info', lambda msg: infos.append(json.loads(msg.data)),
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        probe.create_subscription(JointState, '/action/joint_states', actions.append, 10)
        states = probe.create_publisher(JointState, '/observation/joint_states', 1)
        ready = probe.create_publisher(Bool, '/status/hardware/ready', 1)
        probe.create_subscription(Status, '/status/motion/status', health.append,
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        send_feedback = True
        hardware_ready = True
        joints = JointState(name=list(home), position=list(home.values()), velocity=[0.] * len(home))

        def sample():
            joints.header.stamp = probe.get_clock().now().to_msg()
            if send_feedback:
                states.publish(joints)
            ready.publish(Bool(data=hardware_ready))

        probe.create_timer(.02, sample)
        bridge = RosBridge(dict(domain_id=domain, _control_actor_mode=True, legacy_control_profile=str(profile),
                                servo_j_profile=str(profile), command_mux=dict(enabled=True, source='motion')),
                           lambda *_: None)
        bridge.start()
        wait_for(lambda: infos and health and health[-1].code == Status.OK and bridge.control_ready())
        info = infos[-1]
        assert info['profile_id'] == profile_id
        public_state = info['endpoints']['joint_states']
        assert public_state == '/status/motion/joint_states'
        probe.create_subscription(JointState, public_state, feedback.append, qos_profile_sensor_data)
        wait_for(lambda: feedback)
        status = dict(zip(feedback[-1].name, feedback[-1].position))
        group = info['groups']['left_arm']
        public = group['joint_names']
        assert len(public) == len(left)
        assert [status[name] for name in public] == pytest.approx([home[model] for model in left])
        assert set(left).isdisjoint(status)
        assert states.get_subscription_count() == 1

        servo = probe.create_publisher(JointState, group['endpoints']['servo_j'], 1)
        wait_for(lambda: servo.get_subscription_count() == 1)
        bridge.set_command_output_enabled(True)
        target = JointState(name=public, position=[status[n] for n in public])
        target.position[0] += .005

        def send():
            target.header.stamp = probe.get_clock().now().to_msg()
            servo.publish(target)

        wait_for(lambda: any(abs(dict(zip(m.name, m.position)).get(left[0], math.inf)
                                 - target.position[0]) < 1e-6 for m in actions), send)
        command = dict(zip(actions[-1].name, actions[-1].position))
        assert [command[n] for n in left] == pytest.approx(list(target.position))
        if profile_id == 'HC_REALMAN':
            # The vendor bridge accepts a dual-arm vector on every tick.
            assert [command[n] for n in right] == pytest.approx([home[n] for n in right])
        assert len(probe.get_publishers_info_by_topic('/action/joint_states')) == 1
        right_group = info['groups']['right_arm']
        right_servo = probe.create_publisher(JointState, right_group['endpoints']['servo_j'], 1)
        wait_for(lambda: right_servo.get_subscription_count() == 1)
        right_target = JointState(name=right_group['joint_names'], position=[home[n] for n in right])
        right_target.position[0] += .003
        target.position[0] += .002
        paired_stamp = probe.get_clock().now().to_msg()
        target.header.stamp = right_target.header.stamp = paired_stamp
        servo.publish(target)
        right_servo.publish(right_target)

        def paired_outputs():
            return [dict(zip(m.name, m.position)) for m in actions if m.header.stamp == paired_stamp]

        def received_both():
            frames = paired_outputs()
            return (any(f.get(left[0]) == target.position[0] for f in frames) and
                    any(f.get(right[0]) == right_target.position[0] for f in frames))

        wait_for(received_both)
        if profile_id == 'HC_REALMAN':
            merged = paired_outputs()[-1]
            assert [merged[n] for n in left] == pytest.approx(list(target.position))
            assert [merged[n] for n in right] == pytest.approx(list(right_target.position))
        send_feedback = False
        wait_for(lambda: health[-1].code == Status.STATE_STALE)
        hardware_ready = False
        wait_for(lambda: health[-1].code == Status.DRIVER_FAULT)
    finally:
        if bridge:
            bridge.stop()
        executor.shutdown()
        probe.destroy_node()
        rclpy.shutdown()
