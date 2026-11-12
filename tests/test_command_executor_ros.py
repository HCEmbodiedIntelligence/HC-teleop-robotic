"""Bounded real-ROS test with fake actuators, localhost domain 231 only."""
import json
import os
from pathlib import Path
import threading
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('HC_RUN_LEGACY_ROS_TESTS') != '1',
                                reason='explicit isolated ROS test only')


def _fake_peers(profile, connection):
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    profile=Path(profile)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    import yaml
    from middleware.core.ros_bridge import RosBridge
    from middleware.core.config import ConfigStore
    from middleware.core.dataset_topics import load_joint_groups
    from middleware.core.camera_metadata import load_camera_metadata

    config = yaml.safe_load((profile/'arm_teleop.yaml').read_text())
    arms = [n for side in ('right','left') for n in config['arms'][side]['joint_names']]
    names = arms + config['body']['waist_joint_names']
    values = [float(config['robot']['initial_joints'][n]) for n in names]
    hands = [n for side in ('right','left') for n in config['grippers'][side]['finger_joint_names']]
    hand_values = [q for side in ('right','left') for q in config['grippers'][side]['finger_open']]
    rclpy.init()
    probe = rclpy.create_node('command_queue_fake_peers',enable_rosout=False,start_parameter_services=False)
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    state_pub = probe.create_publisher(JointState,'/observation/joint_states',10)
    hand_pub = probe.create_publisher(JointState,'/observation/hands/joint_states',10)
    ready_pub = probe.create_publisher(Bool,'/status/hardware/ready',10)
    input_pub = probe.create_publisher(String,'/events/teleop/clutch',100)
    ack_pub = probe.create_publisher(String,'/status/control/ack',100)
    arm_pub = probe.create_publisher(JointState,'/internal/control/joint_targets',10)
    finger_pub = probe.create_publisher(JointState,'/internal/control/joint_targets',10)
    permit = {'key':'','phase':''}
    def receive_permit(message):
        pieces = message.frame_id.split('#')
        if len(pieces) == 4: permit.update(key=pieces[0],phase=pieces[1])
    probe.create_subscription(Header,'/control/motion/permit',receive_permit,1)
    phase = ''
    gripped = False
    sequence = 0
    token_key = ''
    sent = {}
    counts = {}
    received = {}
    projected = {}
    observation_sent = {}
    observation_counts = {}
    callback_nodes = {}
    def output(message, channel):
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        item = sent.get(stamp)
        if item:
            received.setdefault((item,channel),[]).append((time.time_ns()-stamp)/1e6)
    for topic,channel in [('/action/joint_states','arms'),('/action/right_hand/joint_states','right'),('/action/left_hand/joint_states','left')]:
        probe.create_subscription(JointState,topic,lambda m,c=channel:output(m,c),qos_profile_sensor_data)
    def projection(message, category, side):
        stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        source_phase=(observation_sent if category=='observation' else sent).get(stamp)
        if not source_phase:return
        expected=config['arms'][side]['joint_names']
        assert list(message.name)==expected
        assert list(message.position)==[float(config['robot']['initial_joints'][n]) for n in expected]
        projected.setdefault((source_phase,category+'/'+side),[]).append((time.time_ns()-stamp)/1e6)
    for category in ['observation','action']:
        for side in ['right','left']:
            probe.create_subscription(JointState,f'/{category}/{side}_arm/joint_states',
                lambda m,c=category,s=side:projection(m,c,s),qos_profile_sensor_data)
    def msg(joints, positions, stamp):
        message = JointState(name=joints,position=positions)
        message.header.stamp = stamp
        return message
    def feedback():
        stamp = probe.get_clock().now().to_msg()
        if phase:
            observation_sent[stamp.sec*10**9+stamp.nanosec]=phase
            observation_counts[phase]=observation_counts.get(phase,0)+1
        message = msg(names,values,stamp);message.velocity=[0.]*len(names)
        state_pub.publish(message)
        ready_pub.publish(Bool(data=True))
    def feed_hands():
        hand_pub.publish(msg(hands,hand_values,probe.get_clock().now().to_msg()))
    def semantic():
        nonlocal sequence
        sequence += 1
        input_pub.publish(String(data=json.dumps(dict(device_instance='test-vr',sequence=sequence,
            stamp_ns=time.time_ns(),valid=True,gripped=gripped,pause_requested=False,resume_requested=False))))
    def acknowledgements():
        for participant in ['teleop','solver']:
            ack_pub.publish(String(data=json.dumps(dict(participant=participant,instance='test-'+participant,
                stamp_ns=time.time_ns(),token=permit['key'],phase=permit['phase'],success=True))))
    def actions():
        if not phase: return
        stamp = probe.get_clock().now().to_msg()
        ns = stamp.sec*10**9+stamp.nanosec
        sent[ns] = phase
        counts[phase] = counts.get(phase,0)+1
        arm = msg(arms,values[:len(arms)],stamp);arm.header.frame_id=token_key+'#'
        hand = msg(hands,hand_values,stamp);hand.header.frame_id=token_key+'#'
        arm_pub.publish(arm);finger_pub.publish(hand)
    probe.create_timer(.01,feedback)
    probe.create_timer(.05,feed_hands)
    probe.create_timer(1/60,semantic)
    probe.create_timer(.05,acknowledgements)
    probe.create_timer(.01,actions)
    deadline=time.monotonic()+25
    try:
        while time.monotonic()<deadline:
            while connection.poll():
                command=connection.recv()
                if 'phase' in command:phase=command['phase']
                if 'gripped' in command:gripped=command['gripped']
                if 'token_key' in command:token_key=command['token_key']
                if command.get('stop'):return
            executor.spin_once(timeout_sec=.002)
    finally:
        executor.shutdown();probe.destroy_node();rclpy.shutdown()
        connection.send({'counts':counts,'received':received,'projected':projected,'observation_counts':observation_counts})
        connection.close()


def test_commands_keep_draining_when_dataset_executor_is_busy(monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID') == '231'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert not os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    assert not os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE')
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Header, String
    import yaml
    from middleware.core.ros_bridge import RosBridge
    from middleware.core.config import ConfigStore
    from middleware.core.dataset_topics import load_joint_groups
    from middleware.core.camera_metadata import load_camera_metadata

    root = Path(__file__).resolve().parents[1]
    profile = root/'adapters/robots/x1'
    cfg = ConfigStore(root/'middleware/config.yaml').load()
    ros = cfg['ros']
    ros.update(domain_id=231, node_name='isolated_command_queue', legacy_control_profile=str(profile))
    ros['intervention']['automatic_recording'] = False
    dataset = ros.setdefault('dataset_topics', {})
    if dataset.get('enabled'):
        dataset['joint_groups'] = load_joint_groups(cfg, root/'middleware')
        dataset['camera_metadata'] = load_camera_metadata(cfg, root/'middleware')
    config = yaml.safe_load((profile/'arm_teleop.yaml').read_text())
    arms = [n for side in ('right','left') for n in config['arms'][side]['joint_names']]
    names = arms + config['body']['waist_joint_names']
    values = [float(config['robot']['initial_joints'][n]) for n in names]
    hands = [n for side in ('right','left') for n in config['grippers'][side]['finger_joint_names']]
    hand_values = [q for side in ('right','left') for q in config['grippers'][side]['finger_open']]
    main_node = {}
    busy = threading.Event()
    original_create = rclpy.create_node
    def create(name, *args, **kwargs):
        node = original_create(name, *args, **kwargs)
        if name == 'isolated_command_queue':
            main_node['node'] = node
            # Represents unrelated dataset/metadata work waiting in the main
            # executor. Sleep releases the GIL and adds no compute stress.
            node.create_timer(.05, lambda: time.sleep(.04) if busy.is_set() else None)
        return node
    monkeypatch.setattr(rclpy, 'create_node', create)
    profiler = None
    if os.environ.get('HC_QUEUE_PROFILE') == '1':
        import cProfile
        profiler = cProfile.Profile(timer=time.thread_time)
        original_timed = RosBridge._timed_callback
        def timed(instance, name, callback):
            if name == 'owned_target':
                original = callback
                def traced(*args, **kwargs):
                    profiler.enable()
                    try:return original(*args, **kwargs)
                    finally:profiler.disable()
                callback = traced
            return original_timed(instance, name, callback)
        monkeypatch.setattr(RosBridge, '_timed_callback', timed)
    import multiprocessing
    context=multiprocessing.get_context('spawn')
    connection, child_connection=context.Pipe()
    child=context.Process(target=_fake_peers,args=(str(profile),child_connection))
    child.start();child_connection.close()
    bridge = RosBridge(ros,lambda *args:None)
    def until(condition, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            if condition():return
            time.sleep(.005)
        raise AssertionError(str(bridge.ownership.snapshot()))
    try:
        bridge.start()
        control = bridge.ownership
        until(lambda:bridge._boundary is not None and control.required_peers<=control.peers.keys())
        control.enable()
        until(lambda:control.input_valid and control.input_armed and bridge.control_ready())
        control.start_auto(recording_ready=False,control_ready=bridge.control_ready())
        until(lambda:control.state=='AUTO')
        connection.send({'gripped':True})
        until(lambda:control.state=='MANUAL')
        connection.send({'token_key':control.token.key(),'phase':'normal'});time.sleep(3)
        busy.set();connection.send({'phase':'main_busy'});time.sleep(3)
        connection.send({'phase':''});busy.clear();time.sleep(.15)
        assert control.state=='MANUAL',control.snapshot()
        control.hold()
        connection.send({'phase':'revoked'});time.sleep(.15)
        connection.send({'stop':True})
        assert connection.poll(5), 'fake peer result timed out'
        result=connection.recv();child.join(3)
        counts,received=result['counts'],result['received']
        callback_nodes={}
        for topic in ['/internal/control/joint_targets','/control/vr/joint_states','/control/vr/right_hand/joint_states',
                      '/hc_teleop/joint_cmd','/hc_teleop/joint_cmd_finger_left']:
            callback_nodes[topic]=[x.node_name for x in main_node['node'].get_subscriptions_info_by_topic(topic) if x.node_name.startswith('isolated_command_queue')]
        metrics={}
        for p in ['normal','main_busy']:
            for channel in ['arms','right','left']:
                ages=sorted(received.get((p,channel),[]))
                metrics[p+'/'+channel]={'sent':counts[p],'received':len(ages),'ratio':len(ages)/counts[p],
                    'p95_ms':ages[int((len(ages)-1)*.95)] if ages else None,'max_ms':max(ages,default=None)}
        dataset_metrics={}
        for p in ['normal','main_busy']:
            for channel in ['observation/right','observation/left','action/right','action/left']:
                ages=sorted(result['projected'].get((p,channel),[]))
                produced=result['observation_counts'][p] if channel.startswith('observation/') else counts[p]
                dataset_metrics[p+'/'+channel]={'sent':produced,'received':len(ages),'ratio':len(ages)/produced,
                    'p95_ms':ages[int((len(ages)-1)*.95)] if ages else None}
        print('COMMAND_QUEUE_RESULT '+json.dumps({'metrics':metrics,'dataset_metrics':dataset_metrics,'subscription_nodes':callback_nodes}),flush=True)
        if profiler is not None:
            import io, pstats
            stream=io.StringIO()
            pstats.Stats(profiler,stream=stream).sort_stats('tottime').print_stats(22)
            print('COMMAND_QUEUE_PROFILE\n'+stream.getvalue(),flush=True)
        assert not any(received.get(('revoked',channel)) for channel in ['arms','right','left'])
        for value in [*metrics.values(),*dataset_metrics.values()]:
            assert value['ratio']>=.95,metrics
            assert value['p95_ms']<25,metrics
        hand_topics = {'/control/vr/right_hand/joint_states', '/hc_teleop/joint_cmd_finger_left'}
        assert all(nodes == ['isolated_command_queue_hands' if topic in hand_topics
                             else 'isolated_command_queue_commands']
                   for topic, nodes in callback_nodes.items()), callback_nodes
    finally:
        busy.clear()
        dataset_pid=bridge.status().get('dataset_process',{}).get('pid')
        bridge.stop()
        assert not bridge._thread.is_alive(), 'bridge shutdown did not finish'
        if dataset_pid is not None:
            assert not Path('/proc',str(dataset_pid)).exists(), 'projection process outlived bridge'
        if child.is_alive():
            connection.send({'stop':True})
            if connection.poll(3):connection.recv()
            child.join(3)
        if child.is_alive():child.terminate();child.join(3)
        connection.close()
