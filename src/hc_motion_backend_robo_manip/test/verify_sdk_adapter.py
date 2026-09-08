"""Opt-in SDK/ROS regression probe; no hardware driver or command publisher is started."""
import os, sys, time, tempfile, subprocess, signal, copy
from pathlib import Path
os.environ.setdefault('ROS_DOMAIN_ID', '187')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import yaml, rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from hc_teleop_interfaces.msg import CartesianTargetArray, CartesianTarget, CartesianStateArray, JointCommandCandidate
root = Path(__file__).resolve().parents[3]
robot = sys.argv[1]
profile = yaml.safe_load((root / f'src/hc_robot_{robot}/config/profile.yaml').read_text())
arms = [c for c in profile['components'] if c['kind'] == 'arm']
params = {'urdf_path': str(root / f"src/hc_robot_{robot}/urdf/{('x1_robot' if robot == 'x1' else 'openarmx_robot')}.urdf"), 'sdk_config_path': str(root / f'src/hc_robot_{robot}/config/motion/robo_manip.yaml'), 'group_names': [a['id'] for a in arms], 'feedback_timeout_sec': 0.1, 'servo_lease_ms': 100, 'nominal_rate_hz': 100.0}
for arm in arms:
    for field, value in [('joint_names', arm['joint_names']), ('base_frame', arm['frames']['base']), ('tip_frame', arm['frames']['tip'])]:
        params[f"{arm['id']}.{field}"] = value
with tempfile.TemporaryDirectory() as temp:
    config = Path(temp) / 'params.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    log = open(Path(temp) / 'sdk.log', 'w+')
    process = subprocess.Popen([str(root / 'build/hc_motion_backend_robo_manip/robo_manip_backend_node'), '--ros-args', '--params-file', str(config), '-r', '__ns:=/pipeline_smoke'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('probe', namespace='/pipeline_smoke')
    feedback_pub = node.create_publisher(JointState, 'state/joints', qos_profile_sensor_data)
    target_pub = node.create_publisher(CartesianTargetArray, 'motion/backend/cartesian_targets', qos_profile_sensor_data)
    poses = {}
    candidates = []
    node.create_subscription(CartesianStateArray, 'state/cartesian', lambda m: poses.update({x.group_name: x for x in m.states if x.valid}), qos_profile_sensor_data)
    node.create_subscription(JointCommandCandidate, 'motion/backend/joint_candidate', lambda m: candidates.append(m), qos_profile_sensor_data)
    feedback = JointState()
    feedback.name = [j for arm in arms for j in arm['joint_names']]
    initial = profile.get('simulation', {}).get('initial_positions', {})
    fallback = yaml.safe_load(Path(params['sdk_config_path']).read_text())['execution']['initial_state']['joint_groups']
    feedback.position = [initial.get(j, fallback[arm['id']][i]) for arm in arms for i, j in enumerate(arm['joint_names'])]
    feedback.velocity = [0.0] * len(feedback.name)
    sequence = 0

    def run(seconds, targets=False, send_feedback=True, session='s1'):
        global sequence
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if process.poll() is not None:
                raise RuntimeError('SDK exited')
            if send_feedback:
                feedback.header.stamp = node.get_clock().now().to_msg()
                feedback_pub.publish(feedback)
            if targets and len(poses) == 2:
                msg = CartesianTargetArray()
                msg.source_id = 'probe'
                msg.session_id = session
                sequence += 1
                msg.sequence = sequence
                msg.valid_until = (node.get_clock().now() + Duration(seconds=0.1)).to_msg()
                for arm in arms:
                    t = CartesianTarget()
                    t.group_name = arm['id']
                    t.reference_frame = arm['frames']['base']
                    t.tip_frame = arm['frames']['tip']
                    t.pose = copy.deepcopy(poses[arm['id']].pose)
                    t.pose_mode = CartesianTarget.FULL_POSE
                    msg.targets.append(t)
                target_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.005)
    try:
        startup = time.monotonic()
        while len(poses) < 2 and time.monotonic() - startup < 25:
            run(0.2)
        assert len(poses) == 2, 'no measured FK'
        run(0.6, True)
        assert {m.group_name for m in candidates} == {a['id'] for a in arms}, 'no dual arm output'
        initial_count = len(candidates)
        run(0.3)
        quiet = len(candidates)
        run(0.15)
        assert len(candidates) == quiet, 'output continued after lease expiry'
        run(0.4, True, session='s2')
        assert len(candidates) > quiet, 'session rollover failed'
        run(0.3, True, False)
        stale_count = len(candidates)
        run(0.15, True, False)
        assert len(candidates) == stale_count, 'stale feedback still emits commands'
        run(0.4, True)
        assert len(candidates) > stale_count, 'feedback recovery failed'
        print(robot, 'PASS', len(candidates), 'candidates; expiry, identity rollover, stale feedback and recovery')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.seek(0)
        logs = log.read()
        Path(f'/tmp/hc_pipeline_{robot}.log').write_text(logs)
