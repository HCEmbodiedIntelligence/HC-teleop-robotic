"""X1 recording format and coordinate compatibility."""
from pathlib import Path


# Historical X1 LeRobot body order. Incoming JointState arrays use msg.name.
X1_BODY_JOINTS = tuple(f'Joint{i}_{side}' for side in ('R', 'L') for i in range(1, 8)) + (
    'leg_1', 'leg_2', 'zhi',
)
X1_JOINT_CONTRACT_KEY = 'x1_joint_contract'


def native_joint_contract(teleop):
    """Describe ordinary v23 capture once, independently of message arrival order."""
    action = '/action/joint_states'
    state = '/observation/joint_states'
    grippers = teleop['grippers']
    simulated = {}
    topics = {
        state: {'source': 'measured_state'},
        action: {'source': 'pre_smoother_target_or_feedback_hold',
                 'provenance_topic': '/events/recording/action_provenance',
                 'position_overrides': simulated},
        '/internal/recording/published_action': {
            'source': 'published_pre_smoother_target', 'wire_copy_of': action},
        '/control/joint_cmd': {'source': 'post_smoother_command'},
        '/control/solver/joint_targets': {'source': 'ik_target_before_arbitration'},
        '/control/vr/joint_states': {'source': 'vr_target_before_arbitration',
                                     'position_overrides': simulated},
        '/observation/hands/joint_states': {'source': 'measured_finger_state'},
    }
    for group, names in (('right_arm', teleop['arms']['right']['joint_names']),
                         ('left_arm', teleop['arms']['left']['joint_names']),
                         ('waist', teleop['body']['waist_joint_names'])):
        for source in (state, action):
            topic = source.replace('/joint_states', f'/{group}/joint_states')
            topics[topic] = {'live_projection_of': source, 'model_joint_order': names}
    for side in ('right', 'left'):
        hand = grippers[side]
        if grippers.get('include_sim_joints', True) and hand.get('sim_joint'):
            simulated[hand['sim_joint']] = {
                'quantity': 'simulated_gripper_angle', 'unit': 'rad',
                'open': 0.0, 'closed': hand['sim_closed_position'],
            }
        spec = {'source': 'finger_target', 'physical_joints': hand['finger_joint_names']}
        if hand.get('binary_state_joint'):
            spec['position_overrides'] = {
                hand['binary_state_joint']: {'quantity': 'grip_state', 'unit': '1',
                                             'open': 0, 'closed': 1}}
        topics[f'/action/{side}_hand/joint_states'] = spec
    return {
        'schema': 'hc-x1-native-joints/v1',
        'message_type': 'sensor_msgs/msg/JointState',
        'name_domain': 'model_name',
        'axis_key': ['topic', 'name'],
        'axis_index': 'per_message_name',
        'header_stamp': {'clock': 'ROS', 'fields': ['sec', 'nanosec']},
        'position': {'quantity': 'absolute_joint_position', 'unit': 'rad',
                     'unavailable': ['absent_name', 'NaN']},
        'velocity': {'unit': 'rad/s', 'empty': 'unavailable'},
        'effort': {'unit': 'N*m', 'empty': 'unavailable'},
        'body_training_order': X1_BODY_JOINTS,
        'body_servo_j_mapping': 'robot_api_info.joints',
        'hand_servo_j_mapping': 'robot_api_info.groups',
        'topics': topics,
    }


def is_x1(config):
    return Path(config.get('legacy_control_profile') or '').name == 'x1'


def ordinary_x1_capture(metadata):
    return metadata.get('profile_id') == 'x1' and not metadata.get('capture_mode')
