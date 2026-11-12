"""Real VR adapter, relative anchor, Motion/IK and driver on an isolated fake rig."""
import json
import time
from pathlib import Path

import pytest
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tests.robot_api_fake_rig import FakeRig


@pytest.mark.parametrize('profile_name', ['six_dof', 'openarmx'])
def test_real_vr_adapter_reanchors_and_fault_recovery_stays_hold(tmp_path, profile_name):
    root = Path(__file__).resolve().parents[1]
    profile = root/('tests/fixtures/robot_api_six_dof' if profile_name == 'six_dof' else 'adapters/robots/openarmx')
    with FakeRig(profile, tmp_path/'real-vr', with_teleop=True) as rig:
        publisher = rig.node.create_publisher(String, '/vrdata', 1)
        group = rig.api.describe()['groups']['left_arm']
        program = rig.node.create_publisher(JointState, group['endpoints']['servo_j'], 1)
        frame = dict(protocol_version=3, sequence=0, vr_timestamp=0.,
            tracking=dict(head=True, left=True, right=True), poses={
                'head':dict(position=[0.,1.5,0.],quaternion=[0.,0.,0.,1.]),
                'left':dict(position=[-.2,1.3,-.4],quaternion=[0.,0.,0.,1.]),
                'right':dict(position=[.2,1.3,-.4],quaternion=[0.,0.,0.,1.])},
            inputs=dict(left=dict(grip=0.,trigger=.5), right=dict(grip=0.,trigger=.5)))
        def feed():
            frame.update(sequence=frame['sequence']+1, vr_timestamp=time.monotonic(), hc_received_ns=time.time_ns())
            publisher.publish(String(data=json.dumps(frame)))
            msg = JointState(name=group['joint_names'], position=[.005]*len(group['joint_names']))
            msg.header.stamp = rig.node.get_clock().now().to_msg()
            program.publish(msg)
        timer = rig.node.create_timer(.01, feed)
        control = rig.bridge.ownership
        opened, closed, actions = [], [], []
        control.open_segment = lambda *args: opened.append(args)
        control.close_segment = lambda *args: closed.append(args)
        rig.bridge._action_sink = actions.append
        try:
            rig.pump(.2)
            control.enable()
            rig.until(lambda: control.input_valid and control.input_armed and len(control.peers) == 2)
            control.start_auto(recording_ready=True, control_ready=rig.bridge.control_ready())
            rig.until(lambda: control.state == 'AUTO')
            rig.pump(.2)
            for _ in range(3):
                frame['inputs']['right']['grip'] = 1.
                rig.until(lambda: control.state == 'MANUAL')
                rig.pump(.15)
                assert actions and actions[-1]['intervention_id'] == control.token.intervention_id
                assert all(arm.active for arm in rig.teleop.arms.values())
                if profile_name == 'openarmx':
                    references = rig.teleop.intervention.grippers
                    assert len(references) == 2
                    for name, (side, trigger, initial, _, finger_open, finger_closed) in references.items():
                        assert abs(rig.positions[name]-initial) < .0001, 'grip must anchor at measured gripper position'
                        frame['inputs'][side]['trigger'] = trigger+(.1 if initial > (finger_open+finger_closed)/2 else -.1)
                frame['poses']['left']['position'][2] -= .001
                rig.pump(.15)
                if profile_name == 'openarmx':
                    for name, reference in references.items():
                        assert abs(rig.positions[name]-reference[2]) > .001
                        public = rig.api.joints[name]['public_name']
                        assert any(public in event['names'] for event in actions)
                frame['inputs']['right']['grip'] = 0.
                rig.until(lambda: control.state == 'AUTO')
                rig.pump(.1)
            assert len(opened) == len(closed) == 3
            frame['inputs']['right']['grip'] = 1.
            rig.until(lambda: control.state == 'MANUAL')
            frame['tracking']['right'] = False
            frame['inputs']['right']['grip'] = 0.
            rig.until(lambda: control.state == 'FAULT')
            frame['tracking']['right'] = True
            rig.pump(.2)
            assert control.state == 'FAULT'
            rig.bridge.recover_intervention()
            rig.until(lambda: control.state == 'HOLD')
            assert not rig.bridge._command_output_enabled
            rig.pump(.1)
            assert control.state == 'HOLD'
        finally:
            control.disable()
            rig.node.destroy_timer(timer)
