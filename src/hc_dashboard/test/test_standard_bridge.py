from types import SimpleNamespace
from hc_dashboard.ros_bridge import DashboardRosBridge
from hc_dashboard.state import DashboardModel
from hc_teleop_interfaces.msg import CartesianStateArray, CartesianState, JointCommand


def test_invalid_feedback_is_not_exported_as_pose():
    calls = []
    bridge = SimpleNamespace(model=DashboardModel('x1'), _publish_pose=lambda *args: calls.append(args), _publish_pose_array=lambda *args: None)
    message = CartesianStateArray()
    for name, valid in [('left_arm', True), ('right_arm', False)]:
        state = CartesianState()
        state.group_name = name
        state.reference_frame = name + '_base'
        state.valid = valid
        message.states.append(state)
    DashboardRosBridge._on_cartesian_state(bridge, message)
    assert len(calls) == 1
    assert calls[0][1:3] == ('left_arm', 'left_arm_base')
    assert bridge.model.snapshot()['cartesian_feedback']['groups'][1]['valid'] is False


def test_standard_command_batch_preserves_order_and_stamp():
    published = []
    bridge = SimpleNamespace(_joint_batches={'commands': {}},
        _group_order=['right_arm', 'left_arm'], _joint_order={'right_arm': ['r1'], 'left_arm': ['l1']},
        _standard_commands=SimpleNamespace(publish=published.append))
    for group, name in [('left_arm', 'l1'), ('right_arm', 'r1')]:
        message = JointCommand()
        message.group_name = group
        message.header.stamp.sec = 12
        message.command.header.stamp.sec = 3
        message.command.name = [name]
        message.command.position = [0.5]
        DashboardRosBridge._publish_joint_batch(bridge, 'commands', message)
    assert len(published) == 1
    assert published[0].header.stamp.sec == 12
    assert published[0].name == ['r1', 'l1']
    assert list(published[0].position) == [0.5, 0.5]
    assert not published[0].velocity


def test_pose_array_order_and_task_frames():
    published = []
    model = DashboardModel('x1')
    bridge = SimpleNamespace(_group_order=['right_arm', 'left_arm'], model=model,
        _pose_arrays={'controller_target_ee_poses': SimpleNamespace(publish=published.append)})
    message = CartesianStateArray()
    for name, x in [('left_arm', 1.), ('right_arm', 2.)]:
        state = CartesianState()
        state.group_name = name
        state.reference_frame = name + '_base'
        state.valid = True
        state.pose.position.x = x
        message.states.append(state)
    DashboardRosBridge._publish_pose_array(bridge, 'controller_target_ee_poses', message.header, message.states, local=True)
    assert published[0].header.frame_id == 'generic_task_bases'
    assert [p.position.x for p in published[0].poses] == [2., 1.]


def test_feedback_reorders_all_arrays_without_fabricating_values():
    from sensor_msgs.msg import JointState
    published = []
    bridge = SimpleNamespace(model=DashboardModel('x1'), _feedback_order=['r1', 'l1'],
        _standard_feedback=SimpleNamespace(publish=published.append))
    message = JointState()
    message.header.stamp.sec = 7
    message.name = ['l1', 'r1']
    message.position = [1., 2.]
    message.velocity = [3., 4.]
    DashboardRosBridge._on_joints(bridge, message)
    assert published[0].name == ['r1', 'l1']
    assert list(published[0].position) == [2., 1.]
    assert list(published[0].velocity) == [4., 3.]
    assert not published[0].effort
    assert published[0].header.stamp.sec == 7
