"""Stage diagnostics use mock callbacks without starting any ROS executor."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip('sensor_msgs.msg')
from sensor_msgs.msg import JointState
import adapters.core.arm_teleop_node as teleop


def command(value=.2):
    message = JointState(name=['arm'], position=[value])
    message.header.stamp.sec = 1
    message.header.frame_id = 'original-source'
    return message


def node_for():
    return SimpleNamespace(generic_command_pipeline=teleop._generic_pipeline_metrics(),
        intervention=None, control_source='vr', source_feedback_pending=False, backend='v23',
        enabled=True, hardware_ready=True, homing=False, generic_command_stamp=0.,
        generic_command_count=0, generic_aux_command={},
        solver_rearm=SimpleNamespace(blocked=False),
        arms={'right': SimpleNamespace(active=True, joint_names=['arm']),
              'left': SimpleNamespace(active=False, joint_names=[])},
        body=SimpleNamespace(active=False, joint_names=[]), last_command={}, joint_state={}, initial_joints={},
        _monotonic=lambda: 2., _ensure_vr_feedback=Mock(return_value=True),
        _collision_command_allowed=Mock(return_value=True), command_pub=SimpleNamespace(publish=Mock()),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_010_000_000)))


@pytest.mark.parametrize('reason,fields', [
    ('intervention_active', {'intervention': SimpleNamespace(active=True)}),
    ('source_not_vr', {'control_source': 'motion'}),
    ('source_feedback_pending', {'source_feedback_pending': True}),
    ('motion_backend', {'backend': 'motion'}),
    ('disabled', {'enabled': False}),
    ('hardware_not_ready', {'hardware_ready': False}),
    ('homing', {'homing': True}),
    ('stale_feedback', {'_ensure_vr_feedback': Mock(return_value=False)}),
    ('collision', {'_collision_command_allowed': Mock(return_value=False)}),
])
def test_early_return_is_counted_without_changing_gate(reason, fields):
    node = node_for()
    vars(node).update(fields)
    assert teleop.RobotArmTeleopNode._generic_command_callback(node, command()) is None
    stats = node.generic_command_pipeline
    assert stats['received_total'] == 1
    assert stats['rejected'][reason] == 1
    assert sum(stats['rejected'].values()) == 1
    assert stats['published_total'] == 0
    assert stats['callback']['samples'] == 1
    node.command_pub.publish.assert_not_called()


@pytest.mark.parametrize('invalid,reason', [
    (JointState(name=['arm'], position=[]), 'length_mismatch'),
    (command(float('nan')), 'nonfinite_position')])
def test_malformed_messages_keep_existing_rejection_and_legacy_counter(invalid, reason):
    node = node_for()
    teleop.RobotArmTeleopNode._generic_command_callback(node, invalid)
    assert node.generic_command_pipeline['received_total'] == 1
    assert node.generic_command_pipeline['rejected'][reason] == 1
    assert node.generic_command_count == 0


def test_success_keeps_header_and_records_call_latency_cpu_and_gap(monkeypatch):
    node = node_for()
    node.get_clock = Mock(side_effect=AssertionError('diagnostics must not read native ROS clock'))
    monkeypatch.setattr(teleop.time, 'time_ns', lambda: 1_010_000_000)
    instants = iter([(1_000_000, 100_000), (2_000_000, 200_000), (5_000_000, 700_000), (6_000_000, 800_000),
                     (40_000_000, 1_100_000), (47_000_000, 1_200_000), (49_000_000, 1_500_000), (50_000_000, 1_600_000)])
    monkeypatch.setattr(teleop, '_measure_clocks', lambda: next(instants))
    message = command()
    for _ in range(2):
        teleop.RobotArmTeleopNode._generic_command_callback(node, message)
    stats = node.generic_command_pipeline
    assert stats['received_total'] == stats['published_total'] == node.generic_command_count == 2
    assert stats['last_receive_source_age_ms'] == 10.
    assert stats['last_receive_source_age_comparable'] is True
    assert stats['receive_source_age_basis'] == 'wall_receive_minus_original_header_diagnostic_only'
    assert stats['last_published_source_stamp_ns'] == 1_000_000_000
    assert stats['max_publication_gap_ms'] == 45.
    assert stats['publish']['max_wall_ms'] == 3.
    assert stats['publish']['max_cpu_ms'] == .5
    assert stats['callback']['max_wall_ms'] == 10.
    for call in node.command_pub.publish.call_args_list:
        assert call.args[0].header == message.header
        assert call.args[0].name == ['arm']
        assert list(call.args[0].position) == [.2]


def test_simulated_epoch_is_not_reported_as_wall_clock_latency(monkeypatch):
    node = node_for()
    node.get_clock = Mock(side_effect=AssertionError('diagnostics must not read native ROS clock'))
    monkeypatch.setattr(teleop.time, 'time_ns', lambda: 1_794_820_000_000_000_000)
    teleop.RobotArmTeleopNode._generic_command_callback(node, command())
    stats = node.generic_command_pipeline
    assert stats['last_receive_source_age_ms'] is None
    assert stats['last_receive_source_age_comparable'] is False
    assert stats['source_age_incomparable_total'] == 1
    assert stats['published_total'] == 1


def test_failed_publish_is_not_counted_as_success_and_still_raises():
    node = node_for()
    node.command_pub.publish.side_effect = RuntimeError('DDS failed')
    with pytest.raises(RuntimeError, match='DDS failed'):
        teleop.RobotArmTeleopNode._generic_command_callback(node, command())
    stats = node.generic_command_pipeline
    assert stats['received_total'] == stats['publish_errors'] == stats['callback_errors'] == 1
    assert stats['published_total'] == 0
    assert stats['last_published_monotonic_ns'] == 0
    assert not node.last_command


def test_status_snapshot_is_json_serializable_and_recent_history_is_bounded(monkeypatch):
    stats = teleop._generic_pipeline_metrics()
    for index in range(1024):
        stats['_publication_gaps'].append((index*10_000_000, float(index)))
        stats['_source_ages'].append((index*10_000_000, float(index)))
    assert len(stats['_publication_gaps']) == len(stats['_source_ages']) == 256
    monkeypatch.setattr(teleop.time, 'monotonic_ns', lambda: 11_000_000_000)
    snapshot = teleop._generic_pipeline_snapshot(stats)
    assert snapshot['max_publication_gap_ms_recent'] == 1023.
    assert not any(key.startswith('_') for key in snapshot)
    json.dumps(snapshot)
    monkeypatch.setattr(teleop.time, 'monotonic_ns', lambda: 14_000_000_000)
    assert teleop._generic_pipeline_snapshot(stats)['max_publication_gap_ms_recent'] is None


def test_tick_samples_one_in_sixteen_and_splits_cpu_wall_without_extra_work(monkeypatch):
    node = SimpleNamespace(control_tick_metrics=dict(ticks=0, sampled_ticks=0, sample_every_ticks=16,
        stages={key: teleop._timing_metric() for key in ('sync_model', 'actual_head_fk', 'remainder', 'total')}),
        _control_tick_sample=None, _monotonic=lambda: 1., joint_state_stamp=1.,
        control={'joint_state_timeout': .5}, _command_seed=Mock(return_value={}),
        _sync_model=Mock(), _publish_actual_poses=Mock(), solver_rearm=SimpleNamespace(blocked=False),
        intervention=None, control_source='motion', _publish_status=Mock())
    clocks = Mock(side_effect=[(0, 0), (1_000_000, 500_000), (3_000_000, 1_500_000),
                              (4_000_000, 2_000_000), (9_000_000, 4_000_000), (12_000_000, 5_000_000)])
    monkeypatch.setattr(teleop, '_measure_clocks', clocks)
    for _ in range(16):
        teleop.RobotArmTeleopNode._control_tick(node)
    metrics = node.control_tick_metrics
    assert metrics['ticks'] == 16 and metrics['sampled_ticks'] == 1
    assert clocks.call_count == 6
    assert node._sync_model.call_count == node._publish_actual_poses.call_count == 16
    for stage, wall, cpu in [('sync_model', 2., 1.), ('actual_head_fk', 5., 2.), ('remainder', 5., 2.), ('total', 12., 5.)]:
        assert metrics['stages'][stage]['samples'] == 1
        assert metrics['stages'][stage]['last_wall_ms'] == wall
        assert metrics['stages'][stage]['last_cpu_ms'] == cpu
    assert node._control_tick_sample is None


def test_command_node_isolates_forwarding_and_concurrent_aux_snapshot(monkeypatch):
    import threading
    from pathlib import Path

    rclpy = pytest.importorskip('rclpy')
    root = Path(__file__).resolve().parents[1]
    cfg = root / 'adapters/robots/x1/arm_teleop.yaml'
    if not cfg.is_file():
        pytest.skip('x1 arm_teleop.yaml missing')

    monkeypatch.setenv('ROS_DOMAIN_ID', '231')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.delenv('FASTRTPS_DEFAULT_PROFILES_FILE', raising=False)
    inited = not rclpy.ok()
    if inited:
        rclpy.init()
    cmd_node = None
    node = None
    try:
        cmd_node = rclpy.create_node('test_cmd_node_isolated', enable_rosout=False, start_parameter_services=False)
        node = teleop.RobotArmTeleopNode(str(cfg), backend='v23', command_node=cmd_node)
        assert node._command_node is cmd_node
        assert any(s.topic_name == node.control.get('generic_command_topic', '/control/solver/joint_targets')
                   for s in cmd_node.subscriptions)
        assert not any(s.topic_name == node.control.get('generic_command_topic', '/control/solver/joint_targets')
                       for s in node.subscriptions)

        errors = []
        stop = threading.Event()
        def writer():
            try:
                for idx in range(200):
                    with node._command_state_lock:
                        node.generic_aux_command['leg_1'] = float(idx) * 0.001
                        node.generic_command_pipeline['_publication_gaps'].append((time.monotonic_ns(), 10.0))
                        node.generic_command_pipeline['_source_ages'].append((time.monotonic_ns(), 1.0))
                stop.set()
            except Exception as exc:
                errors.append(exc)
                stop.set()
        import time
        t = threading.Thread(target=writer)
        t.start()
        while not stop.is_set():
            teleop._generic_pipeline_snapshot(node.generic_command_pipeline, node._command_state_lock)
        t.join(timeout=2.0)
        assert not errors
    finally:
        if node is not None:
            node.destroy_node()
        if cmd_node is not None:
            cmd_node.destroy_node()
        if inited and rclpy.ok():
            rclpy.shutdown()
