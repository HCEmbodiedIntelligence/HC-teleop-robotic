"""Real MCAP -> official LeRobot -> internal ready handoff, with synthetic CDR."""
import json
import time

import pytest
from middleware.core.lerobot_export import writer_python
from middleware.core.recording_service import RecordingService


def test_service_both_publishes_verified_independent_artifacts(tmp_path):
    if not writer_python().is_file():
        pytest.skip('optional official LeRobot environment is not installed')
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState, Image
    cfg = dict(directory=str(tmp_path/'data'), job_state_dir=str(tmp_path/'jobs'),
               artifact_state_dir=str(tmp_path/'ready'), format='both', lerobot_fps=10,
               min_free_bytes=1, action_sources=[dict(topic='/action/left_hand/joint_states',
                   names=['finger'], units=['m'], max_age_sec=.2)])
    ros = dict(enabled=False, subscriptions=[dict(topic='/observation/joint_states',
               type='sensor_msgs/msg/JointState', outputs=['record'])])
    service = RecordingService(cfg, tmp_path, ros, lambda: {'/observation/joint_states': {'state': 'ok'}})
    try:
        service.start(metadata={'profile_id': 'synthetic-openarm-local'})
        origin = time.time_ns()
        for i in range(6):
            for topic, kind, message in [
                ('/observation/joint_states', 'JointState', JointState(name=['arm'], position=[i*.1])),
                ('/action/joint_states', 'JointState', JointState(name=['arm'], position=[i*.1+.01])),
                ('/action/left_hand/joint_states', 'JointState', JointState(name=['finger'], position=[i*.001])),
                ('/cameras/eye/color', 'Image', Image(height=16, width=16, encoding='rgb8', step=48,
                                                    data=bytes([i*40, 0, 255-i*40]*256))),
            ]:
                assert service.record(dict(topic=topic, msg_type='sensor_msgs/msg/'+kind,
                    stamp_ns=origin+i*100_000_000, _raw=bytes(serialize_message(message))))
        service.stop()
    finally:
        service.shutdown()
    job = next(iter(service.jobs.values()))
    assert job['artifact_state'] == 'ready', job
    items = [json.loads(p.read_text()) for p in service.artifact_state_dir.glob('*.json')]
    assert {d['kind'] for d in items} == {'hc_mcap', 'hc_lerobot'}
    mcap = next(d for d in items if d['kind']=='hc_mcap')
    dataset = next(d for d in items if d['kind']=='hc_lerobot')
    assert dataset['source_artifact_id'] == mcap['artifact_id']
    assert any(f['path'].endswith('.mp4') for f in dataset['files'])
    assert any(f['path'].endswith('.parquet') for f in dataset['files'])
    assert mcap['capture_started_at'] <= mcap['capture_ended_at']
    try:
        from humanoid_upload.artifacts import inspect_ready
    except ImportError:
        return  # Standalone B tests also work before C's package is installed.
    for path in service.artifact_state_dir.glob('*.json'):
        inspect_ready(json.loads(path.read_text()), {'data_directory': str(service.directory), 'read_bytes_per_second': 0})
