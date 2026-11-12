"""Actual CDR -> official LeRobot: partial axes, short clips and gap boundaries."""
import json
import os
from pathlib import Path
import subprocess

import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, JointState

from middleware.core.topic_recorder import TopicRecorder
from middleware.core.lerobot_export import prepare_mcap, writer_python


def require_official_writer():
    env = {key: value for key, value in os.environ.items() if key not in {'PYTHONPATH', 'PYTHONHOME'}}
    if not writer_python().is_file():
        pytest.skip('official LeRobot writer unavailable; source/export staging assertions passed')
    probe = subprocess.run([str(writer_python()), '-c',
        "import importlib.util,sys; sys.exit(0 if all(importlib.util.find_spec(n) for n in ('lerobot','av','pyarrow','torch')) else 1)"],
        env=env, capture_output=True, timeout=10)
    if probe.returncode:
        pytest.skip('official LeRobot writer dependencies incomplete; source/export staging assertions passed')
    return env


def test_full_session_exports_all_phases_and_preserves_human_labels_in_both_formats(tmp_path):
    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path)
    recorder.start('session.mcap', {'capture_mode': 'full_session'})
    origin = 1_000_000_000
    for index in range(20):
        stamp = origin + index*100_000_000
        state = JointState(name=['arm'], position=[index/100])
        image = Image(height=16, width=16, encoding='rgb8', step=48, data=bytes([32, 128, 224]*256))
        for topic, message in [('/observation/joint_states', state), ('/test/color/image', image),
                               ('/action/joint_states', JointState(name=['arm'], position=[index/100]))]:
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            recorder.record(dict(topic=topic, msg_type='sensor_msgs/msg/'+type(message).__name__,
                _raw=bytes(serialize_message(message)), stamp_ns=stamp))
    interval = dict(key='human-one', start_ns=origin+500_000_000, end_ns=origin+1_000_000_000)
    session = dict(capture_mode='full_session', start_ns=origin, end_ns=origin+2_000_000_000,
                   interventions=[interval])
    # A manual_close inside a continuous session must not make the exporter
    # select just that interval (or require a correction-only first_action_ns).
    recorder.record(dict(topic='/events/control/intervention', stamp_ns=interval['end_ns'],
        payload=dict(interval, kind='manual_close', capture_mode='full_session')))
    recorder.record(dict(topic='/events/control/intervention', stamp_ns=session['end_ns'],
        payload=dict(session, kind='session_close')))
    recorder.stop()
    recorder.path.with_suffix('.session.json').write_text(json.dumps(session))
    staging = tmp_path/'staging'
    staging.mkdir()
    manifest = prepare_mcap(recorder.path, staging, 10, 'test', 'fake')
    value = json.loads(manifest.read_text())
    assert 'correction' not in value and value['session'] == session
    assert len(value['episodes']) == 1
    rows = value['episodes'][0]
    assert rows[0]['grid_time'] == 0 and rows[-1]['grid_time'] >= 1.9
    assert {row['intervention'] for row in rows} == {False, True}
    assert all(row['intervention'] == (.5 <= row['grid_time'] < 1.) for row in rows)
    from middleware.core.video_bundle import prepare
    bundle = prepare(recorder.path, tmp_path/'bundle', dict(recording_id='session',
        started_at='2026-10-22T00:00:00Z', ended_at='2026-10-22T00:00:02Z'))
    assert json.loads((bundle/'intervention-session.json').read_text()) == session
    assert json.loads((bundle/'session.json').read_text())['videos'][0]['frame_count'] == 20
    env = require_official_writer()
    script = '''
import json,sys
from pathlib import Path
from middleware.core.lerobot_export import write_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset
source=Path(sys.argv[1]); root=source.parent/'dataset'
write_dataset(source,root)
dataset=LeRobotDataset('local/dataset',root=root,video_backend='pyav')
assert dataset.meta.features['intervention']['dtype']=='bool'
assert {bool(dataset[i]['intervention'].item()) for i in range(len(dataset))}=={False,True}
assert len(dataset)>=20
assert json.loads((root/'meta/teleop_sync.json').read_text())['session']['capture_mode']=='full_session'
'''
    result = subprocess.run([str(writer_python()), '-c', script, str(manifest)],
                            text=True, capture_output=True, env=env, timeout=90)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('long_gap', [False, True])
def test_corrections_export_boolean_and_never_joins_across_long_gap(tmp_path, long_gap):
    recorder = TopicRecorder({'directory': str(tmp_path)}, tmp_path)
    recorder.start('human.mcap')
    origin = 1_000_000_000
    offsets = [0, 100_000_000, 700_000_000, 800_000_000] if long_gap else [0, 50_000_000, 100_000_000, 150_000_000]
    end = origin+(900_000_000 if long_gap else 200_000_000)
    for offset in offsets:
        stamp = origin+offset
        state = JointState(name=['left', 'right'], position=[.1, .2])
        image = Image(height=16, width=16, encoding='rgb8', step=48, data=bytes([32, 128, 224]*256))
        for topic, message in [('/observation/joint_states', state), ('/camera/color/image', image),
                               ('/action/joint_states', JointState(name=['left'], position=[.3])),
                               ('/action/joint_states', JointState(name=['right'], position=[.4]))]:
            stamp += 1 if topic == '/action/joint_states' else 0
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            recorder.record(dict(topic=topic, msg_type='sensor_msgs/msg/'+type(message).__name__,
                                 _raw=bytes(serialize_message(message)), stamp_ns=stamp))
    recorder.stop()
    correction = dict(key='hc1:controller:session:1:vr:manual', start_ns=origin, end_ns=end,
                      quality=dict(first_action_ns=origin+1), outcome='unknown', intervention=True)
    recorder.path.with_suffix('.correction.json').write_text(json.dumps(correction))
    staging = tmp_path/'staging'
    staging.mkdir()
    manifest = prepare_mcap(recorder.path, staging, 20 if not long_gap else 10, 'test', 'fake')
    value = json.loads(manifest.read_text())
    assert value['correction']['outcome'] == 'unknown'
    assert value['action_joint_names'] == ['left', 'right']
    assert all(row['action_valid'] == [True, True] for episode in value['episodes'] for row in episode)
    if long_gap:
        assert len(value['episodes']) == 2
        assert value['episodes'][1][0]['grid_time']-value['episodes'][0][-1]['grid_time'] > .2
    else:
        assert len(value['episodes']) == 1 and 1 <= len(value['episodes'][0]) <= 4
    script = '''
import json,sys
from pathlib import Path
from middleware.core.lerobot_export import write_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset
source=Path(sys.argv[1]); root=source.parent/'dataset'
result=write_dataset(source,root)
dataset=LeRobotDataset('local/dataset',root=root,video_backend='pyav')
assert dataset.meta.features['intervention']['dtype']=='bool'
assert all(bool(dataset[i]['intervention'].item()) for i in range(len(dataset)))
assert all(dataset[i]['action.valid'].tolist()==[True,True] for i in range(len(dataset)))
assert json.loads((root/'meta/teleop_sync.json').read_text())['correction']['outcome']=='unknown'
print(json.dumps(result))
'''
    env = require_official_writer()
    result = subprocess.run([str(writer_python()), '-c', script, str(manifest)],
                            text=True, capture_output=True, env=env, timeout=90)
    assert result.returncode == 0, result.stderr
