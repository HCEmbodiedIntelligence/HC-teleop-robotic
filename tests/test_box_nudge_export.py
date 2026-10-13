import copy
import json

import pytest

from middleware.core.lerobot_export import BOX_NUDGE_FIELDS, BOX_NUDGE_TOPIC, parse_box_nudge_sample, prepare_mcap


def sample():
    result = {'schema': 'box_nudge.pico.v1', 'frame_id': 'robot_root',
              'sim_time': 4.2, 'action_age_s': .03,
              'joint_names': [f'joint_{i}' for i in range(17)],
              'tcp_frames': ['flange_L_Link', 'flange_R_Link']}
    for category, fields in BOX_NUDGE_FIELDS.items():
        result[category] = {field: [float(i) for i in range(size)] for field, size in fields.items()}
    result['action']['valid'] = [1]
    result['action']['representable'] = [0]
    return result


def test_atomic_parser_preserves_training_features_and_coordinate_metadata():
    payload = sample()
    values, names, metadata = parse_box_nudge_sample(json.dumps(payload))
    assert values['observation.box_nudge.joint_pos_abs'] == list(range(17))
    assert values['action.box_nudge.policy_raw'] == list(range(14))
    assert values['action.box_nudge.representable'] == [0]
    assert names['action.box_nudge.ee_pose_b'] == [f'{side}_{axis}' for side in ('left', 'right') for axis in ('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')]
    assert names['observation.box_nudge.joint_pos_abs'] == payload['joint_names']
    assert metadata['tcp_frames'] == payload['tcp_frames']


@pytest.mark.parametrize('mutation', [
    lambda p: p.update(schema='unknown'),
    lambda p: p.update(frame_id='world'),
    lambda p: p['observation'].update(joint_pos_abs=[0] * 16),
    lambda p: p['action'].update(ee_pose_b=[0] * 13),
    lambda p: p['action'].update(policy_raw=[float('nan')] * 14),
    lambda p: p['observation'].update(box_size=[0, float('inf'), 0]),
    lambda p: p['action'].update(valid=[2]),
    lambda p: p.update(action_age_s=True),
    lambda p: p.update(joint_names=['duplicate'] * 17),
    lambda p: p['observation'].pop('last_action'),
])
def test_atomic_parser_rejects_invalid_schema_shape_and_numbers(mutation):
    payload = copy.deepcopy(sample())
    mutation(payload)
    with pytest.raises(ValueError):
        parse_box_nudge_sample(json.dumps(payload))


def test_atomic_only_mcap_export_preserves_timeline_and_marks_stale_truth(tmp_path):
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from mcap.writer import Writer

    source = tmp_path / 'source.mcap'
    staging = tmp_path / 'staging'
    staging.mkdir()
    with source.open('wb') as handle:
        writer = Writer(handle)
        writer.start(profile='ros2')
        schema = writer.register_schema('std_msgs/msg/String', 'ros2msg', b'')
        atomic_channel = writer.register_channel(BOX_NUDGE_TOPIC, 'cdr', schema)
        vr_channel = writer.register_channel('/vrdata', 'cdr', schema)
        for channel, stamp, data in [(vr_channel, 1_000_000_000, '{}'),
                                     (atomic_channel, 1_100_000_000, json.dumps(sample())),
                                     (vr_channel, 1_500_000_000, '{}')]:
            writer.add_message(channel, stamp, bytes(serialize_message(String(data=data))), stamp)
        writer.finish()
    manifest = json.loads(prepare_mcap(source, staging, 10, 'nudge', 'x1').read_text())
    rows = manifest['episodes'][0]
    assert len(rows) == 6
    key = 'observation.box_nudge.freshness_valid'
    assert [r['values'][key] for r in rows] == [[0], [1], [1], [1], [0], [0]]
    assert rows[0]['values']['action.box_nudge.policy_raw'] == [0] * 14
    assert rows[-1]['values']['action.box_nudge.policy_raw'] == list(range(14))
    assert rows[-1]['values']['observation.box_nudge.sim_time'] == [4.2]
    assert manifest['joint_names'] == manifest['action_joint_names'] == []
    assert manifest['box_nudge']['frame_id'] == 'robot_root'
    assert manifest['source_frame_ids'][BOX_NUDGE_TOPIC] == 'robot_root'
    assert manifest['source_topics']['action.box_nudge.ee_pose_b'] == BOX_NUDGE_TOPIC
    assert len(set(rows[1]['source_times'])) == 1

    # Exercise the official writer/loader with no generic joint-state streams.
    import os
    import subprocess
    from middleware.core.lerobot_export import ROOT, writer_python
    executable = writer_python()
    if executable.is_file():
        script = '''
import json,sys
from pathlib import Path
from middleware.core.lerobot_export import write_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset
manifest=Path(sys.argv[1]); root=manifest.parent/'dataset'
result=write_dataset(manifest,root)
assert result['frames']==6 and result['episodes']==1
loaded=LeRobotDataset('local/dataset',root=root,video_backend='pyav')
assert loaded.meta.features['observation.box_nudge.joint_pos_abs']['shape']==(17,)
assert loaded.meta.features['action.box_nudge.ee_pose_b']['shape']==(14,)
assert 'action' not in loaded.meta.features
assert float(loaded[0]['observation.box_nudge.freshness_valid'])==0
assert float(loaded[1]['observation.box_nudge.freshness_valid'])==1
assert float(loaded[5]['observation.box_nudge.freshness_valid'])==0
assert loaded[5]['action.box_nudge.policy_raw'].tolist()==list(range(14))
assert json.loads((root/'meta/teleop_sync.json').read_text())['box_nudge']['frame_id']=='robot_root'
'''
        env = {k: v for k, v in os.environ.items() if k not in ('PYTHONPATH', 'PYTHONHOME')}
        result = subprocess.run([str(executable), '-c', script, str(staging / 'manifest.json')],
                                cwd=ROOT, env=env, capture_output=True, text=True, timeout=180)
        assert result.returncode == 0, result.stderr
