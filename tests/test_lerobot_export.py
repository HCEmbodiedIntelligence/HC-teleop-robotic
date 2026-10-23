from types import SimpleNamespace
import pytest
from middleware.core.lerobot_export import joint_vector, latest_sample, synchronized_episodes


def test_joint_names_align_command_to_feedback():
    message = SimpleNamespace(name=['right', 'left'], position=[2, 1])
    assert joint_vector(message, ['left', 'right']) == [1, 2]
    with pytest.raises(ValueError):
        joint_vector(message, ['absent'])


def test_causal_lookup_rejects_future_and_stale():
    samples = [(100, 'old'), (200, 'new')]
    assert latest_sample(samples, 150, 60) == (100, 'old')
    assert latest_sample(samples, 90, 60) is None
    assert latest_sample(samples, 180, 60) is None


def test_stale_samples_preserve_single_episode_with_invalid_audit():
    streams = {key: [(0, [1]), (100_000_000, [2]), (600_000_000, [3]), (700_000_000, [4])] for key in ('observation.state', 'action')}
    audit = {}
    episodes = list(synchronized_episodes(streams, 10, audit))
    assert [len(episode) for episode in episodes] == [8]
    assert [stamp for stamp, _ in episodes[0]] == [i * 100_000_000 for i in range(8)]
    assert episodes[0][5][1]['observation.state'] == (100_000_000, [2])
    assert audit['candidate_frames'] == audit['retained_frames'] == 8
    assert audit['rejected_frames'] == 0
    assert audit['invalid_frames'] == audit['invalid_required_frames'] == 2
    assert audit['invalid_by_stream'] == {'observation.state': 2, 'action': 2}


@pytest.mark.parametrize('optional_key', ['action.base.velocity', 'observation.headset.pose'])
def test_optional_late_start_and_early_end_preserve_required_timeline(optional_key):
    stamps = [index * 100_000_000 for index in range(11)]
    streams = {key: [(stamp, [index]) for index, stamp in enumerate(stamps)]
               for key in ('observation.state', 'action', 'observation.images.eye')}
    streams[optional_key] = [(200_000_000, ['first']), (800_000_000, ['last'])]
    episodes = list(synchronized_episodes(streams, 10))
    assert len(episodes) == 1
    assert [stamp for stamp, _ in episodes[0]] == stamps
    for stamp, selected in episodes[0]:
        expected = (None if stamp < 200_000_000 else
                    (200_000_000, ['first']) if stamp < 800_000_000 else
                    (800_000_000, ['last']))
        assert selected[optional_key] == expected


def test_optional_long_gap_holds_latest_causal_sample_without_splitting():
    stamps = [index * 100_000_000 for index in range(31)]
    streams = {key: [(stamp, [index]) for index, stamp in enumerate(stamps)]
               for key in ('observation.state', 'action', 'observation.images.eye')}
    streams['action.base.velocity'] = [(0, [1, 0, 0]), (3_000_000_000, [2, 0, 0])]
    episodes = list(synchronized_episodes(streams, 10))
    assert [len(episode) for episode in episodes] == [31]
    assert episodes[0][-2][1]['action.base.velocity'] == (0, [1, 0, 0])
    assert episodes[0][-1][1]['action.base.velocity'] == (3_000_000_000, [2, 0, 0])


def test_camera_stale_gap_preserves_single_episode_with_optional_present():
    stamps = [index * 100_000_000 for index in range(11)]
    streams = {key: [(stamp, [index]) for index, stamp in enumerate(stamps)]
               for key in ('observation.state', 'action')}
    streams['observation.images.eye'] = [(0, 'first.png'), (1_000_000_000, 'last.png')]
    streams['action.base.velocity'] = [(0, [0, 0, 0])]
    episodes = list(synchronized_episodes(streams, 10))
    assert [len(episode) for episode in episodes] == [11]
    assert [stamp for stamp, _ in episodes[0]] == stamps
    assert episodes[0][9][1]['observation.images.eye'] == (0, 'first.png')
    assert episodes[0][-1][1]['observation.images.eye'] == (1_000_000_000, 'last.png')


@pytest.mark.parametrize('required_key', ['observation.state', 'action'])
def test_joint_stale_gap_preserves_single_episode_with_optional_present(required_key):
    stamps = [index * 100_000_000 for index in range(11)]
    streams = {key: [(stamp, [index]) for index, stamp in enumerate(stamps)]
               for key in ('observation.state', 'action', 'observation.images.eye')}
    streams[required_key] = [(0, [1]), (1_000_000_000, [2])]
    streams['action.base.velocity'] = [(0, [0, 0, 0])]
    episodes = list(synchronized_episodes(streams, 10))
    assert [len(episode) for episode in episodes] == [11]
    assert [stamp for stamp, _ in episodes[0]] == stamps
    assert episodes[0][9][1][required_key] == (0, [1])
    assert episodes[0][-1][1][required_key] == (1_000_000_000, [2])


def test_default_span_uses_all_stream_first_and_last_messages():
    streams = {
        'observation.state': [(200_000_000, [1]), (600_000_000, [2])],
        'action': [(300_000_000, [3]), (700_000_000, [4])],
        'observation.images.eye': [(400_000_000, 'rgb.png')],
        'action.base.velocity': [(100_000_000, [5]), (1_000_000_000, [6])],
    }
    episodes = list(synchronized_episodes(streams, 10))
    assert [len(episode) for episode in episodes] == [10]
    assert [stamp for stamp, _ in episodes[0]] == [i * 100_000_000 for i in range(1, 11)]
    assert episodes[0][0][1]['observation.state'] is None
    assert episodes[0][2][1]['observation.images.eye'] is None
    assert episodes[0][-1][1]['action'] == (700_000_000, [4])


def test_explicit_recording_range_preserves_pre_first_and_post_last_rows():
    streams = {key: [(200_000_000, [1]), (800_000_000, [2])]
               for key in ('observation.state', 'action')}
    audit = {}
    episodes = list(synchronized_episodes(streams, 10, audit, time_range=(0, 1_200_000_000)))
    assert [len(episode) for episode in episodes] == [13]
    assert [stamp for stamp, _ in episodes[0]] == [i * 100_000_000 for i in range(13)]
    assert episodes[0][0][1] == {'observation.state': None, 'action': None}
    assert episodes[0][1][1] == {'observation.state': None, 'action': None}
    assert episodes[0][-1][1]['action'] == (800_000_000, [2])
    assert audit['retained_frames'] == audit['candidate_frames'] == 13
    assert audit['rejected_frames'] == 0
    assert audit['invalid_required_frames'] == 7


def test_prepare_real_mcap_keeps_full_recording_span_and_missing_masks(tmp_path):
    import json
    import cv2
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message
    from mcap.writer import Writer
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import TwistStamped
    from middleware.core.lerobot_export import prepare_mcap

    source = tmp_path / 'source.mcap'
    staging = tmp_path / 'staging'
    staging.mkdir()
    origin = 1_000_000_000
    with source.open('wb') as handle:
        writer = Writer(handle)
        writer.start(profile='ros2')
        channels = {}
        for topic, kind in [('/vrdata', 'std_msgs/msg/String'),
                            ('/observation/joint_states', 'sensor_msgs/msg/JointState'),
                            ('/action/joint_states', 'sensor_msgs/msg/JointState'),
                            ('/cameras/eye/color', 'sensor_msgs/msg/Image'),
                            ('/action/base/velocity', 'geometry_msgs/msg/TwistStamped')]:
            schema = writer.register_schema(kind, 'ros2msg', b'')
            channels[topic] = writer.register_channel(topic, 'cdr', schema)

        def add(topic, milliseconds, message):
            stamp = origin + milliseconds * 1_000_000
            writer.add_message(channels[topic], stamp, bytes(serialize_message(message)), stamp)

        add('/vrdata', 0, String(data='recording begins before robot data'))
        for milliseconds, value in [(200, 1.), (600, 2.)]:
            add('/observation/joint_states', milliseconds, JointState(name=['a'], position=[value]))
        for milliseconds, value in [(300, 3.), (700, 4.)]:
            add('/action/joint_states', milliseconds, JointState(name=['a'], position=[value]))
        for milliseconds, color in [(400, [255, 0, 0]), (800, [0, 255, 0])]:
            image = Image(height=16, width=16, encoding='rgb8', step=48, data=bytes(color * 256))
            add('/cameras/eye/color', milliseconds, image)
        velocity = TwistStamped()
        velocity.twist.linear.x = .5
        add('/action/base/velocity', 500, velocity)
        add('/vrdata', 1000, String(data='recording ends after robot data'))
        writer.finish()

    manifest = json.loads(prepare_mcap(source, staging, 10, 'full timeline', 'x1').read_text())
    assert manifest['origin_log_time_ns'] == origin
    assert len(manifest['episodes']) == 1
    rows = manifest['episodes'][0]
    assert len(rows) == 11
    assert [row['grid_time'] for row in rows] == [i / 10 for i in range(11)]
    assert rows[0]['values']['observation.state'] == [0.]
    assert rows[0]['values']['action'] == [0.]
    assert rows[0]['values']['action.base.velocity'] == [0.] * 6
    assert rows[0]['source_available'] == rows[0]['source_valid'] == [0.] * 4
    assert rows[0]['source_age'] == rows[0]['source_times'] == [-1.] * 4
    camera = next(key for key in manifest['dimensions'])
    black = cv2.imread(rows[0]['values'][camera])
    assert black.shape == (16, 16, 3) and not black.any()
    assert cv2.imread(rows[4]['values'][camera])[0, 0].tolist() == [0, 0, 255]
    assert cv2.imread(rows[-1]['values'][camera])[0, 0].tolist() == [0, 255, 0]
    assert rows[-1]['values']['observation.state'] == [2.]
    assert rows[-1]['values']['action'] == [4.]
    velocity_index = manifest['source_keys'].index('action.base.velocity')
    assert rows[-1]['source_available'][velocity_index] == 1
    assert rows[-1]['source_valid'][velocity_index] == 0
    assert rows[-1]['source_age'][velocity_index] == .5
    assert rows[-1]['values']['action.base.velocity'][0] == .5
    audit = manifest['sync_audit']
    assert audit['retained_frames'] == audit['candidate_frames'] == 11
    assert audit['rejected_frames'] == 0


@pytest.fixture
def subset_action_joint_manifest(tmp_path):
    """Real CDR feedback has a passive head joint omitted from command messages."""
    pytest.importorskip('rclpy')
    from rclpy.serialization import serialize_message
    from mcap.writer import Writer
    from sensor_msgs.msg import JointState
    from middleware.core.lerobot_export import prepare_mcap

    source = tmp_path / 'joint_subset.mcap'
    staging = tmp_path / 'staging'
    staging.mkdir()
    with source.open('wb') as handle:
        writer = Writer(handle)
        writer.start(profile='ros2')
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        feedback = writer.register_channel('/observation/joint_states', 'cdr', schema)
        command = writer.register_channel('/action/joint_states', 'cdr', schema)
        for index in range(2):
            stamp = 1_000_000_000 + index * 100_000_000
            state = JointState(name=['arm', 'head', 'grip'], position=[1. + index, 99. - index, 2. + index])
            # Command ordering can vary; its own joint set must remain stable.
            action = (JointState(name=['grip', 'arm'], position=[20., 10.]) if index == 0
                      else JointState(name=['arm', 'grip'], position=[11., 21.]))
            writer.add_message(feedback, stamp, bytes(serialize_message(state)), stamp)
            writer.add_message(command, stamp, bytes(serialize_message(action)), stamp)
        writer.finish()
    return prepare_mcap(source, staging, 10, 'partial joint command', 'x1')


def test_prepare_mcap_action_joint_subset_has_independent_names(subset_action_joint_manifest):
    import json
    manifest = json.loads(subset_action_joint_manifest.read_text())
    assert manifest['joint_names'] == ['arm', 'head', 'grip']
    assert manifest['action_joint_names'] == ['arm', 'grip']
    rows = manifest['episodes'][0]
    assert len(rows) == 2
    assert [row['values']['observation.state'] for row in rows] == [[1., 99., 2.], [2., 98., 3.]]
    assert [row['values']['action'] for row in rows] == [[10., 20.], [11., 21.]]
    assert all(row['source_available'] == row['source_valid'] == [1., 1.] for row in rows)


def test_prepare_mcap_preserves_changing_partial_action_with_axis_validity(tmp_path, monkeypatch):
    import json
    import sys
    from types import SimpleNamespace
    from mcap.writer import Writer
    from middleware.core.lerobot_export import prepare_mcap

    monkeypatch.setitem(sys.modules, 'rclpy.serialization', SimpleNamespace(
        deserialize_message=lambda data, _: SimpleNamespace(**json.loads(data))))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', SimpleNamespace(get_message=lambda _: object))
    source = tmp_path / 'changing_action.mcap'
    with source.open('wb') as handle:
        writer = Writer(handle); writer.start()
        schema = writer.register_schema('sensor_msgs/msg/JointState', 'ros2msg', b'')
        state = writer.register_channel('/observation/joint_states', 'cdr', schema)
        action = writer.register_channel('/action/joint_states', 'cdr', schema)
        for index, command in enumerate(({'arm': 1., 'waist': 2.}, {'arm': 3.})):
            stamp = 1_000_000_000 + index * 100_000_000
            writer.add_message(state, stamp, json.dumps({'name': ['arm', 'waist'],
                'position': [10. + index, 20. + index]}).encode(), stamp)
            writer.add_message(action, stamp, json.dumps({'name': list(command),
                'position': list(command.values())}).encode(), stamp)
        writer.finish()
    staging = tmp_path / 'stage'; staging.mkdir()
    manifest = json.loads(prepare_mcap(source, staging, 10, 'changing action', 'x1').read_text())
    assert manifest['action_joint_names'] == ['arm', 'waist']
    assert manifest['action_axis_validity'] is True
    assert [row['values']['action'] for row in manifest['episodes'][0]] == [[1., 2.], [3., 2.]]
    assert [row['action_valid'] for row in manifest['episodes'][0]] == [[True, True], [True, False]]


def test_official_loader_independent_action_joint_dimensions(subset_action_joint_manifest):
    import os
    import subprocess
    from middleware.core.lerobot_export import ROOT, writer_python
    executable = writer_python()
    if not executable.exists():
        pytest.skip('optional LeRobot environment is not installed')
    script = '''
import sys
from pathlib import Path
from middleware.core.lerobot_export import write_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset
manifest=Path(sys.argv[1]); root=manifest.parent/'dataset'
result=write_dataset(manifest,root)
assert result['frames']==2 and result['episodes']==1
loaded=LeRobotDataset('local/dataset',root=root,video_backend='pyav')
assert loaded.meta.features['observation.state']['shape']==(3,)
assert loaded.meta.features['action']['shape']==(2,)
assert loaded.meta.features['observation.state']['names']==['arm','head','grip']
assert loaded.meta.features['action']['names']==['arm','grip']
assert loaded[0]['observation.state'].tolist()==[1.,99.,2.]
assert loaded[0]['action'].tolist()==[10.,20.]
assert loaded[1]['observation.state'].tolist()==[2.,98.,3.]
assert loaded[1]['action'].tolist()==[11.,21.]
'''
    env = {key: value for key, value in os.environ.items() if key not in ('PYTHONPATH', 'PYTHONHOME')}
    result = subprocess.run(
        [str(executable), '-c', script, str(subset_action_joint_manifest)],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=180,
    )
    assert result.returncode == 0, result.stderr


def test_official_writer_loader_roundtrip(tmp_path):
    """Exercise the real optional package, including two short episodes and RGB."""
    import json
    import os
    import subprocess
    from middleware.core.lerobot_export import ROOT, writer_python
    executable = writer_python()
    if not executable.exists():
        pytest.skip('optional LeRobot environment is not installed')
    manifest = tmp_path / 'manifest.json'
    script = '''
import json,sys,numpy as np
from pathlib import Path
from PIL import Image
from middleware.core.lerobot_export import write_dataset
p=Path(sys.argv[1]); image=p.parent/'rgb.png'
Image.new('RGB',(16,16),(255,0,0)).save(image)
row={'values':{'observation.state':[1.,2.],'action':[3.,4.],'observation.images.eye':str(image)},'source_times':[0.,0.,0.],'grid_time':0.}
m={'fps':10,'task':'test','robot_type':'x1','joint_names':['a','b'],'dimensions':{'observation.images.eye':[16,16,3]},'source_keys':['observation.state','action','observation.images.eye'],'source_topics':{},'origin_log_time_ns':0,'episodes':[[row],[row,row]]}
p.write_text(json.dumps(m))
result=write_dataset(p,p.parent/'dataset')
assert result['frames']==3 and result['episodes']==2
from lerobot.datasets.lerobot_dataset import LeRobotDataset
loaded=LeRobotDataset('local/dataset',root=p.parent/'dataset',video_backend='pyav')
assert loaded[0]['observation.state'].tolist()==[1.,2.]
assert loaded[2]['action'].tolist()==[3.,4.]
assert loaded[0]['observation.images.eye'][0,0,0] > .99
assert loaded.meta.features['observation.images.eye']['dtype']=='video'
assert list((p.parent/'dataset'/'videos').rglob('*.mp4'))
'''
    env = {key: value for key, value in os.environ.items() if key not in ('PYTHONPATH', 'PYTHONHOME')}
    result = subprocess.run([str(executable), '-c', script, str(manifest)], cwd=ROOT, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_official_writer_loader_optional_source_masks(tmp_path):
    """Missing and stale held values remain distinguishable after serialization."""
    import os
    import subprocess
    from middleware.core.lerobot_export import ROOT, writer_python
    executable = writer_python()
    if not executable.exists():
        pytest.skip('optional LeRobot environment is not installed')
    script = '''
import json,sys,numpy as np
from pathlib import Path
from PIL import Image
from middleware.core.lerobot_export import write_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset

p=Path(sys.argv[1]); image=p.parent/'rgb.png'
Image.new('RGB',(16,16),(255,0,0)).save(image)
keys=['observation.state','action','observation.images.eye','action.base.velocity']
ages=[-1.,0.,.1,.2,.3]; rows=[]
for index,age in enumerate(ages):
    available=[1.,1.,1.,float(index>0)]
    valid=[1.,1.,1.,float(0<=age<=.2)]
    rows.append({'values':{'observation.state':[float(index)],'action':[float(index+10)],'observation.images.eye':str(image),'action.base.velocity':[0.,0.,0.] if index==0 else [.4,0.,.2]},'source_times':[index/10]*3+[-1. if index==0 else .1],'grid_time':index/10,'source_available':available,'source_valid':valid,'source_age':[0.,0.,0.,age]})
m={'fps':10,'task':'optional-source-mask','robot_type':'x1','joint_names':['a'],'dimensions':{'observation.images.eye':[16,16,3]},'numeric_names':{'action.base.velocity':['forward','lateral','yaw']},'source_keys':keys,'source_topics':{},'origin_log_time_ns':0,'episodes':[rows],'sync_audit':{}}
p.write_text(json.dumps(m)); root=p.parent/'dataset'
result=write_dataset(p,root)
assert result['frames']==5 and result['episodes']==1
loaded=LeRobotDataset('local/dataset',root=root,video_backend='pyav')
for index,source in enumerate(rows):
    row=loaded[index]
    for field in ['source_available','source_valid','source_age']:
        key='observation.'+field
        assert loaded.meta.features[key]['shape']==(4,)
        assert np.allclose(row[key].numpy(),source[field])
    assert np.allclose(row['observation.source_time'].numpy(),source['source_times'])
    assert np.allclose(row['action.base.velocity'].numpy(),source['values']['action.base.velocity'])
assert loaded[0]['observation.source_age'][-1].item()==-1
assert loaded[0]['observation.source_time'][-1].item()==-1
assert loaded[4]['observation.source_available'][-1].item()==1
assert loaded[4]['observation.source_valid'][-1].item()==0
assert loaded[4]['action.base.velocity'][0].item()>.39
'''
    env = {key: value for key, value in os.environ.items() if key not in ('PYTHONPATH', 'PYTHONHOME')}
    result = subprocess.run(
        [str(executable), '-c', script, str(tmp_path / 'manifest.json')],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=180,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('fps', [15, 30])
def test_short_episode_mp4_frame_order_and_timestamps(tmp_path, fps):
    """Verify actual encoding, concatenation and official cross-episode lookup."""
    import os
    import subprocess
    from middleware.core.lerobot_export import ROOT, writer_python
    executable = writer_python()
    if not executable.exists():
        pytest.skip('optional LeRobot environment is not installed')
    script = '''
import json,sys,numpy as np,av
from pathlib import Path
from PIL import Image
from middleware.core.lerobot_export import write_dataset
from lerobot.datasets.lerobot_dataset import LeRobotDataset

p=Path(sys.argv[1]); fps=int(sys.argv[2]); lengths=[1,2,1,3,7,1]
keys=['observation.images.eye','observation.images.wrist_left','observation.images.wrist_right']
episodes=[]; colors=[]; episode_ids=[]
for episode_id,length in enumerate(lengths):
    rows=[]
    for local_index in range(length):
        index=len(colors)
        # Widely separated RGB values distinguish every frame despite H.264 loss.
        color=np.array([40+85*(index%3),40+85*((index//3)%3),40+85*((index//9)%3)],dtype=np.uint8)
        images={}
        for camera,key in enumerate(keys):
            image=p.parent/f'rgb_{camera}_{index:02d}.png'
            Image.new('RGB',(64,48),tuple(map(int,np.roll(color,camera)))).save(image)
            images[key]=str(image)
        colors.append(color); episode_ids.append(episode_id)
        rows.append({'values':{'observation.state':[float(index),float(episode_id)],'action':[float(index+100),float(local_index)],**images},'source_times':[index/fps]*5,'grid_time':index/fps})
    episodes.append(rows)
m={'fps':fps,'task':'short-episode-regression','robot_type':'x1','joint_names':['a','b'],'dimensions':{key:[48,64,3] for key in keys},'source_keys':['observation.state','action',*keys],'source_topics':{},'origin_log_time_ns':0,'episodes':episodes}
p.write_text(json.dumps(m)); root=p.parent/'dataset'
result=write_dataset(p,root)
assert result['episode_lengths']==lengths
assert result['frames']==sum(lengths) and result['episodes']==len(lengths)
loaded=LeRobotDataset('local/dataset',root=root,video_backend='pyav')
assert len(loaded)==sum(lengths)
assert all(loaded.meta.features[key]['dtype']=='video' for key in keys)

# Every row, especially both sides of a short episode boundary, must resolve
# to its own video frame rather than the prior or next episode's frame.
for index,(color,episode_id) in enumerate(zip(colors,episode_ids)):
    row=loaded[index]
    assert row['observation.state'].tolist()==[float(index),float(episode_id)]
    assert int(row['episode_index'].item())==episode_id
    for camera,key in enumerate(keys):
        actual=row[key].numpy().mean(axis=(1,2))*255
        expected=np.roll(color,camera).astype(float)
        assert np.max(np.abs(actual-expected))<12,(index,key,actual,expected)

videos=sorted((root/'videos').rglob('*.mp4'))
assert videos, 'MP4 files were not produced'
for camera,key in enumerate(keys):
    decoded_colors=[]
    camera_videos=sorted((root/'videos'/key).rglob('*.mp4'))
    assert camera_videos,key
    for path in camera_videos:
        with av.open(str(path)) as container:
            stream=container.streams.video[0]
            packets=[packet for packet in container.demux(stream) if packet.dts is not None]
            dts=[packet.dts for packet in packets]
            assert all(abs(float(packet.duration*packet.time_base)-1/fps)<1e-6 for packet in packets)
        assert dts and all(b>a for a,b in zip(dts,dts[1:])),(path,dts)
        with av.open(str(path)) as container:
            frames=list(container.decode(video=0))
            assert abs(float(container.streams.video[0].duration*container.streams.video[0].time_base)-len(frames)/fps)<1e-6
        assert frames and all(frame.pts is not None for frame in frames)
        pts=np.array([float(frame.pts*frame.time_base) for frame in frames])
        assert np.allclose(pts-pts[0],np.arange(len(frames))/fps,atol=1e-6),(path,pts)
        decoded_colors.extend(frame.to_ndarray(format='rgb24').mean(axis=(0,1)) for frame in frames)
    assert len(decoded_colors)==sum(lengths), (key,len(decoded_colors),lengths)
    for index,(actual,color) in enumerate(zip(decoded_colors,colors)):
        expected=np.roll(color,camera).astype(float)
        assert np.max(np.abs(actual-expected))<12,(index,key,actual,expected)
'''
    env = {key: value for key, value in os.environ.items() if key not in ('PYTHONPATH', 'PYTHONHOME')}
    result = subprocess.run(
        [str(executable), '-c', script, str(tmp_path / 'manifest.json'), str(fps)],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=180,
    )
    assert result.returncode == 0, result.stderr


def test_explicit_gripper_sources_keep_independent_age_masks(tmp_path):
    import json
    from mcap.writer import Writer
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from middleware.core.lerobot_export import prepare_mcap
    source=tmp_path/'grasp.mcap';staging=tmp_path/'stage';staging.mkdir()
    mappings=[dict(topic='/action/left_hand/joint_states',names=['finger'],units=['m'],max_age_sec=.3),
              dict(topic='/action/right_hand/joint_states',names=['missing_finger'],units=['m'],max_age_sec=.3)]
    with source.open('wb') as stream:
        writer=Writer(stream);writer.start()
        schema=writer.register_schema('sensor_msgs/msg/JointState','ros2msg',b'')
        for topic,names,stamps in [('/observation/joint_states',['arm'],[0,500_000_000]),
                                   ('/action/joint_states',['arm'],[0,500_000_000]),
                                   (mappings[0]['topic'],['finger'],[100_000_000])]:
            channel=writer.register_channel(topic,'cdr',schema)
            for stamp in stamps:
                writer.add_message(channel,1_000_000_000+stamp,serialize_message(JointState(name=names,position=[.02])),1_000_000_000+stamp)
        writer.finish()
    manifest=json.loads(prepare_mcap(source,staging,10,'grasp','test',action_sources=mappings).read_text())
    assert manifest['action_joint_names']==['arm','finger','missing_finger']
    rows=manifest['episodes'][0]
    index=manifest['source_keys'].index('action.source.0')
    missing=manifest['source_keys'].index('action.source.1')
    assert rows[0]['values']['action']==[.02,0,0]
    assert rows[-1]['values']['action']==[.02,.02,0]
    assert rows[1]['source_valid'][index]==1
    assert rows[-1]['source_valid'][index]==0
    assert all(row['source_available'][missing]==0 for row in rows)
    assert manifest['action_semantics']=='upstream_command_not_execution'
