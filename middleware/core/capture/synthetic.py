"""Synthetic ROS-shaped input -> real Session/MP4/MCAP -> real LeRobot exporter.

No publishers, robot connections, fixture copies, or training-stack dependencies.
"""
import hashlib
import json
from pathlib import Path
import time
import struct

import numpy as np
from PIL import Image as PILImage, ImageDraw
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .alignment import target_time
from .catalog import connection, edits, save_edits
from .clocks import Clocks
from .config import config_hash
from .exporter import export
from .profile import openarm_profile, resolve_capture_config
from .ros_bridge import joint_document, decode_image
from .ros_images import HeaderPairs
from .session import Session


def header(stamp):
    value = Header()
    value.stamp.sec, value.stamp.nanosec = divmod(stamp, 10**9)
    value.frame_id = 'synthetic'
    return value


def synthetic_config(profile=None):
    profile = profile or openarm_profile()
    cameras = {c['id']: {'kind': 'rgbd', 'transport': 'ros_images', 'fps': profile['fps'],
                           'depth_source': {
                               'schema_version':'hc-depth-source/1','encoding':'16UC1','unit':'mm',
                               'scale_m_per_unit':.001,'invalid_values':[0],'invalid_nonfinite':True,
                               'registered_to_rgb':False,'registration_evidence':'synthetic independent pixel grids',
                               'evidence':{'kind':'synthetic_fixture','reference':'middleware.core.capture.synthetic',
                                           'version':'explicit-mm-v1'}},
                           'rgb_topic': '/synthetic/' + c['id'] + '/rgb', 'depth_topic': '/synthetic/' + c['id'] + '/depth'}
               for c in profile['cameras']}
    arms = [a['name'] for a in profile['axes'] if a['group'] == 'arm']
    grippers = [a['name'] for a in profile['axes'] if a['group'] == 'gripper']
    cfg = resolve_capture_config(profile,
        [{'joint_names': arms, 'platform_joint_state_topic': '/synthetic/state', 'platform_joint_command_topic': '/synthetic/action'}],
        [{'gripper_names': grippers, 'position_units': ['m'] * len(grippers),
          'platform_gripper_state_topic': '/synthetic/gripper_state', 'platform_gripper_command_topic': '/synthetic/gripper_action'}],
        cameras, alignment={'camera_validation': 'timestamps', 'delay_ms': 120, 'camera_max_error_ms': 8,
                            'rgbd_max_midpoint_skew_ms': 8, 'camera_pair_max_skew_ms': 8,
                            'joint_max_bracket_gap_ms': 60, 'primary_camera': profile['cameras'][0]['id']})
    # Synthetic rendering/encoding competes with other developer tests. Give the
    # offline publisher a declared delivery budget without relaxing source-time,
    # action freshness or camera association criteria.
    cfg['alignment']['delay_ms'] = 500
    cfg['alignment']['max_catchup_rows'] = 64
    cfg['buffers']['retention_ms'] = 1000
    cfg['storage'].update(minimum_free_bytes=0, numeric_flush_seconds=.02, video_segment_seconds=.3)
    cfg['dataset']['duration_seconds'] = 21 / profile['fps']
    cfg['sources']['synthetic_cloud']={'kind':'pointcloud','transport':'envelope','required':True,
                                      'persist':True,'camera_source_id':profile['cameras'][0]['id']}
    return cfg


def generate(root, profile=None, *, on_complete=None, task_id='synthetic-task-A', session_id='synthetic-openarm-A', raw_only=False):
    from sensor_msgs.msg import Image, PointCloud2, PointField
    root = Path(root)
    root.mkdir(parents=True, exist_ok=False)
    cfg = synthetic_config(profile)
    profile = cfg['capture']['profile']
    reports = {ident: {'registered_monotonic_ns': time.monotonic_ns()} for ident in cfg['sources']}
    identity = {'capture_context': {'local_robot_id': 'synthetic-openarm-local', 'robot_id': 'synthetic-platform-robot',
        'collection_task_id': task_id, 'synthetic': True, 'configuration_sha256': config_hash(cfg),
        'model_sha256': hashlib.sha256(b'synthetic-no-physical-model').hexdigest()}}
    session = Session(root / 'sessions', cfg, Clocks(), reports, identity, session_id=session_id,raw_only=raw_only)
    session.start()
    base = session.t0 if raw_only else time.time_ns() + 30_000_000
    pairs = {c['id']: HeaderPairs() for c in profile['cameras']}
    sequences = {ident: 0 for ident in cfg['sources']}
    tick = camera_frame = 0
    axis_index = {a['name']: i for i, a in enumerate(profile['axes'])}
    try:
        while not session.closed.is_set():
            now = time.time_ns()
            while base + tick * 10_000_000 <= now and tick <= 85:
                stamp = base + tick * 10_000_000
                for ident, source in cfg['sources'].items():
                    if source['kind'] not in {'state', 'action'}:
                        continue
                    names = [a['name'] for a in source['named_axes']]
                    is_gripper = source['named_axes'][0]['group'] == 'gripper'
                    if is_gripper and tick % 5:
                        continue  # actual OpenArm active gripper cadence: 20 Hz
                    # Split every source; interleaving and reversed names are intentional.
                    groups = [[n] for n in names] if is_gripper else [names[:len(names)//2], names[len(names)//2:]]
                    for group in reversed(groups):
                        if not group:
                            continue
                        group = list(reversed(group))
                        values = [(0.01 + .005 * (axis_index[n] % 2) + tick * .00001) if is_gripper
                                  else ((axis_index[n] + 1) * .1 + tick * .001) for n in group]
                        if source['kind'] == 'action':
                            values = [v + (.001 if is_gripper else .05) for v in values]
                        sequences[ident] += 1
                        message = JointState(header=header(stamp), name=group, position=values)
                        doc = joint_document(ident, source, message, sequences[ident], time.time_ns(), time.monotonic_ns(),
                                             evidence={'validity_known': True, 'source_epoch': 'synthetic-control-0'})
                        accepted=session.submit(doc)
                        assert accepted['accepted'] or accepted['reason']=='session_not_accepting', accepted
                tick += 1
            while target_time(base, camera_frame, profile['fps']) <= now and camera_frame < 25:
                stamp = target_time(base, camera_frame, profile['fps'])
                # Primary last ensures all required sources warm up at source frame zero.
                for cindex, camera in reversed(list(enumerate(profile['cameras']))):
                    ident = camera['id']
                    pixels = np.full((camera['height'], camera['width'], 3), (camera_frame * 7 + cindex * 40) % 220, dtype=np.uint8)
                    picture = PILImage.fromarray(pixels)
                    ImageDraw.Draw(picture).text((20, 20), f'SYNTHETIC {ident} FRAME {camera_frame}', fill=(255, 255, 255))
                    pixels = np.asarray(picture)
                    depth = np.full((camera['height'], camera['width']), 1000 + camera_frame, dtype=np.uint16)
                    for key, array, encoding, step in [('rgb', pixels, 'rgb8', camera['width'] * 3),
                                                       ('depth', depth, '16UC1', camera['width'] * 2)]:
                        msg = Image(header=header(stamp), height=camera['height'], width=camera['width'], encoding=encoding,
                                    step=step, data=array.tobytes())
                        pair, dropped = pairs[ident].add(key, msg, time.time_ns(), time.monotonic_ns())
                        assert not dropped
                        if pair:
                            doc = pairs[ident].document(ident, cfg['sources'][ident], pair)
                            if ident==profile['cameras'][0]['id']:
                                cloud=PointCloud2(header=header(stamp),height=1,width=1,
                                    fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1)],
                                    point_step=4,row_step=4,data=struct.pack('<f',float(camera_frame)))
                                cloud_doc={k:v for k,v in doc.items() if k not in ('rgb','depth')}
                                cloud_doc.update(source_id='synthetic_cloud',camera_source_id=ident,
                                    source_depth_seq=doc['depth']['source_seq'],source_rgb_seq=doc['rgb']['source_seq'],
                                    processing_version='synthetic-raw-pointcloud/v1',
                                    pointcloud_layout={'width':cloud.width,'height':cloud.height,'point_step':cloud.point_step,
                                        'row_step':cloud.row_step,'is_bigendian':cloud.is_bigendian,'is_dense':cloud.is_dense,
                                        'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in cloud.fields]})
                                accepted=session.submit(cloud_doc,points=cloud.data)
                                assert accepted['accepted'] or accepted['reason']=='session_not_accepting', accepted
                            accepted=session.submit(doc, decode_image(pair['streams']['rgb'][0]), decode_image(pair['streams']['depth'][0]))
                            assert accepted['accepted'] or accepted['reason']=='session_not_accepting', accepted
                camera_frame += 1
            if time.time_ns() - base > 5_000_000_000:
                raise RuntimeError('Synthetic session timed out: ' + json.dumps(session.status()))
            time.sleep(.001)
        if session.state != 'completed':
            raise RuntimeError(json.dumps(session.status()))
    finally:
        if not session.closed.is_set():
            session.stop()
    if raw_only:
        from .raw_bundle import prepare
        result=prepare(session.path,root/'bundle')
        result.update(session=str(session.path),input_base_ns=base)
        (root/'generation.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
    with connection(session.path) as db:
        rows = [json.loads(body) for body, in db.execute('SELECT body FROM rows ORDER BY k')]
    if len(rows) != 21 or not all(r['valid'] for r in rows):
        raise RuntimeError('Synthetic frame validity: ' + json.dumps([(r['target_frame_index'], r['invalid_reasons']) for r in rows]))
    save_edits(session.path, {'episodes': [
        {'start_frame': 0, 'end_frame': 6, 'task': 'synthetic pick', 'outcome': 'success', 'source_episode_id': 'synthetic-pick'},
        {'start_frame': 15, 'end_frame': 21, 'task': 'synthetic place', 'outcome': 'failure', 'source_episode_id': 'synthetic-place'}],
        'invalid_intervals': [], 'notes': 'Synthetic only; not physical acceptance'}, edits(session.path)['etag'])
    result = export(session.path, root / 'bundle', on_complete=on_complete)
    result['session'] = str(session.path)
    result['input_base_ns'] = base
    (root / 'generation.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    print(json.dumps(generate(parser.parse_args().output), indent=2))
