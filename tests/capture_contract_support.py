"""Synthetic server documents using the parallel-contract.md wire format."""
import copy
from humanoid_upload.capture_plan import digest


def task_document(task='A', source_format='MCAP', *, version=1, revision=1, scheme=None):
    features = {key: {'dtype': 'float32', 'shape': [1], 'names': ['joint'], 'units': ['rad'],
                     'role': role, 'required': True, 'missing': 'presence_and_validity',
                     'validity_feature': None, 'media': {}}
                for key, role in (('observation.state', 'state'), ('action', 'action'))}
    features['action.base.velocity'] = {'dtype': 'float32', 'shape': [6],
        'names': ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'], 'units': [], 'role': 'numeric',
        'required': False, 'missing': 'presence_and_validity', 'validity_feature': None, 'media': {}}
    features['observation.images.head'] = {'dtype': 'video', 'shape': [32, 32, 3],
        'names': ['height', 'width', 'channels'], 'units': [], 'role': 'rgb',
        'required': False, 'missing': 'presence_and_validity', 'validity_feature': None, 'media': {}}
    output = {'algorithm': 'capture-structure/v1', 'frequency_policy': 'PRESERVE',
              'frequency_hz': 15.0, 'time_source': 'STATE_LOG_TIME' if source_format == 'MCAP' else 'FRAME_TIMESTAMP',
              'features': features}
    plan = {'revision': revision, 'plan_sha256': digest([task, revision, version, source_format]),
            'scheme': {'scheme_id': scheme or 'scheme-'+task, 'version': version,
                       'content_sha256': digest([scheme or task, version])},
            'mapping_version': 1, 'mapping_sha256': digest(['mapping', task]),
            'source_format': source_format, 'output': output}
    return dict(name='Task '+task, task_code=task, description='Synthetic capture', project_name='Project '+task,
                project_id='project-'+task, collection_task_id=task, selectable=True,
                ownership=dict(organization_id='org', project_id='project-'+task, dataset_id='dataset-'+task,
                               collection_task_id=task, region_code='region'), capture_plan=plan)


def snapshot_document(task, command, robot_id='robot'):
    plan = task['capture_plan']
    adapter = ({'source_format': 'MCAP', 'parser_version': 'native-mcap-recording/2',
                'configuration': {'state_topic': '/test', 'action_topic': '/test', 'cameras': [],
                                  'time_source': 'STATE_LOG_TIME', 'fps': 15}}
               if command['source_format'] == 'MCAP' else
               {'source_format': 'LEROBOT_V3', 'parser_version': 'lerobot-v3/1',
                'codebase_version': 'v3.0', 'feature_sources': {}})
    content = {'schema_version': 'capture-recording-snapshot/v1', **task['ownership'],
               'robot_id': robot_id, 'recording_id': command['recording_id'],
               'plan_revision': command['expected_plan_revision'],
               **{key: copy.deepcopy(plan[key]) for key in ('plan_sha256', 'scheme', 'mapping_version', 'mapping_sha256', 'output')},
               'source_format': command['source_format'], 'effective_adapter': adapter,
               'output_contract_sha256': digest(plan['output']), 'source': 'TASK_PLAN',
               'prepared_at': '2026-10-21T00:00:00+00:00'}
    return {**content, 'snapshot_sha256': digest(content)}
