"""Read-only runtime/profile/catalog data for the operator console."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes


def profile_summary(path):
    path = Path(path)
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError('profile must be a mapping')
    uri = value.get('resources', {}).get('urdf', '')
    if uri.startswith('package://'):
        package, relative = uri[len('package://'):].split('/', 1)
        urdf = Path(get_package_share_directory(package)) / relative
    else:
        urdf = path.parent / uri
    model = ET.parse(urdf).getroot()
    components = [c for c in value.get('components', []) if c.get('enabled', True)]
    arms = [c for c in components if c.get('kind') == 'arm']
    return {
        'id': value['robot_id'], 'display_name': value.get('display_name', value['robot_id']),
        'schema': value.get('schema', ''), 'urdf': uri,
        'joint_count': len(model.findall('joint')),
        'free_joint_count': len({j for c in arms for j in c.get('joint_names', [])}),
        'task_count': len(arms), 'arm_count': len(arms),
        'groups': [c['id'] for c in arms],
        'feedback_joint_order': value.get('standard_interfaces', {}).get('feedback_joint_order', []),
        'joint_names': {c['id']: c.get('joint_names', []) for c in arms},
        'vr': value.get('vr', {}), 'motion': value.get('motion', {}),
        'recording': value.get('recording', {}),
    }


def profiles(selected):
    result = []
    for package in sorted(get_packages_with_prefixes()):
        if package.startswith('hc_robot_'):
            path = Path(get_package_share_directory(package)) / 'config/profile.yaml'
            if path.is_file():
                try:
                    result.append(profile_summary(path))
                except (OSError, ValueError, KeyError, ET.ParseError, yaml.YAMLError):
                    continue
    path = Path(selected)
    if path.is_file():
        summary = profile_summary(path)
        result = [item for item in result if item['id'] != summary['id']] + [summary]
    return result


def datasets(root):
    """List manifests only; never create directories or scan arbitrary client paths."""
    root = Path(root).expanduser().resolve()
    result = []
    for path in sorted(root.glob('*/manifest.json'), reverse=True):
        if not path.resolve().is_relative_to(root):
            continue
        try:
            record = json.loads(path.read_text(encoding='utf-8'))
            if isinstance(record, dict) and record.get('schema') == 'hc-dataset/v1':
                result.append(record)
        except (OSError, ValueError):
            continue
    return {'directory': str(root), 'items': result}
