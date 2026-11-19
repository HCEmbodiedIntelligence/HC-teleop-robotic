"""Resolve actual startup choices without importing ROS or the web application."""
import argparse
import os
from pathlib import Path
import yaml


def backend(config, override=None):
    value = override or config.get('control', {}).get('backend', 'motion')
    if value != 'motion':
        raise ValueError('control backend must be motion')
    return value


def unified_profile(profile_dir):
    """Use one explicit backend choice for middleware, controller and simulation."""
    profile = Path(profile_dir)
    teleop = profile / 'arm_teleop.yaml'
    config = (yaml.safe_load(teleop.read_text()) or {}) if teleop.is_file() else {}
    unified_enabled = os.environ.get('HC_UNIFIED_ROBOT_API', '0') == '1' or bool(
        config.get('control', {}).get('unified_api', False)
    )
    return (
        unified_enabled
        and (profile / 'robot_api.yaml').is_file()
        and backend(config, os.environ.get('HC_CONTROL_BACKEND')) == 'motion'
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('field', choices=['backend', 'domain'])
    parser.add_argument('config')
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text()) or {}
    if args.field == 'backend':
        print(backend(config, os.environ.get('HC_CONTROL_BACKEND')))
    else:
        value = config.get('ros', {}).get('domain_id', 14)
        if type(value) is not int or not 0 <= value <= 232:
            raise ValueError('ROS domain must be an integer from 0 through 232')
        print(value)


if __name__ == '__main__':
    main()
