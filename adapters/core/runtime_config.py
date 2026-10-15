"""Resolve actual startup choices without importing ROS or the web application."""
import argparse
import os
from pathlib import Path
import yaml


def backend(config, override=None):
    value = override or config.get('control', {}).get('backend', 'motion')
    if value not in {'v23', 'motion'}:
        raise ValueError('control backend must be v23 or motion')
    return value


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
