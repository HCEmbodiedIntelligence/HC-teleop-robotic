"""X1 recording format and coordinate compatibility."""
from pathlib import Path


def is_x1(config):
    return Path(config.get('legacy_control_profile') or '').name == 'x1'


def ordinary_x1_capture(metadata):
    return metadata.get('profile_id') == 'x1' and not metadata.get('capture_mode')
