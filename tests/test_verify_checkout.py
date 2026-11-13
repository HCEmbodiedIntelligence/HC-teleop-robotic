"""Exercise the version gate against real temporary Git repositories."""
import importlib.util
from pathlib import Path
import subprocess

import pytest

script = Path(__file__).with_name('verify_checkout.py')
if not script.is_file():
    script = Path(__file__).resolve().parents[1] / 'tools/runtime/verify_checkout.py'
spec = importlib.util.spec_from_file_location('verify_checkout', script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path),
        '-c', 'user.name=Version test', '-c', 'user.email=test@example.invalid',
        *args], text=True, stderr=subprocess.PIPE).strip()


@pytest.fixture
def checkout(tmp_path):
    child = tmp_path / 'src/humanoid_motion_server'
    child.mkdir(parents=True)
    git(tmp_path, 'init')
    git(child, 'init')
    source = child / 'source.txt'
    source.write_text('first\n')
    git(child, 'add', 'source.txt')
    git(child, 'commit', '-m', 'First')
    pin = git(child, 'rev-parse', 'HEAD')
    git(tmp_path, 'update-index', '--add', '--cacheinfo',
        f'160000,{pin},src/humanoid_motion_server')
    git(tmp_path, 'commit', '-m', 'Pin child')
    return tmp_path, child, source, pin


def test_matching_pin_passes(checkout):
    root, _, _, pin = checkout
    assert module.verify_checkout(root)['motion_server_commit'] == pin


def test_changed_revision_is_rejected(checkout):
    root, child, source, _ = checkout
    source.write_text('second\n')
    git(child, 'commit', '-am', 'Second')
    with pytest.raises(ValueError, match='revision mismatch'):
        module.verify_checkout(root)


def test_dirty_tracked_child_is_rejected(checkout):
    root, _, source, _ = checkout
    source.write_text('local change\n')
    with pytest.raises(ValueError, match='tracked local changes'):
        module.verify_checkout(root)
