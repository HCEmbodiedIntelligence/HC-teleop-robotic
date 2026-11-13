#!/usr/bin/env python3
"""Reject a motion-server checkout that differs from the parent repository pin.

This check is local and read-only: it never fetches, checks out, or repairs code.
Run before starting robot control, while output is disabled.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess


def verify_checkout(root: Path) -> dict[str, str]:
    """Return the verified main/submodule commits, rejecting mixed revisions."""
    root = root.resolve()
    def git(path: Path, *args: str) -> str:
        return subprocess.check_output(
            ['git', '-C', str(path), *args], text=True,
            stderr=subprocess.PIPE, timeout=10).strip()
    main = git(root, 'rev-parse', 'HEAD')
    entry = git(root, 'ls-tree', 'HEAD', 'src/humanoid_motion_server').split()
    if len(entry) < 4 or entry[:2] != ['160000', 'commit']:
        raise ValueError('parent revision has no motion-server submodule pin')
    child = root / 'src/humanoid_motion_server'
    actual = git(child, 'rev-parse', 'HEAD')
    if actual != entry[2]:
        raise ValueError(f'motion-server revision mismatch: expected={entry[2]} actual={actual}')
    if git(child, 'status', '--porcelain', '--untracked-files=no'):
        raise ValueError('motion-server has tracked local changes; cannot verify a unified version')
    return {'main_commit': main, 'motion_server_commit': actual}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    try:
        result = verify_checkout(args.root)
    except (ValueError, subprocess.SubprocessError, OSError) as error:
        print(f'[version-check] BLOCKED: {error}')
        return 1
    print('[version-check] ' + json.dumps(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
