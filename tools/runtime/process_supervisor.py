"""Own one isolated process group and stop it when its launching shell disappears."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def group_members(group):
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == group and fields[0] != 'Z':
                members.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            pass
    return members


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--parent', type=int, required=True)
    parser.add_argument('--grace', type=float, default=3.0)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or args.grace <= 0:
        parser.error('command and positive grace required')
    if os.getpgrp() != os.getpid():
        os.setsid()
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    if os.getppid() != args.parent:
        return 1
    child = subprocess.Popen(command)
    try:
        while child.poll() is None and not stopping and os.getppid() == args.parent:
            time.sleep(0.1)
    finally:
        # Own group only. No process-name matching and no external driver signals.
        os.killpg(os.getpgrp(), signal.SIGTERM)
        deadline = time.monotonic() + args.grace
        while time.monotonic() < deadline:
            child.poll()
            if not group_members(os.getpgrp()):
                break
            time.sleep(0.05)
        if group_members(os.getpgrp()):
            os.killpg(os.getpgrp(), signal.SIGKILL)
        child.wait()
    return child.returncode if child.returncode >= 0 else 128 - child.returncode


if __name__ == '__main__':
    raise SystemExit(main())
