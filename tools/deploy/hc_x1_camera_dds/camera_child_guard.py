"""Fence a replacement camera process to an existing camera launch lifetime.

Used only for a scoped hot replacement: normal launches own their children.
The guard signals only the process group it creates, never an adopted PID.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def process_identity(pid: int) -> str | None:
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return fields[19] if fields[0] != "Z" else None
    except (OSError, ValueError, IndexError):
        return None


def group_alive(group: int) -> bool:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] != "Z" and int(fields[2]) == group:
                return True
        except (OSError, ValueError, IndexError):
            pass
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--launch-pid", type=int, required=True)
    parser.add_argument("--pid-file", type=Path)
    parser.add_argument("--grace", type=float, default=3.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if args.launch_pid <= 1 or args.grace <= 0 or not command:
        parser.error("launch PID > 1, positive grace and command required")
    identity = process_identity(args.launch_pid)
    if identity is None:
        return 1
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    for signum in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop)
    child = subprocess.Popen(command, start_new_session=True)
    try:
        if args.pid_file:
            args.pid_file.write_text(json.dumps({
                "guard_pid": os.getpid(), "child_pid": child.pid,
                "child_starttime": process_identity(child.pid),
                "launch_pid": args.launch_pid, "launch_starttime": identity,
            }))
        # Observe procfs without waitpid/poll, keeping an exited child's PID
        # reserved until the owned process group has been cleaned up.
        while (not stopping and process_identity(child.pid) is not None
               and process_identity(args.launch_pid) == identity):
            time.sleep(0.1)
    finally:
        # This group was created above, and the child is unreaped until cleanup.
        # A vanished/reused external launch can never redirect these signals.
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + args.grace
        while group_alive(child.pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        if group_alive(child.pid):
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        child.wait()
    return child.returncode if child.returncode >= 0 else 128 - child.returncode


if __name__ == "__main__":
    raise SystemExit(main())
