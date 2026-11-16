"""Opt-in, isolated LeRobot converter installation. Never modifies ROS Python."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess

from .artifacts import atomic_json

PACKAGES = ("lerobot==0.4.3", "mcap==1.6.0", "mcap-ros2-support==0.5.7")


def install(directory, *, python="3.12", dry_run=False):
    directory = Path(directory).resolve()
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError(
            "uv is required; this installer does not alter user/global package directories"
        )
    executable = directory / "bin/python"
    marker = directory / "hc-converter.json"
    lockfile = Path(__file__).resolve().parents[3] / "requirements-converter.lock"
    if not lockfile.is_file():
        raise RuntimeError(
            "The repository's platform/requirements-converter.lock is required; refusing an unlocked install"
        )
    if directory.exists() and not marker.exists():
        raise RuntimeError(
            "Refusing to modify an existing environment not created by this installer"
        )
    commands = []
    if not executable.exists():
        commands.append([uv, "venv", "--python", python, str(directory)])
    commands.append(
        [
            uv,
            "pip",
            "install",
            "--python",
            str(executable),
            "--torch-backend",
            "cpu",
            "-r",
            str(lockfile),
        ]
    )
    verification = [
        str(executable),
        "-c",
        "import importlib.metadata as m; import pyarrow,mcap,mcap_ros2; from lerobot.datasets.lerobot_dataset import LeRobotDataset; "
        "assert m.version('lerobot')=='0.4.3'; assert hasattr(LeRobotDataset,'finalize'); print('LeRobot v3 writer dependencies ready')",
    ]
    if dry_run:
        return {
            "commands": commands + [verification],
            "HC_CONVERTER_PYTHON": str(executable),
            "packages": PACKAGES,
        }
    # ROS commonly injects ABI-specific extensions through PYTHONPATH. Those
    # must not enter the separately managed Python 3.12 environment.
    environment = {
        key: value
        for key, value in os.environ.items()
        if key not in {"PYTHONPATH", "PYTHONHOME"}
    }
    environment["PYTHONNOUSERSITE"] = "1"
    if marker.exists():
        atomic_json(
            marker,
            {
                "schema_version": "hc-converter-environment/v1",
                "state": "installing",
                "packages": PACKAGES,
            },
        )
    for index, command in enumerate(commands):
        subprocess.run(command, check=True)
        if index == 0:
            atomic_json(
                marker,
                {
                    "schema_version": "hc-converter-environment/v1",
                    "state": "installing",
                    "packages": PACKAGES,
                },
            )
    subprocess.run(verification, check=True, env=environment)
    freeze = subprocess.run(
        [uv, "pip", "freeze", "--python", str(executable)],
        check=True,
        capture_output=True,
        text=True,
    )
    (directory / "hc-converter.lock").write_text(freeze.stdout)
    atomic_json(
        marker,
        {
            "schema_version": "hc-converter-environment/v1",
            "state": "ready",
            "packages": PACKAGES,
            "python": str(executable),
            "lock": "hc-converter.lock",
        },
    )
    return {"HC_CONVERTER_PYTHON": str(executable), "state": "ready"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--venv", required=True)
    parser.add_argument("--python", default="3.12")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    print(
        json.dumps(
            install(args.venv, python=args.python, dry_run=args.dry_run), indent=2
        )
    )


if __name__ == "__main__":
    main()
