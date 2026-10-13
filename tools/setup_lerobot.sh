#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS stays in its existing Python; LeRobot/torch are isolated here.
command -v uv >/dev/null || { echo 'Install uv first: https://docs.astral.sh/uv/getting-started/installation/' >&2; exit 1; }
uv venv --python 3.12 "$project_root/.venv-lerobot"
uv pip install --python "$project_root/.venv-lerobot/bin/python" 'lerobot==0.4.3' --torch-backend cpu
