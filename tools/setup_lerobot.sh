#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS stays in its existing Python; LeRobot/torch are isolated here.
uv_bin="${UV_BIN:-$(command -v uv || true)}"
if [[ -z "$uv_bin" && -x "$HOME/.local/bin/uv" ]]; then
  uv_bin="$HOME/.local/bin/uv"
fi
if [[ -z "$uv_bin" ]]; then
  echo '[lerobot-setup] uv 未安装，正在安装到用户目录...'
  python3 -m pip install --user --upgrade uv
  uv_bin="$HOME/.local/bin/uv"
fi
if [[ ! -x "$uv_bin" ]]; then
  echo "[lerobot-setup ERR] 找不到可执行的 uv: $uv_bin" >&2
  exit 1
fi
"$uv_bin" venv --python 3.12 "$project_root/.venv-lerobot"
"$uv_bin" pip install --python "$project_root/.venv-lerobot/bin/python" 'lerobot==0.4.3' --torch-backend cpu
