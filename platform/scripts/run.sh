#!/usr/bin/env bash
set -euo pipefail
HC_PLATFORM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
HC_PLATFORM_PYTHON="${HC_PLATFORM_PYTHON:-/usr/bin/python3}"
HC_PLATFORM_ROS_SETUP="${HC_PLATFORM_ROS_SETUP:-/opt/ros/humble/setup.bash}"
HC_PLATFORM_MODE=sim
if [[ "${1:-}" == sim || "${1:-}" == teleop ]]; then
  HC_PLATFORM_MODE="$1"
  shift
fi
HC_PLATFORM_ARGS=()
for arg in "$@"; do
  case "$arg" in
    -h|--help)
      echo "Usage: ./run.sh [sim|teleop] [--headless] [--robot <id>] [--host <ip>] [--port <n>] [--state-dir <dir>]"
      exit 0
      ;;
    --headless) ;; # PyBullet DIRECT and browser 3D are always used.
    middleware) echo 'The new entry starts simulation by default. Use sim or teleop.' >&2; exit 2 ;;
    *) HC_PLATFORM_ARGS+=("$arg") ;;
  esac
done
if [[ ! -r "$HC_PLATFORM_ROS_SETUP" ]]; then
  echo "ROS setup missing: $HC_PLATFORM_ROS_SETUP. Install ROS 2 Humble or set HC_PLATFORM_ROS_SETUP." >&2
  exit 2
fi
set +u
source "$HC_PLATFORM_ROS_SETUP"
set -u
export PYTHONPATH="$HC_PLATFORM_ROOT/.platform-deps:$HC_PLATFORM_ROOT/platform/python${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONUNBUFFERED=1
cd "$HC_PLATFORM_ROOT"
exec "$HC_PLATFORM_PYTHON" -m hc_platform.api --repo "$HC_PLATFORM_ROOT" --mode "$HC_PLATFORM_MODE" "${HC_PLATFORM_ARGS[@]}"
