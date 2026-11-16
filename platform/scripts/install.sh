#!/usr/bin/env bash
set -euo pipefail
HC_PLATFORM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
HC_PLATFORM_PYTHON="${HC_PLATFORM_PYTHON:-/usr/bin/python3}"
HC_PLATFORM_SKIP_PYTHON=0
HC_PLATFORM_SKIP_WEB=0
for arg in "$@"; do
  case "$arg" in
    --skip-python) HC_PLATFORM_SKIP_PYTHON=1 ;;
    --skip-web) HC_PLATFORM_SKIP_WEB=1 ;;
    *) echo 'Usage: ./install.sh [--skip-python] [--skip-web]' >&2; exit 2 ;;
  esac
done
command -v cmake >/dev/null || { echo 'Install cmake and g++ first.' >&2; exit 2; }
[[ -d /usr/include/eigen3 ]] || { echo 'Install libeigen3-dev first.' >&2; exit 2; }
"$HC_PLATFORM_PYTHON" -c 'import sys; assert (3,10) <= sys.version_info < (3,12), "Use the ROS-compatible Python 3.10/3.11 interpreter"'
cd "$HC_PLATFORM_ROOT"
if [[ "$HC_PLATFORM_SKIP_PYTHON" == 0 ]]; then
  "$HC_PLATFORM_PYTHON" -m pip install --upgrade --target "$HC_PLATFORM_ROOT/.platform-deps" -r platform/requirements.lock
fi
cmake -S platform/control -B platform/control/build -DCMAKE_BUILD_TYPE=Release
cmake --build platform/control/build --parallel 2
cmake -S platform/kinematics -B platform/kinematics/build -DCMAKE_BUILD_TYPE=Release
cmake --build platform/kinematics/build --parallel 2
if [[ "$HC_PLATFORM_SKIP_WEB" == 0 ]]; then
  command -v npm >/dev/null || { echo 'Install Node.js 20 or newer first.' >&2; exit 2; }
  npm --prefix platform/web ci
  npm --prefix platform/web run build
fi
echo 'Installed the new process runtime. Start an isolated simulation with ./run.sh sim.'
