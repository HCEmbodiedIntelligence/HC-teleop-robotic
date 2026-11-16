#!/usr/bin/env bash
set -euo pipefail
HC_PLATFORM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
HC_PLATFORM_PYTHON="${HC_PLATFORM_PYTHON:-/usr/bin/python3}"
if [[ -r /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
export PYTHONPATH="$HC_PLATFORM_ROOT/.platform-deps:$HC_PLATFORM_ROOT/platform/python${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd "$HC_PLATFORM_ROOT"
ctest --test-dir platform/control/build --output-on-failure
"$HC_PLATFORM_PYTHON" -m pytest platform/tests -q "$@"
npm --prefix platform/web run build
