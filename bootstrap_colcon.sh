#!/usr/bin/env bash
# Build only the decomposed native ROS 2 packages under src/.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

export COLCON_DEFAULTS_FILE="${PROJECT_ROOT}/colcon_defaults.yaml"

case "${1:-build}" in
  deps)
    rosdep install --from-paths "${PROJECT_ROOT}/src" --ignore-src -r -y
    ;;
  build)
    colcon --log-base "${PROJECT_ROOT}/log" build --base-paths "${PROJECT_ROOT}/src" \
      --build-base "${PROJECT_ROOT}/build" \
      --install-base "${PROJECT_ROOT}/install"
    ;;
  test)
    colcon --log-base "${PROJECT_ROOT}/log" test --base-paths "${PROJECT_ROOT}/src" \
      --build-base "${PROJECT_ROOT}/build" \
      --install-base "${PROJECT_ROOT}/install"
    colcon test-result --test-result-base "${PROJECT_ROOT}/build" --verbose
    ;;
  *)
    echo "usage: $0 [deps|build|test]" >&2
    exit 2
    ;;
esac
