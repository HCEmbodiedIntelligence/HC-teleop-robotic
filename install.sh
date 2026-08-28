#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

case "${1:-deps}" in
  deps)
    rosdep install --from-paths "${PROJECT_ROOT}/src" --ignore-src -r -y
    ;;
  build)
    "${PROJECT_ROOT}/bootstrap_colcon.sh" build
    ;;
  test)
    "${PROJECT_ROOT}/bootstrap_colcon.sh" test
    ;;
  *)
    echo "用法: $0 [deps|build|test]" >&2
    exit 2
    ;;
esac
