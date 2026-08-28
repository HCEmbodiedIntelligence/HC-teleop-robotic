#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Native HC entry point for the decomposed ROS 2 runtime.
if [[ ! -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  echo "HC workspace is not built; run ./bootstrap_colcon.sh build first" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u
exec ros2 launch hc_bringup teleop.launch.py "$@"
