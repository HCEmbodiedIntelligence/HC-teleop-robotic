#!/usr/bin/env bash
# Motion only; does not start a driver, camera, VR receiver or middleware.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: ./tools/run_motion.sh [ROS launch arguments, e.g. urdf_file:=...]'
  echo 'Use --profile adapters/robots/x1 for the X1 model. Default: bundled test humanoid; feedback /observation/joint_states; output /control/motion/joint_targets.'
  exit 0
fi
[[ -f "${PROJECT_ROOT}/install_motion/setup.bash" ]] || { echo 'Run ./bootstrap_colcon.sh build first.' >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
source "${PROJECT_ROOT}/install_motion/setup.bash"
set -u
export PATH="/usr/bin:${PATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
SDK_PREFIX="${HUMANOID_MOTION_SDK_DEPS_PREFIX:-${PROJECT_ROOT}/.deps/robo_manip}"
export LD_LIBRARY_PATH="${SDK_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
ARGS=()
if [[ "${1:-}" == --profile ]]; then
  [[ $# -ge 2 ]] || { echo '--profile requires a profile directory' >&2; exit 2; }
  PROFILE="$(cd -- "$2" && pwd)"; shift 2
  URDF="$(/usr/bin/python3 - "$PROFILE" <<'PYCONF'
import sys,yaml
from pathlib import Path
p=Path(sys.argv[1]);c=yaml.safe_load((p/'arm_teleop.yaml').read_text())
print((p/c['robot']['urdf_path']).resolve())
PYCONF
)"
  ARGS=("params_file:=${PROFILE}/motion/motion.yaml" "channel_config_file:=${PROFILE}/motion/channels.yaml"
        "sdk_config_file:=${PROFILE}/motion/sdk.yaml" "tool_config_file:=${PROFILE}/motion/tools.yaml" "urdf_file:=${URDF}")
fi
exec ros2 launch "${PROJECT_ROOT}/tools/motion.launch.py" "${ARGS[@]}" "$@"
