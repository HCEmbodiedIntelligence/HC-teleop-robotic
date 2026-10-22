#!/usr/bin/env bash
# Motion only; does not start a driver, camera, VR receiver or middleware.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: ./tools/run_motion.sh [ROS launch arguments, e.g. urdf_file:=...]'
  echo 'Use --profile adapters/robots/x1 or the effective HC_ROBOT_NAME/config.yaml selection.'
  echo 'Generates checked runtime parameters; starts Motion only, never hardware.'
  exit 0
fi
MOTION_INSTALL="${HC_MOTION_INSTALL:-${PROJECT_ROOT}/install_motion}"
[[ -f "${MOTION_INSTALL}/setup.bash" ]] || { echo 'Run ./bootstrap_colcon.sh build first.' >&2; exit 2; }
set +u
source /opt/ros/humble/setup.bash
source "${MOTION_INSTALL}/setup.bash"
set -u
export PATH="/usr/bin:${PATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
# The unified SDK runtime verifies its own pinned dependency provenance.
ARGS=()
if [[ "${1:-}" == --profile ]]; then
  [[ $# -ge 2 ]] || { echo '--profile requires a profile directory' >&2; exit 2; }
  PROFILE="$(cd -- "$2" && pwd)"; shift 2
else
  PROFILE="$(/usr/bin/python3 "${PROJECT_ROOT}/middleware/profile_cli.py" profile-dir)"
fi
RUNTIME_DIR="${HC_ROBOT_API_RUNTIME_DIR:-$(mktemp -d /tmp/hc-robot-api.XXXXXX)}"
/usr/bin/python3 "${PROJECT_ROOT}/middleware/profile_cli.py" generate --profile "${PROFILE}" --output "${RUNTIME_DIR}" >&2
URDF="$(/usr/bin/python3 -c 'import sys,yaml; from pathlib import Path; p=Path(sys.argv[1]); print((p/yaml.safe_load((p/"arm_teleop.yaml").read_text())["robot"]["urdf_path"]).resolve())' "${PROFILE}")"
ARGS=("params_file:=${RUNTIME_DIR}/motion.yaml" "channel_config_file:=${RUNTIME_DIR}/channels.yaml"
      "sdk_config_file:=${RUNTIME_DIR}/sdk.yaml" "tool_config_file:=${PROFILE}/motion/tools.yaml" "urdf_file:=${URDF}"
      "robot_api_file:=${RUNTIME_DIR}/robot_api.resolved.yaml" "joint_state_endpoint:=/internal/robot/joint_states")
exec ros2 launch "${PROJECT_ROOT}/tools/motion.launch.py" "${ARGS[@]}" "$@"
