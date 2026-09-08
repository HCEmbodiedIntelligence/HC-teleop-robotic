#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Translate the user-facing switch to a ROS launch argument. Keep every other
# argument intact, including profile paths containing spaces.
LAUNCH_ARGS=()
for argument in "$@"; do
  case "${argument}" in
    --rviz-sim:=*|rviz-sim:=*)
      value="${argument#*:=}"
      case "${value}" in
        true|false) LAUNCH_ARGS+=("rviz_sim:=${value}") ;;
        *) echo "rviz-sim must be true or false" >&2; exit 2 ;;
      esac
      ;;
    *) LAUNCH_ARGS+=("${argument}") ;;
  esac
done

# Native HC entry point for the decomposed ROS 2 runtime.
if [[ ! -f "${SCRIPT_DIR}/install/setup.bash" ]]; then
  echo "HC workspace is not built; run ./bootstrap_colcon.sh build first" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# Record the domain selected by the control stack so helper processes started
# from another terminal (notably RViz) do not inherit a stale ROS_DOMAIN_ID.
mkdir -p "${SCRIPT_DIR}/runtime"
ACTIVE_DOMAIN_FILE="${SCRIPT_DIR}/runtime/active_ros_domain"
ACTIVE_DOMAIN_TMP="${ACTIVE_DOMAIN_FILE}.$$"
printf '%s\n' "${ROS_DOMAIN_ID}" > "${ACTIVE_DOMAIN_TMP}"
mv -f "${ACTIVE_DOMAIN_TMP}" "${ACTIVE_DOMAIN_FILE}"

echo "[HC-Teleop] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec ros2 launch hc_bringup teleop.launch.py "${LAUNCH_ARGS[@]}"
