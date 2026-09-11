#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Translate the user-facing switch to a ROS launch argument. Keep every other
export PATH="/home/maple/.nvm/versions/node/v20.20.2/bin:/usr/bin:${PATH}"

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

case "${1:-web}" in
  mock)
    shift
    exec ros2 launch humanoid_motion_server mock.launch.py "$@"
    ;;
  driver)
    shift
    exec ros2 launch humanoid_driver_runtime bringup.launch.py "$@"
    ;;
  camera)
    shift
    exec ros2 launch humanoid_camera multi_camera.launch.py "$@"
    ;;
  launch)
    shift
    exec ros2 launch "$@"
    ;;
  web|--web)
    [[ "${1:-}" == "web" || "${1:-}" == "--web" ]] && shift || true
    exec "${SCRIPT_DIR}/src/humanoid_adapter_manager/start_configurator.sh" "$@"
    ;;
  help|-h|--help)
    echo "Usage: $0 [web|mock|driver|camera|launch <pkg> <launch_file>] [options]"
    echo ""
    echo "Modes:"
    echo "  web (default)    Start the humanoid web configurator and manager (http://localhost:7876)"
    echo "  mock             Start mock driver + motion server stack"
    echo "  driver           Start humanoid_driver_runtime bringup"
    echo "  camera           Start humanoid_camera multi-camera launch"
    echo "  launch ...       Run arbitrary ros2 launch command"
    exit 0
    ;;
  *)
    exec "${SCRIPT_DIR}/src/humanoid_adapter_manager/start_configurator.sh" "$@"
    ;;
esac
