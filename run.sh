#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Ensure system Python 3.10 and Node v20 take precedence over Conda Python 3.13
export PATH="/home/maple/.nvm/versions/node/v20.20.2/bin:/usr/bin:${PATH}"

# Workspace check
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

# Record domain ID for RViz and helper scripts
mkdir -p "${SCRIPT_DIR}/runtime"
ACTIVE_DOMAIN_FILE="${SCRIPT_DIR}/runtime/active_ros_domain"
ACTIVE_DOMAIN_TMP="${ACTIVE_DOMAIN_FILE}.$$"
printf '%s\n' "${ROS_DOMAIN_ID}" > "${ACTIVE_DOMAIN_TMP}"
mv -f "${ACTIVE_DOMAIN_TMP}" "${ACTIVE_DOMAIN_FILE}"

# Usage help
usage() {
  cat <<EOF
Usage: $0 [mode|profile:=<name>] [options]

Modes:
  web (default)                Start the humanoid web configurator and manager (http://localhost:7876)
  sim [openarmx|x1]            Start full simulation stack for the robot
  mock                         Start mock driver + motion server stack
  driver                       Start humanoid_driver_runtime bringup
  camera                       Start humanoid_camera multi-camera launch
  launch <pkg> <file> [args]   Run arbitrary ros2 launch command

Launch Arguments:
  profile:=<name>              Robot profile name (openarmx, x1; default: openarmx)
  mode:=<sim|real|shadow>      Operating mode (default: sim)
  --rviz-sim:=<true|false>     Start RViz with robot visualization (default: true)
  --headless                   Equivalent to --rviz-sim:=false
  --web:=<true|false>          Start Web Dashboard on port 7876 (default: true)
  --no-web                     Equivalent to --web:=false

Examples:
  ./run.sh                                    # Start Web Dashboard
  ./run.sh sim openarmx                       # Start OpenArmX simulation with RViz & Web Dashboard
  ./run.sh profile:=openarmx mode:=sim        # Standard ROS launch syntax
  ./run.sh profile:=x1 mode:=sim              # Start X1 simulation with RViz & Web Dashboard
  ./run.sh profile:=openarmx mode:=sim --rviz-sim:=false  # Headless sim with Web Dashboard
  ./run.sh profile:=openarmx mode:=sim --no-web           # Sim without Web Dashboard
  ./run.sh mock                               # Low-level mock stack
EOF
}

# Check for special first-position mode keywords
FIRST_ARG="${1:-}"

case "${FIRST_ARG}" in
  help|-h|--help)
    usage
    exit 0
    ;;
  web|--web)
    shift || true
    echo "[HC-Teleop] Starting Web Configurator on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
    exec "${SCRIPT_DIR}/src/humanoid_adapter_manager/start_configurator.sh" "$@"
    ;;
  mock)
    shift
    echo "[HC-Teleop] Starting Mock Stack on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
    exec ros2 launch humanoid_motion_server mock.launch.py "$@"
    ;;
  driver)
    shift
    echo "[HC-Teleop] Starting Driver Runtime on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
    exec ros2 launch humanoid_driver_runtime bringup.launch.py "$@"
    ;;
  camera)
    shift
    echo "[HC-Teleop] Starting Camera Stack on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
    exec ros2 launch humanoid_camera multi_camera.launch.py "$@"
    ;;
  launch)
    shift
    exec ros2 launch "$@"
    ;;
  sim)
    shift
    ROBOT_PROFILE="${1:-openarmx}"
    [[ $# -ge 1 ]] && shift || true
    LAUNCH_ARGS=("profile:=${ROBOT_PROFILE}" "mode:=sim")
    ;;
  "")
    echo "[HC-Teleop] No arguments provided; starting Web Configurator (http://localhost:7876)..."
    exec "${SCRIPT_DIR}/src/humanoid_adapter_manager/start_configurator.sh"
    ;;
  *)
    LAUNCH_ARGS=()
    ;;
esac

# Parse remaining arguments and normalize switches
for argument in "$@"; do
  case "${argument}" in
    --rviz-sim:=*|rviz-sim:=*)
      value="${argument#*:=}"
      case "${value}" in
        true|false) LAUNCH_ARGS+=("rviz_sim:=${value}") ;;
        *) echo "rviz-sim must be true or false" >&2; exit 2 ;;
      esac
      ;;
    --headless)
      LAUNCH_ARGS+=("rviz_sim:=false" "headless:=true")
      ;;
    --web:=*|web:=*)
      value="${argument#*:=}"
      case "${value}" in
        true|false) LAUNCH_ARGS+=("start_web:=${value}") ;;
        *) echo "web must be true or false" >&2; exit 2 ;;
      esac
      ;;
    --no-web)
      LAUNCH_ARGS+=("start_web:=false")
      ;;
    --port:=*|port:=*)
      value="${argument#*:=}"
      LAUNCH_ARGS+=("web_port:=${value}")
      ;;
    --host:=*|host:=*)
      value="${argument#*:=}"
      LAUNCH_ARGS+=("web_host:=${value}")
      ;;
    *)
      LAUNCH_ARGS+=("${argument}")
      ;;
  esac
done

LAUNCH_SCRIPT="${SCRIPT_DIR}/launch/teleop.launch.py"
if [[ ! -f "${LAUNCH_SCRIPT}" ]]; then
  LAUNCH_SCRIPT="${SCRIPT_DIR}/src/hc_teleop_recv/launch/teleop.launch.py"
fi

echo "[HC-Teleop] Starting Teleop/Simulation on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
exec ros2 launch "${LAUNCH_SCRIPT}" "${LAUNCH_ARGS[@]}"
