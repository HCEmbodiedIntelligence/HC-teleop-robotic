#!/usr/bin/env bash
# Single product entry point: complete simulation or real-robot teleoperation.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="middleware"
MIDDLEWARE_CONFIG="${HC_MIDDLEWARE_CONFIG:-${PROJECT_ROOT}/middleware/config.yaml}"
HEADLESS=false
MONITOR=true
DIAGNOSTICS=true
[[ "${TELEOP_DIAGNOSTICS:-1}" != "0" ]] || DIAGNOSTICS=false
LOG_DIR=""
SERVER_HOST=""
SERVER_PORT=""

usage() {
  cat <<EOF
Usage: $0 [middleware|sim|teleop] [options]

  middleware Dashboard + VR gateway; launch PyBullet from System Configuration
  sim       VR discovery + Dashboard + IK/control + PyBullet
  teleop    VR discovery + Dashboard + IK/control for an external real driver

  With no mode, middleware is selected. Import/apply a robot ZIP and start
  PyBullet independently from Dashboard System Configuration.

Options:
  --headless            Run simulation without the PyBullet GUI (sim only)
  --config FILE         Middleware config (default: middleware/config.yaml)
  --log-dir DIR         Session log directory
  --no-monitor          Disable the operation event monitor
  --no-diagnostics      Disable simulation CSV diagnostics
  --host HOST           Override Dashboard bind host
  --port PORT           Override Dashboard port
  -h, --help            Show this help

Examples:
  ./run.sh
  ./run.sh sim --headless
  ./run.sh teleop
EOF
}

if (($#)); then
  case "$1" in
    middleware|sim|teleop)
      MODE="$1"
      shift
      ;;
    simulator)
      echo "Mode '$1' was removed. Use 'sim' or 'teleop'." >&2
      exit 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
  esac
fi

while (($#)); do
  case "$1" in
    --headless)
      HEADLESS=true
      shift
      ;;
    --config)
      [[ $# -ge 2 ]] || { echo "--config requires a file" >&2; exit 2; }
      MIDDLEWARE_CONFIG="$2"
      shift 2
      ;;
    --log-dir)
      [[ $# -ge 2 ]] || { echo "--log-dir requires a directory" >&2; exit 2; }
      LOG_DIR="$2"
      shift 2
      ;;
    --no-monitor)
      MONITOR=false
      shift
      ;;
    --no-diagnostics)
      DIAGNOSTICS=false
      shift
      ;;
    --host)
      [[ $# -ge 2 ]] || { echo "--host requires a value" >&2; exit 2; }
      SERVER_HOST="$2"
      shift 2
      ;;
    --port)
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      SERVER_PORT="$2"
      shift 2
      ;;
    --robot)
      echo "--robot was removed. Select the active profile in Dashboard/config.yaml." >&2
      exit 2
      ;;
    --sim-only|--v23|--reconstructed|--generic|--legacy)
      echo "The duplicate backend option '$1' was removed." >&2
      exit 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -f "${MIDDLEWARE_CONFIG}" ]] || {
  echo "Middleware config not found: ${MIDDLEWARE_CONFIG}" >&2
  exit 2
}

if [[ "${MODE}" == middleware ]]; then
  [[ "${HEADLESS}" == false ]] || { echo "Configure headless mode in Dashboard." >&2; exit 2; }
  unset HC_EXTERNAL_STACK HC_ROBOT_NAME HC_ROBOT_CONFIG_ROOT HC_TELEOP_MODE
  ARGS=(--config "${MIDDLEWARE_CONFIG}")
  [[ -z "${LOG_DIR}" ]] || ARGS+=(--log-dir "${LOG_DIR}")
  [[ "${MONITOR}" == true ]] || ARGS+=(--no-monitor)
  [[ -z "${SERVER_HOST}" ]] || ARGS+=(--host "${SERVER_HOST}")
  [[ -z "${SERVER_PORT}" ]] || ARGS+=(--port "${SERVER_PORT}")
  exec "${PROJECT_ROOT}/middleware/start.sh" "${ARGS[@]}"
fi
export HC_EXTERNAL_STACK=1

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID
  ROS_DOMAIN_ID="$(/usr/bin/python3 - "${MIDDLEWARE_CONFIG}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
print(config.get("ros", {}).get("domain_id", 14))
PY
)"
fi

set +u
source /opt/ros/humble/setup.bash
set -u
export HC_MIDDLEWARE_CONFIG="${MIDDLEWARE_CONFIG}"
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/.deps${PYTHONPATH:+:${PYTHONPATH}}"

ROBOT_CONFIG_ROOT="${HC_ROBOT_CONFIG_ROOT:-}"
ROBOT_NAME="${HC_ROBOT_NAME:-}"
if [[ -z "${ROBOT_CONFIG_ROOT}" ]]; then
  ROBOT_CONFIG_ROOT="$(/usr/bin/python3 -m middleware.profile_cli root --config "${MIDDLEWARE_CONFIG}")"
fi
if [[ -z "${ROBOT_NAME}" ]]; then
  ROBOT_NAME="$(/usr/bin/python3 -m middleware.profile_cli active --config "${MIDDLEWARE_CONFIG}")"
fi
PROFILE_DIR="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}"
if [[ "${MODE}" == teleop && "${HEADLESS}" == true ]]; then
  echo "--headless is only valid in sim mode." >&2
  exit 2
fi

if [[ -z "${LOG_DIR}" ]]; then
  LOG_DIR="${PROJECT_ROOT}/runtime/teleop_logs/session_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${LOG_DIR}/control" "${LOG_DIR}/middleware"
chmod 700 "${LOG_DIR}" "${LOG_DIR}/control" "${LOG_DIR}/middleware"
mkdir -p "${PROJECT_ROOT}/runtime/teleop_logs"
ln -sfn "${LOG_DIR}" "${PROJECT_ROOT}/runtime/teleop_logs/latest"

MIDDLEWARE_ARGS=(
  --config "${MIDDLEWARE_CONFIG}"
  --log-dir "${LOG_DIR}/middleware"
)
[[ "${MONITOR}" == true ]] || MIDDLEWARE_ARGS+=(--no-monitor)
[[ -z "${SERVER_HOST}" ]] || MIDDLEWARE_ARGS+=(--host "${SERVER_HOST}")
[[ -z "${SERVER_PORT}" ]] || MIDDLEWARE_ARGS+=(--port "${SERVER_PORT}")

SIM_ENTRY=""
SIM_ARGS=()
SIM_ROS_ARGS=()
if [[ "${MODE}" == sim ]]; then
  SIM_ENTRY="${PROJECT_ROOT}/simulation/general_sim_robot_control_node_ros2.py"

  [[ -d "${PROJECT_ROOT}/.deps/pybullet-3.2.6.dist-info" ]] || {
    echo "Simulation dependencies not found. Run ${PROJECT_ROOT}/install.sh first." >&2
    exit 2
  }
  [[ -f "${SIM_ENTRY}" ]] || { echo "Bundled simulator not found: ${SIM_ENTRY}" >&2; exit 2; }
  [[ -f "${PROFILE_DIR}/arm_teleop.yaml" || -f "${PROFILE_DIR}/vr_configs.yml" ]] || {
    echo "Active profile has no simulation config: ${PROFILE_DIR}/arm_teleop.yaml or vr_configs.yml" >&2
    exit 2
  }

  export HC_ROBOT_CONFIG_ROOT="${ROBOT_CONFIG_ROOT}"
  export HC_ROBOT_NAME="${ROBOT_NAME}"
  SIM_ARGS=(--profile "${PROFILE_DIR}")
  [[ "${HEADLESS}" == false ]] || SIM_ARGS+=(--headless)
  SIM_ROS_ARGS=(
    --ros-args
    -r /io_teleop/joint_states:=/hc_teleop/joint_states
    -r /io_teleop/joint_cmd:=/hc_teleop/joint_cmd
    -r /io_teleop/target_joint_from_vr:=/hc_teleop/target_joint_from_vr
    -r /io_teleop/target_finger_joints:=/hc_teleop/target_finger_joints
    -r /io_teleop/target_ee_poses:=/hc_teleop/target_ee_poses
    -r /io_teleop/target_gripper_status:=/hc_teleop/target_gripper_status
    -r /io_teleop/target_base_move:=/hc_teleop/target_base_move
    -r /io_teleop/hardware_ready:=/hc_teleop/hardware_ready
  )
fi

MIDDLEWARE_PID=""
CONTROL_PID=""
SIM_PID=""
DIAGNOSTICS_PID=""

collect_process_tree() {
  local parent="$1"
  local child
  while read -r child; do
    [[ -n "${child}" ]] || continue
    collect_process_tree "${child}"
  done < <(pgrep -P "${parent}" 2>/dev/null || true)
  printf '%s\n' "${parent}"
}

stop_tree() {
  local root_pid="$1"
  [[ -n "${root_pid}" ]] || return 0
  local -a tree=()
  mapfile -t tree < <(collect_process_tree "${root_pid}")
  ((${#tree[@]})) || return 0
  kill -TERM "${tree[@]}" 2>/dev/null || true
  for _ in {1..30}; do
    local alive=false
    local pid
    for pid in "${tree[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then alive=true; break; fi
    done
    [[ "${alive}" == true ]] || return 0
    sleep 0.1
  done
  for pid in "${tree[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
}

cleanup() {
  local status=$?
  trap - HUP INT TERM EXIT
  stop_tree "${DIAGNOSTICS_PID}"
  stop_tree "${SIM_PID}"
  stop_tree "${CONTROL_PID}"
  stop_tree "${MIDDLEWARE_PID}"
  [[ -z "${DIAGNOSTICS_PID}" ]] || wait "${DIAGNOSTICS_PID}" 2>/dev/null || true
  [[ -z "${SIM_PID}" ]] || wait "${SIM_PID}" 2>/dev/null || true
  [[ -z "${CONTROL_PID}" ]] || wait "${CONTROL_PID}" 2>/dev/null || true
  [[ -z "${MIDDLEWARE_PID}" ]] || wait "${MIDDLEWARE_PID}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup HUP INT TERM EXIT

# Start discovery first so the headset can pair while IK and simulation load.
export HC_TELEOP_MODE="${MODE}"
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- "${PROJECT_ROOT}/middleware/start.sh" "${MIDDLEWARE_ARGS[@]}" &
MIDDLEWARE_PID=$!
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- "${PROJECT_ROOT}/adapters/start.sh" \
  --config "${MIDDLEWARE_CONFIG}" --log-dir "${LOG_DIR}/control" &
CONTROL_PID=$!

if [[ "${MODE}" == sim ]]; then
  setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 "${SIM_ENTRY}" "${SIM_ARGS[@]}" "${SIM_ROS_ARGS[@]}" &
  SIM_PID=$!
  if [[ "${DIAGNOSTICS}" == true && -f "${PROJECT_ROOT}/tools/diagnostics/teleop_diagnostics.py" ]]; then
    DIAGNOSTICS_LOG="${TELEOP_LOG_PATH:-${LOG_DIR}/teleop_diagnostics.csv}"
    setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 -m tools.diagnostics.teleop_diagnostics \
      --output "${DIAGNOSTICS_LOG}" --rate "${TELEOP_LOG_RATE:-30}" &
    DIAGNOSTICS_PID=$!
    echo "[HC] diagnostics=${DIAGNOSTICS_LOG}"
  fi
fi

echo "[HC] mode=${MODE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} config=${MIDDLEWARE_CONFIG}"
echo "[HC] VR discovery, Dashboard and control are running. Ctrl+C stops this stack."
if [[ "${MODE}" == teleop ]]; then
  echo "[HC] Start the HC_X1 hardware project separately with ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
fi

REQUIRED_PIDS=("${MIDDLEWARE_PID}" "${CONTROL_PID}")
[[ -z "${SIM_PID}" ]] || REQUIRED_PIDS+=("${SIM_PID}")
set +e
wait -n "${REQUIRED_PIDS[@]}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then
  echo "[HC] A required component exited with status ${status}; see ${LOG_DIR}." >&2
fi
exit "${status}"
