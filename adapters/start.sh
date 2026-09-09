#!/usr/bin/env bash
# Generic robot-profile controller: IK solver and VR-to-standard-command adapter.
set -euo pipefail

COMPONENT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${COMPONENT_DIR}/.." && pwd)"
MIDDLEWARE_CONFIG="${HC_MIDDLEWARE_CONFIG:-${PROJECT_ROOT}/middleware/config.yaml}"
LOG_DIR=""
PIDS=()
_CLEANED=0

log() { echo -e "\033[1;34m[HC-Control]\033[0m $*"; }
warn() { echo -e "\033[1;33m[HC-Control WARN]\033[0m $*"; }
err() { echo -e "\033[1;31m[HC-Control ERR]\033[0m $*" >&2; }

cleanup() {
  if [[ "${_CLEANED}" -eq 1 ]]; then return; fi
  _CLEANED=1
  warn "正在关闭通用逆解与遥操作控制节点..."
  local pid
  for pid in "${PIDS[@]}"; do
    kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do wait "${pid}" 2>/dev/null || true; done
  log "通用控制层已退出；外部硬件适配项目未被触碰。"
}
trap cleanup EXIT
trap 'exit 0' HUP INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --log-dir)
      [[ $# -ge 2 ]] || { err "--log-dir 需要目录参数"; exit 2; }
      LOG_DIR="$2"; shift 2 ;;
    --config)
      [[ $# -ge 2 ]] || { err "--config 需要 YAML 路径"; exit 2; }
      MIDDLEWARE_CONFIG="$2"; shift 2 ;;
    -h|--help)
      echo "用法: $0 [--config middleware/config.yaml] [--log-dir DIR]"
      echo "只启动通用 IK 与遥操作控制；X1 等硬件驱动须由独立适配项目启动。"
      _CLEANED=1; exit 0 ;;
    *) err "未知参数: $1"; exit 2 ;;
  esac
done

if [[ -z "${LOG_DIR}" ]]; then
  LOG_DIR="${PROJECT_ROOT}/runtime/control_logs/session_$(date +'%Y%m%d_%H%M%S')"
fi
mkdir -p "${LOG_DIR}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-14}"
export PYTHONUNBUFFERED=1
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/.deps${PYTHONPATH:+:${PYTHONPATH}}"
export HC_MIDDLEWARE_CONFIG="${MIDDLEWARE_CONFIG}"

ROBOT_CONFIG_ROOT="${HC_ROBOT_CONFIG_ROOT:-$(/usr/bin/python3 -m middleware.profile_cli root --config "${MIDDLEWARE_CONFIG}")}"
ROBOT_NAME="${HC_ROBOT_NAME:-$(/usr/bin/python3 -m middleware.profile_cli active --config "${MIDDLEWARE_CONFIG}")}"
export HC_ROBOT_CONFIG_ROOT="${ROBOT_CONFIG_ROOT}"
export HC_ROBOT_NAME="${ROBOT_NAME}"
PROFILE_DIR="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}"
CONTROLLER_YML="${HC_CONTROLLER_CONFIG:-${PROFILE_DIR}/controller_v23.yml}"
ARM_CONFIG="${HC_ARM_TELEOP_CONFIG:-${PROFILE_DIR}/arm_teleop.yaml}"
V23_SCRIPT="${PROJECT_ROOT}/adapters/v23/script/control_v2_3_ros2.py"
CONTROLLER_PREFIX="${HC_CONTROLLER_PREFIX:-${HOME}/miniconda3/envs/hc-teleop-controller}"
CONTROLLER_COMMAND=()

for required in "${V23_SCRIPT}" "${CONTROLLER_YML}" "${ARM_CONFIG}"; do
  if [[ ! -f "${required}" ]]; then err "缺少机器人配置或程序: ${required}"; exit 2; fi
done
if ! /usr/bin/python3 -c 'import numpy, pybullet, rclpy, yaml' 2>/dev/null; then
  err "系统 Python 缺少遥操作运行依赖（numpy/pybullet/rclpy/yaml）"
  err "请运行 ${PROJECT_ROOT}/install.sh"
  exit 2
fi

if [[ -x "${CONTROLLER_PREFIX}/bin/python" ]]; then
  export HC_CONTROLLER_PREFIX="${CONTROLLER_PREFIX}"
  "${PROJECT_ROOT}/run_generic_controller.sh" --check
  CONTROLLER_COMMAND=("${PROJECT_ROOT}/run_generic_controller.sh" v23)
else
  if ! /usr/bin/python3 -c 'import numpy, pinocchio, rclpy, yaml' 2>/dev/null; then
    err "系统 Python 缺少 Pinocchio 控制依赖，且未找到 ${CONTROLLER_PREFIX}"
    err "请运行 ${PROJECT_ROOT}/install.sh 或设置 HC_CONTROLLER_PREFIX"
    exit 2
  fi
  warn "未找到独立控制器环境，使用系统 ROS 2 Python/Pinocchio"
  CONTROLLER_COMMAND=(/usr/bin/python3 -u "${V23_SCRIPT}" "${CONTROLLER_YML}")
fi

log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "机器人配置: ${ROBOT_NAME} (${PROFILE_DIR})"
log "启动 Pinocchio v23 逆解..."
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- "${CONTROLLER_COMMAND[@]}" \
  >"${LOG_DIR}/v23_solver.log" 2>&1 &
PIDS+=("$!")
sleep 1

log "启动 VR 到标准机器人接口控制节点..."
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 -u "${PROJECT_ROOT}/adapters/nodes/arm_controller.py" \
  --config "${ARM_CONFIG}" --backend v23 \
  >"${LOG_DIR}/teleop_controller.log" 2>&1 &
PIDS+=("$!")

log "通用控制层已启动，只使用 /hc_teleop 标准话题。"
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then err "子进程异常退出，请检查 ${LOG_DIR}"; fi
exit "${status}"
