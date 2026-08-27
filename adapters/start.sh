#!/usr/bin/env bash
# Generic robot-profile controller: IK solver and VR-to-standard-command adapter.
set -euo pipefail

COMPONENT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${COMPONENT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd -- "${PROJECT_ROOT}/.." && pwd)"
MIDDLEWARE_CONFIG="${HC_MIDDLEWARE_CONFIG:-${PROJECT_ROOT}/middleware/config.yaml}"
LOG_DIR=""
SOLVER_OVERRIDE="${HC_SOLVER_BACKEND:-}"
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
trap 'exit 0' INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --log-dir)
      [[ $# -ge 2 ]] || { err "--log-dir 需要目录参数"; exit 2; }
      LOG_DIR="$2"; shift 2 ;;
    --config)
      [[ $# -ge 2 ]] || { err "--config 需要 YAML 路径"; exit 2; }
      MIDDLEWARE_CONFIG="$2"; shift 2 ;;
    --solver-backend)
      [[ $# -ge 2 ]] || { err "--solver-backend 需要 v23 或 motion_server"; exit 2; }
      SOLVER_OVERRIDE="$2"; shift 2 ;;
    -h|--help)
      echo "用法: $0 [--config middleware/config.yaml] [--log-dir DIR] [--solver-backend NAME]"
      echo "解算后端可选 v23 或 motion_server；硬件驱动须由独立适配项目启动。"
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

ROBOT_CONFIG_ROOT="${HC_ROBOT_CONFIG_ROOT:-$(/usr/bin/python3 "${PROJECT_ROOT}/robot_profile_cli.py" root --config "${MIDDLEWARE_CONFIG}")}"
ROBOT_NAME="${HC_ROBOT_NAME:-$(/usr/bin/python3 "${PROJECT_ROOT}/robot_profile_cli.py" active --config "${MIDDLEWARE_CONFIG}")}"
export HC_ROBOT_CONFIG_ROOT="${ROBOT_CONFIG_ROOT}"
export HC_ROBOT_NAME="${ROBOT_NAME}"
PROFILE_DIR="${ROBOT_CONFIG_ROOT}/${ROBOT_NAME}"
CONTROLLER_YML="${HC_CONTROLLER_CONFIG:-${PROFILE_DIR}/controller_v23.yml}"
ARM_CONFIG="${HC_ARM_TELEOP_CONFIG:-${PROFILE_DIR}/arm_teleop.yaml}"
V23_SCRIPT="${PROJECT_ROOT}/adapters/v23/script/control_v2_3_ros2.py"
CONTROLLER_PREFIX="${HC_CONTROLLER_PREFIX:-${HOME}/miniconda3/envs/hc-teleop-controller}"
CONTROLLER_COMMAND=()

for required in "${ARM_CONFIG}"; do
  if [[ ! -f "${required}" ]]; then err "缺少机器人配置或程序: ${required}"; exit 2; fi
done
PLUGIN_ARGS=(resolve --config "${ARM_CONFIG}")
if [[ -n "${SOLVER_OVERRIDE}" ]]; then
  PLUGIN_ARGS+=(--override "${SOLVER_OVERRIDE}")
fi
SOLVER_BACKEND="$(/usr/bin/python3 -m adapters.solver_plugins.cli "${PLUGIN_ARGS[@]}")"

if [[ "${SOLVER_BACKEND}" == "v23" ]]; then
  for required in "${V23_SCRIPT}" "${CONTROLLER_YML}"; do
    if [[ ! -f "${required}" ]]; then err "缺少 V2.3 解算资源: ${required}"; exit 2; fi
  done
  if [[ -x "${CONTROLLER_PREFIX}/bin/python" ]]; then
    export HC_CONTROLLER_PREFIX="${CONTROLLER_PREFIX}"
    "${PROJECT_ROOT}/run_generic_controller.sh" --check
    CONTROLLER_COMMAND=("${PROJECT_ROOT}/run_generic_controller.sh" v23)
  else
    if ! /usr/bin/python3 -c 'import numpy, pinocchio, rclpy, yaml' 2>/dev/null; then
      err "系统 Python 缺少 Pinocchio 控制依赖，且未找到 ${CONTROLLER_PREFIX}"
      err "请运行 ${PROJECT_ROOT}/install.sh --sim 或设置 HC_CONTROLLER_PREFIX"
      exit 2
    fi
    warn "未找到独立控制器环境，使用系统 ROS 2 Python/Pinocchio"
    CONTROLLER_COMMAND=(/usr/bin/python3 -u "${V23_SCRIPT}" "${CONTROLLER_YML}")
  fi
else
  HUMANOID_ROOT="${HC_HUMANOID_ROOT:-${WORKSPACE_ROOT}/humanoid}"
  MOTION_SERVER_SETUP="${HC_MOTION_SERVER_SETUP:-${HUMANOID_ROOT}/install/setup.bash}"
  SDK_DEPS_PREFIX="${HUMANOID_MOTION_SDK_DEPS_PREFIX:-${HUMANOID_ROOT}/.sdk_deps}"
  if [[ -d "${SDK_DEPS_PREFIX}" ]]; then
    export HUMANOID_MOTION_SDK_DEPS_PREFIX="${SDK_DEPS_PREFIX}"
    export LD_LIBRARY_PATH="${SDK_DEPS_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  mapfile -t MOTION_RESOURCES < <(
    /usr/bin/python3 -m adapters.solver_plugins.cli motion-resources --config "${ARM_CONFIG}"
  )
  if [[ "${#MOTION_RESOURCES[@]}" -ne 5 ]]; then
    err "Motion Server 资源解析失败: ${ARM_CONFIG}"
    exit 2
  fi
  MOTION_PARAMS="${MOTION_RESOURCES[0]}"
  MOTION_CHANNELS="${MOTION_RESOURCES[1]}"
  MOTION_SDK="${MOTION_RESOURCES[2]}"
  MOTION_TOOLS="${MOTION_RESOURCES[3]}"
  MOTION_URDF="${MOTION_RESOURCES[4]}"
  for required in "${MOTION_SERVER_SETUP}" "${MOTION_PARAMS}" "${MOTION_CHANNELS}" \
    "${MOTION_SDK}" "${MOTION_TOOLS}" "${MOTION_URDF}"; do
    if [[ ! -f "${required}" ]]; then
      err "Motion Server 后端缺少资源: ${required}"
      err "默认同级工作空间: ${HUMANOID_ROOT}"
      err "先构建该 humanoid 工作空间，或设置 HC_HUMANOID_ROOT/HC_MOTION_SERVER_SETUP"
      exit 2
    fi
  done
  set +u
  source "${MOTION_SERVER_SETUP}"
  set -u
  CONTROLLER_COMMAND=(ros2 run humanoid_motion_server humanoid_motion_control_node
    --ros-args --params-file "${MOTION_PARAMS}"
    -p "channel_config_file:=${MOTION_CHANNELS}"
    -p "sdk_config_file:=${MOTION_SDK}"
    -p "tool_config_file:=${MOTION_TOOLS}"
    -p "urdf_file:=${MOTION_URDF}")
fi

log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "机器人配置: ${ROBOT_NAME} (${PROFILE_DIR})"
log "解算插件: ${SOLVER_BACKEND}"
log "启动 ${SOLVER_BACKEND} 解算进程..."
setsid "${CONTROLLER_COMMAND[@]}" \
  >"${LOG_DIR}/${SOLVER_BACKEND}_solver.log" 2>&1 &
PIDS+=("$!")
sleep 1

log "启动 VR 到标准机器人接口控制节点..."
setsid /usr/bin/python3 -u "${PROJECT_ROOT}/adapters/nodes/arm_controller.py" \
  --config "${ARM_CONFIG}" --backend "${SOLVER_BACKEND}" \
  >"${LOG_DIR}/teleop_controller.log" 2>&1 &
PIDS+=("$!")

log "通用控制层已启动，只使用 /hc_teleop 标准话题。"
set +e
wait -n "${PIDS[@]}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then err "子进程异常退出，请检查 ${LOG_DIR}"; fi
exit "${status}"
