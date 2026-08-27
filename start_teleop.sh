#!/usr/bin/env bash
# Hardware-independent HC teleoperation stack: middleware + IK + VR control.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${PROJECT_ROOT}/.." && pwd)"
CONFIG_PATH="${HC_MIDDLEWARE_CONFIG:-${PROJECT_ROOT}/middleware/config.yaml}"
LOG_DIR=""
MIDDLEWARE_ARGS=()
CONTROL_ARGS=()
PIDS=()
_CLEANED=0

log() { echo -e "\033[1;32m[HC-Teleop]\033[0m $*"; }
warn() { echo -e "\033[1;33m[HC-Teleop WARN]\033[0m $*"; }
err() { echo -e "\033[1;31m[HC-Teleop ERR]\033[0m $*" >&2; }

cleanup() {
  if [[ "${_CLEANED}" -eq 1 ]]; then
    return
  fi
  _CLEANED=1
  warn "正在关闭通用中间件、IK 和遥操作控制..."
  local pid
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  log "通用遥操作栈已退出；仿真和外部硬件驱动未被触碰。"
}

trap cleanup EXIT
trap 'exit 0' INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      [[ $# -ge 2 ]] || { err "--config 需要 YAML 路径"; exit 2; }
      CONFIG_PATH="$2"
      shift 2
      ;;
    --log-dir)
      [[ $# -ge 2 ]] || { err "--log-dir 需要目录参数"; exit 2; }
      LOG_DIR="$2"
      shift 2
      ;;
    --solver-backend)
      [[ $# -ge 2 ]] || { err "--solver-backend 需要 v23 或 motion_server"; exit 2; }
      CONTROL_ARGS+=("$1" "$2")
      shift 2
      ;;
    --no-monitor)
      MIDDLEWARE_ARGS+=("$1")
      shift
      ;;
    --host|--port)
      [[ $# -ge 2 ]] || { err "$1 需要参数"; exit 2; }
      MIDDLEWARE_ARGS+=("$1" "$2")
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
用法: ./start_teleop.sh [选项]

一次启动与硬件无关的通用遥操作栈：
  - Dashboard、配置导入、VR UDP 网关和话题录制
  - 可插拔的 Pinocchio v2.3 或 Humanoid Motion Server IK
  - VR 到 /hc_teleop 标准接口的遥操作控制

选项:
  --config YAML   中间件配置文件
  --log-dir DIR   本次会话日志目录
  --solver-backend NAME
                  解算后端：v23（默认）或 motion_server
  --no-monitor    不启动操作事件记录器
  --host HOST     Dashboard 监听地址
  --port PORT     Dashboard 监听端口

本脚本不启动 PyBullet，也不启动 X1 或其他真机驱动。
EOF
      _CLEANED=1
      exit 0
      ;;
    *)
      err "未知参数: $1"
      exit 2
      ;;
  esac
done

if [[ ! -f "${CONFIG_PATH}" ]]; then
  err "中间件配置不存在: ${CONFIG_PATH}"
  exit 2
fi

if [[ -z "${LOG_DIR}" ]]; then
  SESSION_ID="$(date +'%Y%m%d_%H%M%S')"
  LOG_DIR="${PROJECT_ROOT}/runtime/teleop_logs/session_${SESSION_ID}"
fi
mkdir -p "${LOG_DIR}/control" "${LOG_DIR}/middleware"
ln -sfn "${LOG_DIR}" "${PROJECT_ROOT}/runtime/teleop_logs/latest"

# One ROS domain is shared by middleware, controller, simulator, and optional
# hardware adapters. Explicit ROS_DOMAIN_ID wins; the compatibility variable
# comes next; otherwise use middleware/config.yaml.
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  if [[ -n "${HC_TELEOP_ROS_DOMAIN_ID:-}" ]]; then
    export ROS_DOMAIN_ID="${HC_TELEOP_ROS_DOMAIN_ID}"
  else
    export HC_TELEOP_DOMAIN_CONFIG="${CONFIG_PATH}"
    export ROS_DOMAIN_ID="$(/usr/bin/python3 -c 'import os, yaml; cfg=yaml.safe_load(open(os.environ["HC_TELEOP_DOMAIN_CONFIG"])) or {}; print(cfg.get("ros", {}).get("domain_id", 14))')"
    unset HC_TELEOP_DOMAIN_CONFIG
  fi
fi
export HC_MIDDLEWARE_CONFIG="${CONFIG_PATH}"
# HC-teleop-robotic and humanoid use one common parent directory.  Export the
# resolved sibling path so every child process uses the same workspace.
export HC_HUMANOID_ROOT="${HC_HUMANOID_ROOT:-${WORKSPACE_ROOT}/humanoid}"

log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "配置: ${CONFIG_PATH}"
log "Humanoid 工作空间: ${HC_HUMANOID_ROOT}"
log "会话日志: ${LOG_DIR}"
log "启动通用 IK 与遥操作控制..."
setsid "${PROJECT_ROOT}/adapters/start.sh" \
  --config "${CONFIG_PATH}" \
  --log-dir "${LOG_DIR}/control" \
  "${CONTROL_ARGS[@]}" &
CONTROL_PID="$!"
PIDS+=("${CONTROL_PID}")

sleep 2
if ! kill -0 "${CONTROL_PID}" 2>/dev/null; then
  wait "${CONTROL_PID}" || true
  err "通用控制层启动失败，请检查 ${LOG_DIR}/control"
  exit 1
fi

log "启动 Dashboard、VR 网关与话题录制..."
setsid "${PROJECT_ROOT}/middleware/start.sh" \
  --config "${CONFIG_PATH}" \
  --log-dir "${LOG_DIR}/middleware" \
  "${MIDDLEWARE_ARGS[@]}" &
MIDDLEWARE_PID="$!"
PIDS+=("${MIDDLEWARE_PID}")

log "通用遥操作栈已启动；可独立运行，也可按需连接仿真或真机。"
set +e
wait -n "${CONTROL_PID}" "${MIDDLEWARE_PID}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then
  err "子进程异常退出，状态码 ${status}；请检查 ${LOG_DIR}"
fi
exit "${status}"
