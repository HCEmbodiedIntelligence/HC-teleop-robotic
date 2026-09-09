#!/usr/bin/env bash
# Product middleware process group: dashboard/config/profile import/recording/VR gateway.
set -euo pipefail

COMPONENT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${COMPONENT_DIR}/.." && pwd)"
CONFIG_PATH="${HC_MIDDLEWARE_CONFIG:-${COMPONENT_DIR}/config.yaml}"
LOG_DIR=""
START_MONITOR=true
SERVER_ARGS=()
PIDS=()
_CLEANED=0

log() { echo -e "\033[1;32m[HC-Middleware]\033[0m $*"; }
warn() { echo -e "\033[1;33m[HC-Middleware WARN]\033[0m $*"; }
err() { echo -e "\033[1;31m[HC-Middleware ERR]\033[0m $*"; }

cleanup() {
  if [[ "${_CLEANED}" -eq 1 ]]; then
    return
  fi
  _CLEANED=1
  warn "正在关闭网页、VR 网关和录制进程..."
  local pid
  for pid in "${PIDS[@]}"; do
    kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  log "中间件已退出；硬件驱动和控制适配节点未被触碰。"
}

trap cleanup EXIT
trap 'exit 0' HUP INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --log-dir)
      [[ $# -ge 2 ]] || { err "--log-dir 需要目录参数"; exit 2; }
      LOG_DIR="$2"
      shift 2
      ;;
    --config)
      [[ $# -ge 2 ]] || { err "--config 需要 YAML 路径"; exit 2; }
      CONFIG_PATH="$2"
      shift 2
      ;;
    --no-monitor)
      START_MONITOR=false
      shift
      ;;
    -h|--help)
      echo "用法: $0 [--config YAML] [--log-dir DIR] [--no-monitor] [--host HOST] [--port PORT]"
      echo "只启动：前端界面、系统配置、ZIP 解算配置导入、VR 网关与录制。"
      _CLEANED=1
      exit 0
      ;;
    *)
      SERVER_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${LOG_DIR}" ]]; then
  LOG_DIR="${PROJECT_ROOT}/runtime/middleware_logs/session_$(date +'%Y%m%d_%H%M%S')"
fi
mkdir -p "${LOG_DIR}"
chmod 700 "${LOG_DIR}"

if [[ ! -f "${CONFIG_PATH}" ]]; then
  err "中间件配置不存在: ${CONFIG_PATH}"
  exit 2
fi
if [[ ! -d "${PROJECT_ROOT}/.deps/aiohttp" ]] && ! /usr/bin/python3 -c "import aiohttp" 2>/dev/null; then
  err "缺少 aiohttp，请先运行 ${PROJECT_ROOT}/install.sh"
  exit 2
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

export PYTHONUNBUFFERED=1
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/.deps${PYTHONPATH:+:${PYTHONPATH}}"
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export HC_MIDDLEWARE_DOMAIN_CONFIG="${CONFIG_PATH}"
  export ROS_DOMAIN_ID="$(/usr/bin/python3 -c 'import os, yaml; cfg=yaml.safe_load(open(os.environ["HC_MIDDLEWARE_DOMAIN_CONFIG"])) or {}; print(cfg.get("ros", {}).get("domain_id", 14))')"
  unset HC_MIDDLEWARE_DOMAIN_CONFIG
fi

log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "配置: ${CONFIG_PATH}"
log "中间件日志目录: ${LOG_DIR}"

if [[ "${START_MONITOR}" == true ]]; then
    MONITOR_SCRIPT="${PROJECT_ROOT}/middleware/session_monitor.py"
  if [[ -f "${MONITOR_SCRIPT}" ]]; then
    log "启动遥操作事件记录器..."
    setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 -u "${MONITOR_SCRIPT}" \
      --log-file "${LOG_DIR}/teleop_operations.log" > "${LOG_DIR}/monitor.log" 2>&1 &
    PIDS+=("$!")
  fi
fi

log "启动前端、系统配置、ZIP 导入与 MCAP 录制服务..."
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 -u "${PROJECT_ROOT}/middleware/server.py" \
  --config "${CONFIG_PATH}" "${SERVER_ARGS[@]}" > >(tee "${LOG_DIR}/middleware.log") 2>&1 &
SERVER_PID="$!"
PIDS+=("${SERVER_PID}")

set +e
wait "${SERVER_PID}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then
  err "中间件异常退出，状态码 ${status}；请检查 ${LOG_DIR}/middleware.log"
fi
exit "${status}"
