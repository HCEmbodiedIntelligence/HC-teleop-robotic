#!/usr/bin/env bash
# HC-TJ 真机遥操作一键启动脚本
# 支持启动：
#   1. Marvin SDK / 腰部 / 灵巧手 / 相机驱动 (hc_io_suit)
#   2. Pinocchio v23 逆运动学求解器 (control_v2_3_ros2.py)
#   3. 机械臂遥操作控制器 (teleop_arm_controller.py)
#   4. Web 遥操作中间件与录制服务 (middleware_server.py)
#
# 用法:
#   ./start_real_robot_teleop.sh              # 启动完整系统（真机驱动 + 遥控 + 中间件）
#   ./start_real_robot_teleop.sh --no-driver  # 仅启动求解器 + 遥操作 + 中间件（硬件驱动由其他终端单独运行）
#   ./start_real_robot_teleop.sh --arms-only  # 仅启动双臂模式（不带底盘和灵巧手）

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
HC_IO_SUIT_DIR="${HOME}/hc_io_suit"
PIDS=()

log() {
  echo -e "\033[1;32m[HC-Teleop]\033[0m $*"
}

warn() {
  echo -e "\033[1;33m[HC-Teleop WARN]\033[0m $*"
}

err() {
  echo -e "\033[1;31m[HC-Teleop ERR]\033[0m $*"
}

ALL_PROCESS_PATTERNS=(
  "camera_driver"
  "image_jpeg_compressor"
  "depth_png_compressor"
  "camera_calibration_json_bridge"
  "camera_bridge.launch.py"
  "run_camera_bridge.sh"
  "d405_stereo_camera_node"
  "d405_stereo_camera.launch.py"
  "run_camera.sh"
  "hc_tj_marvin_bridge_node"
  "body_bridge_node"
  "omnihand_rs485_bridge_node"
  "run_omnihand_rs485.sh"
  "run_chassis_bridge.sh"
  "run_hc_tj_all.sh"
  "run_hc_tj.sh"
  "hc_real_robot_bridge.py"
  "control_v2_3_ros2.py"
  "teleop_arm_controller.py"
  "teleop_session_monitor.py"
  "middleware_server.py"
)

kill_all_lingering_processes() {
  local force="${1:-false}"
  for pat in "${ALL_PROCESS_PATTERNS[@]}"; do
    pkill -TERM -f "${pat}" 2>/dev/null || true
  done
  sleep 0.6
  for pat in "${ALL_PROCESS_PATTERNS[@]}"; do
    if pgrep -f "${pat}" >/dev/null 2>&1; then
      pkill -KILL -f "${pat}" 2>/dev/null || true
    fi
  done
  sleep 0.3
}

_CLEANED=0
cleanup() {
  if [[ "${_CLEANED}" -eq 1 ]]; then
    return 0
  fi
  _CLEANED=1
  warn "正在关闭所有遥操作、硬件驱动与相机进程..."
  if [[ ${#PIDS[@]} -gt 0 ]]; then
    for pid in "${PIDS[@]}"; do
      kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    done
    local deadline=$((SECONDS + 2))
    for pid in "${PIDS[@]}"; do
      while kill -0 "${pid}" 2>/dev/null && [[ $SECONDS -lt $deadline ]]; do
        sleep 0.1
      done
    done
    for pid in "${PIDS[@]}"; do
      kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    done
  fi
  # 深度清理所有脱离会话组的孤儿进程与硬件驱动节点
  kill_all_lingering_processes true
  log "所有进程与硬件线程已安全彻底退出。"
}

trap cleanup EXIT
trap 'exit 0' INT TERM

LAUNCH_DRIVER=true
DRIVER_MODE="all"
DRIVER_EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-driver)
      LAUNCH_DRIVER=false
      shift
      ;;
    --arms-only)
      DRIVER_MODE="arms"
      shift
      ;;
    --no-camera|--no-head-camera|--no-d405|--no-chassis|--no-hands|--no-vr-trigger)
      DRIVER_EXTRA_ARGS+=("$1")
      shift
      ;;
    -h|--help)
      echo "用法: $0 [--no-driver] [--arms-only] [--no-camera] [--no-d405] [--no-chassis] [--no-hands]"
      exit 0
      ;;
    *)
      DRIVER_EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

# Auto-detect D405 cameras if not explicitly provided
if [[ "${DRIVER_MODE}" == "all" ]] && ! [[ "${DRIVER_EXTRA_ARGS[*]}" =~ "--no-d405" ]]; then
  if ! lsusb 2>/dev/null | grep -qiE "RealSense|Intel Corp"; then
    log "未检测到 Intel RealSense D405 USB 设备，自动添加 --no-d405 避免驱动报错"
    DRIVER_EXTRA_ARGS+=("--no-d405")
  fi
fi

# 0. 启动前深度清理残留后台旧进程（确保相机硬件句柄、串口与端口干净释放）
log "检查并清理旧驱动与残留相机线程..."
kill_all_lingering_processes true

if which uhubctl >/dev/null 2>&1; then
  # 软件切断 1-2.2 闪断端口供电，彻底消除对 Berxel/Angstrong 相机驱动的热插拔重置干扰
  echo 1 | sudo -S uhubctl -l 1-2 -p 2 -a 0 >/dev/null 2>&1 || sudo uhubctl -l 1-2 -p 2 -a 0 >/dev/null 2>&1 || true
fi

# 1. 环境初始化
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  if [[ -f "${HC_IO_SUIT_DIR}/install/setup.bash" ]]; then
    source "${HC_IO_SUIT_DIR}/install/setup.bash"
  fi
  set -u
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
export PYTHONUNBUFFERED=1
log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# 1.5 创建本次遥操作运行日志目录
SESSION_ID="$(date +'%Y%m%d_%H%M%S')"
LOG_DIR="${SCRIPT_DIR}/runtime/teleop_logs/session_${SESSION_ID}"
mkdir -p "${LOG_DIR}"
ln -sfn "${LOG_DIR}" "${SCRIPT_DIR}/runtime/teleop_logs/latest"
log "运行日志已就绪: ${LOG_DIR}"
log "  - 实时操作流: ${LOG_DIR}/teleop_operations.log"
log "  - 遥操作控制: ${LOG_DIR}/teleop_controller.log"
log "  - 逆解求解器: ${LOG_DIR}/v23_solver.log"
log "  - 话题桥接器: ${LOG_DIR}/bridge.log"
log "  - 中间件服务: ${LOG_DIR}/middleware.log"

# 2. 检查并启动硬件驱动（可选）
if [[ "${LAUNCH_DRIVER}" == true ]]; then
  if [[ -d "${HC_IO_SUIT_DIR}/src/hc_tj" ]]; then
    if [[ "${DRIVER_MODE}" == "all" && -f "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj_all.sh" ]]; then
      log "启动 hc_io_suit 全功能真机驱动 (${DRIVER_EXTRA_ARGS[*]:-默认全部})..."
      if [[ ${#DRIVER_EXTRA_ARGS[@]} -gt 0 ]]; then
        setsid bash "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj_all.sh" "${DRIVER_EXTRA_ARGS[@]}" &
      else
        setsid bash "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj_all.sh" &
      fi
      PIDS+=($!)
    elif [[ -f "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj.sh" ]]; then
      log "启动 hc_io_suit 双臂驱动..."
      if [[ ${#DRIVER_EXTRA_ARGS[@]} -gt 0 ]]; then
        setsid bash "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj.sh" "${DRIVER_EXTRA_ARGS[@]}" &
      else
        setsid bash "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj.sh" &
      fi
      PIDS+=($!)
    fi
    sleep 3
  else
    warn "未找到 ${HC_IO_SUIT_DIR}/src/hc_tj，跳过驱动启动，请确认硬件节点已就绪。"
  fi
fi

# 2.5 启动硬件与 HC 标准话题实时桥接
BRIDGE_SCRIPT="${SCRIPT_DIR}/scripts/hc_real_robot_bridge.py"
if [[ -f "${BRIDGE_SCRIPT}" ]]; then
  log "启动 HC 真机标准话题桥接 (/io_teleop <=> /hc_teleop)..."
  setsid /usr/bin/python3 -u "${BRIDGE_SCRIPT}" > "${LOG_DIR}/bridge.log" 2>&1 &
  PIDS+=($!)
  sleep 1
fi

# 3. 启动 Pinocchio v23 逆运动学求解器
CONTROLLER_YML="${SCRIPT_DIR}/robot_configs/hc_tj_description/controller_v23.yml"
V23_SCRIPT="${SCRIPT_DIR}/vendor/io_unicontroller_ros2/control_v23_reconstructed/script/control_v2_3_ros2.py"

if [[ -f "${V23_SCRIPT}" && -f "${CONTROLLER_YML}" ]]; then
  log "启动 Pinocchio v23 逆运动学求解器..."
  setsid /usr/bin/python3 -u "${V23_SCRIPT}" "${CONTROLLER_YML}" > "${LOG_DIR}/v23_solver.log" 2>&1 &
  PIDS+=($!)
  sleep 1
else
  err "未找到 v23 求解器或配置文件！"
  exit 1
fi

# 4. 启动机械臂遥操作控制器
ARM_CONFIG="${SCRIPT_DIR}/robot_configs/hc_tj_description/arm_teleop.yaml"
if [[ -f "${ARM_CONFIG}" ]]; then
  log "启动机械臂遥操作控制器 (backend=v23)..."
  setsid /usr/bin/python3 -u "${SCRIPT_DIR}/teleop_arm_controller.py" --config "${ARM_CONFIG}" --backend v23 > "${LOG_DIR}/teleop_controller.log" 2>&1 &
  PIDS+=($!)
  sleep 1
fi

# 4.5 启动实时遥操作与状态监控记录器
MONITOR_SCRIPT="${SCRIPT_DIR}/scripts/teleop_session_monitor.py"
if [[ -f "${MONITOR_SCRIPT}" ]]; then
  log "启动实时遥操作事件记录器..."
  setsid /usr/bin/python3 -u "${MONITOR_SCRIPT}" --log-file "${LOG_DIR}/teleop_operations.log" > "${LOG_DIR}/monitor.log" 2>&1 &
  PIDS+=($!)
fi

# 5. 启动 Web 遥操作中间件 & 录制服务
log "启动 Web 遥操作中间件与录制服务 (Port 7876)..."
/usr/bin/python3 -u "${SCRIPT_DIR}/middleware_server.py" --config "${SCRIPT_DIR}/middleware.yaml" 2>&1 | tee "${LOG_DIR}/middleware.log"
