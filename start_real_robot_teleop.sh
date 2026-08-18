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

cleanup() {
  if [[ ${#PIDS[@]} -eq 0 ]]; then
    return 0
  fi
  warn "正在关闭所有遥操作进程..."
  for pid in "${PIDS[@]}"; do
    kill -TERM "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -KILL "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  done
  log "所有进程已安全退出。"
}

trap cleanup INT TERM EXIT

LAUNCH_DRIVER=true
DRIVER_MODE="all"

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
    -h|--help)
      echo "用法: $0 [--no-driver] [--arms-only]"
      exit 0
      ;;
    *)
      warn "未知参数: $1"
      shift
      ;;
  esac
done

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
log "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# 2. 检查并启动硬件驱动（可选）
if [[ "${LAUNCH_DRIVER}" == true ]]; then
  if [[ -d "${HC_IO_SUIT_DIR}/src/hc_tj" ]]; then
    if [[ "${DRIVER_MODE}" == "all" && -f "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj_all.sh" ]]; then
      log "启动 hc_io_suit 全功能真机驱动..."
      setsid bash "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj_all.sh" &
      PIDS+=($!)
    elif [[ -f "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj.sh" ]]; then
      log "启动 hc_io_suit 双臂驱动..."
      setsid bash "${HC_IO_SUIT_DIR}/src/hc_tj/run_hc_tj.sh" &
      PIDS+=($!)
    fi
    sleep 3
  else
    warn "未找到 ${HC_IO_SUIT_DIR}/src/hc_tj，跳过驱动启动，请确认硬件节点已就绪。"
  fi
fi

# 3. 启动 Pinocchio v23 逆运动学求解器
CONTROLLER_YML="${SCRIPT_DIR}/robot_configs/hc_tj_description/controller_v23.yml"
V23_SCRIPT="${SCRIPT_DIR}/vendor/io_unicontroller_ros2/control_v23_reconstructed/script/control_v2_3_ros2.py"

if [[ -f "${V23_SCRIPT}" && -f "${CONTROLLER_YML}" ]]; then
  log "启动 Pinocchio v23 逆运动学求解器..."
  setsid /usr/bin/python3 "${V23_SCRIPT}" "${CONTROLLER_YML}" &
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
  setsid /usr/bin/python3 "${SCRIPT_DIR}/teleop_arm_controller.py" --config "${ARM_CONFIG}" --backend v23 &
  PIDS+=($!)
  sleep 1
fi

# 5. 启动 Web 遥操作中间件 & 录制服务
log "启动 Web 遥操作中间件与录制服务 (Port 7876)..."
export PYTHONPATH="${SCRIPT_DIR}/.deps${PYTHONPATH:+:${PYTHONPATH}}"
/usr/bin/python3 "${SCRIPT_DIR}/middleware_server.py" --config "${SCRIPT_DIR}/middleware.yaml"
