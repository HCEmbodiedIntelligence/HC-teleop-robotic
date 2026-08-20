#!/usr/bin/env bash
# 一键同步 HC-teleop-robotic 代码库到 niic 工控机
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_IP="${1:-10.42.0.31}"
TARGET_USER="niic"
TARGET_PASS="1"

echo "[Sync] 正在同步 ${SCRIPT_DIR} 到 ${TARGET_USER}@${TARGET_IP}:~/HC-teleop-robotic/ ..."

sshpass -p "${TARGET_PASS}" rsync -avz --delete \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude '.pytest_cache' \
  --exclude '.deps' \
  --exclude 'runtime/recordings' \
  --exclude 'runtime/teleop_logs' \
  "${SCRIPT_DIR}/" \
  "${TARGET_USER}@${TARGET_IP}:~/HC-teleop-robotic/"

echo "[Sync] 同步完成！"
