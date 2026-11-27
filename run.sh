#!/usr/bin/env bash
# ==============================================================================
# HC-teleop-robotic 统一启动入口脚本
#
# 功能概述：
#   根据传入的模式启动对应的完整服务栈，支持三种运行模式：
#   1. middleware（默认）：仅启动网页控制台（Dashboard）、系统配置与 VR 传输网关，
#      适合在网页端导入机器人配置包并按需独立启停 PyBullet 仿真。
#   2. sim：全栈本地仿真模式，同时启动中间件 + 通用 IK/遥操作控制层 + PyBullet 仿真节点
#      （以及可选的 CSV 遥操作诊断记录器），使用隔离的本机 ROS 2 域。
#   3. teleop：真机遥操作模式，启动中间件与通用控制层（默认由中间件托管），连接外部独立
#      运行的硬件驱动（如 HC_X1），使用硬件 ROS 2 域与跨机 Fast DDS 通信。
# ==============================================================================

# 启用 Bash 严格模式：
#   -e          : 任意命令返回非零退出码时立即终止脚本
#   -u          : 引用未定义的变量时直接报错退出
#   -o pipefail : 管道中任意环节失败，整条管道即返回失败状态码
set -euo pipefail

# ------------------------------------------------------------------------------
# 1. 默认配置与全局变量初始化
# ------------------------------------------------------------------------------
# 解析脚本自身所在目录的绝对路径作为项目根目录，避免受执行时的当前工作目录影响
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 默认运行模式为 middleware（仅启动中间件，不自动拉起控制层和仿真器）
MODE="middleware"
# 中间件配置文件路径：优先使用环境变量 HC_MIDDLEWARE_CONFIG，否则默认为 middleware/config.yaml
MIDDLEWARE_CONFIG="${HC_MIDDLEWARE_CONFIG:-${PROJECT_ROOT}/middleware/config.yaml}"
# 是否开启无头仿真模式（不弹出 PyBullet GUI 窗口，仅 sim 模式有效）
HEADLESS=false
# 是否启用遥操作事件监听器（middleware/session_monitor.py）
MONITOR=true
# 是否在仿真模式下启用 CSV 遥操作诊断记录器（可通过环境变量 TELEOP_DIAGNOSTICS=0 关闭）
DIAGNOSTICS=true
[[ "${TELEOP_DIAGNOSTICS:-1}" != "0" ]] || DIAGNOSTICS=false
# 可选覆盖参数：会话日志目录、Dashboard 监听主机地址、Dashboard 监听端口
LOG_DIR=""
SERVER_HOST=""
SERVER_PORT=""

# 打印命令行使用说明、各运行模式简介、可选参数及调用示例
usage() {
  cat <<EOF
Usage: $0 [middleware|sim|teleop] [options]

  middleware Dashboard + VR transport; launch PyBullet from System Configuration
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
  --no-wifi-preflight   Skip cached, read-only Wi-Fi startup report
  --host HOST           Override Dashboard bind host
  --port PORT           Override Dashboard port
  -h, --help            Show this help

Examples:
  ./run.sh
  ./run.sh sim --headless
  ./run.sh teleop
EOF
}

# ------------------------------------------------------------------------------
# 2. 命令行参数解析
# ------------------------------------------------------------------------------
# 解析可选的第一个位置参数作为运行模式（未指定时保持默认 MODE=middleware）
if (($#)); then
  case "$1" in
    middleware|sim|teleop)
      MODE="$1"
      shift
      ;;
    simulator)
      # 拦截已移除的旧模式名称并提示替代用法
      echo "Mode '$1' was removed. Use 'sim' or 'teleop'." >&2
      exit 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
  esac
fi

# 循环解析剩余的所有选项标志
while (($#)); do
  case "$1" in
    --headless)
      # 开启无头仿真（关闭 PyBullet GUI 界面）
      HEADLESS=true
      shift
      ;;
    --config)
      # 指定自定义的中间件 YAML 配置文件路径
      [[ $# -ge 2 ]] || { echo "--config requires a file" >&2; exit 2; }
      MIDDLEWARE_CONFIG="$2"
      shift 2
      ;;
    --log-dir)
      # 指定本次运行的日志输出目录
      [[ $# -ge 2 ]] || { echo "--log-dir requires a directory" >&2; exit 2; }
      LOG_DIR="$2"
      shift 2
      ;;
    --no-wifi-preflight)
      # 跳过启动前的只读 Wi-Fi 环境预检
      export HC_WIFI_PREFLIGHT=0
      shift
      ;;
    --no-monitor)
      # 禁用遥操作会话事件监听器
      MONITOR=false
      shift
      ;;
    --no-diagnostics)
      # 禁用仿真模式下的 CSV 诊断数据记录
      DIAGNOSTICS=false
      shift
      ;;
    --host)
      # 覆盖 Dashboard Web 服务绑定的主机地址
      [[ $# -ge 2 ]] || { echo "--host requires a value" >&2; exit 2; }
      SERVER_HOST="$2"
      shift 2
      ;;
    --port)
      # 覆盖 Dashboard Web 服务监听的端口号
      [[ $# -ge 2 ]] || { echo "--port requires a value" >&2; exit 2; }
      SERVER_PORT="$2"
      shift 2
      ;;
    --robot)
      # 拦截已废弃参数：当前通过 Dashboard 或 config.yaml 切换激活的机器人配置包
      echo "--robot was removed. Select the active profile in Dashboard/config.yaml." >&2
      exit 2
      ;;
    --sim-only|--v23|--reconstructed|--generic|--legacy)
      # 拦截已移除的历史后端切换选项（当前仓库已统一为 v23 架构）
      echo "The duplicate backend option '$1' was removed." >&2
      exit 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      # 遇到无法识别的参数时报错并打印帮助信息
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

# ------------------------------------------------------------------------------
# 3. 真机安全默认值与配置文件检查
# ------------------------------------------------------------------------------
if [[ "${MODE}" == teleop ]]; then
  # 真机遥操作安全保护：默认禁止控制器启动时自动使能、自动归零（homing）或直接下发运动指令，
  # 需待人工确认就绪后再开启控制（外部显式设置 HC_CONTROL_START_DISABLED=0 可覆盖）
  export HC_CONTROL_START_DISABLED="${HC_CONTROL_START_DISABLED:-1}"
fi

# 校验中间件配置文件必须存在，否则直接退出
[[ -f "${MIDDLEWARE_CONFIG}" ]] || {
  echo "Middleware config not found: ${MIDDLEWARE_CONFIG}" >&2
  exit 2
}

# ------------------------------------------------------------------------------
# 4. 启动前只读 Wi-Fi 无线环境预检（仅 middleware / teleop 模式）
# ------------------------------------------------------------------------------
# 仅读取系统已缓存的 Wi-Fi 扫描信息和信道占用统计（survey），绝不触发离信道主动扫描或修改网络配置，
# 以免干扰正在进行的无线会话。若缺少工具/权限或执行超时，仅打印提示并继续启动，不阻塞主流程。
if [[ "${MODE}" != sim && "${HC_WIFI_PREFLIGHT:-1}" != 0 && -f "${PROJECT_ROOT}/tools/diagnostics/wifi_preflight.py" ]]; then
  PREFLIGHT_DIR="${PROJECT_ROOT}/runtime/network_preflight"
  if mkdir -p "${PREFLIGHT_DIR}"; then
    # 先写入同目录临时文件，成功后再原子重命名为 latest.json，防止生成不完整的 JSON 文件
    PREFLIGHT_TMP="$(mktemp "${PREFLIGHT_DIR}/.report.XXXXXX")"
    # 默认按 80MHz 信道带宽评估（可通过 HC_WIFI_WIDTH 覆盖），若设置了 HC_WIFI_INTERFACE 则指定网卡
    PREFLIGHT_ARGS=(--width "${HC_WIFI_WIDTH:-80}")
    [[ -z "${HC_WIFI_INTERFACE:-}" ]] || PREFLIGHT_ARGS+=(--interface "${HC_WIFI_INTERFACE}")
    # 限时 20 秒执行预检；输出非空时更新 latest.json，否则清理临时文件并忽略错误继续启动
    if timeout 20s /usr/bin/python3 "${PROJECT_ROOT}/tools/diagnostics/wifi_preflight.py" "${PREFLIGHT_ARGS[@]}" > "${PREFLIGHT_TMP}" && [[ -s "${PREFLIGHT_TMP}" ]]; then
      mv -- "${PREFLIGHT_TMP}" "${PREFLIGHT_DIR}/latest.json"
      echo "[HC] Read-only Wi-Fi preflight: ${PREFLIGHT_DIR}/latest.json (unknown values do not imply a clear channel)"
    else
      rm -f -- "${PREFLIGHT_TMP}"
      echo "[HC] Wi-Fi preflight unavailable or timed out; continuing startup." >&2
    fi
  fi
fi

# ------------------------------------------------------------------------------
# 5. 设置全局运行时模式变量（强制由本次命令决定，避免继承父 Shell 残留值）
# ------------------------------------------------------------------------------
# middleware 与 sim 模式均归类为 simulation（隔离仿真域）；仅 teleop 归类为 teleop（真机硬件域）
export HC_RUNTIME_MODE=simulation
[[ "${MODE}" != teleop ]] || export HC_RUNTIME_MODE=teleop

# ------------------------------------------------------------------------------
# 6. middleware 模式专属入口（仅启动中间件，完成后直接退出）
# ------------------------------------------------------------------------------
if [[ "${MODE}" == middleware ]]; then
  # middleware 模式下由网页 Dashboard 系统配置页独立控制仿真和无头开关，不接受命令行 --headless
  [[ "${HEADLESS}" == false ]] || { echo "Configure headless mode in Dashboard." >&2; exit 2; }
  # 清除全栈/外部托管相关环境变量，确保网页端可以自由启停 PyBullet 仿真和导入机器人配置
  unset HC_EXTERNAL_STACK HC_TELEOP_MODE HC_MANAGED_CONTROL HC_CONTROL_LOG_DIR

  # 组装透传给 middleware/start.sh 的启动参数
  ARGS=(--config "${MIDDLEWARE_CONFIG}")
  [[ -z "${LOG_DIR}" ]] || ARGS+=(--log-dir "${LOG_DIR}")
  [[ "${MONITOR}" == true ]] || ARGS+=(--no-monitor)
  [[ -z "${SERVER_HOST}" ]] || ARGS+=(--host "${SERVER_HOST}")
  [[ -z "${SERVER_PORT}" ]] || ARGS+=(--port "${SERVER_PORT}")

  # 保留 run.sh 自身进程在 ps 中可见，并由其托管中间件子进程的完整生命周期
  MIDDLEWARE_PID=""
  cleanup_middleware() {
    local status=$?
    # 清除信号捕获，避免清理过程中重复触发
    trap - EXIT HUP INT TERM
    if [[ -n "${MIDDLEWARE_PID}" ]]; then
      # 通知 process_supervisor 优雅停止整个中间件进程组并等待退出
      kill -TERM "${MIDDLEWARE_PID}" 2>/dev/null || true
      wait "${MIDDLEWARE_PID}" 2>/dev/null || true
    fi
    exit "${status}"
  }
  # 捕获退出与常见终止信号（HUP=128+1, INT=128+2, TERM=128+15）以确保触发清理逻辑
  trap cleanup_middleware EXIT
  trap 'exit 129' HUP
  trap 'exit 130' INT
  trap 'exit 143' TERM

  # 通过 setsid + process_supervisor.py 在独立进程组中拉起 middleware/start.sh；
  # 若本脚本（--parent "$$"）异常终止，supervisor 会自动向其进程组发送 SIGTERM/SIGKILL 清理残留进程
  setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" \
    --parent "$$" -- "${PROJECT_ROOT}/middleware/start.sh" "${ARGS[@]}" &
  MIDDLEWARE_PID=$!
  echo "[HC] run.sh PID=$$; stop this stack with: kill $$"
  # 阻塞等待中间件退出并透传其退出码
  wait "${MIDDLEWARE_PID}"
  exit $?
fi

# ==============================================================================
# 以下为 sim / teleop 全栈启动流程（同时管理中间件、控制层与可选仿真层）
# ==============================================================================

# 标记当前为命令行托管的外部全栈模式，通知网页端禁止重复从 Dashboard 启动仿真
export HC_EXTERNAL_STACK=1

# ------------------------------------------------------------------------------
# 7. 加载 ROS 2 与 Fast DDS 网络隔离/传输环境
# ------------------------------------------------------------------------------
# ROS 2 的 setup.bash 内部可能引用未初始化变量，因此加载前临时关闭 -u 检查，加载后恢复
set +u
source /opt/ros/humble/setup.bash
set -u

export HC_MIDDLEWARE_CONFIG="${MIDDLEWARE_CONFIG}"
# 配置 Python 模块搜索路径：项目根目录、上传客户端目录、本地依赖目录 .deps 以及已有 PYTHONPATH
export PYTHONPATH="${PROJECT_ROOT}:${PROJECT_ROOT}/upload_client:${PROJECT_ROOT}/.deps${PYTHONPATH:+:${PYTHONPATH}}"

# 调用 runtime_mode 模块计算本次会话的 ROS 2 通信域：
#   - simulation 模式：自动分配一个 100~220 之间且避开硬件域的随机仿真域，实现仿真与真机域隔离
#   - teleop 模式：直接使用配置文件（或环境变量）指定的硬件域
DOMAIN_VALUES="$(/usr/bin/python3 -m middleware.core.runtime_mode "${MIDDLEWARE_CONFIG}" --shell-values)"
read -r ROS_DOMAIN_ID HC_HARDWARE_DOMAIN_ID <<< "${DOMAIN_VALUES}"
export ROS_DOMAIN_ID HC_HARDWARE_DOMAIN_ID

if [[ "${HC_RUNTIME_MODE}" == simulation ]]; then
  # 仿真模式：缓存仿真域 ID 供所有子节点复用，并强制 ROS_LOCALHOST_ONLY=1 仅限本机回环通信，杜绝跨机误控真机
  export HC_SIMULATION_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_LOCALHOST_ONLY=1
else
  # 真机遥操作模式：清除仿真域标记并允许跨机 DDS 发现（ROS_LOCALHOST_ONLY=0）
  unset HC_SIMULATION_DOMAIN_ID
  # HC_X1 硬件端与中间件必须使用一致的 Fast DDS 传输配置；若缺少该配置，Fast DDS 可能选到 wlan0，
  # 而 HC_X1 仅绑定机器人内部与推理以太网口，会导致两侧 DDS 发现图分裂、无法通信。
  export ROS_LOCALHOST_ONLY=0
  if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
    # 若外部已显式指定 Fast DDS 配置文件，校验文件存在性后导出
    [[ -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]] || {
      echo "Fast DDS profile not found: ${FASTRTPS_DEFAULT_PROFILES_FILE}" >&2
      exit 2
    }
    export FASTRTPS_DEFAULT_PROFILES_FILE
  elif [[ -f "${HOME}/shm_profile.xml" ]]; then
    # 否则若用户主目录下存在默认共享内存/网卡配置文件 shm_profile.xml，则自动启用
    export FASTRTPS_DEFAULT_PROFILES_FILE="${HOME}/shm_profile.xml"
  else
    # 两者均不存在时回退至中间件默认 DDS 传输设置并打印提示
    unset FASTRTPS_DEFAULT_PROFILES_FILE
    echo "[HC] Fast DDS profile not found; using the middleware default transport." >&2
  fi
fi

# ------------------------------------------------------------------------------
# 8. 解析当前激活的机器人配置包（Profile）与模式参数合法性检查
# ------------------------------------------------------------------------------
# 通过 profile_cli 读取 config.yaml 中当前激活的机器人配置目录绝对路径
PROFILE_DIR="$(/usr/bin/python3 -m middleware.profile_cli profile-dir --config "${MIDDLEWARE_CONFIG}")"
# 拆分出机器人配置仓库根目录（父目录）与当前机器人名称（目录名）
ROBOT_CONFIG_ROOT="$(dirname -- "${PROFILE_DIR}")"
ROBOT_NAME="$(basename -- "${PROFILE_DIR}")"

# 真机模式下不允许传入 --headless
if [[ "${MODE}" == teleop && "${HEADLESS}" == true ]]; then
  echo "--headless is only valid in sim mode." >&2
  exit 2
fi

# ------------------------------------------------------------------------------
# 9. 初始化会话日志目录结构
# ------------------------------------------------------------------------------
# 若未通过 --log-dir 指定路径，则按当前时间戳在 runtime/teleop_logs 下新建会话目录
if [[ -z "${LOG_DIR}" ]]; then
  LOG_DIR="${PROJECT_ROOT}/runtime/teleop_logs/session_$(date +%Y%m%d_%H%M%S)"
fi
# 分别创建控制层和中间件日志子目录，并设置 700 权限（仅当前用户可访问）
mkdir -p "${LOG_DIR}/control" "${LOG_DIR}/middleware"
chmod 700 "${LOG_DIR}" "${LOG_DIR}/control" "${LOG_DIR}/middleware"
mkdir -p "${PROJECT_ROOT}/runtime/teleop_logs"
# 更新 latest 软链接指向最新一次会话日志目录，便于快速定位排查
ln -sfn "${LOG_DIR}" "${PROJECT_ROOT}/runtime/teleop_logs/latest"

# 组装传给 middleware/start.sh 的命令行参数
MIDDLEWARE_ARGS=(
  --config "${MIDDLEWARE_CONFIG}"
  --log-dir "${LOG_DIR}/middleware"
)
[[ "${MONITOR}" == true ]] || MIDDLEWARE_ARGS+=(--no-monitor)
[[ -z "${SERVER_HOST}" ]] || MIDDLEWARE_ARGS+=(--host "${SERVER_HOST}")
[[ -z "${SERVER_PORT}" ]] || MIDDLEWARE_ARGS+=(--port "${SERVER_PORT}")

# ------------------------------------------------------------------------------
# 10. sim 仿真模式专属依赖检查与 ROS 2 话题重映射准备
# ------------------------------------------------------------------------------
SIM_ENTRY=""
SIM_ARGS=()
SIM_ROS_ARGS=()
if [[ "${MODE}" == sim ]]; then
  # 内置通用 PyBullet ROS 2 仿真控制节点入口
  SIM_ENTRY="${PROJECT_ROOT}/simulation/general_sim_robot_control_node_ros2.py"

  # 校验 PyBullet 依赖是否已通过 install.sh 安装到项目本地 .deps 目录
  [[ -d "${PROJECT_ROOT}/.deps/pybullet-3.2.6.dist-info" ]] || {
    echo "Simulation dependencies not found. Run ${PROJECT_ROOT}/install.sh first." >&2
    exit 2
  }
  # 校验仿真脚本与当前机器人 Profile 的仿真配置文件是否存在
  [[ -f "${SIM_ENTRY}" ]] || { echo "Bundled simulator not found: ${SIM_ENTRY}" >&2; exit 2; }
  [[ -f "${PROFILE_DIR}/arm_teleop.yaml" || -f "${PROFILE_DIR}/vr_configs.yml" ]] || {
    echo "Active profile has no simulation config: ${PROFILE_DIR}/arm_teleop.yaml or vr_configs.yml" >&2
    exit 2
  }

  # 导出当前机器人配置根目录与名称，供仿真节点及控制层读取
  export HC_ROBOT_CONFIG_ROOT="${ROBOT_CONFIG_ROOT}"
  export HC_ROBOT_NAME="${ROBOT_NAME}"
  SIM_ARGS=(--profile "${PROFILE_DIR}")
  [[ "${HEADLESS}" == false ]] || SIM_ARGS+=(--headless)

  # 将仿真节点内部默认的 /io_teleop/* 话题重映射到平台标准 ROS 2 话题命名空间
  # （/observation、/action、/control、/status），使仿真器与真机驱动对外接口完全一致
  SIM_ROS_ARGS=(
    --ros-args
    -r /io_teleop/joint_states:=/observation/joint_states               # 仿真关节状态观测 -> 标准关节状态话题
    -r /io_teleop/joint_cmd:=/action/joint_states                       # 关节控制指令 <- 标准关节动作话题
    -r /io_teleop/target_joint_from_vr:=/control/solver/target_joint_from_vr  # VR 逆解目标关节角
    -r /io_teleop/target_finger_joints:=/action/hands/joint_targets     # 灵巧手手指目标关节角
    -r /io_teleop/target_ee_poses:=/action/ee_poses                     # 末端执行器目标位姿
    -r /io_teleop/target_gripper_status:=/action/hands/gripper_status   # 夹爪开合状态指令
    -r /io_teleop/hardware_ready:=/status/hardware/ready                # 仿真/硬件就绪状态反馈
  )
fi

# ------------------------------------------------------------------------------
# 11. 进程树管理与优雅退出清理机制
# ------------------------------------------------------------------------------
# 记录各子系统对应的 process_supervisor 进程 PID
MIDDLEWARE_PID=""   # 中间件（Dashboard Web、VR 网关、录制与事件监听）
CONTROL_PID=""      # 通用控制层（v23 Pinocchio 逆解 + VR 到标准机器人控制节点）
SIM_PID=""          # PyBullet 仿真器节点（仅 sim 模式）
DIAGNOSTICS_PID=""  # 遥操作 CSV 诊断记录器（仅 sim 模式且未禁用时）

# 深度优先递归收集指定 PID 及其全部后代子进程（按“子进程在前、父进程在后”的顺序输出）
collect_process_tree() {
  local parent="$1"
  local child
  while read -r child; do
    [[ -n "${child}" ]] || continue
    collect_process_tree "${child}"
  done < <(pgrep -P "${parent}" 2>/dev/null || true)
  printf '%s\n' "${parent}"
}

# 停止指定根 PID 对应的整棵进程树：先发送 SIGTERM 优雅关闭，最多等待 3 秒，超时则 SIGKILL 强杀
stop_tree() {
  local root_pid="$1"
  [[ -n "${root_pid}" ]] || return 0
  local -a tree=()
  mapfile -t tree < <(collect_process_tree "${root_pid}")
  ((${#tree[@]})) || return 0
  # 批量发送 SIGTERM 请求进程自行清理退出
  kill -TERM "${tree[@]}" 2>/dev/null || true
  # 轮询最多 30 次（每次 0.1 秒，共 3 秒），确认是否全部进程均已退出
  for _ in {1..30}; do
    local alive=false
    local pid
    for pid in "${tree[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then alive=true; break; fi
    done
    [[ "${alive}" == true ]] || return 0
    sleep 0.1
  done
  # 仍有进程未响应 SIGTERM 时，强制发送 SIGKILL 清理，防止遗留僵尸/孤儿进程
  for pid in "${tree[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
}

# 统一退出清理回调：按依赖逆序关闭全部子进程树并回收状态
cleanup() {
  local status=$?
  # 注销信号捕获，避免清理阶段收到信号造成递归调用
  trap - HUP INT TERM EXIT
  # 按逆序依次停止：诊断器 -> 仿真器 -> 控制层 -> 中间件
  stop_tree "${DIAGNOSTICS_PID}"
  stop_tree "${SIM_PID}"
  stop_tree "${CONTROL_PID}"
  stop_tree "${MIDDLEWARE_PID}"
  # 等待各子进程彻底回收，避免产生僵尸进程
  [[ -z "${DIAGNOSTICS_PID}" ]] || wait "${DIAGNOSTICS_PID}" 2>/dev/null || true
  [[ -z "${SIM_PID}" ]] || wait "${SIM_PID}" 2>/dev/null || true
  [[ -z "${CONTROL_PID}" ]] || wait "${CONTROL_PID}" 2>/dev/null || true
  [[ -z "${MIDDLEWARE_PID}" ]] || wait "${MIDDLEWARE_PID}" 2>/dev/null || true
  exit "${status}"
}
# 绑定挂断、Ctrl+C、终止信号及脚本退出事件到 cleanup
trap cleanup HUP INT TERM EXIT

# ------------------------------------------------------------------------------
# 12. 按顺序拉起各核心服务组件
# ------------------------------------------------------------------------------
export HC_TELEOP_MODE="${MODE}"
if [[ "${MODE}" == teleop ]]; then
  # 真机模式下默认开启 HC_MANAGED_CONTROL=1，由中间件（Web 后端 ControlRuntime）托管控制层生命周期，
  # 支持在网页端切换配置并自动重启控制器；并将控制日志目录传入中间件
  export HC_MANAGED_CONTROL="${HC_MANAGED_CONTROL:-1}" HC_CONTROL_LOG_DIR="${LOG_DIR}/control"
else
  # 仿真模式下由本脚本直接启动控制层，不走中间件托管
  unset HC_MANAGED_CONTROL HC_CONTROL_LOG_DIR
fi

# (1) 最先启动中间件（VR 设备发现、VR 网关、网页 Dashboard、录制与事件监听），
#     让 VR 头显在后端 IK 求解器与仿真器加载期间即可开始配对连接
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- "${PROJECT_ROOT}/middleware/start.sh" "${MIDDLEWARE_ARGS[@]}" &
MIDDLEWARE_PID=$!

# (2) 启动通用遥操作控制层（v23 IK 逆解 + arm_controller 适配节点）：
#     仅在非 teleop 模式，或外部显式设置 HC_MANAGED_CONTROL=0（关闭中间件托管）时由本脚本直接拉起
if [[ "${MODE}" != teleop || "${HC_MANAGED_CONTROL:-0}" != 1 ]]; then
setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- "${PROJECT_ROOT}/adapters/start.sh" \
  --config "${MIDDLEWARE_CONFIG}" --log-dir "${LOG_DIR}/control" &
CONTROL_PID=$!
fi

# (3) 仿真模式专属：拉起 PyBullet 仿真器节点，以及可选的 CSV 遥操作诊断记录器
if [[ "${MODE}" == sim ]]; then
  setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 "${SIM_ENTRY}" "${SIM_ARGS[@]}" "${SIM_ROS_ARGS[@]}" &
  SIM_PID=$!
  if [[ "${DIAGNOSTICS}" == true && -f "${PROJECT_ROOT}/tools/diagnostics/teleop_diagnostics.py" ]]; then
    # 默认输出至 ${LOG_DIR}/teleop_diagnostics.csv（可由 TELEOP_LOG_PATH 覆盖），
    # 默认采样率 30Hz（可由 TELEOP_LOG_RATE 覆盖），同步记录手柄位姿、末端目标/实际位姿与关节状态
    DIAGNOSTICS_LOG="${TELEOP_LOG_PATH:-${LOG_DIR}/teleop_diagnostics.csv}"
    setsid /usr/bin/python3 "${PROJECT_ROOT}/tools/runtime/process_supervisor.py" --parent "$$" -- /usr/bin/python3 -m tools.diagnostics.teleop_diagnostics \
      --output "${DIAGNOSTICS_LOG}" --rate "${TELEOP_LOG_RATE:-30}" &
    DIAGNOSTICS_PID=$!
    echo "[HC] diagnostics=${DIAGNOSTICS_LOG}"
  fi
fi

# 打印当前运行摘要与操作指引
echo "[HC] mode=${MODE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} config=${MIDDLEWARE_CONFIG}"
echo "[HC] VR discovery, Dashboard and control are running. Ctrl+C stops this stack."
if [[ "${MODE}" == teleop ]]; then
  # 提醒用户在真机模式下需使用相同的 ROS_DOMAIN_ID 独立启动外部 HC_X1 硬件驱动工程
  echo "[HC] Start the HC_X1 hardware project separately with ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
fi

# ------------------------------------------------------------------------------
# 13. 阻塞监控核心组件并在任一关键组件退出时联动收尾
# ------------------------------------------------------------------------------
# 仅将关键运行组件加入监控列表（中间件、可选控制层、可选仿真器）；
# 诊断记录器（DIAGNOSTICS_PID）属于辅助工具，其异常退出不会导致整个服务栈中止
REQUIRED_PIDS=("${MIDDLEWARE_PID}")
[[ -z "${CONTROL_PID}" ]] || REQUIRED_PIDS+=("${CONTROL_PID}")
[[ -z "${SIM_PID}" ]] || REQUIRED_PIDS+=("${SIM_PID}")

# 使用 wait -n 等待任意一个必需组件率先退出（临时关闭 -e 以捕获其真实退出码）
set +e
wait -n "${REQUIRED_PIDS[@]}"
status=$?
set -e
if [[ ${status} -ne 0 ]]; then
  echo "[HC] A required component exited with status ${status}; see ${LOG_DIR}." >&2
fi
# 脚本退出时会自动触发 EXIT trap 执行 cleanup，联动停止剩余所有子进程并返回状态码
exit "${status}"
