# HC Robot Platform — X1 进程架构

此分支将默认入口切换到 `platform/` 中的新实现：C++ 控制权威与运动学内核，Python asyncio 业务服务，独立的 VR、IK、机器人接口、视频、录制、回放和作业进程，以及 React / TypeScript 六页控制台。

它不启动旧的 `middleware/` 和 Python 遥操控制链。原始机器人配置包继续使用；旧实现保留供接口对照，原入口另存为 `run-legacy.sh`、`install-legacy.sh`，原说明位于 [README-legacy.md](README-legacy.md)。不要同时启动新旧控制链。

## 本机启动

验证环境：Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10、C++17、Eigen 3、Node.js 20。先准备系统依赖：

```bash
sudo apt-get install cmake g++ libeigen3-dev python3-pip ffmpeg
./install.sh
./run.sh sim --robot x1
```

打开 `http://127.0.0.1:7876`。仿真使用 PyBullet DIRECT，网页可打开三维视图。仿真 ROS Domain 自动隔离，`ROS_LOCALHOST_ONLY=1`。启动、重启和配置切换后均保持软件停止；点击“故障复位／恢复许可”后选择并开始控制会话。不会启动即回零。

安装器把依赖放在项目的 `.platform-deps/`，不替换 ROS 的 Python。依赖锁在 [platform/requirements.lock](platform/requirements.lock) 和 [platform/web/package-lock.json](platform/web/package-lock.json)。不需要原来的 Pinocchio / CasADi Conda 控制环境。

```bash
# 原生内核和 Python/ROS/数据/视频端到端测试；不连接真机
bash platform/scripts/test.sh

# 程序控制示例：只允许仿真，4 秒后软件停止
PYTHONPATH="$PWD/platform/python:$PWD/.platform-deps" \
  /usr/bin/python3 platform/examples/program_control.py
```

## X1 真机入口

硬件驱动由独立的 `HC_x1` 仓库启动。本仓库不会启停厂商驱动，也不会把模拟状态当成硬件反馈。

```bash
# 在硬件端先按 HC_x1 的说明启动驱动，ROS Domain 必须一致。
# 本仓库：默认只授权右臂与右夹爪。
./run.sh teleop --robot x1 --domain 14 --host 0.0.0.0
```

默认 X1 契约为用户提供的 `HC_x1` 仓库所用的 `legacy-model-topic`：`/action/joint_states` 和 `/observation/joint_states` 使用 URDF 关节名。若硬件已经迁移至新的内部模型接口，可显式选择：

```bash
HC_X1_CONTRACT=internal-model-v2 ./run.sh teleop --robot x1 --domain 14
```

两套接口不会同时发布。必须核对硬件版本，不能凭机器人名称自动猜测。完整映射、底盘顺序、手部标定及软件停止边界见 [X1_INTERFACE.md](platform/docs/X1_INTERFACE.md)。

双臂、腰部或底盘需要在完成对应验收后显式授权，例如：

```bash
./run.sh teleop --robot x1 --domain 14 \
  --groups right_arm,left_arm,right_gripper,left_gripper,body,base
```

`--groups` 是启动授权上限；配置界面不能将范围扩大到该参数之外。当前实际真机验收仍待完成，仿真和 ROS mock 测试不能替代真机验收。[验收步骤](platform/docs/ACCEPTANCE.md)包括离合、方向、停止、断流、DAgger、回零和回放。

PICO 的 UDP 端口为输入 `5005`、发现 `5006`、状态回传 `5007`；HTTP 视频端口默认 `7876`，支持 `/offer`、`/offer/{camera_id}` 和 `/api/camera/status`。输入兼容 102 字节 v1 与 162 字节 v2；v1 没有按键，不能凭空提供 Grip / Trigger。

网页操作默认只允许本机。远程网页和算法写入需配置 `HC_PLATFORM_TOKEN`，请求带 `Authorization: Bearer ...`。令牌在控制台“系统配置”中只保存在浏览器当前会话。外部模型使用 [asyncio SDK](platform/python/hc_platform/sdk.py) 或 [API 文档](platform/docs/API.md)。

## 录制、转换与平台

普通录制与 DAgger 共用一个录制租约。控制出口命令、实际提交审计和所有选定 ROS 话题都会进入 MCAP；配置 ZIP、有效参数、时间信息、控制来源及接管区间与数据一起保存。标记不自动等于任务成功或质检通过。

本地格式为 MCAP、MCAP＋MP4、LeRobot v3、MCAP＋LeRobot。LeRobot 转换在独立环境中执行，避免 Torch / PyArrow 污染 ROS：

```bash
# 需先安装 uv；只在项目内创建转换环境。
PYTHONPATH="$PWD/platform/python:$PWD/.platform-deps" /usr/bin/python3 \
  -m hc_platform.data.install_converter --venv "$PWD/.venv-lerobot-platform"
HC_CONVERTER_PYTHON="$PWD/.venv-lerobot-platform/bin/python" ./run.sh sim
```

全局保存位置通过 `--state-dir` 设置；录制界面可选该数据根目录内的子目录和 MCAP 文件名。停止录制先完成文件封口，再运行视频/转换/上传作业。失败保留源数据；后台队列持久化在 SQLite，重启可恢复。

平台连接支持连接链接或兼容配置。任务、采集方案、凭据引用和上传格式在录制开始时固定；本地格式与上传格式独立。历史数据另投任务创建新上传作业。上传完成、平台处理、质检是三个独立状态。接口与恢复语义见 [DATA.md](platform/docs/DATA.md)。不连接平台也能完整进行本地采集。

## 配置与能力

控制台包含状态监控、话题录制、数据集管理、上传管理、运行日志、系统配置。配置修改使用校验→停止→加载新配置→确认进程→持久化的事务流程；失败回滚，恢复后仍需明确授权。

```json
{
  "control_backend": "motion",
  "teleop": {
    "control": {"position_scale": 0.5, "rotation_scale": 1.0}
  },
  "camera": {"encoder_backend": "auto"}
}
```

默认 `control_backend=v23` 是兼容的基础遥操能力档位，新 IK 实际使用独立进程中的 Eigen 原生内核；它不代表仍运行旧 ControllerV23，也不开放全部 Motion API。显式选 `motion` 后才提供 ServoJ / ServoP / MoveJ / MoveL / MoveP 和运动任务状态、结果、取消。各关节组的实际支持范围通过能力查询返回。

X1 优先提供真实驱动适配。OpenArmX 和 HC_REALMAN 配置包可导入、查询和用于仿真；本分支没有将它们标记为已验证真机。颈部默认关闭；DAgger 仅接管双臂及受支持夹爪，身体保持。外骨骼 ROS 桥默认关闭，设备驱动与标定仍由外部提供。

软件保护包括单一输出权威、世代/租约/参考校验、反馈和输入超时、限位/速度/加速度约束、按配置启用的几何检查及停止后拒绝旧命令。普通 Linux 下这是带可观测期限的软实时系统，未声称硬实时或安全认证；物理急停和驱动级保护仍由硬件系统实现。

详细实现边界、进程线程与异步模型见 [ARCHITECTURE.md](platform/docs/ARCHITECTURE.md)。
