> 当前 HC / OpenArmX 集成工作区入口见 [../start.md](../start.md)。第三方控制与订阅接口、换机兼容范围和 demo 见 [对外接口说明](docs/ROBOT_PUBLIC_API.md)。

# HC Teleop

一个与机器人后端无关的通用遥操作栈：接收 PICO 位姿和手柄数据，通过单一 `/vrdata` 发布到 ROS 2，并提供机器人导入、话题录制、状态监控、Pinocchio IK 和遥操作控制。

## 代码分层

产品现在分为通用软件和机型硬件两个独立仓库。本仓库只包含通用软件：

| 部分 | 目录 | 职责 | 独立启动 |
| --- | --- | --- | --- |
| 中间件 | `middleware/` | `core/` 中的前端、配置导入、VR 网关、录制与回放，以及 `config.yaml` 系统配置 | `./middleware/start.sh` |
| 通用控制/解算 | `adapters/` | VR 控制、Pinocchio v2.3 逆解，以及 `robots/<机器人ID>/` 中由网页导入的 URDF/mesh/参数 | `./adapters/start.sh` |

真机入口 `./run.sh teleop` 会同时启动上面两部分，不判断也不要求真机在线；仿真入口 `./run.sh sim` 还会启动 PyBullet。X1 相机、机械臂、底盘、腰部和灵巧手驱动已独立为同级仓库 `HC_X1`。本仓库不再依赖或 source `hc_io_suit`，停止通用入口也不会启动、停止或清理硬件。新机型接入见 [硬件适配项目接入指南](docs/HARDWARE_ADAPTER_GUIDE.md)。

统一入口默认读取 `middleware/config.yaml` 中的 ROS Domain；显式 `ROS_DOMAIN_ID` 优先。仿真或硬件适配项目必须使用相同的 Domain。

统一模式下 OpenArmX、X1 的 VR 位姿和程序 ServoP 共用 Motion 逆解，与 ServoJ/MoveJ 共享最终关节 RTC。公共双臂名为 `left_joint_1…N`、`right_joint_1…N`，能力从 `/status/robot/info` 在线读取。中间件按 `/control/source` 选择公共目标并发布 `/action/joint_states`，共同硬件边界转换到 `/internal/robot/joint_commands`；驱动模型反馈走 `/internal/robot/joint_states`，HC 转成公共 `/observation/joint_states`。硬件仓库继续独立负责厂商通信和执行保护。外骨骼使用 `/control/exoskeleton/joint_states`，需按当前 profile 提供公共名称；切源、停止和恢复仍执行原有许可代际检查。

原有的 `d435_webrtc_server.py` 与手动 UDP 调试工具
`tools/diagnostics/udp_receiver.py` 仍然保留。新服务兼容它们的关键协议：

- PICO 位姿与手柄输入：UDP `5005`，v2 二进制格式 `<4sBIdB21f3H6f3H6f>`，共 162 字节；同时兼容旧版 v1 位姿包。
- PICO 发现：向 UDP `5006` 发送 `PICO_DISCOVER_V1`。
- D435 WebRTC：`POST /offer`（同时提供新版路径 `/api/webrtc/offer`），只协商 H.264。
- 健康检查：`GET /health`。

## 快速开始

系统中的 ROS 2 Humble 使用 Python 3.10，而当前用户默认 `python3` 可能是 3.13。安装脚本会明确使用 `/usr/bin/python3`，将网页依赖安装在项目内的 `.deps`，并继续使用系统 ROS 包：

```bash
cd /home/maple/test/HC-teleop-robotic
chmod +x install.sh run.sh
./install.sh
./run.sh middleware
```

然后访问 `http://<机器人IP>:7876/dashboard/#config`。当前机器人包的 `middleware.yaml` 提供相机、订阅话题、安全和 VR 超时配置。需要启用 WebRTC 服务时安装可选依赖：

```bash
./install.sh --camera
```

接真机时只需额外启动硬件适配项目：

```bash
# X1 Jetson：只启动硬件与标准接口
cd ~/HC_X1
ROS_DOMAIN_ID=14 ./start.sh

# 通用系统：中间件、ZIP/URDF 配置、逆解、控制和录制
cd ~/HC-teleop-robotic
ROS_DOMAIN_ID=14 ./run.sh teleop
```

Jetson 首次部署时，先安装 Conda，再运行 `./install.sh`。默认 Conda 路径为
`~/miniconda3/bin/conda`；其他安装位置通过 `CONDA_BIN` 指定。安装脚本会创建
独立的 `~/miniconda3/envs/hc-teleop-controller` 环境，提供带 CasADi 绑定的
Pinocchio 3.7 和 CasADi 3.7；系统 Python 中只有 Pinocchio 并不足以启动逆解。
自定义环境位置时，安装和运行都需要设置相同的 `HC_CONTROLLER_PREFIX`。
安装完成后先执行以下只读检查：

```bash
./run_generic_controller.sh --check
```

同时将 `HC_X1` 更新到提供 `/action`、`/observation`、`/control`、`/status`
接口的版本；旧 `/hc_teleop` 接口不能直接连接当前遥操控制器。更新硬件部署时
保留机器专用的相机配置。遥操测试期间停止其他关节命令源，包括 HoloBrain
HTTP 控制网关；两个项目使用相同的 `ROS_DOMAIN_ID`。

相机、话题和安全配置随机器人包提供，无需额外的 X1 配置参数：

```bash
ROS_DOMAIN_ID=14 ./run.sh
```

该命令只启动网页和 VR 网关，不启动控制器。默认模式下，网关、网页启动的
PyBullet 和控制器统一使用自动分配的本机专用 ROS 域，并强制 localhost 通信；
传入的域 14 不会用于仿真，也不会连接同域的 HC_X1。有效仿真域会显示在网页。系统配置分别显示“当前实际运行 ROS 域”和“默认 ROS 域（保存配置）”；显式 `ROS_DOMAIN_ID` 启动参数优先于保存的默认值。
在网页应用机器人包后，从 PyBullet 面板勾选同时启动解算与 VR 仿真控制，再点击启动。
VR 数据、关节反馈与动作均在同一个仿真域内流转。`run.sh` 的默认模式保持 `middleware`。

控制真机必须显式启动 `ROS_DOMAIN_ID=14 ./run.sh teleop`，此时使用指定的域与
HC_X1 通信，网页禁止启动 PyBullet。切换模式时先停止当前入口，再启动另一种模式。
`./run.sh sim` 仍可从命令行启动完整隔离仿真，同样不会使用真机域。
旧的 `--config middleware/config.x1_jetson.yaml` 用法仍兼容，但不再承载另一份机器人配置。

每个包中的 `middleware.yaml` 统一保存 `camera`、`safety`、ROS 订阅/命令话题和
`vr.pose_timeout_ms`。启动自动加载当前包；应用包会重新加载相机与订阅，同名包也可重新应用。
网页修改这些设置时保存回当前包；服务器地址、ROS 域、VR 网络端口和录制目录保留在本机主配置。
ZIP 不带 `middleware.yaml` 时导入器生成安全默认值（相机关闭、标准关节话题、启动软件停止）；
需要相机的机器人包应填写自己的流和话题。应用或重新应用包会停止当前仿真，之后按需重新启动。

网页内查看三维仿真时，真机只需启动默认入口：

```bash
cd ~/HC-teleop-robotic
ROS_DOMAIN_ID=14 ./run.sh
```

本机浏览器打开真机网页，在 PyBullet 面板启动仿真，并按需开启“显示三维视图”。
开关默认关闭，不自动记忆开启状态：关闭时不会加载三维库、机器人模型或网格，
不会建立三维订阅，也不会运行三维渲染循环。打开时才加载本地静态资源并订阅显示帧。
仿真和控制器始终在远程以无窗口模式运行，本机浏览器负责绘制；不需要 SSH X11
或单独的 Python 显示客户端。开关视图不重启仿真、不改变 VR 控制。

最后一个查看者关闭三维视图后，远程停止生成显示帧和计算 marker；
多个浏览器可各自独立开关。视图包含左右机械臂末端、末端目标、头部相机实际和
目标位姿的坐标轴与标签；实际和目标标签使用不同颜色。目标只在收到对应数据后显示，
头部目标超时会移除。支持鼠标旋转和滚轮缩放。
目前网页模型支持 URDF 基本几何体及 STL 网格；其他格式会明确报错。

原 `tools/runtime/simulation_viewer.py` 仍作为可选兼容工具保留，不是网页流程的必需项。

X1 真机目前仅提供底盘速度接口，未提供有效的 `/observation/base/pose`。
`run.sh teleop` 使用 `body.head_pose_base.teleop_mode: planar_velocity`：按住左 Grip
建立头显参考，前后/左右移动和转头分别映射为底盘前后、横移和旋转。
松开 Grip、追踪失效、输入超时或急停后停止并清除参考，下次按住重新定中心。
左/右摇杆手动底盘命令优先；摇杆回中后若仍按住 Grip，以当前头显位置重新定中心。
目标是相对位移，不是持续倾身测速：头显停下后底盘根据命令积分估计到位后停止。
估计值不是底盘实测位姿；打滑、驱动限速和丢包会产生误差，不能保证精确距离。
头显跟随限速为前后/横向各 `0.1 m/s`、旋转 `0.2 rad/s`，需与 HC_X1
的 `max_vx/max_vy/max_vw` 保持一致。状态中 `feedback_source: command_estimate`
明确标记估计来源。原 `yaw_velocity` 配置仍可选择只跟随转向。
仿真继续使用实测位姿闭环；真机提供有效位姿后可将 `teleop_mode` 改为 `pose`。

根目录入口 `./run.sh` 默认只启动中间件（等价于 `./run.sh middleware`）。在网页“系统配置”导入并应用机器人 ZIP 后，可独立启动、停止或重启 PyBullet，并选择关节调试和是否同时启动 IK/VR 控制。按需开启网页三维视图；后台仿真不依赖桌面 DISPLAY。初始姿态和相机配置读取包内 `vr_configs.yml`。停止仿真保留网页、VR 网关和录制服务；导入、切换机器人或修改仿真选项、ROS Domain ID 会先停止当前仿真，再从网页重新启动。

使用 `ps -ef | grep '[r]un.sh'` 查找启动主进程，执行 `kill <PID>` 即可关闭该入口启动的整套服务，也可在启动终端按 Ctrl+C。默认模式保留 `run.sh` 主进程，启动日志会打印 PID；请通过根目录入口启动。

显式 `./run.sh sim` 仍启动完整仿真，`./run.sh teleop` 启动连接真机所需的 VR、Dashboard 和 IK/控制；这两种方式由命令行管理进程，不允许从网页另起一套仿真。网页管理模式下，中间件退出会清理其启动的仿真与控制进程。

### 网页导入机器人配置

“系统配置”页顶部提供 HC 机器人配置工作流：选择已导入机器人、导入新配置、应用配置。导入时同时选择一个 URDF 和一个 YAML，配置 ID 只能使用字母、数字、点、下划线或短横线。文件会保存到 `adapters/robots/<配置ID>/`，X1 的内置资料位于 `adapters/robots/x1/`。

支持两种 YAML：

- HC 通用 `vr_configs.yml`：包含 `urdf_path`、`arms`、`controller_indices`，可选 `folding_waist`。导入器会校验所有 joint/link 索引，并生成标准接口的 `controller_v23.yml`；双臂配置还会生成 `arm_teleop.yaml`。
- 重构控制器 `controller_v23.yml`：包含 `model.free_joints`、`task` 和 `limit`。导入器会校验 URDF 关节与任务 link，并统一 ROS 话题和 URDF 相对路径。该格式只提供 IK 控制配置，不包含 HC PyBullet 仿真所需的 `vr_configs.yml`。

完整统一 API 的 ZIP 还包含 `robot_api.yaml` 和 `motion/` 配置，引用现有分组和 home；只导入上述旧格式不会自动获得统一 Servo 能力，列表会显示缺少的配置。点击“应用配置”会写入 `middleware/config.yaml` 的 `robot_profiles.active` 并立即下发软件停止。重启中间件、控制和硬件适配器以一致加载新 profile；显式 `HC_ROBOT_NAME`、`HC_ROBOT_CONFIG_ROOT` 优先于配置文件。统一 API 的启动和迁移步骤见 [对外接口说明](docs/ROBOT_PUBLIC_API.md)。

导入接口对齐当前遥操作链路的标准话题：`/observation/joint_states`、`/control/solver/joint_targets`、`/action/joint_states`、`/control/solver/ee_targets`、`/action/ee_poses`、`/observation/ee_poses`、`/status/solver/joint_states` 和 `/action/base/command`。状态监控页可在 VR 与外骨骼控制源之间切换，并实时显示命令、反馈关节角和误差。

### 话题录制

“话题录制”页从 ROS Graph 读取当前话题和消息类型，可直接添加自定义消息，也可在“系统配置 → HC 标准 ROS 2 接口”中切换内置消息是否录制。点击“保存录制配置”后写回 `middleware/config.yaml`；启用录制时，消息按启动会话写入 `runtime/topic_recordings/`。MCAP 录制使用独立 ROS Context 和多线程执行器，不与 Dashboard 健康检测或 WebRTC 视频回调共用执行器。X1 默认录制规范化彩色/深度压缩图像、`/observation/cameras/head/calibration` 内参 JSON、`/observation/cameras/head/extrinsics` 外参 JSON，以及机器人基座坐标系中的 `/observation/head/pose` 实际头部相机位姿和 `/action/head/pose` 目标位姿。中间件优先转发 HC_X1 原始标定消息；旧发布端缺失时，从 CameraInfo 生成内参，从 TF 或实测关节 + URDF 生成 `zhi_Link → camera_head` 外参（2 Hz），源数据过期后停止生成。配置与 JSON 字段见 [相机内外参接口](docs/dataset_topics.md)。同时将 `/status/head/tracking_poses` 的 `[actual, target]` 拆分为两条标准 PoseStamped；新格式 MCAP 转为旧格式时可将这两个 Pose 分别补到反馈和目标双臂 Pose 后作为第三项，无需回放 TF 重建。其他同源 `/io_teleop` 图像别名和大体积未压缩回退流默认不勾选，仍可在界面按需启用。

### 自动上传到数据平台

`main` 保留平台任务、共享采集方案、持久上传队列、分片续传、重试/取消和处理状态面板，并整合 Motion 与统一机器人 API。默认仍为 v23；网页三维仿真、ROS 域隔离和机器人包配置保持可用。录制支持 MCAP、MCAP＋独立 RGB MP4 原始包、LeRobot 和双格式；深度消息在 MCAP 中原样保留。标注、裁剪、审计和恢复工具由数据平台负责，不在机器人侧重复提供。

现有部署先运行 `./install.sh --upload`。新版上传管理页支持粘贴平台一次性连接链接、按名称选择任务，并独立选择上传格式。下列 YAML 绑定方式用于兼容旧部署：

```yaml
uploads:
  enabled: true
  auto_enqueue: true
  queue_directory: ../runtime/upload_queue
  platform_base_url: https://your-data-platform.example
  credential_ref: env:HC_UPLOAD_TOKEN
  robot_id: platform-robot-id
  local_robot_id: x1
  task_id: collection-task-id
```

令牌由启动中间件进程的环境变量提供；也可使用 `file:/绝对路径/凭据文件`（仅服务用户可读，权限 0600）。仓库默认 `enabled: false`、`auto_enqueue: false`，不会自动上传历史数据。修改启用配置后重启中间件；“上传管理”页可切换默认机器人/任务绑定。启用自动上传但尚未选择绑定时，开始录制会明确报错，不会把数据归入未知任务。

本地保存格式在“话题录制”页设置（MCAP、LeRobot 或两者），上传格式在“上传管理”页单独选择 MCAP 或 LeRobot。例如本地保存两种格式、平台仅上传 LeRobot。上传 LeRobot 时，本地保存方式须包含 LeRobot；本地仅保存 MCAP 时会提示先修改保存方式，不会静默生成或替换数据。每次录制开始时固定任务、上传格式与采集开始时间，停止写入后记录采集结束时间。所选 MCAP 正常关闭后或 LeRobot 导出成功后，后台生成产物 ID、校验和与完成标记，再自动入队；修改默认设置不改变已有录制和队列。转换失败保留源文件，不把未完成的 LeRobot 当作可上传数据。状态分别保存在 `runtime/recording_uploads/`、`runtime/artifact_state/` 和 `runtime/upload_queue/`，重启可接续已正常关闭源文件的产物交接和上传队列；采集中断不会自动标记为完整录制。

数据集列表中的“上传”按钮可把已就绪的产物送到上传管理页手动入队。新功能安装前的旧文件没有可信采集生命周期/完成标记，不会仅凭文件名或修改时间自动上传；安装后未绑定的录制也不会在切换默认任务后静默绑定。上传读取期间删除会被拒绝，取消后需等读锁释放。上传完成、平台处理完成和质检通过是不同状态。

上传客户端沿用 humanoid 的平台 API 契约。本地回环服务测试不代替目标平台的真实接收验收；需使用匹配的服务器地址和凭据。配置与客户端细节见 [上传集成说明](upload_client/HC_INTEGRATION.md)。

### LeRobot 数据集保存

录制页可选择 MCAP、LeRobot 或两者同时保存，并设置数据集帧率和任务描述。MCAP 保留原始话题（包括 `/vrdata`）；LeRobot 在停止录制后异步生成，相机保存为 H.264 MP4，状态和动作保存为 Parquet。录制页及数据集管理页显示转换阶段、百分比和处理数量，完成后支持特征查看、相机视频预览、完整 ZIP 下载及删除。

LeRobot 是可选依赖。在项目根目录运行：

```bash
./tools/setup_lerobot.sh
```

脚本优先使用 `PATH` 中的 `uv`，其次查找 `~/.local/bin/uv`；仍未找到时会通过
`python3 -m pip install --user uv` 自动安装。随后创建 Python 3.12 的
`.venv-lerobot` 环境，安装固定版本 `lerobot==0.4.3` 和 CPU PyTorch；ROS
继续使用系统 Python。无需激活该虚拟环境，中间件会在每次开始 LeRobot
录制时检查并直接调用其中的解释器。也可通过 `LEROBOT_PYTHON` 指定兼容的
独立解释器。

安装完成后可执行只读检查：

```bash
.venv-lerobot/bin/python -c \
  "import lerobot,av,pyarrow,torch; from lerobot.datasets.lerobot_dataset import LeRobotDataset; print(lerobot.__version__, torch.__version__)"
```

原始 MCAP 不会因转换失败而删除；仅选 LeRobot 时，原始文件位于录制目录的
`.lerobot_sources/`，成功数据集位于 `lerobot/<录制名称>/`。

一次录制对应一个 episode，按 MCAP 消息接收时间的完整范围生成固定帧率时间轴。采样帧率不会提高相机实际采集频率。各字段只选择当前时间及之前的最后一个样本；过期值保留并标记无效，首次收到数据之前使用明确标记的零值/黑图占位，不借用未来数据，也不因短暂缺失删帧或切分 episode。

数据集中的 `observation.source_available`、`observation.source_valid`、`observation.source_age`、`observation.source_time` 向量按各自 feature 的 `names` 对应来源字段。`available=0` 表示尚未收到样本；`valid=0` 表示缺失或过期，训练时应按字段使用该掩码。缺失样本的年龄和来源时间为 `-1`。数值数据有效期为 0.2 秒，相机为 0.25 秒；旧底盘命令仅代表最后收到的值，不代表机器人仍执行该速度。

`observation.state` 使用反馈关节，`action` 使用实际下发的关节；按名称对齐，允许二者维度不同，不用反馈补造缺失命令。同步规则、来源话题、关节名称及无效采样统计保存在 `meta/teleop_sync.json`。

### PICO 客户端版本

手柄按键、Trigger、Grip 和摇杆要求 PICO 客户端发送协议 v2。网页右上角应显示“协议 v2”，ROS 2 中应出现 `/vrdata`；旧版 v1 只有位姿，所有手柄输入都会显示为零。

当前配套 Unity 工程和最新 APK 位于移动盘：

```text
/media/maple/B81666081665C7C8/Users/maple/HC-Teleop
/media/maple/B81666081665C7C8/Users/maple/HC-Teleop/HC-Teleop.apk
```

PICO 通过 USB 连接后可更新安装：

```bash
adb devices -l
adb install -r /media/maple/B81666081665C7C8/Users/maple/HC-Teleop/HC-Teleop.apk
```

更新 APK 或本项目网页后，重新打开 PICO 应用，并在浏览器执行一次强制刷新。

如果已有工作空间，在运行前先 source 对应的 `install/setup.bash`；启动脚本会自动 source `/opt/ros/humble/setup.bash`。ROS 域仍由标准环境变量控制：

```bash
ROS_DOMAIN_ID=12 ./run.sh teleop
```

## 数据流

```text
ROS 2 topics ──> dedicated recording executor ──> recording rules ──> runtime/topic_recordings/*.mcap

PICO UDP :5005 ──> packet validation / sequence check ──┬─> /vrdata (String JSON)
                                                        └─> /ws dashboard status

ROS 2 feedback ──> optional UDP :5007 ──> PICO

D435 ──> latest frame only ──> WebRTC H.264 /offer
```

ROS 转发到 VR 的 UDP 消息是 UTF-8 JSON，最大为一个 UDP 数据报。通用信封如下：

```json
{
  "version": 1,
  "kind": "ros_message",
  "source": "ros2",
  "timestamp": 1786694400.0,
  "topic": "/joint_states",
  "msg_type": "sensor_msgs/msg/JointState",
  "payload": {"name": ["joint1"], "position": [0.1]}
}
```

`/vrdata` 使用 `std_msgs/msg/String`，每条 JSON 同时包含 `tracking`、头显/左右手柄 `poses`、左右手柄 `inputs`、序号和 VR 时间戳。`inputs` 内含 Trigger、Grip、两个摇杆、`held`、`pressed`、`released` 及对应位掩码，因此位姿、连续状态和按键边沿不再拆成多个 ROS 话题。

位姿中断超过 `vr.pose_timeout_ms`（当前配置为 600 ms）、VR 时间戳停止推进或头显跟踪失效时，服务向 `/control/emergency_stop` 发布 `std_msgs/msg/Bool(data=true)`。启动和配置热重载是否急停由 `safety.stop_on_startup` 控制。

## HTTP / WebSocket 接口

| 接口 | 用途 |
| --- | --- |
| `GET /api/status` | ROS、VR、相机和客户端状态 |
| `GET/PUT /api/config` | 读取或保存配置；保存后热重载 |
| `GET /api/robot-profiles` | 已导入机器人、当前选择和标准话题 |
| `POST /api/robot-profiles/import` | 以 multipart 导入 URDF 和 YAML |
| `POST /api/robot-profiles/{id}/activate` | 应用已导入配置 |
| `GET /api/ros/topics` | 当前 ROS Graph 话题 |
| `POST /api/ros/publish` | 通用 ROS 消息发布 |
| `POST /api/safety/stop` | 人工急停 |
| `GET /ws` | 实时 JSON 事件与消息 |
| `POST /api/webrtc/offer` | D435 WebRTC SDP 协商 |

`POST /api/ros/publish` 示例：

```json
{
  "topic": "/teleop/test",
  "type": "std_msgs/msg/String",
  "data": {"data": "hello"}
}
```

## 安全与部署说明

- Dashboard 当前设计用于可信机器人局域网，没有账号认证。不要直接暴露到公网；生产部署应通过防火墙限制来源，或在前面增加带认证的反向代理。
- UDP 不保证送达。关节状态等高频实时数据适合 UDP；任务指令和模式切换应使用 WebSocket/ROS service/action，并在应用层确认。
- 急停话题只是软件联锁，不能替代硬件急停回路。
- Drive 回放会临时独占最终关节命令通道，并继续受 `hardware_ready` 和命令输出开关约束；结束、停止或异常后必须按 A 或调用安全恢复接口，实时控制源才可重新接管。
- 网页“保存并应用”会原子写回 `middleware/config.yaml` 并热重载。`server.host`、`server.port` 或 `robot_profiles.root` 改动会保存，但需要重启进程；其余配置立即应用。

## 验证

```bash
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 -m compileall -q middleware adapters
bash -n run.sh middleware/start.sh adapters/start.sh
```

## HC-TJ 机械臂仿真遥操作

VR 到 HC-TJ 双臂、腰部、底盘和夹爪的离合控制见 [TELEOP.md](TELEOP.md)。仿真使用一个统一入口；脚本负责让 VR、控制和仿真使用同一个 ROS Domain，并在任一必要组件退出时清理整套进程：

双臂与腰部回零：不用按 Grip，同时把左主摇杆向左、右主摇杆向右拨到底一次；回零后先让两个摇杆回中，才能再次触发。回零期间命令合并层会将双臂和 `body.waist_joint_names` 中的腰关节限速拉回 `initial_joints`，避免 7DoF 冗余解只让末端到位却永久卡在 homing。反馈到位后系统会清除旧 IK 积分状态，并等待一帧复位后的新解；此时需要先松开右 Grip，再重新按下才能恢复双臂控制，旧 IK 消息不会重新接管。

```bash
./install.sh --sim

# 一条命令启动 VR 自动发现、Dashboard、IK/控制和 PyBullet
./run.sh sim

# 无显示器环境
./run.sh --headless
```

普通 `./install.sh` 已同时安装 PyBullet shadow model 依赖，并创建与开发板一致的
`hc-teleop-controller` Conda 环境（Pinocchio 3.7 + CasADi 3.7）；`--sim` 作为兼容选项保留。
默认启动已完成数值验证的重构 v2.3 后端。
根目录不再保留旧的分段启动脚本，也不提供单独 PyBullet 产品模式。真机统一使用 `./run.sh teleop` 并在 `HC_X1` 仓库启动硬件；仿真统一使用 `./run.sh sim`。旧 `--robot`、`--generic`、`--legacy` 和 `--sim-only` 选项均已移除。

仿真启动时会自动以 30 Hz 将手柄位姿、目标/实际末端位姿、关节命令/反馈和离合状态写入 `runtime/teleop_logs/`。复现抖动时按住右 Grip 并尽量保持双手静止 5–10 秒，退出仿真后分析对应日志：

```bash
/usr/bin/python3 -m tools.diagnostics.analyze_teleop_log runtime/teleop_logs/teleop_YYYYMMDD_HHMMSS.csv
```

若日志同时包含主动移动和静止保持，可加 `--start 秒数 --end 秒数` 只分析静止区间。

可用 `TELEOP_DIAGNOSTICS=0` 禁用记录，或用 `TELEOP_LOG_RATE=60` 调整记录频率。诊断写盘由独立进程完成，不占用遥操作控制循环。

仿真底盘坐标约定为 `+X` 前进、`+Y` 向左，底盘消息顺序为 `[yaw, forward, lateral]`。PyBullet 窗口需要先点击获得焦点；按住 `Ctrl` 并拖动鼠标左键旋转视角，按住 `Ctrl` 并拖动中键平移视角，滚轮缩放。修改仿真相机或坐标配置后需要退出并重新运行脚本。

X1 默认开启 `body.head_pose_base.enabled`：按住左 Grip 时记录头显与底盘参考位姿，之后头显水平位移和偏航同时形成底盘相对位置、相对朝向目标；到位后停止。松开 Grip、输入/底盘反馈超时、急停或回零时停止并清除参考，下次按住重新定中心。头显高度与俯仰不驱动平面底盘，脖子仍独立跟随头显。开启此模式时左摇杆可独立平移（无需 Grip），推杆期间优先手动控制并清除头显底盘目标；回中后若仍按住左 Grip 则重新建立参考，否则停止，避免回拉。右摇杆和旧的朝向/伸展辅助不叠加到底盘命令；设为 `false` 恢复旧模式。

相对位姿模式使用 `/observation/base/pose`（`geometry_msgs/msg/PoseStamped`）真实反馈，Isaac 和 PyBullet 已发布此话题。位置使用固定 `world` 坐标，姿态 yaw 表示物理 base_link 前进轴朝向，PyBullet 与 Isaac 均使用 +X 前进、+Y 左移；硬件适配器也必须提供同样约定的实际定位反馈，无反馈时不会运动。`position_scale/yaw_scale` 控制位移/角度比例，`max_displacement/max_yaw` 限制每次参考的目标范围，现有三个 `max_*_velocity` 限速。修改后重启控制器；Isaac 同时需要重启场景加载进程以启用新反馈。

X1 的 `./run.sh sim` 和 `./run.sh teleop` 共用 `arm_teleop.yaml` 中唯一的 `body` 配置。摇杆轴、方向、头显跟随、脖子与腰部参数不再随运行模式覆盖。`body.enabled` 和 `body.waist_joint_names` 统一决定是否启用腰部；当前默认开启，关节为 `[leg_1, leg_2, zhi]`，采用高度/俯仰双自由度控制。修改后重启控制器；从网页启动的仿真需点击“重启仿真”。仿真按时间积分速度命令，200 ms 未收到新底盘命令即停止。

机械臂只使用手柄相对位姿增量：当前 VR 数据协议中手柄 `+Z` 向前，对应胸部 `zhi_Link` 的 `+X` 向前。目标先在胸部坐标系生成，再转换到左右肩部任务坐标交给 v2.3 求解器；腰部运动不会改变这项视觉/手柄约定。

X1 仿真头部显示 `HEAD ACTUAL`（青色标签、短坐标轴）与 `HEAD TARGET`（橙色标签、长坐标轴），橙色连线表示位置误差。实际位姿由关节反馈和 `camera_head` FK 得到，目标是头显输入映射到腰部升降/俯仰后对应的机器人头部位姿，并非 VR 房间中的原始绝对位置。松开左 Grip 或停用控制时目标回到实际位姿；数据断流 0.5 s 后隐藏标记。话题 `/status/head/tracking_poses` 使用 `geometry_msgs/msg/PoseArray`，顺序为 `[actual, target]`，`header.frame_id=robot_base` 表示 PyBullet 模型根部位姿坐标系。需要带 GUI 启动仿真才能看到标记。

默认链路为 `controller_target_ee_poses → ControllerV23 → FrameTask/AxisTask/JointTask → solve_ik → 速度及一步位置限位 → Pinocchio integrate → joint_cmd_arm → VR 适配器/夹爪合并 → joint_cmd_vr → Command Mux → joint_cmd`。`target_ee_poses` 和 `actual_ee_poses` 专供仿真显示/诊断，始终使用胸部 `zhi_Link` 坐标，使 marker 与法兰直观对应；内部控制目标才转换为左右肩基坐标。源码位于 `adapters/v23/`，X1 参数位于 `adapters/robots/x1/controller_v23.yml`。

新机器人优先从网页或 CLI 导入包含 URDF、YAML、Mesh 和 `arm_teleop.yaml` 的 ZIP 压缩包；也可手工在 `adapters/robots/<机器人名>/` 配置这些文件。默认选择来自 `middleware/config.yaml`，`HC_ROBOT_NAME` 与 `HC_ROBOT_CONFIG_ROOT` 仍可作为启动时覆盖项。

X1 头显位移映射：原始 VR `+Y` 控制腰部升高、`-Y` 控制降低；低头/抬头的俯仰旋转控制腰部俯仰（`pitch_source: orientation`）。水平位移交给底盘，不再重复驱动腰部。按住左 Grip 时建立参考，松开再按可重新定中心。

腰部俯仰使用头显朝前方向相对水平面的仰俯角，并减去左 Grip 接管时的参考角。偏航转身（包括保持 Grip 转向）不会把俯仰转换成侧倾；升降仍使用竖直位移，`pitch_direction` 仅负责设备方向标定。

相机 WebRTC 仅在收到新图像后发送，不再以固定 30 FPS 重发缓存图像或占位图。视频时间戳使用图像到达中间件的单调时钟（压缩图像在解码前记录），因此表示实际接收节奏而非传感器硬件曝光时间；不直接使用可能暂停/回退的 ROS 仿真时间。编码较慢时只取最新帧，允许丢帧但不补帧；源断流时等待恢复。修改后重启中间件并让头显重新连接。

## Motion 与统一机器人 API

默认配置沿用 v23 与原生关节名。启用 Motion 前先构建两个子模块，并使硬件适配器切换到统一 API 的 internal 接口。完整启动、名称契约和使能规则见 [Motion 工作区](docs/MOTION.md) 与 [机器人公共 API](docs/ROBOT_PUBLIC_API.md)。

```bash
git submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
./bootstrap_colcon.sh build
HC_CONTROL_BACKEND=motion ./run.sh sim
# 硬件适配器已按统一 API 配置后：
HC_CONTROL_BACKEND=motion ROS_DOMAIN_ID=14 ./run.sh teleop
```

`HC_CONTROL_BACKEND=motion` 同时决定中间件公共名称转换、遥操控制链和仿真话题；切换后端需停止旧入口再重新启动。旧配置不带统一 API 文件时继续支持 v23。双方功能清单及本次合并范围见 [分支整合说明](docs/MAIN_HUMANOID_INTEGRATION.md)。

DAgger 人工接管与纠正采集已从本地工作恢复：Motion 模式下握住右 Grip 接管，正常松手交还程序，每段纠正独立保存并沿用 main 的任务和上传流程。配置及使用见 [DAgger 说明](docs/DAGGER.md)。
