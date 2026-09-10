> 旧双臂遥操作栈：该目录保留左右双臂及历史夹爪/腰部假设，不作为通用机器人前端扩展。当前平台统一入口使用 `hc_teleop_recv` 与模型插件配置。

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

真机双臂采用分层结构：本仓库 v23 逆解生成内部 VR 命令 `/hc_teleop/joint_cmd_vr`，外骨骼发布 `/hc_teleop/joint_cmd_exoskeleton`；中间件通过可锁存的 `/hc_teleop/control_source` 广播当前选择，并在状态监控页选择其中一路转发到标准 `/hc_teleop/joint_cmd`。每个硬件仓库用独立固定频率节点完成速度/加速度约束和厂商命令下发。OpenArmX 零重力主动端可使用 `/home/maple/hc_openarmx` 中的 `make teleop-gravity-hc` 接入；切换到外骨骼时会锁存主动端与机器人当前位置，再按关节增量控制。

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

然后访问 `http://<机器人IP>:7876/dashboard/#config`。相机不在系统配置页中管理；需要独立启用 WebRTC 服务时安装可选依赖：

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

根目录入口 `./run.sh` 默认只启动中间件（等价于 `./run.sh middleware`）。在网页“系统配置”导入并应用机器人 ZIP 后，可独立启动、停止或重启 PyBullet，并选择无窗口模式、关节调试和是否同时启动 IK/VR 控制。窗口显示在运行中间件的服务器桌面；无桌面环境请选择无窗口模式。初始姿态和相机配置读取包内 `vr_configs.yml`。停止仿真保留网页、VR 网关和录制服务；导入、切换机器人或修改仿真选项、ROS Domain ID 会先停止当前仿真，再从网页重新启动。

显式 `./run.sh sim` 仍启动完整仿真，`./run.sh teleop` 启动连接真机所需的 VR、Dashboard 和 IK/控制；这两种方式由命令行管理进程，不允许从网页另起一套仿真。网页管理模式下，中间件退出会清理其启动的仿真与控制进程。

### 网页导入机器人配置

“系统配置”页顶部提供 HC 机器人配置工作流：选择已导入机器人、导入新配置、应用配置。导入时同时选择一个 URDF 和一个 YAML，配置 ID 只能使用字母、数字、点、下划线或短横线。文件会保存到 `adapters/robots/<配置ID>/`，X1 的内置资料位于 `adapters/robots/x1/`。

支持两种 YAML：

- HC 通用 `vr_configs.yml`：包含 `urdf_path`、`arms`、`controller_indices`，可选 `folding_waist`。导入器会校验所有 joint/link 索引，并生成标准接口的 `controller_v23.yml`；双臂配置还会生成 `arm_teleop.yaml`。
- 重构控制器 `controller_v23.yml`：包含 `model.free_joints`、`task` 和 `limit`。导入器会校验 URDF 关节与任务 link，并统一 ROS 话题和 URDF 相对路径。该格式只提供 IK 控制配置，不包含 HC PyBullet 仿真所需的 `vr_configs.yml`。

点击“应用配置”会写入 `middleware/config.yaml` 的 `robot_profiles.active` 并立即下发软件停止。真机模式退出并重新执行 `./run.sh teleop`；仿真模式重新执行 `./run.sh sim`。机器人型号以 Dashboard/`middleware/config.yaml` 中当前激活的 profile 为准。

导入接口对齐当前遥操作链路的标准话题：`/hc_teleop/joint_states`、`/hc_teleop/joint_cmd_arm`、`/hc_teleop/joint_cmd`、`/hc_teleop/controller_target_ee_poses`、`/hc_teleop/target_ee_poses`、`/hc_teleop/actual_ee_poses`、`/hc_teleop/sol_q` 和 `/hc_teleop/target_base_move`。状态监控页可在 VR 与外骨骼控制源之间切换，并实时显示命令、反馈关节角和误差。

### 话题录制

“话题录制”页从 ROS Graph 读取当前话题和消息类型，可直接添加自定义消息，也可在“系统配置 → HC 标准 ROS 2 接口”中切换内置消息是否录制。点击“保存录制配置”后写回 `middleware/config.yaml`；启用录制时，消息按启动会话写入 `runtime/topic_recordings/`。MCAP 录制使用独立 ROS Context 和多线程执行器，不与 Dashboard 健康检测或 WebRTC 视频回调共用执行器。X1 默认只录制规范化压缩相机话题；同源 `/io_teleop` 别名和大体积未压缩回退流默认不勾选，仍可在界面按需启用。

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

位姿中断超过 `vr.pose_timeout_ms`（当前配置为 600 ms）、VR 时间戳停止推进或头显跟踪失效时，服务向 `/teleop/emergency_stop` 发布 `std_msgs/msg/Bool(data=true)`。启动和配置热重载是否急停由 `safety.stop_on_startup` 控制。

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

按住左手柄中指 Grip 后，左主摇杆 Y 控制底盘前进/后退，X 控制左/右横移。摇杆平移直接使用左手柄 `Joy` 数据，不再依赖头显跟踪是否有效；松开 Grip、输入超时、急停或双臂与腰部回零时都会发布零命令。

X1 的 `./run.sh sim` 会应用 `arm_teleop.yaml` 中的 `simulation_body`，启用三个腰关节。按住左 Grip 时以当前头显姿态为参考：转头控制底盘转向，头部位移控制腰部升降/俯仰；松开后再次按住会重新设定参考。底盘偏航死区为 0.18 rad（约 10°），最大角速度为 0.45 rad/s。仿真按时间积分速度命令，200 ms 未收到新底盘命令即停止；真机模式保留原有腰部关闭配置。

机械臂只使用手柄相对位姿增量：当前 VR 数据协议中手柄 `+Z` 向前，对应胸部 `zhi_Link` 的 `+X` 向前。目标先在胸部坐标系生成，再转换到左右肩部任务坐标交给 v2.3 求解器；腰部运动不会改变这项视觉/手柄约定。

X1 仿真头部显示 `HEAD ACTUAL`（青色标签、短坐标轴）与 `HEAD TARGET`（橙色标签、长坐标轴），橙色连线表示位置误差。实际位姿由关节反馈和 `camera_head` FK 得到，目标是头显输入映射到腰部升降/俯仰后对应的机器人头部位姿，并非 VR 房间中的原始绝对位置。松开左 Grip 或停用控制时目标回到实际位姿；数据断流 0.5 s 后隐藏标记。话题 `/hc_teleop/head_tracking_poses` 使用 `geometry_msgs/msg/PoseArray`，顺序为 `[actual, target]`，`header.frame_id=robot_base` 表示 PyBullet 模型根部位姿坐标系。需要带 GUI 启动仿真才能看到标记。

默认链路为 `controller_target_ee_poses → ControllerV23 → FrameTask/AxisTask/JointTask → solve_ik → 速度及一步位置限位 → Pinocchio integrate → joint_cmd_arm → VR 适配器/夹爪合并 → joint_cmd_vr → Command Mux → joint_cmd`。`target_ee_poses` 和 `actual_ee_poses` 专供仿真显示/诊断，始终使用胸部 `zhi_Link` 坐标，使 marker 与法兰直观对应；内部控制目标才转换为左右肩基坐标。源码位于 `adapters/v23/`，X1 参数位于 `adapters/robots/x1/controller_v23.yml`。

新机器人优先从网页或 CLI 导入包含 URDF、YAML、Mesh 和 `arm_teleop.yaml` 的 ZIP 压缩包；也可手工在 `adapters/robots/<机器人名>/` 配置这些文件。默认选择来自 `middleware/config.yaml`，`HC_ROBOT_NAME` 与 `HC_ROBOT_CONFIG_ROOT` 仍可作为启动时覆盖项。

X1 头显位移映射：原始 VR `+Y` 控制腰部升高、`-Y` 控制降低；`+Z`（前移）控制前倾、`-Z`（后移）控制后仰。俯仰输入采用位置增量，不使用头显低头/抬头的旋转角度。按住左 Grip 时建立参考，松开再按可重新定中心。
