# HC Teleop Middleware

一个面向机器人遥操作的局域网中间件：接收 PICO 位姿和手柄数据，通过单一 `/vrdata` 发布到 ROS 2，并提供机器人导入、话题录制、状态监控和配置网页。

原有的 `d435_webrtc_server.py` 和 `udp_receiver_test.py` 保留不变。新服务兼容它们的关键协议：

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
./run.sh
```

然后访问 `http://<机器人IP>:7876/dashboard/#config`。相机不在系统配置页中管理；需要独立启用 WebRTC 服务时安装可选依赖：

```bash
./install.sh --camera
```

### 网页导入机器人配置

“系统配置”页顶部提供 HC 机器人配置工作流：选择已导入机器人、导入新配置、应用配置。导入时同时选择一个 URDF 和一个 YAML，配置 ID 只能使用字母、数字、点、下划线或短横线。文件会保存到 `robot_configs/<配置ID>/`，原文件不会覆盖已有配置。

支持两种 YAML：

- HC 通用 `vr_configs.yml`：包含 `urdf_path`、`arms`、`controller_indices`，可选 `folding_waist`。导入器会校验所有 joint/link 索引，并生成标准接口的 `controller_v23.yml`；双臂配置还会生成 `arm_teleop.yaml`。
- 重构控制器 `controller_v23.yml`：包含 `model.free_joints`、`task` 和 `limit`。导入器会校验 URDF 关节与任务 link，并统一 ROS 话题和 URDF 相对路径。该格式只提供 IK 控制配置，不包含 HC PyBullet 仿真所需的 `vr_configs.yml`。

点击“应用配置”会写入 `middleware.yaml` 的 `robot_profiles.active` 并立即下发软件停止。退出并重新执行 `./run_sim_teleop.sh` 后，仿真、VR 适配器和 v2.3 控制器会共同读取所选配置。也可临时通过 `HC_ROBOT_NAME`、`HC_ROBOT_CONFIG_ROOT` 覆盖网页选择。

导入接口对齐当前遥操作链路的标准话题：`/hc_teleop/joint_states`、`/hc_teleop/joint_cmd_arm`、`/hc_teleop/joint_cmd`、`/hc_teleop/controller_target_ee_poses`、`/hc_teleop/target_ee_poses`、`/hc_teleop/actual_ee_poses`、`/hc_teleop/sol_q` 和 `/hc_teleop/target_base_move`。网页仅保留配置和运行状态，原“实时事件”页面不再显示。

### 话题录制

“话题录制”页从 ROS Graph 读取当前话题和消息类型，可直接添加自定义消息，也可在“系统配置 → HC 标准 ROS 2 接口”中切换内置消息是否录制。点击“保存录制配置”后写回 `middleware.yaml`；启用录制时，消息按启动会话写入 `runtime/topic_recordings/ros2_YYYYMMDD_HHMMSS.jsonl`。

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

如果已有工作空间，在运行前先 source 对应的 `install/setup.bash`；`run.sh` 会自动 source `/opt/ros/humble/setup.bash`。ROS 域仍由标准环境变量控制：

```bash
ROS_DOMAIN_ID=12 ./run.sh
```

## 数据流

```text
ROS 2 topics ──> recording rules ──> rate limit ──> runtime/topic_recordings/*.jsonl

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

位姿中断超过 200 ms 或头显跟踪失效时，服务向 `/teleop/emergency_stop` 发布 `std_msgs/msg/Bool(data=true)`。启动和配置热重载也默认急停，可在配置页关闭。

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
- 网页“保存并应用”会原子写回 `middleware.yaml` 并热重载。`server.host`、`server.port` 或 `robot_profiles.root` 改动会保存，但需要重启进程；其余配置立即应用。

## 验证

```bash
/usr/bin/python3 -m unittest discover -s tests -v
/usr/bin/python3 -m compileall -q hc_teleop_middleware middleware_server.py
```

## HC-TJ 机械臂仿真遥操作

VR 到 HC-TJ 双臂、腰部、底盘和夹爪的离合控制见 [TELEOP.md](TELEOP.md)。快速启动：

双臂回零：不用按 Grip，同时把左主摇杆向左、右主摇杆向右拨到底一次；回零后先让两个摇杆回中，才能再次触发。回零期间命令合并层会限速拉回 `initial_joints`，避免 7DoF 冗余解只让末端到位却永久卡在 homing；完成后可直接再次按右 Grip 和 Trigger，无需急停重启。

```bash
./install.sh --sim
./run_sim_teleop.sh
```

`--sim` 会额外创建与开发板一致的 `hc-teleop-controller` Conda 环境
（Pinocchio 3.7 + CasADi 3.7）。默认启动已完成数值验证的重构 v2.3 后端。
原加密通用链可用 `./run_sim_teleop.sh --generic` 做 A/B 对照；旧版
PyBullet IK 可用 `./run_sim_teleop.sh --legacy` 排障。

仿真启动时会自动以 30 Hz 将手柄位姿、目标/实际末端位姿、关节命令/反馈和离合状态写入 `runtime/teleop_logs/`。复现抖动时按住右 Grip 并尽量保持双手静止 5–10 秒，退出仿真后分析对应日志：

```bash
/usr/bin/python3 analyze_teleop_log.py runtime/teleop_logs/teleop_YYYYMMDD_HHMMSS.csv
```

若日志同时包含主动移动和静止保持，可加 `--start 秒数 --end 秒数` 只分析静止区间。

可用 `TELEOP_DIAGNOSTICS=0` 禁用记录，或用 `TELEOP_LOG_RATE=60` 调整记录频率。诊断写盘由独立进程完成，不占用遥操作控制循环。

仿真底盘坐标约定为 `+X` 前进、`+Y` 向左，底盘消息顺序为 `[yaw, forward, lateral]`。PyBullet 窗口需要先点击获得焦点；按住 `Ctrl` 并拖动鼠标左键旋转视角，按住 `Ctrl` 并拖动中键平移视角，滚轮缩放。修改仿真相机或坐标配置后需要退出并重新运行脚本。

按住左手柄中指 Grip 后，左主摇杆 Y 控制底盘前进/后退，X 控制左/右横移。摇杆平移直接使用左手柄 `Joy` 数据，不再依赖头显跟踪是否有效；松开 Grip、输入超时、急停或双臂回零时都会发布零命令。

机械臂只使用手柄相对位姿增量：手柄 `+Z` 向前对应胸部 `zhi_Link` 的 `+X` 向前。目标先在胸部坐标系生成，再转换到左右肩部任务坐标交给 v2.3 求解器；腰部运动不会改变这项视觉/手柄约定。

默认链路为 `controller_target_ee_poses → ControllerV23 → FrameTask/AxisTask/JointTask → solve_ik → 速度及一步位置限位 → Pinocchio integrate → joint_cmd_arm → VR 适配器/夹爪合并 → joint_cmd`。`target_ee_poses` 和 `actual_ee_poses` 专供仿真显示/诊断，始终使用胸部 `zhi_Link` 坐标，使 marker 与法兰直观对应；内部控制目标才转换为左右肩基坐标。源码位于 `vendor/io_unicontroller_ros2/control_v23_reconstructed`，参数位于 `robot_configs/hc_tj_description/controller_v23.yml`。压缩包中错误的相对 Jacobian 已按 HC-TJ 有限差分结果修正，重构说明同时保留了尚未确认的行为假设。

新机器人优先从网页或 CLI 导入包含 URDF、YAML、Mesh 和 `arm_teleop.yaml` 的 ZIP 压缩包；也可手工在 `robot_configs/<机器人名>/controller_v23.yml` 与 `robot_configs/<机器人名>/arm_teleop.yaml` 配置 URDF、`free_joints`、pose/axis/joint tasks、权重、限速及手柄映射。默认选择来自 `middleware.yaml`，`HC_ROBOT_NAME` 与 `HC_ROBOT_CONFIG_ROOT` 仍可作为启动时覆盖项。

