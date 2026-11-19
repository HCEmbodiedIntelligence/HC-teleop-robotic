# HC-TJ VR 遥操作

本系统读取中间件发布的头显、左右手柄位姿和 `Joy` 输入，默认使用 `motion` 控制后端：双臂逆运动学（`qp_servo` / `placo`）与实时关节滤波（RTC）由 ROS 2 包 `humanoid_motion_server`（`humanoid_motion_control_node`）求解，VR 控制节点（`RobotArmTeleopNode`）负责坐标映射、离合、底盘、腰部 IK（仅配置了 `body.waist_joint_names` 的机型）、夹爪和网页状态，并经 `StandardMotionTeleopBridge` 与 Motion 节点闭环对接（原 `v23` 后端与 `adapters/v23/` 已移除）。

## 离合状态机

| 状态 | 左手中指 Grip | 右手中指 Grip | 动作 |
| --- | --- | --- | --- |
| 保持 | 松开 | 松开 | 所有子系统保持 |
| 仅左离合 | 按住 | 松开 | 左摇杆控制底盘；头显相对运动控制腰部；双臂和夹爪保持 |
| 仅右离合 | 松开 | 按住 | 左右手柄 6D 位姿同时控制双臂；两侧食指 Trigger 控制对应夹爪 |
| 双离合 | 按住 | 按住 | 底盘、腰部、双臂、夹爪并发 |

离合采用相对映射：按下瞬间记录当前位置，之后只跟随相对变化。松开或数据超时立即停止发布该子系统的新目标，保持最后位置。

## 动作映射

- 按住左 Grip 时，左主摇杆 X 控制底盘左右横移，Y 控制前进后退；该平移通道只依赖左手柄输入，头显暂时丢帧时仍可使用。
- 头显绕 OpenXR 竖直 `Y` 轴的相对转头：经设备符号补偿后同向映射为机器人 `Z-yaw`；左右歪头不会触发底盘旋转。
- 头显向上/向下：腰部同向抬升伸直/下沉弯曲。
- 头显前后位置增量：经设备符号补偿后控制同向躯干俯仰。
- 左右手柄 6D 位姿：对应侧机械臂末端位姿。
- 手柄位姿只使用相对增量；OpenXR 手柄 `-Z` 向前映射为胸部 `zhi_Link` 的 `+X` 向前，左右臂目标统一在胸部坐标系中解算。
- 左右食指 Trigger：对应夹爪从张开到闭合的连续位置。
- 左手 Grip：底盘/腰部离合；右手 Grip：双臂/夹爪总离合。
- 双主摇杆同时向外拨到底一次（左摇杆向左、右摇杆向右）：停止当前离合，并由命令合并层限速将双臂关节明确拉回 `robot.initial_joints`。这避免 7DoF 冗余 IK 在末端到位后留下不同关节解而卡住 homing；两个摇杆回中后可直接重新使用右 Grip 和 Trigger。

### Joy axes 下标契约

PICO 协议 v2 的左右手柄输入会转换成 ROS `sensor_msgs/msg/Joy`。配置中的
`trigger_axis`、`clutch_axis`、`stick_x_axis`、`stick_y_axis` 和
`home_gesture_axis` 都是下面这个 `Joy.axes` 数组的下标，不是机器人或头显的
XYZ 坐标轴：

| 下标 | `Joy.axes` 内容 | PICO v2 原始字段 |
| ---: | --- | --- |
| 0 | Trigger | `trigger` |
| 1 | Grip | `grip` |
| 2 | 主摇杆 X | `primary_axis[0]` |
| 3 | 主摇杆 Y | `primary_axis[1]` |
| 4 | 副摇杆 X | `secondary_axis[0]` |
| 5 | 副摇杆 Y | `secondary_axis[1]` |

等价的数组排列为：

```text
Joy.axes = [
  trigger,
  grip,
  primary_axis.x,
  primary_axis.y,
  secondary_axis.x,
  secondary_axis.y,
]
```

例如，`stick_x_axis: 2`、`stick_y_axis: 3` 表示底盘使用主摇杆的
X/Y 分量；`clutch_axis: 1` 表示 Grip 是离合输入。

双臂末端目标带位置/姿态死区、低通滤波及末端限速，用于抑制静止手柄追踪噪声与突变阶跃；随后经 `StandardMotionTeleopBridge` 转发到 `humanoid_motion_control_node` 的 `servo_p` 通道，在 SDK（`qp_servo` / `placo` + 最终关节 RTC）中求解关节命令并按模型限位输出。

Motion 的通道与运动学参数位于 `adapters/robots/<机器人名>/motion/{motion.yaml,sdk.yaml,channels.yaml}`，遥操作映射与体/手参数位于 `adapters/robots/<机器人名>/arm_teleop.yaml`。桥接与控制源码位于 `adapters/nodes/` 与 `adapters/core/`，Motion ROS 2 包位于 `src/humanoid_motion_interfaces` 与 `src/humanoid_motion_server`。

双臂末端目标统一保存在胸部参考坐标系（如 X1 为 `zhi_Link`）中。腰部运动时，末端目标会随躯干整体运动，不会反向补偿成世界坐标不动。

`/action/ee_poses` 与 `/observation/ee_poses` 都使用机身参考坐标（如 `zhi_Link`），供仿真 marker、Web 3D 视图和 Motion `servo_p` 桥接消费。

## 启动

首次安装 Python 依赖与编译 Motion ROS 2 包：

```bash
./install.sh
./install.sh --camera   # 需启用 WebRTC 视频推流时安装 av/aiortc/pyrealsense2
git submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
./bootstrap_colcon.sh build
```

命令行启动完整网页、VR 自动发现、Motion 逆解/控制和 PyBullet 图形仿真窗口：

```bash
./run.sh sim
```

无显示器或只使用网页端三维仿真视图时（不弹出独立 PyBullet GUI 窗口）：

```bash
./run.sh sim --headless
```

也可直接运行 `./run.sh`（默认仅启动中间件，在网页“系统配置 → PyBullet 仿真”按需无窗口启动仿真与 VR 控制，并可开启“显示三维仿真视图”）。

脚本读取 `middleware/config.yaml` 中 `robot_profiles.active` 指向的 Dashboard 当前配置；网页切换配置后需要退出并重新启动。

连接真机时，本仓库只启动通用遥操作部分，硬件由 `HC_X1` 独立启动：

```bash
ROS_DOMAIN_ID=14 ./run.sh teleop

# 另一个终端
cd ~/HC_X1
./start.sh
```

根目录只保留 `run.sh` 产品入口，旧 `--robot/--v23/--generic/--legacy/--sim-only` 与 `run_generic_controller.sh` 均已移除。

两边必须使用相同的 `ROS_DOMAIN_ID`。启动后松开两个 Grip；准备好再按对应离合。

完整仿真入口默认将手柄、目标/实际末端位姿及关节命令/反馈记录到 `runtime/teleop_logs/`。复现抖动后可运行 `python3 -m tools.diagnostics.analyze_teleop_log <CSV日志>` 定位输入、IK 或关节跟踪环节。

## ROS 接口

输入：

- `/vrdata`（`std_msgs/msg/String` JSON，包含头显/双手位姿、摇杆、Trigger、Grip 和按键边沿）
- `/observation/joint_states`
- `/status/solver/joint_states`（求解器健康/关节目标；原版后端也用它连接 PID）
- `/control/emergency_stop`
- `/control/teleop/enabled`（`Bool`，网页或 ROS 命令使能/停用）

输出：

- `/control/vr/joint_states`（VR/IK 内部命令源）
- `/control/exoskeleton/joint_states`（外骨骼 `sensor_msgs/msg/JointState` 输入）
- `/control/source`（当前控制源，`std_msgs/msg/String`，瞬态本地持久化；值为 `vr` 或 `exoskeleton`）
- `/action/joint_states`（中间件选择后的唯一标准机器人命令）
- `/control/solver/joint_targets`（Motion 桥接经过限流与许可过滤后的内部双臂关节输出，由 VR 适配器合并夹爪/腰/头）
- `/action/base/command`
- `/action/ee_poses`
- `/control/solver/ee_targets`（内部肩基任务目标）
- `/observation/ee_poses`
- `/action/left_hand/joint_states`、`/action/right_hand/joint_states`
- `/status/teleop`

控制服务：

```bash
ros2 service call /teleop/arm/enable std_srvs/srv/Trigger '{}'
ros2 service call /teleop/arm/disable std_srvs/srv/Trigger '{}'
ros2 service call /teleop/arm/reset_reference std_srvs/srv/Trigger '{}'
```

## 实机切换前必须修改

当前 X1 配置位于 `adapters/robots/x1/arm_teleop.yaml`。连接其他实机时至少要：

1. 将 `control.enabled_on_start` 改为 `false`，由人工服务显式使能。
2. 将 `body.base_command_mode` 改为 `velocity`。
3. 将 `grippers.include_sim_joints` 改为 `false`，避免向 Marvin 臂桥发送仿真夹爪关节名。
4. 在低速、空载、有硬件急停的条件下重新标定坐标轴、工作空间和速度限制。

软件急停不能替代硬件急停。
