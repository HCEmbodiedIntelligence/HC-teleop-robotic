# HC-TJ VR 遥操作（仿真第一阶段）

本系统读取中间件发布的头显、左右手柄位姿和 `Joy` 输入，默认使用重构的 ControllerV23 任务/限位差分 IK 生成关节目标。VR 节点只负责坐标映射、离合、底盘、夹爪和网页状态；重构后端负责双臂/腰部的 FrameTask、限位求解及 Pinocchio 积分。当前交付默认只用于 PyBullet 仿真。

## 离合状态机

| 状态 | 左手中指 Grip | 右手中指 Grip | 动作 |
| --- | --- | --- | --- |
| 保持 | 松开 | 松开 | 所有子系统保持 |
| 仅左离合 | 按住 | 松开 | 左摇杆控制底盘；头显相对运动控制腰部；双臂和夹爪保持 |
| 仅右离合 | 松开 | 按住 | 左右手柄 6D 位姿同时控制双臂；两侧食指 Trigger 控制对应夹爪 |
| 双离合 | 按住 | 按住 | 底盘、腰部、双臂、夹爪并发 |

离合采用相对映射：按下瞬间记录当前位置，之后只跟随相对变化。松开或数据超时立即停止发布该子系统的新目标，保持最后位置。

## 动作映射

- 左主摇杆 X：底盘左右；Y：底盘前后。
- 头显绕 OpenXR 竖直 `Y` 轴的相对转头：经设备符号补偿后同向映射为机器人 `Z-yaw`；左右歪头不会触发底盘旋转。
- 头显向上/向下：腰部同向抬升伸直/下沉弯曲。
- 头显前后位置增量：经设备符号补偿后控制同向躯干俯仰。
- 左右手柄 6D 位姿：对应侧机械臂末端位姿。
- 手柄位姿只使用相对增量；手柄 `+Z` 向前映射为胸部 `zhi_Link` 的 `+X` 向前，左右臂目标统一在胸部坐标系中解算。
- 左右食指 Trigger：对应夹爪从张开到闭合的连续位置。
- 左手 Grip：底盘/腰部离合；右手 Grip：双臂/夹爪总离合。
- 双主摇杆同时向外拨到底一次（左摇杆向左、右摇杆向右）：停止当前离合，并由命令合并层限速将双臂关节明确拉回 `robot.initial_joints`。这避免 7DoF 冗余 IK 在末端到位后留下不同关节解而卡住 homing；两个摇杆回中后可直接重新使用右 Grip 和 Trigger。

双臂末端目标带位置/姿态死区和低通滤波，用于抑制静止手柄追踪噪声；v2.3 每周期求解带速度及一步关节位置边界的加权最小二乘，再通过 Pinocchio `integrate()` 生成小步关节命令。

v2.3 的 URDF、任务权重和速度限制来自 `robot_configs/hc_tj_description/controller_v23.yml`，关节位置限制来自 URDF。第四个 task 标量目前按 gain 解释；HC-TJ 双臂使用 `3.0`，躯干使用 `1.0`，兼顾手臂响应和身体平稳，这仍属于待与 vendor 差分验证的行为假设。原加密 `Controller.Run_IK + PID` 后端保留为 `--generic` 回退。

双臂末端目标统一保存在胸部 `zhi_Link` 坐标系中。腰部运动时，末端目标会随躯干整体运动，不会反向补偿成世界坐标不动。

`/io_teleop/target_ee_poses` 与 `/io_teleop/actual_ee_poses` 都使用胸部坐标，供仿真 marker 和诊断直接比较。VR 节点另将目标转换到左右肩基，并通过 `/io_teleop/controller_target_ee_poses` 送入控制器；显示和 IK 坐标不能混用。

## 启动

首次安装仿真依赖：

```bash
cd /home/maple/test/HC-teleop-robotic
./install.sh --sim
```

终端 1 启动 VR 中间件：

```bash
./run.sh
```

终端 2 启动 HC-TJ 图形仿真和遥操作控制：

```bash
./run_sim_teleop.sh
```

没有图形显示时：

```bash
./run_sim_teleop.sh --headless
```

后端对照：`--v23`（默认重构）、`--generic`（原加密控制器和 PID）、`--legacy`（PyBullet IK）。

两边必须使用相同的 `ROS_DOMAIN_ID`。启动后松开两个 Grip；准备好再按对应离合。

`run_sim_teleop.sh` 默认将手柄、目标/实际末端位姿及关节命令/反馈记录到 `runtime/teleop_logs/`。复现抖动后可运行 `analyze_teleop_log.py <CSV日志>` 定位输入、IK 或关节跟踪环节。

## ROS 接口

输入：

- `/vr/head_pose`
- `/vr/left_controller_pose`、`/vr/right_controller_pose`
- `/vr/left_controller/input`、`/vr/right_controller/input`
- `/io_teleop/joint_states`
- `/io_teleop/sol_q`（求解器健康/关节目标；原版后端也用它连接 PID）
- `/teleop/emergency_stop`
- `/teleop/arm/enabled`（`Bool`，网页或 ROS 命令使能/停用）

输出：

- `/io_teleop/joint_cmd`
- `/io_teleop/joint_cmd_arm`（v2.3 或原版 PID 的内部输出，由 VR 适配器合并夹爪）
- `/io_teleop/target_base_move`
- `/io_teleop/target_ee_poses`
- `/io_teleop/controller_target_ee_poses`（内部肩基任务目标）
- `/io_teleop/actual_ee_poses`
- `/io_teleop/joint_cmd_finger_left`、`/io_teleop/joint_cmd_finger_right`
- `/teleop/arm/status`

控制服务：

```bash
ros2 service call /teleop/arm/enable std_srvs/srv/Trigger '{}'
ros2 service call /teleop/arm/disable std_srvs/srv/Trigger '{}'
ros2 service call /teleop/arm/reset_reference std_srvs/srv/Trigger '{}'
```

## 实机切换前必须修改

当前 [arm_teleop.yaml](arm_teleop.yaml) 是仿真配置。连接 TJ 实机前至少要：

1. 将 `control.enabled_on_start` 改为 `false`，由人工服务显式使能。
2. 将 `body.base_command_mode` 改为 `velocity`。
3. 将 `grippers.include_sim_joints` 改为 `false`，避免向 Marvin 臂桥发送仿真夹爪关节名。
4. 在低速、空载、有硬件急停的条件下重新标定坐标轴、工作空间和速度限制。

软件急停不能替代硬件急停。
