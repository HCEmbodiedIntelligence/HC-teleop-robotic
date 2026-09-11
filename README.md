# HC Humanoid Teleop

HC 通用遥操作框架的官方模块化解耦 ROS 2 工作区。系统基于 `HCEmbodiedIntelligence` 官方 7 个核心独立解耦仓库及辅助遥操作接收端组装：

1. `humanoid_motion_interfaces`：通用 ROS 2 消息/服务/Action 通信规范定义
2. `humanoid_driver_interface`：统一 pluginlib 驱动规范（`RobotDriverPlugin`、`GripperDriverPlugin`）
3. `humanoid_driver_runtime`：硬件运行层与驱动加载器（内置 `RosTopicRobotDriver`，看门狗保护，发布 `/hc_teleop/joint_states`）
4. `humanoid_gripper`：独立夹爪驱动体系（内置 `RosTopicGripperDriver`）
5. `humanoid_camera`：RealSense 多相机管理、曝光中点时间戳对齐与可靠 QoS
6. `humanoid_motion_server`：权威运动控制、IK 解算与限位保护（权威发布 `/hc_teleop/joint_cmd`）
7. `humanoid_adapter_manager` (`humanoid_manager`)：Web 管理控制台（端口 7876）、机器人组合配置与 MCAP 数据录制
8. `hc_teleop_recv`：VR 遥操作手柄与位姿 UDP 接收端

---

## 快速构建与启动

### 1. 编译全工作区

```bash
cd /home/maple/test/HC-teleop-robotic
./bootstrap_colcon.sh build
```

### 2. 启动方式

#### 方式 A：启动 Web 管理配置后台（推荐）
```bash
./run.sh
# 浏览器打开 http://localhost:7876/dashboard/#robots 即可可视化管理、配置、启动与急停机器人
```

#### 方式 B：启动 Mock 仿真与运动规划闭环
```bash
./run.sh mock
```

#### 方式 C：启动驱动运行层
```bash
./run.sh driver
```

---

## 使用 Humanoid Motion Server

`hc_motion_backend_robo_manip` 使用项目内连接的
`src/humanoid_motion_server` 的完整 `CommandPipeline`，统一执行 ServoP 会话、
反馈超时、Servo 租约、关节限位检查和最终 RTC，并提供实测关节 FK。Motion
Server 与接口仓以固定提交的 Git submodule 保存，仍保持各自独立历史。后端是
独立进程，只发布 candidate，不会绕过 HC 安全仲裁器。

首次拉取和构建：

```bash
# 新 clone 推荐直接使用 git clone --recurse-submodules。
git submodule update --init --recursive

# 指向 ruckig 0.17.3、toppra 0.6.8 等 ABI 固定依赖的安装前缀。
# 默认读取本工作区 .deps/robo_manip；其他安装位置必须显式指定。
export HUMANOID_MOTION_SDK_DEPS_PREFIX=/path/to/robo_manip/dependencies

./bootstrap_colcon.sh build
./run.sh profile:=x1 mode:=sim
```

默认 `motion_backend:=profile` 读取机器人 profile；X1 和 OpenArmX 当前均默认
使用 `robo_manip`，无需显式指定。旧 KDL 后端已移除；独立扩展后端使用
`motion_backend:=external`，并自行启动符合 HC 消息契约的后端。
运行时仅 source ROS 2 和本工作区的安装环境，不再 source 外部 Humanoid underlay。

## X1 真机

厂商驱动继续由独立的 `HC_X1` 仓库启动，本仓库不会 source 或修改它。两边使用
同一个 ROS Domain；`run.sh` 和 HC_X1 均默认使用 14。

```bash
# 终端 A：真机硬件仓库
cd /home/maple/test/HC_X1
ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0 ./start.sh

# 终端 B：先做只读 shadow 验证，不会向 HC_X1 发布命令
cd /home/maple/test/HC-teleop-robotic
ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0 \
  ./run.sh profile:=x1 mode:=shadow motion_backend:=robo_manip

# 验证反馈、方向、限位和急停后才切换真机输出
ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0 \
  ./run.sh profile:=x1 mode:=real motion_backend:=robo_manip \
  start_auto_lease:=true
```

`mode:=real` 会自动启动本仓库的 X1 兼容适配器，把 HC_X1 的
`/hc_teleop/joint_states` 转换为 `/robots/x1/state/joints`，并将唯一仲裁后的
强类型命令合并、校验后发送到 `/hc_teleop/joint_cmd`。`start_auto_lease:=true`
会在发现 PICO 会话后自动启用控制，只应在低速真机验收阶段使用。

## 新增机械臂与末端工具开发

无论是接入单臂、双臂、三臂还是不同自由度（3~7轴）机械臂与末端夹爪，均由 `URDF + profile.yaml` 统一配置驱动。详细接入步骤请查阅文档：
👉 **[新增机械臂与末端工具控制接入指南 (docs/ADD_NEW_ROBOT_GUIDE.md)](docs/ADD_NEW_ROBOT_GUIDE.md)**

