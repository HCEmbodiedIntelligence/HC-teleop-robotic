# HC Humanoid Teleop

HC 通用遥操作框架的官方模块化解耦 ROS 2 工作区。系统基于 `HCEmbodiedIntelligence` 官方 7 个核心独立解耦仓库及辅助遥操作接收端组装：

1. **`humanoid_motion_interfaces`**：通用 ROS 2 消息/服务/Action 通信规范定义
2. **`humanoid_driver_interface`**：统一 pluginlib 驱动规范（`RobotDriverPlugin`、`GripperDriverPlugin`）
3. **`humanoid_driver_runtime`**：硬件运行层与驱动加载器（内置 `MockRobotDriver`、`RosTopicRobotDriver`，看门狗保护，权威发布 `/hc_teleop/joint_states`）
4. **`humanoid_gripper`**：独立夹爪驱动体系（内置 `RosTopicGripperDriver`）
5. **`humanoid_camera`**：RealSense 多相机管理、曝光中点时间戳对齐与可靠 QoS
6. **`humanoid_motion_server`**：权威运动控制、IK 解算与限位保护（权威发布 `/hc_teleop/joint_cmd`）
7. **`humanoid_adapter_manager` (`humanoid_manager`)**：Web 管理控制台（端口 7876）、机器人组合配置与 MCAP 数据录制
8. **`hc_teleop_recv`**：VR 遥操作手柄与位姿 UDP 接收端，支持 `/teleop/emergency_stop` 急停联动

---

## 快速构建与启动

### 1. 编译全工作区

```bash
cd /home/maple/test/HC-teleop-robotic
git submodule update --init --recursive
./bootstrap_colcon.sh build
```

### 2. 启动仿真与运行

工作区提供统一入口 `./run.sh`，支持多种启动模式：

#### 方式 A：一键启动机器人仿真（含 RViz 3D 可视化）

```bash
# 启动 OpenArmX 双臂仿真（Driver + Motion Server + Teleop VR + TF + RViz）
./run.sh sim openarmx
# 或使用经典 ROS 参数语法：
./run.sh profile:=openarmx mode:=sim

# 启动 X1 双臂人形机器人仿真
./run.sh sim x1
# 或：
./run.sh profile:=x1 mode:=sim

# 无头模式启动仿真（不弹出 RViz 窗口）
./run.sh sim openarmx --headless
# 或：
./run.sh profile:=openarmx mode:=sim --rviz-sim:=false
```

#### 方式 B：启动 Web 管理控制台后台

```bash
./run.sh
# 或
./run.sh web
```
在浏览器中打开 **http://localhost:7876/dashboard/** 即可进行可视化配置、导入导出插件、点动测试夹爪、配置多相机对齐采集以及录制 MCAP 数据。

#### 方式 C：启动官方 Mock 验证闭环

```bash
./run.sh mock
```
直接启动官方 `humanoid_motion_server` 与 `humanoid_driver_runtime` 的最小闭环 Mock 仿真流水线。

---

## 独立启动 RViz 可视化

仿真在后台或另一个终端运行时，可以通过 `./rviz.sh` 独立打开 RViz 监视模型状态：

```bash
# 打开 OpenArmX 可视化
./rviz.sh openarmx

# 打开 X1 可视化
./rviz.sh x1

# 强制打开（无需等待后台话题发布）
./rviz.sh --force openarmx
```

`rviz.sh` 自动读取 `runtime/active_ros_domain` 中记录的 ROS Domain ID（默认为 14），并订阅标准 `/robot_description` 模型与 `/hc_teleop/joint_states` 实时关节流。

---

## 话题与接口协议

- **关节状态反馈**：`/hc_teleop/joint_states` (`sensor_msgs/msg/JointState`)
- **权威执行命令**：`/hc_teleop/joint_cmd` (`sensor_msgs/msg/JointState`)
- **遥操作笛卡尔目标**：`/teleop/<arm>/servo_p` (`geometry_msgs/msg/PoseStamped`)
- **正向运动学反馈**：`/teleop/<arm>/fk_pose` (`geometry_msgs/msg/PoseStamped`)
- **系统急停**：`/teleop/emergency_stop` (`std_msgs/msg/Bool`)
- **夹爪状态与命令**：`/hc_teleop/gripper_states`, `/hc_teleop/gripper_commands`
- **机器人模型与坐标变换**：`/robot_description`, `/tf`, `/tf_static`

---

## 测试与诊断

```bash
# 运行全套单元测试与闭环验证
./bootstrap_colcon.sh test

# 检查 ROS 话题与节点连通性
source install/setup.bash
export ROS_DOMAIN_ID=14
ros2 topic list
ros2 topic echo /hc_teleop/joint_states
```
