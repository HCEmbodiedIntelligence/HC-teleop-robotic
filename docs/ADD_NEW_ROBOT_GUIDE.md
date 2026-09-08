# 新增机械臂与末端工具控制接入指南 (Adding New Robots & Tools Guide)

本工程采用 **"URDF + Profile (YAML)" 统一驱动架构**。无论是**单臂、双臂、三臂**，还是 **3轴、6轴、7轴** 机械臂，以及各类**二指夹爪、气动夹具或灵巧手**，均可实现**零代码开发、即插即用**的通用遥操作控制。

---

## 目录
- [一、整体架构与数据流](#一整体架构与数据流)
- [二、接入流程四步法](#二接入流程四步法)
  - [步骤 1：准备机器人 URDF 模型](#步骤-1准备机器人-urdf-模型)
  - [步骤 2：配置 profile.yaml](#步骤-2配置-profileyaml)
  - [步骤 3：选择 IK/FK 运动学后端](#步骤-3选择-ikfk-运动学后端)
  - [步骤 4：运行与调试](#步骤-4运行与调试)
- [三、工具与夹爪控制配置 (Gripper / Tools)](#三工具与夹爪控制配置-gripper--tools)
- [四、完整示例：配置单臂 6 轴机械臂 (Single Arm Example)](#四完整示例配置单臂-6-轴机械臂-single-arm-example)
- [五、关键参数调优与常见问题](#五关键参数调优与常见问题)

---

## 一、整体架构与数据流

```text
       [VR 头显 / 手柄]
              │ UDP /vrdata (VrFrame)
              ▼
   ┌──────────────────────┐
   │    hc_vr_gateway     │  输入层：无机器人依赖，纯粹解析 VR 位姿与按键
   └──────────┬───────────┘
              │ input/vr_frame
              ▼
   ┌──────────────────────┐
   │    hc_vr_mapper      │  映射层：根据 profile.yaml 中的手柄绑定和 axis_mapping
   └──────────┬───────────┘  由 Grip 键作为离合器，计算末端相对目标与夹爪扳机模拟量
              │ teleop/cartesian_targets  &  control/joint_candidate
              ▼
   ┌──────────────────────┐
   │   IK/FK 后端选择器   │  运动学层：从 URDF 自动提取关节链条，完成正逆运动学
   │  ┌────────────────┐ │  - Motion Server CommandPipeline (ServoP / RTC / 反馈检查)
   │  │ hc_motion_...  │ │  - RoboManip / Humanoid Motion Server (Pinocchio/RTC动力学)
   │  └────────────────┘ │
   └──────────┬───────────┘
              │ control/joint_candidate (各轴目标关节角)
              ▼
   ┌──────────────────────┐
   │   command_arbiter    │  安全仲裁层：速度/加速度限幅、租约管理与超时看门狗
   └──────────┬───────────┘
              │ control/joint_command
              ▼
     [机器人硬件驱动 / 仿真]
```

---

## 二、接入流程四步法

### 步骤 1：准备机器人 URDF 模型
将机械臂的 URDF 文件放置于功能包目录下（例如 `src/hc_robot_<model>/urdf/<model>.urdf`）。
* **重要校验**：
  1. 运动链各关节必须具备完整限位：`<limit lower="..." upper="..." velocity="..." />`。
  2. 记下机械臂基座连杆名（`base_frame`，如 `base_link`）和末端工具中心点连杆名（`tip_frame`，如 `tool0` 或 `flange`）。

### 步骤 2：配置 `profile.yaml`
在 `src/hc_robot_<model>/config/profile.yaml` 中声明机器人组件与遥操作绑定：

```yaml
schema: hc-teleop-profile/v1
robot_id: my_robot
display_name: My Robot Name

resources:
  urdf: package://hc_robot_<model>/urdf/<model>.urdf

components:
  # 1. 声明机械臂 (支持单臂、双臂或多臂)
  - id: my_arm
    kind: arm
    enabled: true
    joint_names:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    frames:
      base: base_link
      tip: tool0
    command_modes: [servo_j, servo_p]

  # 2. 声明夹爪/末端工具 (可选)
  - id: my_gripper
    kind: gripper
    enabled: true
    attached_to: my_arm
    joint_names: [gripper_joint]
    frames:
      base: tool0
      tip: finger_tip

teleop:
  position_scale: 0.85          # 移动缩放灵敏度
  clutch_threshold: 0.5         # 抓握侧键 (Grip) 激活门限 (0.0 ~ 1.0)
  clutch_controller: right      # 指定离合器控制手柄: 'left', 'right', 或 'binding' (各自独立)

  # 手柄与机械臂绑定
  bindings:
    - group: my_arm
      controller: right         # 用右手手柄控制该机械臂
      # PICO 坐标系到机械臂基座坐标系的 3x3 旋转矩阵转换
      # (PICO: +X 左, +Y 上, +Z 后; ROS: +X 前, +Y 左, +Z 上)
      axis_mapping:
        - [0.0, 0.0, 1.0]
        - [1.0, 0.0, 0.0]
        - [0.0, 1.0, 0.0]

  # 手柄与夹爪工具绑定
  gripper_input: trigger        # 使用扳机键 (Trigger) 连续模拟量开合
  tools:
    - group: my_gripper
      controller: right
      joint_names: [gripper_joint]
      open: [0.0]               # 扳机完全松开时的关节位置 (m 或 rad)
      closed: [0.08]            # 扳机完全扣紧时的关节位置 (m 或 rad)
```

### 步骤 3：选择 IK/FK 运动学后端
在 `profile.yaml` 的 `motion` 节点中选择默认后端：

```yaml
motion:
  backend_package: hc_motion_backend_robo_manip
  backend_executable: robo_manip_backend_node
  servo_nominal_rate_hz: 100.0
  robo_manip_servo_lease_ms: 100
  robo_manip_joint_max_velocity_rad_s: 0.6
  robo_manip_joint_max_acceleration_rad_s2: 2.4
  robo_manip_joint_max_jerk_rad_s3: 9.6

```

还需在 `resources.robo_manip_sdk` 指定机器人的 SDK YAML，配置关节组、模型和 RTC；
可参考 `src/hc_robot_x1/config/motion/robo_manip.yaml`。

### 步骤 4：运行与调试
启动仿真或真机进行验证：
```bash
# 1. 运行仿真模式验证运动学与映射
./run.sh profile:=<robot_id> mode:=sim

# 2. 显式选择当前运动后端
./run.sh profile:=<robot_id> mode:=sim motion_backend:=robo_manip

# 3. 打开 RViz 查看 3D 机械臂模型与位姿
./rviz.sh <robot_id>
```

---

## 三、工具与夹爪控制配置 (Gripper / Tools)

系统支持将手柄食指扳机键（Trigger，0.0~1.0 连续模拟量）线性映射到任意关节：

```yaml
teleop:
  tools:
    # 示例 1: 单自由度二指夹爪
    - group: left_gripper
      controller: left
      joint_names: [finger_joint]
      open: [0.0]
      closed: [0.05]

    # 示例 2: 联动多关节夹爪 (双指同步运动)
    - group: dual_finger_gripper
      controller: right
      joint_names: [left_finger_joint, right_finger_joint]
      open: [0.0, 0.0]
      closed: [0.04, 0.04]
```

---

## 四、完整示例：配置单臂 6 轴机械臂 (Single Arm Example)

若要接入一台名为 `ur5_single` 的单臂机器人，只需在 `src/hc_robot_ur5/config/profile.yaml` 编写如下极简内容：

```yaml
schema: hc-teleop-profile/v1
robot_id: ur5_single
display_name: UR5 Single Arm Robot

resources:
  urdf: package://hc_robot_ur5/urdf/ur5.urdf
  robo_manip_sdk: package://hc_robot_ur5/config/motion/robo_manip.yaml

components:
  - id: arm
    kind: arm
    enabled: true
    joint_names:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    frames:
      base: base_link
      tip: tool0

  - id: gripper
    kind: gripper
    enabled: true
    attached_to: arm
    joint_names: [robotiq_85_left_knuckle_joint]
    frames:
      base: tool0
      tip: robotiq_coupler

motion:
  backend_package: hc_motion_backend_robo_manip
  backend_executable: robo_manip_backend_node

teleop:
  position_scale: 0.8
  clutch_threshold: 0.5
  clutch_controller: right
  bindings:
    - group: arm
      controller: right
      axis_mapping:
        - [0.0, 0.0, 1.0]
        - [1.0, 0.0, 0.0]
        - [0.0, 1.0, 0.0]
  tools:
    - group: gripper
      controller: right
      joint_names: [robotiq_85_left_knuckle_joint]
      open: [0.0]
      closed: [0.8]
```

只需上述一个文件，无需修改任何代码，运行：
```bash
./run.sh profile:=ur5_single mode:=sim
```
系统即可自动完成 URDF 链条提取、手柄绑定与实时遥操作。

---

## 五、关键参数调优与常见问题

1. **机械臂移动反向或轴向不一致**：
   - 检查 `teleop.axis_mapping`（或每条臂独立的 `axis_mapping`）。它是手柄朝向与机器人基座朝向的对齐矩阵，旋转 90 度或取反即可调整对应的轴向运动方向。
2. **手柄按住 Grip 机械臂不动**：
   - 查看终端是否提示 `fresh measured FK is required before clutch engagement`。
   - 离合器需要机械臂的当前实测末端位姿（`state/cartesian`）作为锚点。RoboManip 后端通过实测关节 FK 计算并发布；如使用真机，需确保电机驱动正在向 `state/joints` 发布有效的关节角度。
3. **运动卡顿或超速保护跳闸**：
   - 先检查输入/求解延迟与反馈新鲜度，再核对 `motion.robo_manip_joint_max_velocity_rad_s`、`motion.robo_manip_joint_max_acceleration_rad_s2` 和 `motion.robo_manip_joint_max_jerk_rad_s3`。
