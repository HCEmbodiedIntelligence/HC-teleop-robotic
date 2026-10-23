# main 通用话题与 VLA 采集

适用于当前 `main` 的 `hc-teleop-hardware-interface/v2`。采集端订阅机器人反馈、选中的动作目标及相机，保存为 MCAP；训练时再对齐为 `observation.state / action / observation.images.*`。本页不依赖 Motion、MoveJ、ServoJ、ServoP。

## 订阅与录制清单

表中所有话题都是**采集端订阅**。可选设备未接入时可以没有数据；频率为契约目标，不代表现场实测值。

| 用途 | 话题 | ROS 2 类型 | 说明 |
| --- | --- | --- | --- |
| 状态，必录 | `/observation/joint_states` | `sensor_msgs/msg/JointState` | 实测关节；推荐 100 Hz，契约最低 50 Hz |
| 动作，必录 | `/action/joint_states` | `sensor_msgs/msg/JointState` | 选中的绝对关节位置目标，目标 100 Hz；不是执行成功确认 |
| RGB，VLA 按实际相机必录 | `/observation/cameras/{id}/color/compressed` | `sensor_msgs/msg/CompressedImage` | 目标 30 Hz；`id` 可为 `head/overhead/d405_left/d405_right` |
| 深度，可选 | `/observation/cameras/{id}/depth/compressed` | `sensor_msgs/msg/CompressedImage` | 契约使用 PNG；按实际相机选择，不作为普通 RGB 视频 |
| 手/夹爪反馈，可选 | `/observation/hands/joint_states` | `sensor_msgs/msg/JointState` | 目标 10 Hz；部分机型的夹爪反馈包含在整机关节状态中 |
| 左右手/夹爪动作，可选 | `/action/left_hand/joint_states`、`/action/right_hand/joint_states` | `sensor_msgs/msg/JointState` | 目标 10 Hz；名称、开合单位和范围以机型配置为准 |
| 底盘动作，可选 | `/action/base/command` | `std_msgs/msg/Float64MultiArray` | 目标 60 Hz；速度模式为 `[yaw, forward, lateral]`，单位 `[rad/s, m/s, m/s]` |
| 底盘位置，可选 | `/observation/base/pose` | `geometry_msgs/msg/PoseStamped` | 仿真/有定位的硬件提供；当前 X1 真机不保证提供 |
| 就绪 | `/status/hardware/ready` | `std_msgs/msg/Bool` | 硬件就绪心跳 |
| 控制来源 | `/control/source` | `std_msgs/msg/String` | 当前 main 为 `vr` 或 `exoskeleton` |
| 软件停止 | `/control/emergency_stop` | `std_msgs/msg/Bool` | `true` 表示停止事件，用于标记采集片段 |

启用 `ros.dataset_topics.enabled` 后，还可录制以下派生数据。整机与分组数据选一种作为训练来源，避免重复拼接：

| 话题 | 类型与语义 |
| --- | --- |
| `/{observation,action}/{left_arm,right_arm,waist}/joint_states` | `JointState`；从整机消息按配置抽取，腰部只在机型支持时发布 |
| `/{observation,action}/{left_arm,right_arm}/ee_pose` | `PoseStamped`；反馈是本地模型计算，目标来自 VR 位姿链路，不能当作所有控制源的统一末端动作 |
| `/action/base/velocity` | `TwistStamped`；上述底盘命令的派生表示，`linear.x/y` 为前向/横向速度，`angular.z` 为偏航速度 |

分组动作、末端位姿和 `/action/base/velocity` 用于观测/录制，不是第二套驱动输入。X1 双臂末端使用 `zhi_Link`；其他机型按实际 `header.frame_id` 解释。

## 字段与训练约定

- `JointState.name[i]` 对应 `position[i]`；名称与当前机型 URDF 一致，例如 X1 的 `Joint1_L`、OpenArmX 的 `openarmx_left_joint1`，不是统一的 `left_joint_1`。旋转关节位置/速度/力矩用 rad、rad/s、N·m；直线关节用 m、m/s、N。
- `velocity/effort` 可以为空，表示未提供。状态和动作可能包含不同关节集合；分别固定名称顺序，不用反馈值补造动作，也不假定维度一定是 14。
- 保存原始 `header.stamp`、接收时间、任务文本及当前机型的 URDF/配置。不同话题异步到达；同一次打印的最新值不是同步训练帧。训练时依据明确时间轴对齐，处理过期/缺失值及停止片段。
- 当前默认 LeRobot 导出将总反馈映射为 `observation.state`，总命令映射为 `action`，RGB 映射为图像特征。**独立手/夹爪动作话题不会自动并入主 action**：若总命令没有夹爪，需在训练预处理中按名称、单位和时间显式合并；本 demo 会保留这些原始消息。
- 状态/图像订阅用 Best Effort、Volatile；`/control/source` 用 Reliable、Transient Local，才能读取加入前锁存的来源。

## Python demo

[vla_topics_demo.py](../tools/examples/vla_topics_demo.py) 默认订阅总状态、总动作、手/夹爪、ready、控制来源及停止事件；每秒打印消息数量、原始字段和接收年龄。加 `--output` 保存标准 ROS 2 CDR MCAP，完整保留消息，不重采样、不发布控制指令。

```bash
cd /home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=14        # 必须与运行端一致；仿真填网页显示的实际域
export ROS_LOCALHOST_ONLY=0    # 跨机器；同机隔离仿真设为 1

# 先观察状态/动作；Ctrl+C 退出
/usr/bin/python3 tools/examples/vla_topics_demo.py

# 录制一段 30 秒数据；相机名称替换为实际存在的话题
/usr/bin/python3 tools/examples/vla_topics_demo.py \
  --output runtime/vla_demo/episode_001.mcap --duration 30 \
  --task '把物体放入盒子' --robot-type x1 \
  --camera /observation/cameras/head/color/compressed
```

多相机重复添加 `--camera`。使用 `--topic TOPIC TYPE` 增加其他源，例如：

```bash
# 将需要的选项加到上面的录制命令末尾
--topic /action/base/velocity geometry_msgs/msg/TwistStamped
--topic /observation/base/pose geometry_msgs/msg/PoseStamped
--topic /observation/left_arm/ee_pose geometry_msgs/msg/PoseStamped
--topic /clock rosgraph_msgs/msg/Clock
# 未压缩 RGB 相机示例：
--topic /camera/color/image_raw sensor_msgs/msg/Image
```

用 `ros2 topic list -t` 确认现场名称与类型。脚本复用项目 `.deps` 中的 `mcap` 和消息定义解析器；默认只读显示无需 mcap。文件不覆盖已有录制；Ctrl+C 正常封存。状态、动作或所选相机整段无消息时，录制退出码为 2，并列出缺失源，已收到的数据仍保留。消息数量大于零也不代表时序完整或已通过训练数据质检。

## 转为训练数据

已有网页录制也可直接勾选上述话题并选择 MCAP / LeRobot / 两者，同时填写任务文本。demo 的 MCAP 可用当前仓库导出器转换；先完成 README 中的 `./tools/setup_lerobot.sh` 环境安装：

```bash
export PYTHONPATH="$PWD/.deps${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m middleware.core.lerobot_export \
  runtime/vla_demo/episode_001.mcap runtime/vla_demo/episode_001_lerobot \
  --fps 15 --task '把物体放入盒子' --robot-type x1
```

这里生成的是按接收时间对齐的数据。训练时使用 `observation.source_available/source_valid/source_age`，名称顺序与来源见 `meta/teleop_sync.json`；数值默认有效期 0.2 秒、图像 0.25 秒。深度和独立手部原始消息保留在 MCAP，不会自动成为主动作或 RGB 特征。

接口来源：[硬件契约](../interfaces/hardware_interface_v2.yaml)、[分组话题](dataset_topics.md)、[LeRobot 导出实现](../middleware/core/lerobot_export.py)。
