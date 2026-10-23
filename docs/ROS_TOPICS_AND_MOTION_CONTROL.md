# ROS 接口速查：相机、机械臂、夹爪 / X1 手

## 1. 运行环境

以下以 X1、ROS Domain 14 为例，在仓库根目录执行：

```bash
cd /home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic
source /opt/ros/humble/setup.bash
source "${HC_MOTION_INSTALL:-$PWD/install_motion}/setup.bash"  # MoveJ / MoveL 消息包
export ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0
```

机械臂 Move/Servo 需要机器人端用 `HC_CONTROL_BACKEND=motion HC_ROBOT_NAME=x1 ROS_DOMAIN_ID=14 ./run.sh teleop` 启动。在 Dashboard 选择 **Move / Servo → 恢复输出**，并确保硬件、相机驱动已启动。切换来源会停用 VR 输出，避免与程序的手部命令同时发送。构建说明见 [MOTION.md](MOTION.md)。

## 2. 获取相机和机械臂状态

| 数据 | Topic | 消息类型 |
| --- | --- | --- |
| 头部 RGB | `/observation/cameras/head/color/compressed` | `sensor_msgs/msg/CompressedImage` |
| 头部深度 | `/observation/cameras/head/depth/compressed` | `sensor_msgs/msg/CompressedImage` |
| 头部相机内参 | `/observation/cameras/head/calibration` | `std_msgs/msg/String`（JSON） |
| 头部相机外参 | `/observation/cameras/head/extrinsics` | `std_msgs/msg/String`（JSON） |
| 整机关节实际状态 | `/observation/joint_states` | `sensor_msgs/msg/JointState` |
| 左臂实际状态 | `/observation/left_arm/joint_states` | `sensor_msgs/msg/JointState` |
| 右臂实际状态 | `/observation/right_arm/joint_states` | `sensor_msgs/msg/JointState` |
| 左臂 TCP 实际位姿（Motion FK） | `/observation/motion/left_arm/ee_pose` | `geometry_msgs/msg/PoseStamped` |
| 右臂 TCP 实际位姿（Motion FK） | `/observation/motion/right_arm/ee_pose` | `geometry_msgs/msg/PoseStamped` |
| 手部实际状态（驱动可选提供） | `/observation/hands/joint_states` | `sensor_msgs/msg/JointState` |

关节按 `name` 取值，机械臂 `position` 单位为 **rad**。`/action/joint_states` 是控制目标，读取实际位置用上面的 observation 话题。

下面可保存为 `read_robot.py`，用 `/usr/bin/python3 read_robot.py` 运行；`images`、`joints` 和 `metadata` 分别保存最新图像、关节与内外参数据：

```python
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

rclpy.init()
node = Node('read_robot')
images, joints, metadata = {}, {}, {}
calibration_topic: str = '/observation/cameras/head/calibration'
extrinsics_topic: str = '/observation/cameras/head/extrinsics'
calibration_qos = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
extrinsics_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

def on_image(msg, key):
    images[key] = cv2.imdecode(
        np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)

def on_metadata(msg, key):
    metadata[key] = json.loads(msg.data)

subs = [
    node.create_subscription(CompressedImage,
        '/observation/cameras/head/color/compressed',
        lambda msg: on_image(msg, 'rgb'), qos_profile_sensor_data),
    node.create_subscription(CompressedImage,
        '/observation/cameras/head/depth/compressed',
        lambda msg: on_image(msg, 'depth'), qos_profile_sensor_data),
    node.create_subscription(JointState, '/observation/joint_states',
        lambda msg: joints.update(zip(msg.name, msg.position)), qos_profile_sensor_data),
    node.create_subscription(String, calibration_topic,
        lambda msg: on_metadata(msg, 'calibration'), calibration_qos),
    node.create_subscription(String, extrinsics_topic,
        lambda msg: on_metadata(msg, 'extrinsics'), extrinsics_qos),
]
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
```

当前 X1 的 RGB 解码后是 BGR；深度为 `uint16` 毫米，`1000 ≈ 1 m`，`0` 为无效值。实际使用时检查 `msg.header.stamp`，缓存值不会因断流自动清空。

两条内外参话题沿用旧 `/io_teleop/camera_head/{calibration,extrinsics}` 的 JSON 格式，客户端修改上面的 `calibration_topic`、`extrinsics_topic` 即可。

| 数据 | JSON schema | 主要字段 |
| --- | --- | --- |
| 内参 | `hc_camera_calibration/v1` | `camera=eye`；`resolution.color/depth` 的宽高；`rgb/depth` 中的 `fx/fy/cx/cy`、`distortion_model`、`distortion_coefficients` |
| 外参 | `hc_robot_camera_extrinsics/v1` | `common_frame=zhi_Link`；`transforms[]` 中的 `parent_frame`、`child_frame`、`translation_m`、`rotation_xyzw` |

内参来自实际 `CameraInfo`。例如读取 `metadata['calibration']['rgb']['fx']` 获取 RGB 的焦距；只收到 RGB 时没有 `depth` 字段。本地生成的内参还保留各流的 `frame_id`、`camera_info_topic` 和原始 `stamp`。

X1 外参为 `zhi_Link → camera_head`：平移单位 **m**，四元数顺序 **x/y/z/w**，坐标换算为 `p_zhi = R × p_camera_head + t`。优先读取 TF（`source=tf`）；TF 不可用时由当前实测关节和 URDF 计算（`source=urdf_joint_states`），顶层 `stamp` 保留来源时间。头部运动时外参会更新。URDF 计算结果表示模型中的相机安装位姿，不能直接当作实测手眼标定或驱动 optical frame 的外参。

启用 `middleware/config.yaml` 中的 `ros.dataset_topics.enabled` 后，X1 默认提供这两条话题；更新代码后需重启中间件。本地生成频率为 **2 Hz**，均为 Reliable；内参还使用 Transient Local 保留最近一条。旧驱动仍在发布内外参时优先原样转发。缺少有效来源时不生成对应消息；关节反馈默认 **1 秒**超时，过期后停止用它计算外参。上面示例的缓存不会自动清空，使用前应检查内参各流或外参顶层的 `stamp`，也不能把各话题的最新值当作与图像同步的一帧。

内参来源配置在 [X1 arm_teleop.yaml](../adapters/robots/x1/arm_teleop.yaml) 的 `camera_metadata.camera_info_topics`：默认订阅 `/observation/cameras/head/{color,depth}/camera_info`，兼容 `/cameras/head/{color,depth}/camera_info`；驱动名称不同则填写实际话题。没有相机或 CameraInfo 时不会生成内参。

```bash
# ROS_DOMAIN_ID 必须与运行端一致；仿真使用 Dashboard 显示的实际域。
ros2 topic echo /observation/cameras/head/calibration --once --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /observation/cameras/head/extrinsics --once --qos-reliability reliable
```

完整字段与来源约定见[相机内外参接口](dataset_topics.md)。X1 网页录制默认已包含这两条话题。

## 3. 控制机械臂

| 控制方式 | 左臂接口 | 右臂接口 | 类型 |
| --- | --- | --- | --- |
| MoveJ：一次关节运动 | `/control/motion/left_arm/move_j` | `/control/motion/right_arm/move_j` | Action：`humanoid_motion_interfaces/action/MoveJ` |
| MoveL：一次末端直线运动 | `/control/motion/left_arm/move_l` | `/control/motion/right_arm/move_l` | Action：`humanoid_motion_interfaces/action/MoveL` |
| ServoJ：连续关节目标 | `/control/motion/left_arm/servo_j` | `/control/motion/right_arm/servo_j` | Topic：`sensor_msgs/msg/JointState` |
| ServoP：连续末端位姿目标 | `/control/motion/left_arm/servo_p` | `/control/motion/right_arm/servo_p` | Topic：`geometry_msgs/msg/PoseStamped` |

Motion 模式的关节名：`left_joint_1..7`、`right_joint_1..7`；默认 v23 反馈仍是 `Joint1_L..7_L`、`Joint1_R..7_R`。位姿位置用 **m**，姿态用四元数 **x/y/z/w**；X1 的 `frame_id` 是 `zhi_Link`。

直接使用现有 [robot_api_demo.py](../examples/robot_api_demo.py)，目标都从当前反馈计算：

```bash
# MoveJ：左臂第 4 轴增加 0.02 rad
/usr/bin/python3 examples/robot_api_demo.py move-j --group left_arm --joint left_joint_4 --delta 0.02 --scale 0.1

# ServoJ：同样的关节目标，以 50 Hz 持续发送 2 秒
/usr/bin/python3 examples/robot_api_demo.py servo-j --group left_arm --joint left_joint_4 --delta 0.02 --hz 50 --duration 2

# ServoP：TCP 沿基坐标 X 增加 5 mm，保持姿态，持续 2 秒
/usr/bin/python3 examples/robot_api_demo.py servo-p --group left_arm --dx 0.005 --hz 50 --duration 2
```

自己发 Servo 时，每帧更新 `header.stamp`，建议 `50 Hz`；当前目标超过 `100 ms` 不刷新会失效。ServoJ 填绝对关节角，ServoP 填绝对 TCP 位姿。

MoveL 构造示例：放入自己的 ROS 节点；`node` 是节点，`fresh_fk` 是刚从左臂 Motion FK 话题收到的新鲜 `PoseStamped`。

```python
import copy
from rclpy.action import ActionClient
from humanoid_motion_interfaces.action import MoveL

client = ActionClient(node, MoveL, '/control/motion/left_arm/move_l')
if not client.wait_for_server(timeout_sec=5.0):
    raise RuntimeError('MoveL 服务未启动')
goal = MoveL.Goal()
goal.group_name = 'left_arm'
goal.tip_frame = 'io_teleop_ee_link_left'
goal.target_pose = copy.deepcopy(fresh_fk)
goal.target_pose.pose.position.x += 0.005
goal.target_pose.header.stamp = node.get_clock().now().to_msg()
goal.options.velocity_scale = 0.1
goal.options.acceleration_scale = 0.1
goal.options.jerk_scale = 0.1
goal.options.timeout_sec = 30.0
future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
handle = future.result()
if not handle.accepted:
    raise RuntimeError('MoveL 目标被拒绝')
result = handle.get_result_async()
rclpy.spin_until_future_complete(node, result)
print(result.result().status, result.result().result.status)
```

Action 成功需 ROS 状态为 `4 / SUCCEEDED`，且业务 `status.code == 0`。右臂同时改话题、`group_name` 和 `tip_frame=io_teleop_ee_link_right`。

## 4. 控制夹爪 / X1 手：0 张开，1 闭合

| 部位 | Topic | 消息类型 |
| --- | --- | --- |
| 左夹爪 / 左手 | `/action/left_hand/joint_states` | `sensor_msgs/msg/JointState` |
| 右夹爪 / 右手 | `/action/right_hand/joint_states` | `sensor_msgs/msg/JointState` |

**调用时只传 0 或 1。** 以下函数按机型配置生成实际消息：X1 展开为十个手指角度并附带 `L_ban/R_ban` 的 0/1 标记；OpenArmX 转为主指位置，`0.044 m` 张开、`0 m` 闭合。无需自己填写各手指角度。

保存为 `control_hand.py`，在仓库根目录用 `/usr/bin/python3 control_hand.py` 运行；默认右手闭合，按 Ctrl+C 停止发送：

```python
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

profile = 'x1'  # OpenArmX 改成 'openarmx'
cfg = yaml.safe_load(Path(
    f'adapters/robots/{profile}/arm_teleop.yaml').read_text())['grippers']
rclpy.init()
node = Node('control_hand')
pubs = {side: node.create_publisher(JointState, cfg[side]['finger_topic'], 10)
        for side in ('left', 'right')}

def set_hand(side, closed):
    if closed not in (0, 1):
        raise ValueError('0 张开，1 闭合')
    hand = cfg[side]
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = list(hand['finger_joint_names'])
    msg.position = [float(q) for q in hand['finger_closed' if closed else 'finger_open']]
    if hand.get('binary_state_joint'):
        msg.name.append(hand['binary_state_joint'])
        msg.position.append(float(closed))
    pubs[side].publish(msg)

timer = node.create_timer(0.05, lambda: set_hand('right', 1))  # 20 Hz；改为 ('left', 0) 即左手张开
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
```

这是普通程序控制示例；DAgger 会话按 [DAGGER.md](DAGGER.md) 管理控制权。手部必须有实际驱动订阅，停发后由驱动处理保持；反馈读第 2 节的话题，OpenArmX 夹爪反馈也可在整机关节状态中。

快速检查：`ros2 topic info /action/right_hand/joint_states -v` 查看实际订阅节点；`ros2 action list -t` 查看 MoveJ/MoveL 是否启动。
