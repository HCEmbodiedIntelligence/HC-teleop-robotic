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
| 整机关节实际状态 | `/observation/joint_states` | `sensor_msgs/msg/JointState` |
| 左臂实际状态 | `/observation/left_arm/joint_states` | `sensor_msgs/msg/JointState` |
| 右臂实际状态 | `/observation/right_arm/joint_states` | `sensor_msgs/msg/JointState` |
| 左臂 TCP 实际位姿（Motion FK） | `/observation/motion/left_arm/ee_pose` | `geometry_msgs/msg/PoseStamped` |
| 右臂 TCP 实际位姿（Motion FK） | `/observation/motion/right_arm/ee_pose` | `geometry_msgs/msg/PoseStamped` |
| 手部实际状态（驱动可选提供） | `/observation/hands/joint_states` | `sensor_msgs/msg/JointState` |

关节按 `name` 取值，机械臂 `position` 单位为 **rad**。`/action/joint_states` 是控制目标，读取实际位置用上面的 observation 话题。

下面可保存为 `read_robot.py`，用 `/usr/bin/python3 read_robot.py` 运行；`images` 和 `joints` 保存最新数据：

```python
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState

rclpy.init()
node = Node('read_robot')
images, joints = {}, {}

def on_image(msg, key):
    images[key] = cv2.imdecode(
        np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)

subs = [
    node.create_subscription(CompressedImage,
        '/observation/cameras/head/color/compressed',
        lambda msg: on_image(msg, 'rgb'), qos_profile_sensor_data),
    node.create_subscription(CompressedImage,
        '/observation/cameras/head/depth/compressed',
        lambda msg: on_image(msg, 'depth'), qos_profile_sensor_data),
    node.create_subscription(JointState, '/observation/joint_states',
        lambda msg: joints.update(zip(msg.name, msg.position)), qos_profile_sensor_data),
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
