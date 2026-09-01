# 硬件适配项目接入指南

`HC-teleop-robotic` 不包含任何机型厂商 SDK，也不直接访问串口、Redis、CAN、EtherCAT 或相机。每个机型建立独立项目（例如 `HC_X1`），并只通过 `interfaces/hardware_interface_v1.yaml` 与本系统通信。

## 1. 项目职责

硬件项目负责启动相机、机械臂、腰部、底盘和灵巧手驱动；把厂商私有状态规范化为 `/hc_teleop/*`；把标准命令转换成厂商命令；实现固定频率、限速/限加速度、超时保持、单发布者检查和硬件急停。

通用项目负责 Dashboard、PICO/VR 数据、机器人 ZIP 导入、Pinocchio 逆解、遥操作逻辑、标准命令生成和 MCAP 录制。通用项目不得 source 硬件工作空间，也不得引用厂商 SDK 路径。

## 2. 标准 ROS 2 契约

完整机器可读定义见 `interfaces/hardware_interface_v1.yaml`。最低接入要求只有两条：

- 硬件以至少 50 Hz 发布 `sensor_msgs/msg/JointState` 到 `/hc_teleop/joint_states`；推荐 100 Hz。
- 硬件订阅 `/hc_teleop/joint_cmd`，按消息中的关节名映射到驱动；收到命令超过 150 ms 后必须停止继续下发旧目标并安全保持。

所有关节名必须与导入 URDF 完全一致。不要依赖数组固定顺序；必须按 `JointState.name` 映射。底盘命令为 `[yaw, forward, lateral]`。相机供录制时优先发布压缩流，深度使用 PNG 压缩的 `CompressedImage`，避免同时发布/录制同源别名和未压缩图像。

ROS Domain 默认是 14。硬件项目和通用项目必须设置相同的 `ROS_DOMAIN_ID`，跨机器还需 `ROS_LOCALHOST_ONLY=0`。

## 3. 新机型实施步骤

1. 新建独立仓库 `HC_<MODEL>`，将厂商驱动、消息包、SDK、udev 规则和系统服务要求全部放入该仓库。
2. 驱动可以保留内部私有命名空间；新增一个 `standard_adapter` 节点完成私有话题与 `/hc_teleop` 的转换。
3. 为关节命令建立独立固定频率执行节点。逆解回调、相机回调和话题桥不能直接向伺服驱动发布。
4. 加入单发布者检查、反馈同步、输入超时、速度/加速度限制以及真实硬件急停联锁。
5. 准备机器人 ZIP：`profile.yaml`、URDF、全部 mesh、`controller_v23.yml` 和 `arm_teleop.yaml`。在 Dashboard 导入并应用。
6. 分别启动硬件项目和通用项目，并按下面的验收项检查。

## 4. 启动与验收

```bash
# 终端 A：机型硬件项目
cd ~/HC_<MODEL>
ROS_DOMAIN_ID=14 ./start.sh

# 终端 B：通用中间件、逆解与控制
cd ~/HC-teleop-robotic
ROS_DOMAIN_ID=14 ./run.sh teleop
```

先确认 `ros2 topic hz /hc_teleop/joint_states` 稳定，再连接头显。空闲时 `/hc_teleop/joint_cmd` 不应被多个控制器发布；操作时频率应接近 100 Hz。拔掉 VR 数据或松开离合后，硬件命令必须在约定超时内停止。逐项检查关节方向、零位、限位、回零、底盘轴向、灵巧手开合和相机帧率，最后才进行大范围运动。

## 5. 安全要求

软件碰撞检测和逆解限位不能代替硬件限位及急停。硬件适配项目是最后一道执行边界：即使上游发出非法、NaN、超限、过期或关节名不完整的消息，也必须拒绝。启动测试时使用低速度、留出工作空间，并保持实体急停可触达。
