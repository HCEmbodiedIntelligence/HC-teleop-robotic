# Observation / action 标准接口

VLA 接入的精简清单、Python 订阅/MCAP 录制 demo 及 LeRobot 转换命令见 [main 通用话题与 VLA 采集](VLA_MAIN_TOPICS.md)。

公共控制接口已迁移至 /action、/observation、/control、/status。中间件 `ros.dataset_topics.enabled: true` 时同时发布以下分组投影。整机原子命令为 `/action/joint_states`，反馈为 `/observation/joint_states`；硬件不订阅分组投影，避免重复执行。
需一起重启中间件、控制节点及对应仿真或硬件适配节点，旧进程不会自动更换话题。网页自定义话题录制规则中默认已选中这些话题以及 `/vrdata`、`/clock`，开始录制后写入同一个 MCAP。

| 分类话题 | 类型 | 来源 |
| --- | --- | --- |
| `/observation/left_arm/joint_states` | JointState | `/observation/joint_states` 左臂关节 |
| `/observation/right_arm/joint_states` | JointState | 同上，右臂关节 |
| `/observation/waist/joint_states` | JointState | 同上，leg_1、leg_2、zhi |
| `/action/left_arm/joint_states` | JointState | `/action/joint_states` 左臂最终命令 |
| `/action/right_arm/joint_states` | JointState | 同上，右臂最终命令 |
| `/action/waist/joint_states` | JointState | 同上，腰部最终命令 |
| `/observation/left_arm/ee_pose`、`/observation/right_arm/ee_pose` | PoseStamped | `/observation/ee_poses` |
| `/action/left_arm/ee_pose`、`/action/right_arm/ee_pose` | PoseStamped | `/action/ee_poses` |
| `/observation/base/pose` | PoseStamped | `/observation/base/pose` |
| `/action/base/velocity` | TwistStamped | `/action/base/command` |
| `/observation/headset/pose` | PoseStamped | `/vrdata` 的 poses.head |

关节按配置的名称顺序抽取，源消息缺少任一组内关节时不发布该组。
position/velocity/effort 中源消息未提供的数组保持为空，不能将空数组解释为零。
关节和末端消息保留源 header；双臂末端顺序源为右、左，分类后不再依赖数组下标。
X1 末端目标与实际话题均使用 zhi_Link；actual 是控制节点本地模型计算结果，不是独立末端传感器。
肩部求解器坐标系的 `/control/solver/ee_targets` 仍可从原始录制读取，不能直接与 zhi_Link 位姿比较。

底盘 action 使用 linear.x=前向速度（m/s）、linear.y=横向速度（m/s）、angular.z=偏航角速度（rad/s）；frame_id=base_command 表示源命令的坐标约定，不额外旋转。
底盘 observation 保留源 world 坐标系及位置/朝向。
头显 observation 保留 VR 原始坐标，不转成 ROS world；frame_id=vr_tracking，tracking.head=false 时不发布。
头显是操作者观测输入，实际机器人头部位姿不能用此话题替代。

无 header 的底盘命令、VR 消息使用中间件 ROS 时钟的接收时间作为派生 header。
原始 `/vrdata` 同时保存，保留 vr_timestamp、sequence、tracking 和双手输入。
各源时钟不一定相同，分类转发不做时钟同步或重采样。
MCAP 索引使用接收系统时间；仿真时间由 `/clock` 单独保留。

本次提供原始分类序列，不生成运动时长/距离累计话题。
离线可从底盘 observation 计算世界平面位移、累计路程、转角和运动时段；应按录制段初始化，并处理场景重置/跳变。
速度命令积分只能代表指令位移，不能当成机器人真实路程。

关节分组自动从当前机器人 Profile 的 arm_teleop.yaml 读取 arms.*.joint_names 和 body.waist_joint_names，无腰部的机器人不发布腰部分组。已迁移 X1、OpenArmX、Baxter、FR3 Duo、G1 Dual Arm 配置。

公共接口中 `/action/base/command` 仍是 Float64MultiArray，顺序 `[yaw, forward, lateral]`，用于驱动；`/action/base/velocity` 是便于分析的 TwistStamped，不是第二条驱动输入。
就绪 `/status/hardware/ready`、急停 `/control/emergency_stop`、控制源 `/control/source`、遥操作状态 `/status/teleop`、求解器复位 `/control/solver/reset`。
HC_X1 内部仍使用 `/io_teleop/*` 与设备驱动通信；Isaac 和 PyBullet 启动映射已同步。原生相机输入 `/cameras/*` 和 ROS 的 `/tf`、`/clock` 不变。
旧 MCAP 文件本身不改写；旧接口录制文件如需驱动回放，应明确设置旧话题到新接口的映射。
