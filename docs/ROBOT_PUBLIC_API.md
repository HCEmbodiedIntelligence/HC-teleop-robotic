# 第三方机械臂控制与订阅接口

显式启用 `HC_CONTROL_BACKEND=motion` 时，本仓库提供 `hc-robot-api/v1`。默认 v23 模式沿用原生名称与接口，不发布统一 API 能力。OpenArmX、X1 的统一模式均使用 Motion；算法客户端在线获取本机能力，无需机型参数或厂商 SDK。机械臂名称为 `left_joint_1…N`、`right_joint_1…N`，数量和限位由 profile 决定。专用六轴测试模型验证了动态 DOF。

humanoid 分支的历史软件验收使用实际 Motion/IK/最终 RTC、HC 中间件、实际适配器和假硬件闭环。详细结论、失败及复测日志见工作区 [本次验收报告](../../../reports/unified-robot-api-20261015-165101/RESULT.md)。没有连接真实电机或 CAN；没有运行 Isaac 图形仿真、跨电脑 DDS 或外部 X1 真机驱动。

该分支历史上另使用 PyBullet 仿真和两种机型原有 URDF，分别通过实际 `RecordingService` 录制公共关节状态、动作及 FK，并用 `UploadService` 上传至本机平台的独立仿真任务。对象存储读回校验一致；此次没有相机流。MCAP 原始文件接收已验证，Episode/质检未完成，不等同于训练数据就绪。

## 在线能力与公共端点

先订阅 `/status/robot/info`：`std_msgs/msg/String`，内容为版本化 JSON，QoS 为 **reliable、transient-local、depth 1**。晚加入订阅者也能取得当前启动时的描述。

描述含 `schema`、`profile_id`、`revision`、`groups`、全体 `joints`、`poses`、`endpoints`、`servo`。每组给出公共名称、数量、单位、限位、支持命令及端点、实际 base/tip frame 和 FK 话题。`whole_body` 是组合控制组，没有独立分组状态投影，`joint_states` 为 null。未支持的命令不应由客户端假定存在。

| 用途 | 端点 | ROS 类型与含义 |
| --- | --- | --- |
| 实测关节状态 | `/observation/joint_states` | `sensor_msgs/msg/JointState`；公共名称，不依赖 VR 或控制源 |
| 分组状态 | `/observation/{group}/joint_states` | 同上；按 profile 顺序，从总反馈投影 |
| 实测模型 FK | `/observation/motion/{group}/ee_pose` | `geometry_msgs/msg/PoseStamped`；有效关节反馈经本机模型 FK 计算 |
| 连续关节目标 | `/control/motion/{group}/servo_j` | `sensor_msgs/msg/JointState`；公共名称 |
| 连续末端目标 | `/control/motion/{group}/servo_p` | `geometry_msgs/msg/PoseStamped`；本机 frame |
| 回位／固定关节位置 | `/control/motion/{group}/move_j` | `humanoid_motion_interfaces/action/MoveJ` |
| 选中的最终关节目标 | `/action/joint_states` | `sensor_msgs/msg/JointState`；公共名称；不是实测值或硬件执行确认 |
| 能力描述 | `/status/robot/info` | `std_msgs/msg/String`；配置描述，不代表 ready 或许可 |
| 硬件就绪心跳 | `/status/hardware/ready` | `std_msgs/msg/Bool`；失效／断流撤销输出 |
| 当前来源 | `/control/source` | `std_msgs/msg/String`；reliable、transient-local |
| 显式硬件使能事件 | `/control/hardware/enabled` | `std_msgs/msg/Bool`；reliable、volatile，不周期重发 true |

`{group}` 从在线描述选择；双臂组为 `left_arm`、`right_arm`。现有 MoveL/MoveP 端点仍在能力表内声明，输入和 Action 反馈／结果同样经过公共边界及来源检查，本次完整闭环验收重点为 ServoJ、ServoP、MoveJ。

旋转关节使用 rad、rad/s、N·m；直线关节使用 m、m/s、N。额外关节以描述中的名称和单位为准。OpenArmX 描述包括四个夹爪滑动关节；X1 包括腰部、头部、仿真夹爪及轮关节。描述模型可映射的关节不等于硬件持续提供所有测量：当前 X1 通用驱动配置管理双臂、腰部、头部共 19 轴，独立灵巧手沿用自己的话题。上游提供的已声明辅助关节会保留，不补造缺失测量。

`velocity`、`effort` 未提供时保持为空。Motion 控制需要完整且新鲜的组内位置和速度反馈。总状态及分组投影保留源时间戳，读取时按 `name` 匹配，不能依赖某次硬件数组顺序。`/{observation,action}/{group}/ee_pose` 是既有 VR PoseArray 数据集投影；实际模型 TCP 请使用 `/observation/motion/{group}/ee_pose`。

OpenArmX 的 base frame 是 `openarmx_body_link0`，X1 是 `zhi_Link`。保留各机型坐标约定，不保证同一个 ServoP 数值在不同机器人上到达同一物理位置。目标 frame 必须是本机已知 frame，并可转换到该组 base；未知 frame、非有限值、零四元数会拒绝。

## Servo 和 Move 的执行语义

Servo 输入采用 best-effort、volatile、keep-last 1；可靠发布者也可与该订阅匹配。每条目标应使用当前同步 ROS 时钟的非零时间戳。服务端依据描述中的 `input_stamp_max_age_s`、未来容差、反馈时效、许可代际及 Servo lease 拒绝陈旧输入。

当前 profile 控制频率为 100 Hz、Servo lease 为 100 ms；demo 默认连续发送 50 Hz。停止发送后 lease 到期，Motion 停止产生该 Servo 的目标，硬件驱动按原有 watchdog 保持。停止订阅或仅发布一次目标均不是持续 Servo 的替代。

ServoJ 与 MoveJ 允许组内具名子集，由服务端用同一份新鲜实测反馈补齐本组其余轴；demo 默认发送完整组。重复名、未知名、跨组、数组长度错误、NaN/Inf、越限目标拒绝。命令不静默截断到限位。SDK 计算结果只允许机器浮点舍入量级的边界误差，物理越限仍拒绝。

ServoP 经实际 SDK 位姿处理／逆解，随后与 ServoJ、MoveJ 共享最终关节 RTC。不可解或计算超时会报告原因并停止该次执行；连续输入可能重试，客户端应监控 FK 误差及新鲜度，不能把发布成功当作到达。MoveJ 成功需同时满足 ROS Action 成功终态和业务 `status.code == 0`；feedback/result 的关节名均为公共名。

## 名称和单位转换责任

| 边界 | 名称／单位 | 责任 |
| --- | --- | --- |
| 公共 JointState、Action | `left_joint_1…N` 等；公共 SI | `robot_api.yaml` 描述；Python/C++ 消费同一解析产物 |
| 内部模型与最终 RTC | URDF 原生名；rad、m | 公共映射只在公共／模型边界执行 |
| SDK API | SDK 分组；degree、mm 等 SDK 单位 | 既有 `SdkMotionBackend` 执行单位转换；生成的 SDK 限位从 Motion SI 派生 |
| 厂商／控制器接口 | 厂商名称和单位 | 驱动 `vendor_to_logical_scales/offsets` 或既有硬件适配器转换 |

公共到模型约定 `q_m = s*q_p + b`、`v_m = s*v_p`、`tau_m = tau_p/s`；反向为 `q_p = (q_m-b)/s`、`v_p = v_m/s`、`tau_p = s*tau_m`。反算位置区间后重新排序；速度／加速度／jerk 幅值除以 `abs(s)`。旋转偏置使用 `offset_rad`，直线偏置使用 `offset_m`。公共映射不重复执行厂商或 SDK 单位换算。

## 数据流、来源与安全门

```text
公共 ServoJ/MoveJ ─公共名转模型─┐
公共 ServoP ─程序来源检查──────┼─ 同一 Motion / IK / 最终 RTC
VR 映射位姿 ─独立 VR 来源检查─┘
                       ↓ 模型转公共并按来源路由
 /control/motion/joint_targets 或 /control/vr/joint_states
                       ↓ 唯一 HC 选择器
                 /action/joint_states
                       ↓ 公共转模型、反馈／限位／启用的碰撞检查
           /internal/robot/joint_commands → 机型驱动

驱动 → /internal/robot/joint_states → Motion / 遥操作模型
                                  → 公共转换 → /observation/joint_states
```

| 内部通道 | 表示与消费者 |
| --- | --- |
| `/internal/robot/joint_states` | URDF 模型名；驱动到 Motion、遥操作及公共转换 |
| `/internal/robot/joint_commands` | URDF 模型名；公共硬件边界到驱动 |
| `/internal/vr/{group}/servo_p` | 映射后的本机位姿；仅 VR 来源许可 |
| `/internal/vr/{group}/move_j` | 公共 JointState 目标；VR home 的独立来源通道 |
| `/internal/motion/vr_joint_targets` | 已经共同 RTC 处理的公共关节目标；VR 桥转发 |
| `/internal/vr/aux_joint_targets` | 仅辅助关节的模型名目标；VR 桥转成公共名，不再覆盖机械臂 |
| `/internal/vr/home`、`home_cancel`、`home_result` | VR home 请求、取消和双臂动作结果 |

公共程序 Servo/Move 只在 `source=motion` 时允许，VR 内部输入只在 `source=vr` 时允许。共享计算不改变来源隔离。VR 断流在 motion 来源下只撤销 VR 输入；显式停止、硬件故障仍影响所有来源。

`/control/teleop/enabled` 只管理 VR 输入。硬件执行由 `/control/hardware/enabled` 管理，恢复需要新显式使能和新目标。切源、停止、反馈／ready 失效和需复位的回放结束撤销旧许可；恢复反馈不自动恢复输出。内部 Header 许可 `/control/motion/permit` 包含 `motion:epoch` 或 `vr:epoch`，用于服务端执行门，客户端只读检查。

程序 home 和 VR home 都提交普通 MoveJ 执行。VR 使用独立内部授权入口，公共 MoveJ 在 VR 来源下仍拒绝。Move 占用相应物理关节，期间 Servo 不能插队；完成／取消后旧 Servo 不续播。VR 需新的输入及离合重新锚定。Home 不隐式使能；腰部／头部回位和手指控制保留既有辅助逻辑。

启用的碰撞检查位于所有选中来源共用的模型命令边界，状态见 `/status/robot/collision`。不改变各真实 profile 的原有碰撞开关；专门几何测试模型验证了程序和 VR 两路拦截。禁止第二个发布者直接向 `/action/joint_states` 或厂商控制器争抢命令所有权。

## Profile 选择、迁移与启动

统一解析器为 `middleware/core/robot_api.py`。`robot_api.yaml` 引用 `arm_teleop.yaml` 的 arms/body 有序分组和 `robot.initial_joints`，结合 URDF 及 Motion 限位解析；不维护第二套模型或 home。Motion/SDK 分组必须一致，不支持的配置在启动前拒绝。

导出的完整 profile ZIP 保留 `robot_api.yaml`、Motion 配置和资源；导入时重新校验。只含 URDF/旧 VR 配置的历史 profile 保留旧能力，但明确标记统一 API 不可用及原因。OpenArmX、X1 的统一模式要求 Motion；显式使用 v23 时整条链恢复为旧接口模式，不声明统一 API 能力。

各入口共同遵循 `HC_ROBOT_NAME`、`HC_ROBOT_CONFIG_ROOT` 优先于配置文件；配置中的相对 root 相对配置文件目录解析。运行时固定所选 profile。切换／修改配置应停止旧输出，并一致重启中间件、Motion、遥操作和硬件适配器；不能在旧控制器仍运行时只换能力描述。

以下是部署操作说明，本次验收没有执行真机启动：

```bash
source /home/hc_op/workspace/teleop_ws/setup_hc.bash
cd /home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic
export HC_CONTROL_BACKEND=motion
export HC_ROBOT_NAME=openarmx     # 换 X1 时设为 x1
# 可选 export HC_ROBOT_CONFIG_ROOT=/path/to/profiles
/usr/bin/python3 middleware/profile_cli.py api
/usr/bin/python3 middleware/profile_cli.py generate --output /tmp/hc-api-preview
```

生成目录含可审阅的 `robot_api.resolved.yaml`、`robot_info.json`、`motion.yaml`、`sdk.yaml`、`channels.yaml`、`driver.yaml`。`tools/run_motion.sh` 与 `tools/run_robot_driver.sh` 分别生成并实际消费对应参数，驱动连接话题来自 profile。

构建 HC 内 Motion 后设置与构建一致的 `HC_MOTION_INSTALL`；`HC_CONTROL_BACKEND=motion ./run.sh teleop` 启动中间件和统一控制链。硬件项目单独启动厂商控制器，适配器使用 `bash tools/run_robot_driver.sh` 消费当前 profile。OpenArmX `--fake` 只选择假硬件健康契约，不会自动创建假控制器。避免同时启动旧话题参数的第二个 adapter。选择 profile、启动中间件或订阅能力都不会自动连接或使能真机。

硬件契约见 [hardware_interface_v2.yaml](../interfaces/hardware_interface_v2.yaml)：统一模式增加 `robot_api_schema: hc-robot-api/v1`，驱动使用 internal 模型话题，公共话题由 HC 转换持有。旧原生名进程必须整体迁移或隔离到显式 legacy 命名空间，不能在同一公共话题混发。

## Demo 与数据采集

在线客户端需要 ROS 2 Humble、`rclpy` 和标准消息；MoveJ/home 另需与服务端匹配的 `humanoid_motion_interfaces`。ROS 使用 `/usr/bin/python3`。两端 Domain 和时钟需一致；跨电脑 DDS 尚未实测。

```bash
/usr/bin/python3 examples/robot_api_demo.py describe
/usr/bin/python3 examples/robot_api_demo.py subscribe --duration 5

# 下列命令要求机器人端已经选择 motion 并显式恢复输出
/usr/bin/python3 examples/robot_api_demo.py servo-j --group left_arm --joint left_joint_4 --delta 0.02 --duration 2
/usr/bin/python3 examples/robot_api_demo.py servo-p --group left_arm --dx 0.005 --duration 2
/usr/bin/python3 examples/robot_api_demo.py home --group left_arm
/usr/bin/python3 examples/robot_api_demo.py move-j --group left_arm --joint left_joint_4 --delta 0.02 --scale 0.1

# 唯一需要本地 profile 的可选离线检查
/usr/bin/python3 examples/robot_api_demo.py describe --profile adapters/robots/openarmx
```

操作方可通过 Dashboard 或既有 `POST /api/teleop/source`（`{"source":"motion"}`），再 `POST /api/safety/resume`（`{}`）显式授予控制。demo 不自动切源或恢复硬件。ServoJ 从新鲜测量构造整组目标；ServoP 从新鲜 FK 构造小位移、保留本机 frame 和姿态。退出／异常停发 Servo；MoveJ 异常／Ctrl+C 请求取消并等待终态。

录制元数据保存公共 `free_joints`、`robot_api_schema`、`robot_api_revision`、完整 `robot_api_info`。公共总状态、分组状态、选中目标和 Action 使用同一名称表。Drive 回放要求当前 schema/revision 匹配，并再次校验目标；显式授予回放后才将历史采样时间换成当前执行时间。缺少统一 API 元数据的历史原生名录制明确拒绝驱动，不静默当作新格式；只读查看不受此迁移限制。需驱动历史数据时，应先进行显式离线转换并审阅。

外骨骼仍是独立选中来源，输入 `/control/exoskeleton/joint_states` 必须使用公共名；原生名目标在选择器入口拒绝。夹爪手指、底盘和相机沿用各自可选话题，不能仅根据双臂 API 假定硬件具备这些能力。更多示例与复测入口见 [examples/README.md](../examples/README.md)。

当前 main 的录制生命周期由 TopicRecorder/TaskRecorder 管理，支持 MCAP、MCAP＋RGB MP4（深度消息原样保留）、LeRobot 与双格式。历史验收中提到的 RecordingService、平台数据工作台及相机标定录制不在本次合并范围。当前结果以 [分支整合说明](MAIN_HUMANOID_INTEGRATION.md) 为准。
