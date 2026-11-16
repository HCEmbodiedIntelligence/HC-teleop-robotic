# X1 / PICO 接入契约与验证范围

本目录是新的平台接入实现，不导入旧 middleware、ControllerV23 或旧 PyBullet 控制器。旧机型包仅提供 URDF、Mesh、关节和标定数据。没有连接真实机器人或头显进行本次验证。

核对来源：HC-Teleop `436516cbe8ba59caed8cd3c75f9bdb71bf273a04` 的 `Assets/UdpPoseSender.cs`、`Docs/UDP_PROTOCOL_V2.md`；HC_x1 `b3f2e436bb9d773f01bd83631bb0071dfed2f4e3` 的 `hc_x1/control_adapter.py`、`hc_x1/joint_command_smoother.py`、`config/x1_hardware.yaml`、`src/hc_tj/launch/hc_tj.launch.py`。

## 进程与线程

`python -m hc_platform.adapters.runtime --role vr|ik|backend --config config.json --socket /tmp/.../control.sock --state-dir /tmp/...`

backend 另选 `--backend simulation|x1`。启动不恢复运动。控制核心负责显式恢复、会话、租约、epoch、参考代际和唯一最终输出。

| 进程 | 并发方式 | 边界 |
| --- | --- | --- |
| VR | 一个 asyncio loop；UDP callback 只解析、入有界队列 | 最多 64 个输入事件；溢出锁停；不能丢掉 Grip 边沿后继续运动 |
| IK | 一个 asyncio loop 读 IPC；一个专用线程调用 Eigen C++ native FK/Jacobian/DLS | 每次只计算一个请求，等待期间只保留最新请求；迭代和时间有上限；结果受原始截止时间约束 |
| X1 backend | asyncio IPC + 一个 ROS SingleThreadedExecutor 线程 | 只有 ROS 线程发布；输出 mailbox 长度 1；fence 事件优先清空输出并执行软件保持/底盘零 |
| 仿真 backend | 一个 asyncio loop，PyBullet DIRECT/GUI 原生步进 | 关节位置电机；反馈来自 PyBullet；底盘为明确标注的速度积分运动学模型 |
| ROS observation | 独立进程，消费有界 Unix datagram 镜像 | 发布仿真实际反馈、实际提交动作、模型 FK/TF/外参及 VR 输入；镜像拥塞只计数丢弃，不阻塞控制 |

IPC 为 Unix SOCK_SEQPACKET。数值限位、源授权和期限由核心检查；backend 在真正发送前再次检查 boot/epoch/seq/绝对 monotonic deadline。`output_ack` 表示适配器实际提交事实，不是电机到位确认。核心或 IPC 断开时，backend 退出前执行停止；驱动自身超时保护仍必须启用。

`fence_ack` 是软件边界确认。HC_x1 标准话题没有实体驱动队列取消确认，因此 `physical_stop_ack=false`。不能把 ROS publish 成功写成机器人已经物理停止。若关节已发送、某个手部通道因反馈过期未发送，ACK 用 `accepted=false` 和 `PARTIAL_GRIPPER_FEEDBACK_STALE` 明确报告部分提交。

## PICO 字节协议

默认 pose UDP 5005，discovery UDP 5006，头显返回事件 UDP 5007。

- v1：102 bytes，little endian `<4sBIdB21f`。
- v2：162 bytes，little endian `<4sBIdB21f3H6f3H6f`。
- Header：`PICO`、version uint8、sequence uint32、Unity 实际采样时间 float64、tracking flags uint8。
- 三个位姿依次 head/left/right，每个 `xyz + quaternion xyzw`。
- v2 左／右输入：held/pressed/released uint16，trigger/grip float，primary axis xy，secondary axis xy。
- Grip bit 2，Trigger bit 3；左 X／右 A 为 bit 0，左 Y／右 B 为 bit 1，主摇杆 click 为 bit 5。

`sequence` 使用 uint32 回绕比较；采样 timestamp 与包序号分别检查。发送线程在采样冻结时仍可能增加 sequence，因此冻结 timestamp 不刷新控制输入时效。合法静止位姿但 timestamp 推进不会误报冻结。旧 timestamp、重复／乱序、NaN／Inf、非法 quaternion、非法模拟量范围均拒绝。

发现请求 `PICO_DISCOVER_V1`，空闲回应 `PICO_RECEIVER_V1|5005`，占用回应 `PICO_RECEIVER_BUSY|5005|owner_ip`；`PICO_RELEASE_V1` 只能释放同一来源 IP 的 lease。现有客户端没有 token/加密身份，兼容模式是 IP 独占，不是密码学身份认证。同一网络需部署访问隔离。v1 无控制器输入，不能宣称支持 Trigger/Grip/DAgger。

输入送到核心时保持 Unity tracking 原始坐标，标记 `source_frame=unity_tracking`，由 IK worker 唯一执行配置中的基变换。位置 `p'=S p`，相对旋转 `R'=S R S^T`。跨左右手系的 S 不能直接变成 TF quaternion。

头显事件为 UTF-8 JSON `{"kind":"safety_stop","source":"middleware","timestamp":...,"payload":{"reason":"..."}}`。录制字段使用客户端实际支持的 `recording/filename/path/action/message/error/marked/marked_at`。回放字段 `state/requires_reset/message/error`。安全状态周期重发同一时间戳，避免单次 UDP 丢包后长期不一致；现有客户端不支持 ACK，因此不声称可靠事件投递。

头显相机授权还依赖 `GET /api/status`（默认 7876）中的 `vr.is_locked` 与字符串 `vr.active_peer`。VR runtime 原子写 `vr.json`，API读取并附加进程存活状态。只看到发现回应不授予相机或运动控制权。

## 两套硬件话题必须显式选择

| contract | 最终关节输出（URDF 名） | 关节反馈（URDF 名） |
| --- | --- | --- |
| `legacy-model-topic`，默认，匹配提供的 HC_x1 | `/action/joint_states` | `/observation/joint_states` |
| `internal-model-v2`，适用于已升级驱动 | `/internal/robot/joint_commands` | `/internal/robot/joint_states` |

不会自动猜测、同时发布两套，或直接发布私有 `/io_teleop/joint_cmd`。HC_x1 自己的 100 Hz smoother 是私有输出唯一拥有者。

两套都使用 `/status/hardware/ready` Bool；底盘 `/action/base/command` Float64MultiArray 顺序为 `[yaw,forward,lateral]`，与核心 `[vx,vy,wz]` 明确换序；手部 `/action/{left,right}_hand/joint_states` 使用真实 finger joint names 及配置的 open/closed 校准插值。手部反馈 `/observation/hands/joint_states` 按真实关节名检查新鲜度。`L_ban/R_ban` 只是 URDF 仿真几何，不虚构它们为真实编码器反馈。

ROS 保持 depth 1 / reliable / volatile。硬件配置的 `allowed_groups` 是独立显式授权；不从 hardware_ready 推断左右臂、腰部或底盘控制权限。默认物理配置只允许 right_arm/right_gripper，左臂和身体须显式启用。提供的 HC_x1 launch 默认左右臂和腰部皆可控，不能将该默认误认为现场已授权。需要同时核对硬件 launch 开关。

backend 检查同一最终关节话题的其他发布者；检测冲突立即停止自身新输出并报告。ROS graph 检查并非安全访问控制；部署中必须停止旧控制进程，通用发布入口也不得绕过核心写这些话题。

## 机型与模型限制

双臂 FK/IK 从 URDF 名称解析链路，由 `platform/kinematics` 的 Eigen C++ 库执行几何 Jacobian 与阻尼最小二乘、关节限位、有限步数和截止时间。Python 负责验证模型数据、输入映射和调用边界。默认要求 native 库；无库明确启动失败。可显式设置 `kinematics_backend=python` 使用参考实现，状态会报告降级。无法收敛不伪造成功。软件不包含厂商驱动、电机内部回路、推理引擎或完整逐手指 VR 手势跟踪。

构建 native 库：`cmake -S platform/kinematics -B platform/kinematics/build -DCMAKE_BUILD_TYPE=Release`，再执行 `cmake --build platform/kinematics/build -j2`。也可用 `HC_KINEMATICS_LIBRARY` 指向 ABI v1 的共享库。

X1 普通模式可按配置使用腰部高度／俯仰两任务求解、摇杆底盘速度与 Grip 内头显平面偏移；后者是无里程计条件下的速度意图，不是实测定位闭环。DAgger 由核心关闭 body/base 输出。当前 X1 颈部默认关闭；显式启用时必须配置有效 yaw/pitch 关节并取得新鲜反馈，映射受模型与配置限位约束，配置缺失则拒绝启动。

`collision_avoidance_enabled=true` 时启用独立 shadow PyBullet 模型，对当前构型到候选构型的关节路径分段检查自碰撞；按配置排除近邻、同臂或指定 link 对。检查要求完整新鲜配置关节反馈，受原始请求 deadline 约束；依赖缺失、无检查对、状态不完整、路径超预算和碰撞均不放行。已处于间隙阈值内时，只有显式启用 escape 且路径不恶化、终点改善才放行。碰撞锁存需要松开后重新建立参考。当前 X1 默认该配置为 false，此时状态明确 `collision_checked=false`。检查只覆盖 URDF 几何及所选 link 对，不包括未建模环境，也不能作为真机碰撞安全认证。

仿真停止边沿锁存保持目标；后续 stop 心跳不重新采样反馈，避免把重力误差累计成目标漂移。原 X1 URDF 个别固定连杆无惯性数据，PyBullet会使用默认惯性，因此仿真通过不等同于验证真实负载动力学。

观测进程用 `python -m hc_platform.adapters.observations --config config.json --state-dir /tmp/...` 启动，真机只能使用 `--vr-only`。仿真相机外参明确 `source=model_fk, calibrated=false`；底盘状态来自模拟器真实构型并注明运动学底盘模型。`/action` 镜像只来自 backend 已提交的输出，不用核心尚未提交候选冒充动作。Unix datagram 丢弃计数在 adapter 状态中公开。

## 验证

`PYTHONPATH=platform/python <python3.10> -m pytest platform/tests/test_adapters*.py`

包含精确协议布局、回绕／乱序、独占／释放、冻结与合法静止、真实本地 UDP 发现／回传、URDF 校验、native/reference FK与Jacobian对照、IK 可达／不可达、反射变换、X1模型按名加载、输出代际／期限／授权、路径中间段碰撞、neck限位、停止保持以及四个真实进程的 PICO→IK→core→PyBullet 电机闭环。另已在隔离 loopback ROS Domain 验证 X1 topic 发送/反馈、旧代际拒绝，以及模拟观测发布和模型外参标识；不涉及真实硬件。

PyBullet 与 NumPy ABI必须兼容；模块路径存在不代表导入成功。ROS Python 环境须保留 `/opt/ros/humble` 的 PYTHONPATH。没有 ROS／PyBullet 依赖时应显示缺失原因，不允许假装真实后端已启动。

## 媒体与相机元数据

媒体单独进程提供头显所需 `streams[].state/capture_fps/webrtc_send_fps`，offer 返回 `telemetry_session_id`，`POST /receiver/{id}` 检查会话、摄像头、递增 sequence/sample_ms 及有限计数。收到的丢包／解码／冻结指标明确是 receiver_reported，不能作为机器人控制输入。

编码使用固定版本 aiortc 1.15.0。私有 sender encoder 与 RTP 发送观测 hook 只在 `video_encoding.py` 内；版本不符明确拒绝 offer。H264、RTX 在处理远端 offer 前设置，保留 NACK/PLI。NACK/RTX 是实际 SDP 协商结果；RED/ULPFEC sender 当前未实现，`fec.available=false`，不把重传称作 FEC。发送 FPS 按完整帧 marker packet 成功提交给 DTLS transport 计数，接收端解码 FPS另列。

`encoder_actual` 只有真实帧产生有效 H264 位流后才设定。可选择 libx264、NVENC、V4L2 M2M 或 auto；硬件打开／编码错误逐项保留，再尝试 libx264。本环境实际验证 NVENC 编码与 WebRTC 解码成功；V4L2 在本机打开失败，错误来自实际 avcodec_open2 尝试。VAAPI 尚无 device/upload 实现，会明确报告并回落。连接结束保留 last_encoder_attempt；空闲不能报告正在使用某硬件编码器。

输出尺寸和 FPS 实际应用。当前协商 H264 level 3.1，宽高须为偶数，最多 3600 macroblocks/frame、108000 macroblocks/s；例如 1280×720@30 在范围内。高于该范围需要未来扩展 SDP level 协商，不能只改配置宣称支持。ROS 回调只保留最新压缩字节或原始图像，解码与编码在媒体 worker 执行；所有录制用 topic 即使没有 websocket/UDP 输出，也统计接收频率、age、缺数／订阅错误。raw image 统计和 JSON 展示不展开巨型像素数组。某个自定义消息类型缺失只使该订阅 unavailable，其他话题与相机继续工作。

相机元数据同时保留 hardware JSON、CameraInfo 和 TF/TF_static 来源。可在 stream.metadata 指定 calibration_topics、extrinsics_topics、camera_info_topics、parent_frame、camera_frame。head 默认读取公共元数据话题，以及 HC_x1 私有只读 calibration/extrinsics 兼容源。来自 URDF FK 的外参固定 calibrated=false；HC_x1 static_transform_config 外参是配置变换，不是实测标定。CameraInfo K 非零表示发布者提供内参，不能等同于本系统独立验证了物理标定。source、source_topic、age、frame、calibrated 和 calibration_verified 独立保留。源 ROS 话题的原始消息应由录制选择显式纳入；媒体缓存不会替代原始采集记录。

媒体验证：软件与 NVENC 两个真实 WebRTC loopback，检查实际接收分辨率与帧时间、RTP 发送 FPS、H264/RTX/NACK/PLI、接收遥测会话与旧序号拒绝；真实编码器软件／NVENC／V4L2／VAAPI 成功或回退；隔离 ROS Domain 197 验证 record-only 图像频率、缺自定义类型局部错误、CameraInfo、JSON、TF 元数据。未连接物理摄像头或 PICO 头显，未宣称它们已做现场验证。
