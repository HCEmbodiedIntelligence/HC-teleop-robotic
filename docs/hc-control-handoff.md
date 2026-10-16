# HC 控制与硬件接入

使用 `HC_ROBOT_NAME=openarmx` 或 `x1` 选择 profile；`HC_ROBOT_CONFIG_ROOT` 可指定根目录，环境变量优先于 middleware 配置。统一解析入口为 `middleware/profile_cli.py` / `core/robot_api.py`。OpenArmX、X1 统一模式均要求 Motion，不允许覆盖回 v23；旧后端只服务明确的 legacy profile。启动日志输出最终 backend 和 ROS_DOMAIN_ID。

`bootstrap_colcon.sh build` 仅构建 HC 内两个 motion 包，支持 HC_MOTION_BUILD、HC_MOTION_INSTALL、HC_MOTION_LOG 隔离目录；启动时设置相同 HC_MOTION_INSTALL。不再构建外层同名包，不在新源码上应用旧快照兼容补丁。完整原生 motion 的服务、示例、SDK、规划、仿真和调试工具继续保留。SDK 与 /usr/local 中依赖的 ABI、散列由子模块既有检查验证。

普通运动在 middleware 选择 `ros.command_mux.source=motion` 并显式启用。目标端点为 `/control/motion/{left,right}_arm/{move_j,move_l,move_p}`（对应 humanoid_motion_interfaces action）与 `{servo_j,servo_p}`（JointState/PoseStamped）。公共名称由 `/status/robot/info` 获取。输出为 `/control/motion/joint_targets`，经唯一 mux 到公共 `/action/joint_states`，再通过共同边界检查及转换到模型 `/internal/robot/joint_commands`。驱动模型反馈进入 `/internal/robot/joint_states`，HC 唯一转换为公共反馈。普通运动不需要 VR 手柄握持；切换 source、停止、回放或 ready 丢失会撤销旧权限。

硬件执行使能独立使用 `/control/hardware/enabled`；`/control/teleop/enabled` 只启停 VR 输入，不能接到硬件停止门。中间件显式 resume 为当前控制代际发送硬件使能，切源、停止、ready 失效和回放结束会撤销；不周期重发 true。OpenArmX 与通用臂/夹爪 HC 配置已同步更新，自定义配置需要同步中间件 `hardware_enable_topic` 和驱动 `enable_topic` / `output_enable_topic`。接口与 demo 见 [ROBOT_PUBLIC_API.md](ROBOT_PUBLIC_API.md)。

内部 `/control/motion/permit` 用 Header 表示限时许可，frame_id=`motion:epoch` 或 `vr:epoch`，20 Hz，0.3 秒过期；profile 的 output_permit_topic 必须与 middleware motion_permit_topic 相同。公共 Move/Servo 需要 motion 许可；VR 使用独立 `/internal/vr/{group}/servo_p` 和 `move_j`，VR home 复用 MoveJ，不能由此授权公共 Move。Move 占用相应关节，期间 Servo 被拒绝，释放后旧目标不续播。许可代际变化取消 session，旧 worker、排队 Servo 和已算出的命令不能在恢复后继续发布。这个话题不是执行成功证据。

运动桥接按 profile 的 arms、关节映射、渠道端点和帧配置工作；测量状态保留原采样时间，每侧独立新鲜度。VR 和程序位姿共用同一 SDK/IK/最终 RTC，辅助腰部、头部、夹爪在独立通道保留，不再次覆盖臂目标。已启用的碰撞检查位于共同模型命令边界。配置和客户端不固定七轴。

`tools/run_motion.sh` 生成并消费解析后的 Motion/SDK 配置；`tools/run_robot_driver.sh` 独立生成并消费 profile 连接参数，不启动厂商控制器或 CAN。中间件不自动启动硬件。统一 API 的本次软件验收见工作区 `reports/unified-robot-api-20261015-165101/RESULT.md`，包含隔离假硬件闭环，以及 OpenArmX/X1 现有 URDF 的 PyBullet 状态/动作录制和本机平台上传读回校验；不包含相机流、外部 X1 真机驱动、真实电机或跨电脑 DDS。

OpenArmX 原生启动、输出 adapter、控制器顺序、执行占用和 fake 模式见 hc_openarmx/docs/hc-control.md。通用接入见 humanoid_driver_runtime/docs/hc-integration.md；相机启动基础来自 robot_bringup，不再依赖 manager。公共 app/config/UI 由 B 接入，字段需求在交接 requests/A.md。

旧pin差异审计见 motion子模块 docs/hc-baseline-audit.md。用户已取消旧 registered_robot 入口及说明，后续提交将它们移除，不再引入旧 manager/teleop_vr_recv 启动依赖。
