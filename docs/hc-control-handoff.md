# HC 控制与硬件接入

使用 `HC_ROBOT_NAME=openarmx` 选择 OpenArmX；其他机器人继续使用自己的 profile，不更改全局机器人默认值。`HC_CONTROL_BACKEND=motion|v23` 显式覆盖所选 arm_teleop.yaml 的 `control.backend`；启动日志输出最终 backend 和 ROS_DOMAIN_ID。OpenArmX 保留 v23 默认兼容，完整 MoveJ/MoveL/MoveP/ServoJ/ServoP 使用 motion。X1 明确记录原来实际使用的 motion 默认。

`bootstrap_colcon.sh build` 仅构建 HC 内两个 motion 包，支持 HC_MOTION_BUILD、HC_MOTION_INSTALL、HC_MOTION_LOG 隔离目录；启动时设置相同 HC_MOTION_INSTALL。不再构建外层同名包，不在新源码上应用旧快照兼容补丁。完整原生 motion 的服务、示例、SDK、规划、仿真和调试工具继续保留。SDK 与 /usr/local 中依赖的 ABI、散列由子模块既有检查验证。

普通运动在 middleware 选择 `ros.command_mux.source=motion` 并显式启用。目标端点为 `/control/motion/{left,right}_arm/{move_j,move_l,move_p}`（对应 humanoid_motion_interfaces action）与 `{servo_j,servo_p}`（JointState/PoseStamped）。输出为 `/control/motion/joint_targets`，经唯一 mux 到 `/action/joint_states`；必须有新鲜 hardware ready。普通运动不需要 VR 手柄握持；切换 source、停止、回放或 ready 丢失会撤销旧权限。

内部 `/control/motion/permit` 用 Header 表示限时许可，frame_id=`motion:epoch` 或 `vr:epoch`，20 Hz，0.3 秒过期；profile 的 output_permit_topic 必须与 middleware motion_permit_topic 相同。VR 只允许 Servo，普通 Move 需要 motion 许可。许可代际变化取消 session，旧 worker、排队 Servo 和已算出的命令不能在恢复后继续发布。这个话题不是执行成功证据。

运动桥接按 profile 的 arms、关节映射、渠道端点和帧配置工作；测量状态保留原采样时间，每侧独立新鲜度。配置文件不应依赖 X1 关节或固定双臂集合。

OpenArmX 原生启动、输出 adapter、控制器顺序、执行占用和 fake 模式见 hc_openarmx/docs/hc-control.md。通用接入见 humanoid_driver_runtime/docs/hc-integration.md；相机启动基础来自 robot_bringup，不再依赖 manager。公共 app/config/UI 由 B 接入，字段需求在交接 requests/A.md。

旧pin差异审计见 motion子模块 docs/hc-baseline-audit.md。旧 registered_robot.launch.py 与 registering_robot.md 已原样恢复，仅作为可选原生入口；start_teleop=true 需另行安装 teleop_vr_recv。
