# Motion 工作区

`src/humanoid_motion_interfaces` 和 `src/humanoid_motion_server` 是独立仓库中的 ROS 2 包，通过 Git 子模块嵌入本仓库。父仓库固定其提交，提供 profile、控制桥接、启动脚本与构建入口；子模块源码修改应在各自仓库维护。

## 构建

```bash
git submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
./bootstrap_colcon.sh build
./bootstrap_colcon.sh test
```

依赖安装和 SDK 版本以 [`src/humanoid_motion_server/README.md`](../src/humanoid_motion_server/README.md) 为准。当前子模块固定 `vendor/robo_manip/`（`librobo_manip.so`、`libmotion_control.so`、`liblibplaco.so` 及头文件），构建会校验 `SHA256SUMS` 与固定依赖版本；ROS 自带的同名库不能替代。依赖目录支持 `HUMANOID_MOTION_SDK_DEPS_PREFIX`、仓库 `.sdk_deps` 或 `/usr/local`。构建目录为 `build_motion/`、`install_motion/`、`log_motion/`，可分别用 `HC_MOTION_BUILD`、`HC_MOTION_INSTALL`、`HC_MOTION_LOG` 指定。

## 控制模式

当前所有内置机型（`x1`、`openarmx`、`HC_REALMAN`）已移除原 `v23` 后端，统一默认使用 `control.backend: motion`（由 `StandardMotionTeleopBridge` 对接 `humanoid_motion_control_node` 完成双臂 `ServoP` QP Servo IK 与 RTC 平滑）：

```bash
./run.sh sim             # 完整仿真（含 PyBullet GUI 窗口）
./run.sh sim --headless  # 无窗口仿真（可在网页按需打开三维视图）
ROS_DOMAIN_ID=14 ./run.sh teleop
```

如需启用统一机器人公共 API（`UnifiedMotionTeleopBridge`），可为整套进程设置 `HC_UNIFIED_ROBOT_API=1`（或配置 `control.unified_api: true`）。中间件、仿真、控制器必须从同一个入口和后端选择启动，切换前停止原进程。仿真继续使用本机隔离 ROS 域；真机模式禁止网页另起仿真。

统一模式下，驱动使用 `/internal/robot/joint_states` 与 `/internal/robot/joint_commands`；公共 API 负责名称与单位转换，在线能力发布到 `/status/robot/info`。在网页选择 Move / Servo 后还需显式恢复输出。VR、程序与外骨骼共用命令选择器；切源、反馈过期、ready 失效或停止会撤销旧许可。

单独启动 Motion：

```bash
./tools/run_motion.sh --profile adapters/robots/x1
```

此命令使用给定 profile 生成 Motion/SDK 参数，不启动硬件。`tools/run_robot_driver.sh` 供已迁移到统一接口的独立硬件包使用。不要在同一域混用旧原生名驱动与统一公共名接口。API、Servo 租约和 Move 动作语义详见 [机器人公共 API](ROBOT_PUBLIC_API.md)。

## DAgger 录制范围

在“人工干预与纠正采集”中，开启会话前选择录制范围：默认“仅人工接管片段”，每次右 Grip 接管单独保存；“整个会话”从开启到退出连续保存程序运行、人工接管及暂停期间的数据。两种模式都使用“话题录制”中勾选的数据项，会话期间不能更改范围。整段录制保留接管起止事件和会话元数据；MCAP + MP4 包包含 `intervention-session.json`，LeRobot 导出包含逐帧 `intervention` 标记（表示人工控制时段，不表示任务成功）。

`POST /api/control/intervention/enable` 的可选字段 `capture_mode` 为 `corrections_only`（默认）或 `full_session`，其余请求身份与控制权校验保持一致。整段模式不会因缺少观测而丢弃已经收到的数据；开始程序仍要求必要观测、控制器和有效 VR 跟踪就绪。

“控制源切换”用于普通控制时选择 VR、Move / Servo 程序或外骨骼。DAgger 会话内由控制权状态机自动切换，面板锁定；握住右 Grip 接管，从程序控制接管后正常松手自动交还。若从暂停状态接管，松手回到保持。选择程序控制或点击“开始程序”只开放相应控制入口，外部程序仍需持续发送动作。
