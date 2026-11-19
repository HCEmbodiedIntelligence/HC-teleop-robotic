# Generic Robot Control

这一部分是与厂商硬件无关的控制和桥接层（原 `v23` / Pinocchio 控制器已移除，双臂逆解与实时轨迹统一交由 `humanoid_motion_server`）：

- 启动 `motion` 桥接节点与 VR 遥操作控制器（`RobotArmTeleopNode` + `StandardMotionTeleopBridge` / `UnifiedMotionTeleopBridge`）；
- 只读写 `/action`、`/observation`、`/control`、`/status` 标准接口；
- 可执行入口位于 `nodes/`，公共控制与几何计算位于 `core/`（含本地 PyBullet 正运动学与可选腰部 DLS IK / 碰撞校验）；
- 内置机型（`x1`、`openarmx`、`HC_REALMAN`）及网页导入的机器人资料统一位于 `robots/<配置ID>/`，包含 URDF、Meshes、`arm_teleop.yaml` 与 `motion/{motion.yaml,sdk.yaml,channels.yaml}`。

独立启动：

```bash
./adapters/start.sh
```

这是内部组件。正常使用请从项目根目录运行 `./run.sh`（网页按需启动）或 `./run.sh teleop`（实机遥操作），一次启动本组件、Motion 节点和 Web/VR 中间件；命令行仿真使用 `./run.sh sim` 或 `./run.sh sim --headless`。停止该组件只会停止逆解与控制进程，不会停止任何外部硬件项目。

双臂命令链路固定为：

```text
/action/ee_poses -> StandardMotionTeleopBridge -> /control/motion/{left,right}_arm/servo_p
  -> humanoid_motion_control_node (QP Servo IK / Placo + RTC)
  -> /control/motion/joint_targets -> /control/solver/joint_targets
  -> RobotArmTeleopNode (合并腰/头/灵巧手) -> /control/vr/joint_states
  -> Command Mux -> /action/joint_states -> 硬件驱动 / 仿真
```
