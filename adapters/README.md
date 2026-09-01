# Generic Robot Control

这一部分是与厂商硬件无关的控制和解算层：

- 启动 Pinocchio v2.3 逆解和 VR 遥操作控制器；
- 只读写 `/hc_teleop` 标准接口；
- 可执行节点位于 `nodes/`，运动控制实现位于 `core/`，v2.3 逆解位于 `v23/`；
- X1 与网页导入的机器人资料统一位于 `robots/<配置ID>/`，每个目录自包含 URDF、Meshes 和控制参数。

独立启动：

```bash
./adapters/start.sh
```

这是内部组件。正常使用请从项目根目录运行 `./run.sh teleop`，一次启动本组件和 Web/VR 中间件；仿真使用 `./run.sh sim`。停止该组件只会停止逆解与控制进程，不会停止任何外部硬件项目。

双臂命令链路固定为：

```text
controller_target_ee_poses -> v23 sol_q -> /hc_teleop/joint_cmd
  -> 独立机型项目固定频率执行器 -> 厂商驱动
```
