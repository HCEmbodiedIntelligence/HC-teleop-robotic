# 可选 Motion 工作区

本分支沿用主分支的中间件、录制、机器人适配和 PyBullet 结构。只增加以下两个 ROS 2 子模块：

- `src/humanoid_motion_interfaces`：运动消息、服务和 Action。
- `src/humanoid_motion_server`：运动仲裁、IK、Move/Servo 与关节轨迹输出。

原 humanoid 分支中的 driver、gripper、camera、adapter manager、VR 接收及机器人 ROS 包已移出工作区，不再作为本分支的独立组件构建。主分支原有 `middleware/`、`adapters/` 等功能保留，`./run.sh` 和 `./install.sh` 仍使用主分支接口。

## 构建

```bash
git submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
# 指向合格的 RoboManip 依赖安装目录，含 ruckig、Pinocchio、hpp-fcl 等：
export HUMANOID_MOTION_SDK_DEPS_PREFIX=/path/to/robo_manip/dependencies
./bootstrap_colcon.sh deps
./bootstrap_colcon.sh build
./bootstrap_colcon.sh test
```

本机 `.deps/robo_manip` 已连接到既有 SDK 依赖目录。父仓库保留 Humble hpp-fcl 2.4.5 兼容补丁，构建入口会幂等应用它；原有两处本地 SDK 修改保留。构建、安装和日志分别放入 `build_motion/`、`install_motion/`、`log_motion/`，不加载旧整栈的安装 overlay。

## 单独启动运动服务

```bash
./tools/run_motion.sh
./tools/run_motion.sh --show-args
```

这个入口只启动运动服务，默认模型是运动包自带的测试人形机器人，**不是 X1 配置**。更换机器人需同时提供匹配的 URDF、SDK、通道、工具及 ROS 参数配置：

```bash
./tools/run_motion.sh \
  urdf_file:=/absolute/robot.urdf \
  params_file:=/absolute/motion_control.yaml \
  sdk_config_file:=/absolute/robo_manip.yaml \
  channel_config_file:=/absolute/channels.yaml \
  tool_config_file:=/absolute/tools.yaml
```

默认反馈为 `/observation/joint_states`，输出为 `/control/motion/joint_targets`；可通过同名 launch 参数覆盖。反馈应为可靠 QoS 的 `sensor_msgs/JointState`，关节名称与模型一致，name/position/velocity 同长度且数值有效；反馈缺失或过期会阻止运动输出。

本分支网页“同时启动 Motion 逆解与 VR 遥操作控制”开关默认启动 X1 motion 服务、motion 桥接和 VR 适配器，不再启动 v23。`./run.sh sim`、`./run.sh teleop` 的控制入口同样默认使用 motion。配置位于 `adapters/robots/x1/motion/`；其他机器人需提供对应四配置，缺少配置会明确报错。

链路为 `/action/ee_poses`（右、左 TCP，zhi_Link）→ `/control/motion/{side}_arm/servo_p` → SDK IK/RTC → `/control/motion/joint_targets` → 桥接 `/control/solver/joint_targets` → VR 适配器 `/control/vr/joint_states` → 中间件选择器 `/action/joint_states`。桥接只在硬件和反馈新鲜、手臂 Grip 激活、控制使能时使用 motion 结果；闲置时以实测关节保持，过期结果不重放。腰部、底盘、夹具和原有关节回零逻辑仍由 VR 适配器管理。

只启动原始 motion 时可运行 `./tools/run_motion.sh --profile adapters/robots/x1`。网页已经启动控制时不要重复手动启动同一个 motion 节点。需要临时使用原 v23 时，在启动中间件前设置 `HC_CONTROL_BACKEND=v23`（网页开关文字仍为本分支默认的 Motion）。

子模块自带的 mock/bringup/registered_robot launch 属于上游完整栈示例，依赖本分支不包含的 driver/VR 包；在这个两包工作区请使用父仓库的 `tools/run_motion.sh`。


## 双臂跟踪回归

运动节点将控制定时器、反馈和 Servo 输入分到不同回调组，避免双臂解算饿死输入并造成 100 ms lease 反复过期。保活时限和 SDK 限速保持不变；桥接等待结果时保留新鲜目标授权，不会误删正在传输的结果。

对应修复已提交到 motion 子模块并由本仓库固定其提交引用。可以运行隔离域 201 的假反馈回归（不会连接真实运行域；若域已占用则中止）：

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="$PWD:$PWD/.deps:$PYTHONPATH" /usr/bin/python3 tests/motion_follow_probe.py
```

该回归用 13 Hz 反馈测试双臂同时前伸 30 mm，检查经 motion 和桥接后的实际 FK 误差小于 1 mm，且轨迹没有异常中止。
