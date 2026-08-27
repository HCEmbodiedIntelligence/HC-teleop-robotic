# Solver plugins

解算后端按独立进程隔离，VR 控制器只负责生成末端目标：

- `v23`：当前 Pinocchio/CasADi 解算包，接收成组 `PoseArray`，输出
  `/hc_teleop/joint_cmd_arm`；
- `motion_server`：通过左右 ServoP `PoseStamped` 接收目标，输出统一到
  `/hc_teleop/joint_cmd_vr`。

两种后端最终都经过中间件 command mux，只有 mux 发布
`/hc_teleop/joint_cmd`。可用 `./start_teleop.sh --solver-backend v23` 或
`./start_teleop.sh --solver-backend motion_server` 切换。后者默认读取
`HC-teleop-robotic` 同级目录下的 `humanoid/install/setup.bash`。也可设置
`HC_HUMANOID_ROOT` 覆盖工作空间根目录，或用 `HC_MOTION_SERVER_SETUP`
直接指定安装环境。

机器人配置若要支持 Motion Server，需在 `arm_teleop.yaml` 增加
`motion_server` 资源与左右 ServoP 通道；OpenArmX 内置配置可作为模板。
