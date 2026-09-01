# HC Teleop

HC 通用遥操作框架的原生 ROS 2 工作区。当前分支只包含解耦后的运行时，
不再保留旧 middleware、Python adapters、PyBullet 控制链或兼容转发入口。

## 构建与启动

```bash
cd /home/maple/test/HC-teleop-robotic
git submodule update --init --recursive
./bootstrap_colcon.sh build
./run.sh profile:=openarmx mode:=sim
```

默认启动以下组件：

- `hc_vr_gateway`：PICO UDP v1/v2 解码
- `hc_teleop_core`：VR 映射、clutch/deadman、多组命令仲裁和 watchdog
- `hc_motion` + `hc_motion_backend_kdl`：目标路由与轻量 IK
- `hc_adapter_openarmx`：OpenArmX 仿真设备适配
- `robot_state_publisher`：URDF 状态发布

调试时使用 `composition:=isolated`，对比真实设备时使用
`mode:=shadow`；真实设备必须由独立 `hc-adapter-*` 包提供唯一硬件命令发布者。

## 工作区布局

业务代码全部位于 `src/hc_*`：接口、VR 网关、遥操作核心、运动后端、设备 SDK、
设备适配、机器人 profile、数据集控制面和 bringup。机器人 URDF 与 profile
由 `hc_robot_<model>` 包提供，厂商 SDK 后端作为独立可选包构建。

```bash
hcctl doctor --profile openarmx
./bootstrap_colcon.sh test
```

所有话题在启动时统一放到 `/robots/<robot_id>/` 命名空间；最终执行命令只允许
安全仲裁器发布。

## 使用 Humanoid Motion Server

`hc_motion_backend_robo_manip` 使用项目内连接的
`src/humanoid_motion_server` 的 ServoP、RTC 和实测关节 FK替换 KDL。Motion
Server 与接口仓以固定提交的 Git submodule 保存，仍保持各自独立历史。后端是
独立进程，只发布 candidate，不会绕过 HC 安全仲裁器。

首次拉取和构建：

```bash
# 新 clone 推荐直接使用 git clone --recurse-submodules。
git submodule update --init --recursive

# 指向 ruckig 0.17.3、toppra 0.6.8 等 ABI 固定依赖的安装前缀。
# 当前开发机未设置时会兼容检测 /home/maple/test/humanoid/.sdk_deps。
export HUMANOID_MOTION_SDK_DEPS_PREFIX=/home/maple/test/humanoid/.sdk_deps

./bootstrap_colcon.sh build
./run.sh profile:=x1 mode:=sim motion_backend:=robo_manip
```

默认 `motion_backend:=profile` 仍读取机器人 profile；X1 当前默认 KDL，显式传
`robo_manip` 才切换。运行时不再 source 外部 Humanoid underlay。
