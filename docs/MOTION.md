# Motion 工作区

`src/humanoid_motion_interfaces` 和 `src/humanoid_motion_server` 是独立仓库中的 ROS 2 包，通过 Git 子模块嵌入本仓库。父仓库固定其提交，提供 profile、控制桥接、启动脚本与构建入口；子模块源码修改应在各自仓库维护。

## 构建

```bash
git submodule update --init -- src/humanoid_motion_interfaces src/humanoid_motion_server
./bootstrap_colcon.sh build
./bootstrap_colcon.sh test
```

依赖安装和 SDK 版本以 `src/humanoid_motion_server` 的说明为准。当前子模块固定 RoboManip 二进制及依赖版本，构建会校验哈希和 `/usr/local` 依赖来源；ROS 自带的同名库不能替代。构建目录为 `build_motion/`、`install_motion/`、`log_motion/`，可分别用 `HC_MOTION_BUILD`、`HC_MOTION_INSTALL`、`HC_MOTION_LOG` 指定。

## 控制模式

X1、OpenArmX 默认沿用 main 的 v23 控制，原生关节名与硬件话题不变。统一 API 与 Motion 通过同一个显式后端选择启用：

```bash
HC_CONTROL_BACKEND=motion ./run.sh sim
HC_CONTROL_BACKEND=motion ROS_DOMAIN_ID=14 ./run.sh teleop
```

也可在机型 `arm_teleop.yaml` 中设置 `control.backend: motion`。环境变量优先。中间件、仿真、控制器必须从同一个入口和后端选择启动，切换前停止原进程。仿真继续使用 main 的本机隔离 ROS 域；真机模式禁止网页另起仿真。

统一模式下，驱动使用 `/internal/robot/joint_states` 与 `/internal/robot/joint_commands`；公共 API 负责名称与单位转换，在线能力发布到 `/status/robot/info`。在网页选择 Move / Servo 后还需显式恢复输出。VR、程序与外骨骼共用命令选择器；切源、反馈过期、ready 失效或停止会撤销旧许可。

单独启动 Motion：

```bash
./tools/run_motion.sh --profile adapters/robots/x1
```

此命令使用给定 profile 生成 Motion/SDK 参数，不启动硬件。`tools/run_robot_driver.sh` 供已迁移到统一接口的独立硬件包使用。不要在同一域混用旧原生名驱动与统一公共名接口。API、Servo 租约和 Move 动作语义详见 [机器人公共 API](ROBOT_PUBLIC_API.md)。
