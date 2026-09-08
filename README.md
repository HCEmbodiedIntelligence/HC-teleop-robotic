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

`run.sh` 默认同时打开当前机器人 profile 的 RViz。无需界面时关闭：

```bash
./run.sh profile:=x1 mode:=sim --rviz-sim:=false
# 显式开启（也是默认值）
./run.sh profile:=x1 mode:=sim --rviz-sim:=true
```

也支持 `rviz-sim:=false`；直接使用 ROS launch 时参数名为 `rviz_sim:=false`。
RViz 随 launch 一起退出，并使用同一 ROS Domain。此开关只控制 RViz 显示，
不改变 `mode` 或仿真设备节点的启动。

需要之后单独打开界面时，仍可使用 `./rviz.sh x1`。该脚本读取
`runtime/active_ros_domain` 并等待 URDF、TF 和关节状态就绪；手动覆盖 Domain
可使用 `HC_ROS_DOMAIN_ID=14 ./rviz.sh x1`。

默认启动以下组件：

- `hc_vr_gateway`：PICO UDP v1/v2 解码
- `hc_teleop_core`：VR 映射、clutch/deadman、多组命令仲裁和 watchdog
- `hc_motion` + `hc_motion_backend_robo_manip`：目标路由与 Motion Server 控制流水线
- `hc_adapter_openarmx`：OpenArmX 仿真设备适配
- `robot_state_publisher`：URDF 状态发布
- `rviz2`：机器人模型与状态显示

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
`src/humanoid_motion_server` 的完整 `CommandPipeline`，统一执行 ServoP 会话、
反馈超时、Servo 租约、关节限位检查和最终 RTC，并提供实测关节 FK。Motion
Server 与接口仓以固定提交的 Git submodule 保存，仍保持各自独立历史。后端是
独立进程，只发布 candidate，不会绕过 HC 安全仲裁器。

首次拉取和构建：

```bash
# 新 clone 推荐直接使用 git clone --recurse-submodules。
git submodule update --init --recursive

# 指向 ruckig 0.17.3、toppra 0.6.8 等 ABI 固定依赖的安装前缀。
# 默认读取本工作区 .deps/robo_manip；其他安装位置必须显式指定。
export HUMANOID_MOTION_SDK_DEPS_PREFIX=/path/to/robo_manip/dependencies

./bootstrap_colcon.sh build
./run.sh profile:=x1 mode:=sim
```

默认 `motion_backend:=profile` 读取机器人 profile；X1 和 OpenArmX 当前均默认
使用 `robo_manip`，无需显式指定。旧 KDL 后端已移除；独立扩展后端使用
`motion_backend:=external`，并自行启动符合 HC 消息契约的后端。
运行时仅 source ROS 2 和本工作区的安装环境，不再 source 外部 Humanoid underlay。

## X1 真机

厂商驱动继续由独立的 `HC_X1` 仓库启动，本仓库不会 source 或修改它。两边使用
同一个 ROS Domain；`run.sh` 和 HC_X1 均默认使用 14。

```bash
# 终端 A：真机硬件仓库
cd /home/maple/test/HC_X1
ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0 ./start.sh

# 终端 B：先做只读 shadow 验证，不会向 HC_X1 发布命令
cd /home/maple/test/HC-teleop-robotic
ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0 \
  ./run.sh profile:=x1 mode:=shadow motion_backend:=robo_manip

# 验证反馈、方向、限位和急停后才切换真机输出
ROS_DOMAIN_ID=14 ROS_LOCALHOST_ONLY=0 \
  ./run.sh profile:=x1 mode:=real motion_backend:=robo_manip \
  start_auto_lease:=true
```

`mode:=real` 会自动启动本仓库的 X1 兼容适配器，把 HC_X1 的
`/hc_teleop/joint_states` 转换为 `/robots/x1/state/joints`，并将唯一仲裁后的
强类型命令合并、校验后发送到 `/hc_teleop/joint_cmd`。`start_auto_lease:=true`
会在发现 PICO 会话后自动启用控制，只应在低速真机验收阶段使用。

## 新增机械臂与末端工具开发

无论是接入单臂、双臂、三臂还是不同自由度（3~7轴）机械臂与末端夹爪，均由 `URDF + profile.yaml` 统一配置驱动。详细接入步骤请查阅文档：
👉 **[新增机械臂与末端工具控制接入指南 (docs/ADD_NEW_ROBOT_GUIDE.md)](docs/ADD_NEW_ROBOT_GUIDE.md)**

