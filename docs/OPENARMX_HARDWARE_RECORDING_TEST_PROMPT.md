# OpenArmX 真机录制与上传测试执行提示词

日期：2026-10-16。将本文交给负责真机测试的终端执行。本文是测试要求，不代表真机测试已经通过。

请接手 OpenArmX 真实硬件环境下的数据录制和本机平台上传测试，实际执行并给出结果，不要只提供方案。

## 用户已授权的工作

- 检查已连接的 OpenArmX、CAN 和相机，进行静态数据录制。
- 沿用本机 `http://127.0.0.1:8088` 平台，登录并创建新的专用硬件测试任务，完成上传验证。
- 保留工作区现有未提交修改；必要修复完成后进行对应验证。

## 交接背景

统一接口的软件验收，以及 OpenArmX、X1 使用各自原有 URDF 的 PyBullet 录制、上传和对象读回校验已经通过；真机录制尚未验证。仿真上传为 `COMMITTED`，但 Episode/质检尚未完成，不能称为全链路验收完成。六轴模型只是测试 fixture，真实 OpenArmX 必须使用项目原有 URDF。

上一终端没有启动真机驱动或进行真机录制。请以当前源码、设备和运行状态为准，核实已有证据，不把仿真结果当作真机结果。

## 工作路径

- 工作区：`/home/hc_op/workspace`
- 当前主仓库：`/home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic`
- OpenArmX 驱动：`/home/hc_op/workspace/teleop_ws/src/hc_openarmx`
- ROS 环境：`/home/hc_op/workspace/teleop_ws/setup_hc.bash`
- 数据平台：`/home/hc_op/workspace/hc_DataPlatform`
- 既有验收资料：`/home/hc_op/workspace/teleop_ws/reports/unified-robot-api-20261015-165101`

先读取相关 `AGENTS.md`，以及主仓库中的：

- `docs/ROBOT_PUBLIC_API.md`
- `docs/hc-control-handoff.md`
- `docs/recording-workflows.md`

## 执行要求

### 1. 重新确认设备和运行状态

上一终端只读检查发现：`can0`、`can1` 均为 `DOWN/STOPPED`；USB 有两台 D405、一台 D435、两个 PEAK PCAN-USB FD。`ROS_DOMAIN_ID=14` 当时没有发现业务节点。这只是检查时的状态，请重新核实，确认左右臂 CAN 映射、相机序列号和配置后再启动。

### 2. 使用当前真实硬件链路

检查驱动启动是否会自动使能、回零或移动。此次先做静态录制，不主动发送回零、Servo 或轨迹指令；若读取反馈必须进入会产生运动的模式，先说明具体原因。

使用 `teleop_ws` 当前源码和正确 overlay；历史验收的新构建位于上述报告目录的 `install`。不要误用旧 `ws_teleop` 源码或默认安装目录中的旧版本。ROS 2 Humble 使用系统 `/usr/bin/python3` 或与其兼容的 Python 3.10 环境。

参照 `tools/run_robot_driver.sh` 使用统一接口生成的驱动参数，核对内部原始话题与公共话题的边界，避免原生关节名直接混入公共录制数据。录制不需要的控制服务无需启动。

### 3. 实际录制

确认真实关节反馈和启用的相机 RGB、深度数据持续更新后，通过现有 `RecordingService` 的正常预检查、启动、停止和落盘流程，先录制约 10～15 秒。

核验录制文件可读、关节名称与 profile 一致、数值有效、时间戳连续，以及实际启用相机的图像解码、尺寸、帧率、深度编码、同步和丢帧情况。

保存录制任务状态、metadata、ready 描述和日志；预检查失败时查明原因，不绕过检查。所有结果明确标记为真实硬件、静态录制。

### 4. 新建专用硬件测试任务并上传

不要复用仿真任务 `00000009`、`00000010`，也不要把测试数据混入原有业务任务或队列。

本机平台已有管理登录辅助工具，不要一开始就向用户索要管理员密码：

`/home/hc_op/workspace/hc_DataPlatform/backend/src/hc_data_platform/tools/local_storage_cli.py`

其中 `local_operator_client()` 可建立本机临时管理会话。参考既有验收目录中的 `platform-setup.py` 和 `platform-verify.py`，通过正常平台 API 创建任务和所需绑定。既有脚本在 `hc-data-platform-dev-api-1` 容器内调用该辅助工具；先核实当前容器状态。保持正常认证和权限校验，不重置管理员密码。

上传身份必须允许实际录制格式；凭据保存为私有文件，不打印到日志。不要直接复用仿真机器人身份，或修改现有业务绑定。

### 5. 验证真实上传结果

使用项目现有 `UploadService` 完成上传，检查本地队列、平台任务归属、上传回执和资产状态，并读回对象验证大小与 SHA-256。

分别报告录制成功、原始数据上传提交、Episode 生成、质检的实际状态。`COMMITTED` 不能代替 Episode 或质检通过；MCAP 不应调用仅支持 LeRobot V3 的 processing 查询接口。

### 6. 留下可复查结果

在 `/home/hc_op/workspace/teleop_ws/reports` 下新建独立的硬件测试目录，保存执行命令、设备映射、实际源码与构建路径、日志、录制统计、文件校验值、任务 ID 和上传 ID。

仅清理自己启动的临时测试进程，保留其他终端已有服务和未提交工作。不要提交、推送或自动部署本次修改。

请持续执行到取得明确结果。最后直接告诉我：

- 真机录制是否通过，实际覆盖了哪些关节和相机；
- 上传到了哪个任务，读回校验是否通过；
- Episode/质检到了哪一步；
- 还有哪些未完成及具体原因；
- 数据和报告的绝对路径。
