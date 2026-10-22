# HC 通用机器人客户端

`robot_api_demo.py` 在线读取 `/status/robot/info`，支持 describe、subscribe、连续 ServoJ/ServoP、home 和固定关节 MoveJ。OpenArmX、X1 和六轴测试模型共用相同客户端，不需要品牌、profile 或厂商 SDK。

```bash
source /home/hc_op/workspace/teleop_ws/setup_hc.bash
cd /home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic
/usr/bin/python3 examples/robot_api_demo.py describe
/usr/bin/python3 examples/robot_api_demo.py subscribe --duration 5
```

有意控制时，先由机器人操作方选择 motion、确认 ready 并显式恢复输出。demo 不自动授予控制权：

```bash
/usr/bin/python3 examples/robot_api_demo.py servo-j --group left_arm --joint left_joint_4 --delta 0.02 --duration 2
/usr/bin/python3 examples/robot_api_demo.py servo-p --group left_arm --dx 0.005 --duration 2
/usr/bin/python3 examples/robot_api_demo.py home --group left_arm
/usr/bin/python3 examples/robot_api_demo.py move-j --group left_arm --joint left_joint_4 --delta 0.02 --scale 0.1
```

Servo 默认 50 Hz，检查新鲜反馈和许可代际；退出后停发，由服务端 lease 停止。MoveJ 在异常或 Ctrl+C 后取消并确认终态。`servo-p --pose x y z qx qy qz qw` 使用本机 frame，`--delta-xyz dx dy dz` 或 `--dx/--dy/--dz` 在测量 FK 基础上偏移。所有有运动的命令使用在线描述；`describe --profile PATH` 仅用于离线检查。

接口、时间戳／QoS、公共名称、profile 迁移及启动职责见 [对外接口说明](../docs/ROBOT_PUBLIC_API.md)。Motion 子模块中的原生品牌示例仍使用旧命名空间和固定轴数，应使用本目录示例接入统一 API。

验收使用 `tests/robot_api_fake_rig.py` 启动实际 Motion/IK/RTC、HC 边界、OpenArmX adapter 或通用驱动；仅硬件控制器与测量由假硬件提供。`test_unified_robot_api_ros.py` 验证三模型主闭环、VR 等价输入、来源／恢复／回位和所有在线 demo，并另测非单位映射及改动驱动连接；`test_unified_robot_api_aux_ros.py` 验证实际 VR 辅助功能、开启碰撞和 MCAP 回放。测试固定 localhost domain 217，锁定并检查空域，分进程运行；既有 HTTP 回归在 domain 216。

本次构建、依赖、完整复测脚本、A1–A12 结论及日志位于工作区 [unified-robot-api-20261015-165101](../../../reports/unified-robot-api-20261015-165101/RESULT.md)。这不包含真机、CAN、跨电脑 DDS、Isaac 图形仿真或外部 X1 硬件项目验收。历史 68 项使能修复报告保留在 `reports/robot-api-20261015/`，不计作此次新增功能的验证。

报告内的 `sim-record.sh openarmx` / `sim-record.sh x1` 使用 localhost domain 218、两机型原有 URDF 和仓库 PyBullet DIRECT 节点，实际录制约 7 秒公共状态、动作及 FK。`sim-upload.py` 使用独立队列上传到本机专用任务，`platform-verify.py` 从对象存储完整读回核对 SHA-256。没有相机录制；平台 MCAP 接收成功与后续 Episode/质检分开报告。
