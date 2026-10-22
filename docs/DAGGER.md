# DAgger 人工接管与纠正采集

2026-10-22：从本地未上传的 humanoid 工作恢复到 main。父仓库来源为
`32e0350797c9ee73289aa0e96ffbdad29a01e51e` stash，Motion 子模块来源为
`92c5de19b29999dc9735802ad78336364b49b07c` stash。两个原 stash 均保留。

## 使用

需要带统一 Robot API 的机器人配置，以及已构建的 Motion 后端和 VR 接收器。
按 [统一接口说明](ROBOT_PUBLIC_API.md) 准备依赖；启动时显式设置
`HC_CONTROL_BACKEND=motion`，例如 `HC_CONTROL_BACKEND=motion ./run.sh sim`。
默认 v23 及原来的普通录制、回放、任务选择和上传流程继续保留。

1. 在“话题录制”中选择观测关节、相机等来源及保存格式，在“上传管理”中选择任务或本地保存。
2. 在概览的“人工干预与纠正采集”中开启会话。保持有效跟踪，松开右 Grip，等待采集资源就绪。
3. 点击“开始程序”。外部程序继续使用现有 ServoJ/ServoP/Move 接口。
4. 握住右 Grip 接管双臂；OpenArmX 支持带反馈的夹爪。正常松手自动交还程序。
5. 每次人工接管独立保存。暂停后接管再松手回到保持状态；跟踪丢失、断流和急停不会被当作松手恢复程序。
6. 退出会话后恢复普通录制。会话内 X/Y 普通录制按键停用；配置、任务和上传格式在会话期间固定。

DAgger 会话只开放双臂和支持的夹爪，腰部、头部、底盘保持。普通遥操作的腰部安全规则仍有效：恢复后需新的左 Grip 按下沿。

## 数据

沿用 main 的 `ros.recording.format`：`mcap`、`lerobot`、`both`、`mcap_mp4`。
`mcap_mp4` 使用 main 的 RGB MP4＋无损深度包；深度原始消息保留在 `raw.mcap` 中。
不需要 `workflow`、`capture_config_file`，也不依赖旧的 RecordingService 或相机标定模块。

每段包含控制会话、接管身份、epoch、源时间、最终 HC 发出的人工动作，以及采样缺口信息。
`intervention=true` 表示人工干预，`outcome=unknown` 不代表任务成功。
MCAP 同时保存事件和同名 `.correction.json`；MP4 深度包包含 `correction.json` 并将其列入资产校验。
LeRobot 导出包含 `intervention`、各动作轴有效性和原始时间信息，长缺口分开保留。
选择 LeRobot 前需准备官方读写环境，见 `tools/setup_lerobot.sh`。

片段使用 main 的 TaskRecorder 固定任务、方案快照和上传格式，后台整理后进入原上传流程；
未绑定任务时保留本地文件。空的预备槽不会作为录制上传。下载沿用现有录制列表入口。
本地不包含数据预览、标注、裁剪、审计和恢复工作台。

可选配置（省略时使用这些默认值）：

```yaml
ros:
  recording:
    corrections:
      prepared_slots: 4
      action_queue_items: 4096
      total_queue_bytes: 268435456
      observation_cache_bytes: 67108864
      observation_max_age_sec: 0.25
      late_message_sec: 0.2
      long_gap_sec: 0.2
```

控制切换使用不可变身份和实际确认，在最终发送前再次检查权限；写盘、编码和上传不在 Grip 回调内执行。
没有新增 ROS 消息类型，也不要求外部推理程序实现握手协议。

## 验证范围

测试覆盖真实 Motion 进程、HC 门控、驱动适配器及隔离的模拟输入，包含连续短接管、旧指令拒绝、
队列满时停止、相机深度原字节回读和既有功能回归。测试与恢复来源保存在本机
`/home/hc_op/workspace/reports/dagger-main-20261022/`。
这些测试不代表真机运动、真实 VR 链路和长时间高分辨率采集负载验收。

[原接管设计](DAGGER_CONTROL_TAKEOVER_DESIGN.md) 和
[原纠正录制设计](DAGGER_CORRECTION_RECORDING_DESIGN.md) 保留了 10 月 19 日的历史方案与验证记录；
main 的入口和目录以本文为准。
