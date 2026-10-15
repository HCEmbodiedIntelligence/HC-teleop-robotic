# 录制、数据工具与上传交接

录制入口仍为 `/api/recording/start`、`stop`、`precheck`、`status`。HTTP、网页和 VR 使用同一 RecordingService。VR 命令通过有界串行后台入口执行，MCAP 写入、raw 封存、LeRobot 编码不在控制回调中运行。停止返回已接受；通过作业状态判断源关闭和产物完成。

## 配置

旧配置缺省 `workflow: hc`，继续支持 `mcap`、`lerobot`、`both`。不改变机器人默认 profile，不自动启用上传。

```yaml
ros:
  recording:
    workflow: hc
    format: both
    directory: ../runtime/topic_recordings
    job_state_dir: ../runtime/recording_jobs
    artifact_state_dir: ../runtime/artifact_ready
    capture_config_file: ""
    max_pending_jobs: 8
    min_free_bytes: 268435456
    action_sources: []
    gap_rules: []  # 例如 [{topic: /observation/joint_states, max_gap_seconds: 0.2}]
```

所有相对路径相对 middleware 配置文件目录。录制格式等非路径配置可保存供下一段使用；已有作业保留配置快照。运行路径的变化要求先结束采集和后台收尾。没有配置历史库。

`platform_raw` 必须给出 `capture_config_file`，内容沿用迁入 capture.config.validate 的完整 JSON/YAML 配置。storage.directory/export_directory 相对该采集配置文件。保留 `retain_original_ros`、`depth_video_encoding`、既有缓冲/磁盘/编码限制及三种媒体策略。raw 配置不由 HC 的 format/fps 覆盖。独立相机节点及自定义消息包需要正确安装，不能靠旧 manager Python 环境补依赖。

subscriptions 增加 `required: true|false` 与 `recording_mode: continuous|event`。缺省 `/action/`、`/control/`、`/status/` 为 event，其余 continuous。必需 continuous 源检查新鲜度/频率，event 不要求操作者先运动。开始仍需至少一个选择的源；服务统一预检，不使用 HTTP force 绕过。消息缺失和整段无动作不补造 action。

OpenArmX 可显式使用以下附加动作映射；有效期是数据采样新鲜度规则，须按 A 的实际指令语义校准，不代表电机采用或执行：

```yaml
action_sources:
  - topic: /action/left_hand/joint_states
    names: [openarmx_left_finger_joint1]
    units: [m]
    max_age_sec: 0.2
  - topic: /action/right_hand/joint_states
    names: [openarmx_right_finger_joint1]
    units: [m]
    max_age_sec: 0.2
```

action 主向量末尾按配置顺序附加夹爪；每个来源保留独立 availability、age、valid 掩码及原 topic。缺失值为带掩码的占位值，过期不变成已执行值。`meta/teleop_sync.json` 保存名称、单位、映射和 upstream_command_not_execution 语义。深度不进入有损 RGB 编码；原始深度留在 MCAP/raw 既有格式中。

## 身份、恢复与接口

每段在启动写入前持久生成 recording_id，路径带此身份，独立 IPC 队列、写入进程和标记状态。停止后独立排空；单个后台 worker 串行封存/导出，允许下一段录制。后台积压和磁盘限制拒绝新段，源文件保留。进程中断的活动源标为 interrupted，不自动就绪；正常关闭后的后台任务可以重启继续。

- `GET /api/recording/jobs`：capture_state、artifact_state、错误、冻结 profile/config、产物身份。
- `POST /api/recording/jobs/{recording_id}/retry`：仅重试源已完成的失败收尾任务。
- `POST /api/recording/jobs/{recording_id}/seal`：提交不可变绑定并立即返回 job_id/state。
- `DELETE /api/recording/artifacts/{artifact_id}`：通过共享上传锁删除完成产物；含 raw 整包。

`request_raw_seal(recording_id, binding_snapshot)` 绑定字段为 binding_id、platform_base_url、robot_id、collection_task_id，不接收凭据。源完成后未绑定状态为 awaiting_binding。原始 session_id、raw 标记版本与原生身份算法保留；已有源 metadata.capture_context.local_robot_id 优先保留，没有时使用本次实际 profile_id。平台 local_robot_id 配置必须与此一致。更改绑定不能覆盖已封存作业。

ready 描述在独立 artifact_state_dir 原子写入，绝不放进 raw 包。采集开始/停止请求的 UTC 时间保存为 capture_started_at/capture_ended_at，与源消息时间和 completed_at 分开。MCAP 与 LeRobot 独立 artifact；LeRobot 关联 source_artifact_id。恢复/裁剪是新身份，默认未绑定，不自动归属新任务。文件完成不表示平台质检通过。

上传端实现来自 C：UploadService、register_upload_routes、humanoid_upload.artifacts.ArtifactLocks。B 在删除临界区调用 cancel_artifact 并取得独占 delete 锁，worker 持读锁时拒绝删除。持久 tombstone 和锁文件保留。没有上传模块时本地录制继续运行；删除已封存产物仍需要共享锁包，不能退化成一次状态检查。

## 数据工具

统一页面“录制作业与数据工具”支持 MCAP 概要、浏览器预览、播放/暂停/位置/速度、备注、valid/invalid/note 标记、裁剪副本、审计和恢复副本。预览从不调用 TopicPlayer 或 ROS 发布。原机器人回放继续独立走原有权限/ready 门控。

- `GET /api/data/{filename}` 与 `/frames`：只读概要/消息。
- `POST /api/data/{filename}/edit`：需要最新 etag，源身份变化或多窗口覆盖明确失败。
- `POST /api/data/{filename}/{audit|export|recover}`：返回后台 job_id。
- `GET /api/data/jobs`：持久任务状态；中断任务标为 interrupted。
- `POST /api/data-preview/{play|seek|pause|stop}`：只读播放器。

兼容 `.mcap.edit.json`、`.mcap.session.json`、`.mcap.quality.json` 和旧嵌入元数据命名。有效标记不覆盖 invalid 区间。原字节/时间戳保留；缺少自定义解码类型仍可字节裁剪；CRC/数值/倒退/间隔审计结果绑定源身份。恢复不依赖概要读取成功；只保留完整可读前缀，新副本明确可能不完整。raw 整包和 LeRobot 不冒充单 MCAP 裁剪对象。

## 安装与验证

`./install.sh --capture` 安装 requirements-capture.txt，并在 C upload_client 已合入时安装其原包。普通 HC 基础依赖仍在 requirements.txt，官方 LeRobot 写入器继续使用 `./tools/setup_lerobot.sh` 的独立 0.4.3 环境（LEROBOT_PYTHON 可显式指定）。控制器、相机节点、自定义 ROS 消息按 A 的唯一来源构建安排安装。

迁移来源及许可证见 `middleware/core/capture/ORIGIN.md`。旧源码保留。`tests/capture` 保留 raw/原子相机/媒体/深度/动作证据/历史共享模块边界测试。相机测试通过 HC_CAMERA_SOURCE 指定保留的相机项目。G0 外部冻结 fixture 缺失时显式跳过相关两个对照测试，不伪造样本。

联调仍需用户安排真机停止/断流、长时资源压力和实际平台处理窗口。离线合成数据不构成硬件、物理同步或平台质检验收。
