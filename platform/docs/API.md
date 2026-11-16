# 对外 API

控制台与程序共用 HTTP `:7876`，OpenAPI 位于 `/docs`。写操作从本机访问可用；远程需 `HC_PLATFORM_TOKEN` 与 `Authorization: Bearer <token>`。浏览器跨源写操作被拒绝。普通消息发布还要经过话题白名单，不能直接发布执行器命令。

## 控制与能力

| 入口 | 用途 |
| --- | --- |
| `GET /api/status` | 运行模式、核心、VR、后端、IK、录制和自己启动的进程 |
| `GET /api/control/state` | 轻量、当前 core 状态，用于外部实时输入 |
| `GET /api/robot/info` | 名称、分组、SI 单位、限制、命令接口版本与能力 |
| `GET /api/observations` | 当前关节状态、已配置观察话题和相机元数据 |
| `POST /api/control/operations` | `recover/stop/pause/resume/session_start/session_end/set_dagger/reference_reset/home` |
| `POST /api/control/command` | `program` 或 `exo` 的短生命周期命名关节目标 |
| `PUT /api/dagger/recording` | 会话之间设置自动录制策略 `enabled/mode/required` |
| `WS /api/ws` | 最新系统状态；有界客户端数量和发送期限 |
| `WS /api/topics/ws` | 配置话题的最新观测与接收指标 |

操作示例：

```json
{"op":"session_start","mode":"program","groups":["right_arm"],"dagger_enabled":true,"request_id":"caller-unique-id","expected_boot_id":"from-state","expected_epoch":3}
```

软件停止先进入 core，再清理慢任务，不需要等待录制、转换或下载。HTTP 202 表示核心已接受操作；控制切换是否完成应观察 `transition_pending/owner/epoch`，不能仅凭 HTTP 状态认定物理动作完成。

关节命令携带当前状态中的 `boot_id/epoch/lease_id/reference_epoch`、递增 `seq`、`ttl_ms`、`joint_names` 和 `positions`。本地单调时钟域可提供 `sample_mono_ns/deadline_mono_ns`，两者不能延长目标寿命。跨机器建议用 [SDK](../python/hc_platform/sdk.py)，不要直接把客户端的 monotonic 时间当服务器时间。来自 HTTP 入口且没有采样时间的旧兼容调用，以 API 接收时刻作为可验证的时间边界。

不会在命令失败后自动 recover、resume 或重发旧动作。调用方须重新查询控制权，并生成新的算法动作。DAgger 的人工接管期间模型可以继续推理，但输出权威会拒绝无权来源。

## Motion

先设置 `control_backend=motion`（系统配置，或启动环境 `HC_CONTROL_BACKEND=motion`）。只有 `program` 会话且对应分组就绪时允许执行。

| 入口 | 命令 |
| --- | --- |
| `GET /api/motion/capabilities`、`/info` | 命令语义、分组、坐标系、限制与几何检查能力 |
| `POST /api/motion/servo-j` | ServoJ 命名关节目标 |
| `POST /api/motion/servo-p` | ServoP 末端目标 |
| `POST /api/motion/move-j` | MoveJ 关节轨迹任务 |
| `POST /api/motion/move-l` | MoveL 笛卡尔直线路径 |
| `POST /api/motion/move-p` | MoveP 点位 IK + PTP |
| `GET /api/motion/tasks[/{id}]` | 执行阶段、进度、反馈和终态 |
| `POST /api/motion/tasks/{id}/cancel` | 取消并等待撤销；终态要求显式 resume |

详细载荷与语义见 [Motion README](../python/hc_platform/motion/README.md)。每个任务冻结原始控制世代；暂停、接管、停止、换配置后不能继续输出旧轨迹。

## 配置、设备与相机

`GET /api/robots` 列出内置/导入机型及校验问题。`POST /api/robots/{id}/import` 接收 ZIP multipart；`POST .../apply`、`GET .../export`、`DELETE ...` 管理版本。内置包、当前包和被数据引用的包不可删除。

`GET /api/config` 返回实际运行配置与持久 overrides。`POST /api/config/validate` 只校验候选，`PUT /api/config` 应用覆盖配置。失败回滚；成功后仍停止。模型拓扑、Mesh、手部标定变更通过新配置包管理。

`GET/PUT/DELETE /api/topics` 管理录制/网页/UDP 目的及 QoS；`GET` 同时返回实时发现和频率。`POST /api/messages/publish` 发布白名单外部话题。`WS /api/messages/ws` 的远程/令牌模式首帧为 `{"authorization":"Bearer ..."}`，随后每帧为 topic/type/message；没有执行器旁路。

相机：`GET /api/cameras`（头显兼容 `/api/camera/status`）、`GET /api/cameras/{id}/snapshot`、`.../preview`、`POST /offer` 或 `/offer/{id}`。offer 使用标准 SDP JSON，支持 `camera_ids` 选择 1–4 路；答复的 `telemetry_session_id` 用于 `POST /api/webrtc/receiver/{id}`，接收端统计也有序号和会话校验。

## 数据与平台

`POST /api/recordings/preflight` 检查格式、话题、类型和必需来源。`POST /api/recordings` 冻结 `topics/format/filename/save_subdirectory/task_description/dataset_fps`；可另选 `upload_format/auto_upload/platform_task_id`。`POST /api/recordings/stop` 封口，`.../mark` 保存用户标记。

`GET /api/datasets` 返回轻量摘要，`GET /api/datasets/{id}` 返回完整 manifest/features。`POST /api/datasets/import` 导入 MCAP。单个产物可 `convert/upload/delete/download`；`POST /api/datasets/download` 接收 `artifact_ids` 批量打包。视频通过 `.../asset?path=...` 预览，路径受 manifest 限制。

`GET /api/jobs` 查询持久队列，`POST /api/jobs/{id}/cancel|retry|refresh` 操作作业。refresh 用于取平台处理结果，不能把传输完成推断为质检通过。

`GET /api/platform/status`、`PUT /api/platform/config`、`GET /api/platform/tasks` 和 `POST /api/platform/preflight` 管理连接、任务与方案。配置接受连接 `link`，或 `platform_base_url` 加 `credential_env/credential_file`；凭据文件需 0600，不在响应/日志中回传凭据。

回放：`POST /api/replay/start` 接收 `artifact_id, mode=observe|drive, topics, mapping, speed, loop`；`pause/resume/stop/speed` 独立操作，`GET /api/replay/status` 返回进度。drive 要求配置/接口/单位匹配并重新取得租约，aux 夹爪和底盘须显式选择。结束后保持停止，需要明确恢复。

运行日志通过 `GET /api/logs?session=&level=&subsystem=&q=` 查询，`/api/logs/export` 导出 JSONL，`/api/log-sessions` 列出会话。`GET /api/diagnostics` 汇总网络端口、ROS 隔离、进程、磁盘、编码与链路统计；`POST /api/runtime/restart` 重启自己创建的控制组件。仿真入口另有 `/api/runtime/simulation/start|stop|restart`。
