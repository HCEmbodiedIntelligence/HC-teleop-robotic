# 数据服务：录制、数据集、派生作业和平台上传

`hc_platform.data` 是新的独立数据进程，不导入旧 `middleware` 或 `humanoid_upload` 运行时代码。控制循环不等待这个服务；ROS capture bridge 应用自己的有界队列提交样本。Python 3.10 可运行。

## 启动与进程边界

```bash
PYTHONPATH=platform/python /usr/bin/python3 -m hc_platform.data \
  --state-dir /var/lib/hc-platform/data \
  --socket /run/user/1000/hc-platform/data.sock
```

可选 `--platform-config /private/platform.json`，只允许凭据引用。可选 `--converter-python /opt/hc-lerobot/bin/python` 或环境变量 `HC_CONVERTER_PYTHON` 将 LeRobot 放在独立环境中。跨解释器转换只注入本项目 `platform/python`，不继承 ROS 或主进程的 ABI 专属 `PYTHONPATH`、`PYTHONHOME`。

可选依赖安装命令如下。安装器通过 `uv` 下载/创建 Python 3.12 环境，不依赖 ROS Python 3.10 的 `ensurepip`，不修改系统包。没有 uv 时给出明确错误；已有非本安装器创建的目录会拒绝覆盖。

```bash
PYTHONPATH=platform/python /usr/bin/python3 -m hc_platform.data.install_converter \
  --venv .venv-lerobot-platform
export HC_CONVERTER_PYTHON="$PWD/.venv-lerobot-platform/bin/python"
```

安装默认使用仓库 `platform/requirements-converter.lock` 锁定全部依赖，包含 LeRobot 0.4.3、MCAP 1.6.0、mcap-ros2-support 0.5.7 和 CPU PyTorch；环境内另保存实际 `hc-converter.lock` 和验证状态。缺锁文件会拒绝无锁安装。`--dry-run` 仅展示命令。主进程无需安装 Torch、PyArrow 或 LeRobot。该环境已通过真实 X1 命名状态/动作、RGB 视频、断流掩码、官方 v3 reader 的集成测试。

| 执行单元 | 并发及职责 |
| --- | --- |
| jobs-data 主事件循环 | Unix IPC、状态机、异步 HTTP 上传、作业协调 |
| `data-sqlite` 专用线程 | SQLite 唯一连接和写者；WAL、FULL 同步、短事务 |
| `hc-mcap-writer` 子进程 | 每个会话一个 MCAP 写者；有界消息数和字节数；独立磁盘故障域 |
| 转换/视频子进程 | 同时最多一个 CPU 派生作业；终止整个进程组，包含 FFmpeg |
| HTTP 上传协程 | 默认同时两个作业；每个作业逐资产、逐分片传输 |
| 文件 IO 线程 | 哈希、复制、封口、打包在 `to_thread` 中，离开事件循环 |

状态目录有进程级 `flock`，第二个 service 实例明确返回 `DATA_WRITER_EXISTS`。不要给 API 的每个 worker 单独实例化 `DataService`；API 使用 `DataClient`。

## API/IPC 契约

```python
from hc_platform.data import DataClient

data = DataClient('/run/user/1000/hc-platform/data.sock')
session = await data.start_session({
    'robot_id': 'x1',
    'config_hash': 'sha256-of-effective-config',
    'local_format': 'mcap',
    'mode': 'ordinary',
    'selected_topics': ['/joint_states', '/hc/control/command'],
}, request_id='operator-start-42')
await data.append_sample(session['id'], '/hc/control/command', {
    'name': ['left_joint_1', 'left_joint_2'],
    'position': [0.1, 0.2],
    'control_epoch': 7,
})
result = await data.stop_session(session['id'])
await data.close()
```

也支持 `await data.call('get_dataset', {'artifact_id': identity})`。底层协议是 UTF-8 JSON 行：

```json
{"id":"request-uuid","method":"status","params":{}}
```

响应 `{"id":"...","result":...}` 或 `{"id":"...","error":{"code":"...","message":"...","retryable":false}}`。socket 权限为 0600；这不是网络认证接口，远程权限由 console-api 管理。最大请求 24 MiB，样本 body 最大 16 MiB。大图像的持续高吞吐应在 capture bridge 做合适的采样/编码，不能把 UI WebSocket 当图像总线。

| 方法 | 关键参数/返回 |
| --- | --- |
| `status()` | capabilities、active_session_id/state、recorder_pid/alive、recording_root、作业统计；statistics 含 accepted/written/dropped/errors/queue_bytes/queue_limit_bytes/topics/last_error/statistics_stale |
| `start_session(spec, request_id)` | 冻结 JSON 配置；重复同 request_id 返回原会话，内容冲突拒绝 |
| `append_sample(session_id, topic, payload, source_time_ns=None, sample_monotonic_ns=None)` | 非阻塞入有界队列；普通录制返回 accepted/sequence，DAgger 返回 accepted/buffered/armed |
| `set_intervention(session_id, active, event)` | 已确认的控制所有权边界，见下文 |
| `stop_session(session_id, cutoff_monotonic_ns=None)` | 排空/封口/发布；DAgger可传已确认审计屏障的单调截止时间；派生格式返回 POSTPROCESSING |
| `get_session / list_sessions` | 录制状态及错误 |
| `list_datasets(limit=100, offset=0)` | 默认不返回大 manifest；summary 含 robot_id、size_bytes、duration_s、message_count、frame_count、topic_count、camera_count、任务/质量信息 |
| `get_dataset(id)` | 完整 manifest、同样的 summary、video_assets 和 references；`list_datasets(include_manifest=True)` 用于内部恢复 |
| `import_mcap(path, request_id, spec=None)` | 校验并复制历史 MCAP；调用方负责本机文件读权限 |
| `enqueue_conversion(artifact_id, request_id, options)` | LeRobot 持久作业 |
| `enqueue_video(artifact_id, request_id, options)` | MCAP+MP4 持久派生作业 |
| `enqueue_upload(artifact_id, request_id, target)` | target 必须明确 robot_id 和 collection_task_id |
| `get_job / list_jobs / job_events` | 持久状态、尝试次数、结果和错误 |
| `cancel_job / retry_job` | 取消或显式重试；原 UUID 和不可变 manifest 保留 |
| `acquire_artifact(id, owner, ttl_seconds=300)` | 下载/预览/回放租约，最长 3600 秒，长任务要续租 |
| `resolve_asset(id, path, owner)` | 只允许 manifest 内路径及活跃租约；拒绝穿越和符号链接 |
| `release_artifact(id, owner)` | 释放读引用 |
| `prepare_download(id, owner=None)` | 生成 TAR，返回路径/字节数/租约；API 流式发送后释放租约、清理下载文件 |
| `delete_dataset(id)` | 事务检查引用并 tombstone；使用中的资产拒绝删除 |

批量操作由 API 对这些幂等单项接口按有限并发调用，逐项返回结果；不能用一个失败掩盖其余结果。

`recording_id` 可由控制台提前生成 UUID 并传给平台 prepare-recording，随后传入 spec；省略时按 request_id 稳定派生 UUID。存在 capture_snapshot 时其 recording_id 必须完全一致。录制 `filename` 为安全 basename，默认 `capture.mcap`，省略扩展名会补 `.mcap`；文件名真正用于 MCAP 落盘和后续读取。`save_subdirectory`（兼容 `selected_save_dir`）只能位于 `--state-dir/artifacts` 内，支持 `task/day1` 这样的相对子目录。全局保存根由启动参数配置，HTTP 不能指定外部任意目录。

status 的写入量来自子进程共享计数器，读取不等待磁盘或慢 RPC；计数器短暂不可取时返回最近快照和 statistics_stale。writer退出、写盘错误和溢出会明确显示 PARTIAL。

## MCAP 和 ROS 消息

MCAP `profile=ros2`。普通 JSON 控制事件编码为 `std_msgs/msg/String` 的 CDR，string 内容为 `hc.sample/v1` JSON envelope。ROS capture bridge 传入以下结构时直接保留原始 CDR，不再次 JSON 包装：

```python
payload = {
    '__cdr_b64': base64.b64encode(serialized_ros_message).decode(),
    '__ros_type': 'sensor_msgs/msg/JointState',
    '__ros_schema': full_ros2_message_definition_with_dependencies,
}
await data.append_sample(session_id, topic, payload, source_time_ns=published_ns)
```

channel encoding 为 `cdr`，schema encoding 为 `ros2msg`。完整消息定义须包含依赖，桥接层不能仅传消息类型名字。同一个话题录制中改变类型/定义会使录制变为 PARTIAL。普通 JSON 中的 NaN/Infinity 被拒绝。

`log_time` 使用采集端接收墙钟；`publish_time` 使用调用方提供的发布时间，未知时取接收时间。JSON envelope 另存接收单调时间。不要把多台机器的单调时钟直接相减；传感器自身时间保留在原 ROS header，转换默认根据接收时间建立因果时间轴。这里没有假装实现跨设备时钟同步。

队列默认 4096 消息、64 MiB，两个限制同时生效，schema 仅发送一次并计入预算。录制队列满时返回 `RECORDING_QUEUE_OVERFLOW`，会话标记 PARTIAL，已采集文件保留，不发布 READY 产物。控制进程不能等待磁盘腾出空间；是否暂停模型由任务的“录制必需”策略在控制层决定。

## DAgger 边界

`spec.mode` 为 `dagger_corrections` 或 `dagger_full` 时必须明确 `auto_record=true`。普通录制与 DAgger 共用一份会话租约，无法并开。

corrections 会话启动后处于 ARMED，订阅先建立，并持续保留有限的重排窗口。默认 `reorder_window_ms=500`（范围 10–10000ms），窗口同时受 queue_messages/queue_bytes 限制，上限 64MiB；最终纠正文件只写入已确认的人工区间。接管时调用：

```python
await data.set_intervention(session_id, True, {
    'event_id': 'core-epoch-7-human',
    'confirmed': True,
    'control_epoch': 7,
    'monotonic_ns': actual_switch_time_ns,
})
```

释放时传 active=False 及新的已确认 epoch。原始 Grip 请求不能冒充这个事件。事件在窗口内按实际 monotonic_ns/epoch 排序，允许结束事件先到、开始事件后到；重复幂等，同 ID 内容冲突拒绝。序列必须最终交替且 epoch 单调增长。边界写入 `/hc/control/intervention` 和 interventions.json，返回 active 是排序后的状态。

来源时标优先使用核心 issued_mono_ns/mono_ns 和 capture 的 received_monotonic_ns；DataClient会在请求连接锁之前填 sample_monotonic_ns，避免同连接前序请求耗时改变样本所属区间。本地直调没有可靠时标的样本会计入 sample_clock_unknown 排除数，不能伪装成人工纠正。整个会话用固定的同机 monotonic→wall 映射写 MCAP log_time，原始发布时间仍保留。CDR消息不修改，逐消息 `<filename>.samples.jsonl` 以 topic/channel_sequence 关联源时间、控制epoch/来源/stage和人工区间。晚到的自动命令会按样本时间、来源和epoch排除，即使其到达时人工正在接管。

窗口后的样本形成提交水位；迟于该水位的 ownership 事件、未能补齐的乱序事件、capture gap 或窗口溢出会使录制 PARTIAL，不发布 READY 数据集。没有静默丢弃边界事件。stop_session 在不持session锁的情况下等待一个窗口接收尾部事件；runtime还必须先停止capture并消费核心审计屏障之前的事件，不能只依赖超时猜测“不会再有迟到消息”。显式截止时间后样本不进入最终纠正集。尚未收到释放事件的区间进入 manifest 的 incomplete_intervention，不编造正常释放；task_success始终为null。

## 文件发布与恢复

```text
PREPARING → RECORDING/ARMED → FINALIZING → READY
                                 └──→ PARTIAL
READY(raw) → POSTPROCESSING → READY / DERIVATION_FAILED
```

写入 `staging/<id>.partial/`，完成时关闭并 fsync MCAP，读取验证，计算资产 SHA-256/CRC64，写原子 manifest，再同文件系统 rename 到 `artifacts/[save_subdirectory/]<id>/`，最后登记数据库。manifest 记录不可变配置/任务归属、资产清单、统计和来源。

控制台注册表生成完整 URDF/Mesh/参数 ZIP，并覆盖 `spec.robot_package_archive`，不能信任 HTTP 提交的源路径；可同时提供 `robot_package_archive_sha256`（ZIP 字节摘要，不是 package_hash）。数据服务在准备阶段通过文件线程复制为 `robot_package.zip`，校验源文件稳定性、摘要和ZIP路径，记录 `manifest.session.robot_package`。MCAP 及派生 LeRobot/MP4 产物均封存自己的 ZIP，原配置包删除或迁机后仍能取回；引用的绝对运行路径仅为采集时信息。

文件系统 rename 与数据库事务不是跨资源原子事务。重启扫描已发布 manifest 并校验以补登记；没有完成封口的录制变为 INTERRUPTED，partial 文件保留供修复。原来 RUNNING 的作业回到 QUEUED 并按同 UUID 继续；已请求取消的作业恢复为 CANCELED。删除的 tombstone 同样在启动时完成。

独立回放/下载进程可能比 jobs-data 活得更久，因此重启保留尚未到期的读租约，直到调用方续租/释放或TTL到期；不会因数据服务重启而允许删除仍在使用的产物。回放在校验/规划阶段也有独立的续租任务，长文件校验不影响租约保持。

LeRobot-only 也先使用 MCAP 作为恢复源；当前策略保留该恢复源，不自动销毁原始数据。界面应把 requested_format 和实际/派生产物分别显示，禁止将“停止录制”显示为“训练数据已就绪”。

`local_format` 和 `upload_format` 独立：本地 MCAP + 上传 LeRobot 会为上传生成派生作业；本地 LeRobot + 上传 MCAP 会立即排队源 MCAP 上传。所需本地产物生成失败显示 DERIVATION_FAILED 并保留源；仅上传所需转换失败不把已就绪的本地 MCAP 改成失败。自动上传目标、任务快照和平台身份/凭据引用在 start_session 固定，后续连接修改不会改变这段录制的目标。发布后派生/上传路由是可恢复的持久作业；重启会补齐发布与入队之间的中断。

## LeRobot v3 转换

```python
await data.enqueue_conversion(artifact_id, 'convert-request-42', {
    'joint_names': ['left_joint_1', 'left_joint_2'],
    'state_topic': '/joint_states',
    'action_topic': '/hc/control/command',
    'fps': 30,
    'max_age_ns': 200_000_000,
    'task': 'pick up the cup',
    'robot_type': 'x1',
    'cameras': {'observation.images.head': '/head/rgb'},
    'only_interventions': False,
    'lerobot_version': '0.4.3',
})
```

`lerobot_version` 可选，若和环境版本不同明确失败。自动及历史转换默认从冻结 robot_config 的 allowed_groups、joint_names 和 joint_units 建立命名映射，状态/动作默认为 `/platform/control/state`、`/platform/control/commands`，只将实际录到的已配置 RGB 相机加入映射；历史 MCAP 没有这些元数据时必须明确给出 joint_names 和话题。JSON 关节数据接受 `name/names/joint_names` 和 `position/positions/values`；ROS `JointState` 直接解码。图像支持 RGB/BGR/RGBA/BGRA/mono8 和 JPEG/PNG，深度和其他格式明确拒绝。

worker 使用固定 FPS 因果保持，超过 max_age 或缺必需流则切断 episode，不能用零或过期样本当有效数据。所有采样 tick 的缺失/过期和 presence/validity 掩码进入 sample-provenance.jsonl；有效训练帧包含 observation.source_valid 和 observation.source_age_s，保留原时间、所选话题、控制来源/epoch/stage，采样规则进入 conversion.json。正常收尾调用官方 writer.finalize()，用官方 reader 重开，父进程再验证 Parquet footer、总行数和视频后发布。跨环境的 validator 也在同一固定解释器执行。缺 LeRobot/PyArrow/图像依赖时作业 FAILED/unavailable，不生成伪造的 v3 目录。

`action` 的语义是选定命令话题里的 commanded target，采集方必须选择仲裁/安全限制之后的命令话题。驱动接受结果和真实关节反馈应另录；下发命令不自动解释为实际执行。

## MCAP + 独立 MP4

`local_format=mcap_mp4` 在录制封口后自动启动 video 作业。也可对历史 MCAP 调用 enqueue_video。options 可指定 `topics`、`fps`、`max_age_ns`、`max_cameras`、`max_output_frames`。

媒体在独立 FFmpeg 子进程编码 H.264，源 MCAP 保留。每路 MP4 有 timestamps.jsonl：输出帧、视频时间、接收时间、原发布时间、年龄、重复和有效标记。固定 FPS 使用因果保留上一帧；断流产生的重复/过期帧明确标记，绝不能作为连续完整采集。没有图像流、分辨率变化、编码失败或帧预算超限都会失败并保留源。完成时用 ffprobe 验证每路尺寸和帧数，再发布资产。

此离线保存编码当前使用 libx264 CPU 后端；实时视频硬件编码和后端回退归 media service，不能把本 worker 的能力显示成 NVENC/VAAPI 已验证。

## 平台协议与凭据

```python
await data.configure_platform({
    'platform_base_url': 'https://platform.example',
    'credential_env': 'HC_PLATFORM_TOKEN',
    'robot_id': 'platform-x1-id',
    'part_size_bytes': 8 * 1024 * 1024,
})
await data.platform_status()
await data.platform_tasks({'source_format': 'MCAP', 'search': 'cup'})
```

也支持 `connect_platform(link, local_robot, request_id)` 对 `#robot-connect=...` 链接执行公开 `connections:redeem` 协议；票据不存盘，尝试 ID 持久化，token 写入 0700 目录下的 0600 文件。`platform_preflight` 检查任务可选性，`platform_prepare_recording` 冻结 prepare 请求，验证返回任务/机器人/录制/版本及快照摘要。调用方须将准备结果固定到开始录制的 spec，不能在录制中改投任务。

上传独立实现 `/api/v1/robot-ingest/uploads`、`:authorize-parts`、资产 `:complete`、上传 `:commit`。每份作业冻结目标、配置引用、内容及分片规划。同 UUID 的 POST/状态对账解决创建、PUT 和 commit 回包丢失；已确认分片不重传。对象存储 PUT 不携带平台 bearer，不跟随重定向，不允许 HTTPS 降级到 HTTP。HTTP 仅 loopback 默认允许，明确信任 LAN 时才配置 allow_insecure_lan。

状态 `QUEUED → RUNNING → SUCCEEDED` 只表示本地作业/传输；`result.remote_state`、`processing_status`、`quality_status` 分开显示。调用 `refresh_upload(job_id)` 查询新状态；配置 processing_endpoint=true 时支持 LeRobot 的 `/processing`。不把 COMMITTED 映射为平台质检通过。取消只保证本地不继续传输；远端可能已经提交，返回 remote_cancelled=false 并保留对账信息。

网络/408/429/5xx 最多自动尝试 10 次，指数退避加抖动；认证、内容、身份错误需要显式修复/retry。凭据轮换只更新相同 env/file 引用，不重建作业。历史资产改投任务需新 request_id/new upload job，原始 session 归属不变。

`platform_status` 返回所配置的 `robot_id` 供历史上传默认身份使用，不返回密钥，不能用本地 `x1` 替代平台UUID。prepare返回 `plan_revision/plan_sha256/snapshot_sha256/recording_id`；上传 target 中的 capture_plan 直接使用这四个固定引用，完整 capture_snapshot 不作为上传 HTTP manifest 的替代字段。

## 独立回放进程

console-api 持有 `ReplayManager(runtime)`，提供 `start(artifact_id, mode='observe', topics=None, mapping=None, speed=1, loop=False, groups=None, include_grippers=False, include_base=False)`、`pause/resume/stop/status/set_speed/close`。它经 supervisor 启动 `python -m hc_platform.replay --job ...`；工作进程自己持有 replay 角色的核心连接、数据集租约和0600管理socket。status返回 PREPARING/RUNNING/PAUSING/PAUSED/RESUMING/STOPPING/COMPLETED/STOPPED/FAILED，以及 elapsed_s/duration_s/messages/total_messages/loop_count/speed/error/requires_explicit_recovery。

观察回放通过ROS动态消息类反序列化原CDR，默认发布到 `/replay/<原话题>`；映射也限定在此命名空间。按需流式读MCAP，不把视频全部载入内存，ROS缺依赖/未知消息类型明确失败。需要控制核心idle，开始时软件stop，结束后仍需显式recover。

驱动回放只接受经过core_gate的 `/platform/control/commands`。要求录制与运行的机器人/config_hash/关节名称及rad、m单位/`hc-control-ipc/v1`版本完全匹配；不读取原lease/seq/epoch作为授权。独立worker取得新session与tokens，用新序号、sample_mono_ns和短TTL发送。准备阶段核对反馈、关节限位、全轨迹速度/加速度及起止静止条件；loop要求严格起终点重合。默认仅选中授权关节组，夹爪和底盘需显式启用，非零底盘速度还要求显式 base_acceleration_limit 及起终点静止。

配置启用 collision_avoidance_enabled 时，在后台线程用独立模型检查每一轨迹段；缺完整实时反馈、碰撞依赖或计算预算会拒绝。开始和恢复前重新验证反馈/所有权，暂停后改变身体位置会触发剩余轨迹重新碰撞检查。该检查针对已加载机器人模型；不能视为未知环境障碍物已建模。驱动倍速只允许在PAUSED状态修改，并重新预检运动限制；调度卡顿不追发旧命令，而是停止。

start入口立刻可见PREPARING，并在读取资产前固定boot/epoch。stop可取消尚在读取或规划中的start，不等待准备锁；正式session_start前再次核对固定令牌，worker从不自行recover。工作进程结束/失败/退出只停止其持有的控制session，解除数据集引用后保持软件停止，等待操作者明确恢复。console-api须在全部活动phase阻止录制、配置apply、普通session_start和外部recover，以保持回放与实时控制互斥。

## 验证与实际边界

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=.platform-deps:platform/python \
/usr/bin/python3 -m pytest platform/tests/test_data*.py platform/tests/test_replay.py -q
```

测试包括真实 MCAP/CDR 读回、独占写者、Unix RPC、队列溢出、/dev/full、DAgger延迟和乱序裁边/水位迟到拒绝/尾部排空、资产引用/路径隔离、配置ZIP独立副本、封口失败、作业重启/取消、本地和上传格式独立路由、真实 loopback HTTP 丢 PUT/commit 回包后对账、真实 FFmpeg 派生和逐帧时间 sidecar。LeRobot集成从实际X1配置创建MCAP，包含RGB图像、命名重排和故意断流，用官方v3 reader重新取帧；安装环境缺失时该项明确skip。回放测试使用真实C++核心、mock反馈后端、独立数据/回放进程，覆盖新授权、暂停/倍速/恢复、结束需recover和prepare-stop竞态。

测试中的平台是协议测试服务器，不能据此宣称生产平台权限、真实网络或平台质检已经验证。LeRobot已在本机固定环境完成真实产物验证，部署机仍需安装并运行同一测试。回放未据此宣称真机轨迹已经验证。相机时间采用接收时钟映射，不宣称已完成硬件同步或实测标定。
