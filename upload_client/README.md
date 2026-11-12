> 单链接连接、按任务名称录制和自动上传请使用机器人页面，详见 [平台任务录制](../docs/PLATFORM_TASK_RECORDING.md)。以下手动字段与 CLI 保留用于旧部署维护。

> HC 迁入版本 1.4.0：普通 HC 产物、服务/API、面板和文件锁接线请先阅读 [HC_INTEGRATION.md](HC_INTEGRATION.md)。以下保留原客户端协议说明；旧 manager 路径和 G0 验收记录属于来源实现，不是本轮集成通过证据。

# 独立机器人上传器 v1

消费 A 的最终 `export-complete.json`，通过公开 robot-ingest/v1 HTTP API 上传。包依赖 Python 3.10+、jsonschema 和 mcap（用于共享方案元数据一致性检查）；不导入 ROS、LeRobot、平台代码、数据库或 Temporal。schema 基于 G0 `g0-20260922.1`，本次 HC 扩展增加可选 capture_plan 引用和独立处理回执状态；原始上游哈希与扩展后的文件哈希分别保存在 `humanoid_upload/schemas/provenance.json`。服务端协议由协调终端集成发布。

## 安装与启动

以下命令从 D/robot 执行；交付 wheel 和离线依赖放在 `artifacts/g0/{dist,wheelhouse}`。另一台机器人只需复制这些 wheel、配置和完成包，不需要本仓库或平台仓库。离线 wheelhouse 本次验证为 Linux x86_64 / Python 3.10；其他平台按 `requirements.lock` 下载相应 wheel。

```bash
cd /home/hc_op/workspace/openarm-integration/D/robot
uv venv --python /usr/bin/python3 install/uploader
uv pip install --python install/uploader/bin/python --no-cache --no-index \
  --find-links artifacts/g0/wheelhouse --find-links artifacts/g0/dist \
  humanoid-upload-client==1.0.0
cp src/humanoid_manager/upload_client/config.example.json artifacts/upload-config.json
# 编辑公开 URL、机器人映射、冻结任务、本地导出与队列目录。
# credential_ref 使用 env:OPENARM_PLATFORM_TOKEN 或 file:/绝对路径。
# 文件必须由运行用户拥有，权限为 0600。不要把 token 写入配置 JSON。
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json check-config
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json run \
  >> log/uploader.jsonl 2>&1
```

`-I` 避免继承主机 ROS/PYTHONPATH。使用环境凭据时由操作环境注入变量；凭据文件无需 shell 注入。企业代理环境中应按实际网络设置 NO_PROXY；客户端保留代理策略、验证 TLS、不跟随重定向、不改写签名 URL 的 hostname。远程平台签发 loopback URL 会给出 `UPLOAD_URL_NOT_EXTERNAL`。

仅处理一次、手工通知和查看状态：

```bash
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json scan
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json enqueue /绝对路径/完成包
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json step
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json status
```

`run` 每 30 秒扫描完成 marker，补偿导出后通知前崩溃；失败目录和无 marker 的半成品不入队。`enqueue` 是独立进程命令，**不能在控制回调里同步执行**。A 无需等待它：完成包的持久 marker 是事实来源。

## 恢复与状态

网络/5xx/429/PUT 签名过期自动进入 `retry_wait`，1/2/4/8/16/32/60 秒退避、抖动，并遵守 Retry-After。每次恢复先 GET upload，或在 create 响应丢失时用原 UUID 重放创建；授权接口返回的分片清单决定续传。PUT/complete/commit 结果不确定时不生成新 UUID。分片大小冻结在队列中。反复 409 停止并要求人工核对。

进程中断后直接重启相同 `run` 命令；最迟在持久化的 `next_at` 到期后恢复。`step` 也遵守这个时间，不代表强制立即联网。队列 worker 使用 OS flock 单实例锁，SIGKILL 后自动释放；SQLite WAL、FULL synchronous 与唯一键确保原子入队。保留整个队列目录，勿只复制运行中的 sqlite3 主文件。磁盘满/不可写会输出 `QUEUE_IO_FAILED` 并退出，不回调机器人控制。

上传与处理分别展示：`raw_committed` 不等于 `ready`；READY/PENDING、质量风险、质检拒绝、部分失败均不能显示就绪。终态停止平台轮询。CLI 回执和 Web 页面保留源 session/export/episode、Raw/平台 episode/dataset、错误码及下一步。它们不会删除任何原始或导出资产。

修复凭据或平台故障后先停止 worker，再操作并重启；下面的 ID 是 `status` 输出的 client_upload_id：

```bash
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json retry UUID
# 仅针对服务端 PAUSED：
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json retry UUID --resume
# C 的逐 episode / 重试接口部署且 processing_endpoint=true 时：
install/uploader/bin/python -I -m humanoid_upload.cli --config artifacts/upload-config.json retry UUID --processing
```

普通 retry 对已提交 Raw 只重新查询。`--processing` 持久保存 request_id 后调用 C 的 `:retry-processing`，响应丢失沿用同一 request_id。CANCELED/FAILED 上传不自动新建；会话过期需由平台确认既有事实后处置。编辑内容必须新 export_id；切换任务必须显式生成冻结新任务的新包。修改全局配置不会改变已排队条目的 URL/身份/任务/凭据引用/分片规划。

队列不复制大文件，只保存不可变清单；入队、发送前和 commit 前复核完整字节。入队后的包必须保留在原路径且禁止修改；本地变更停止上传，新内容须重新导出。客户端校验 hash/CRC、完整文件清单、身份/任务、profile 维数、源映射有效性与 MP4 容器收尾；不包含视频解码和 Parquet 训练读取器，A 的导出验收与 B 的内容校验仍是必需的。C 校验整个对象后才允许 commit。

单个 JSON 上限 16 MiB；文件 hash 与 PUT 按最多 64 KiB 流式读取，单 worker，不编码视频。默认网络 4 MiB/s、校验读取 16 MiB/s，进程 nice +10；示例 systemd 另提供 CPU/IO 配额。参数不代表实机实时性合格线。

## 测试与交付接口

```bash
cd /home/hc_op/workspace/openarm-integration/D/robot
# 首次准备离线依赖（开发环境）：
.venv/bin/python -m pip download --dest artifacts/g0/wheelhouse \
  -r src/humanoid_manager/upload_client/requirements.lock
bash src/humanoid_manager/upload_client/scripts/test-client.sh "$PWD"
```

测试使用 G0 公共 fixture/schema/状态示例，HTTP 契约服务会校验逐资产字节、CRC、manifest 和身份。Bubblewrap 安装测试只挂载系统 Python、独立 wheels 和样本，文件系统没有 `/home` 和平台源码；明确断言平台/数据库/工作流/ROS 包不存在。压力测试是独立进程的 100 Hz/30 Hz 模拟调度器，不是实际 ROS/电机验收。

真实 C 联调入口（C/E 先提供与封存包一致的机器人身份、任务、受限凭据及可达 API/签名 origin）：

```bash
install/uploader/bin/python -I src/humanoid_manager/upload_client/scripts/real-service-smoke.py \
  --config artifacts/c-real-config.json --export-root /绝对路径/完成包 \
  --output artifacts/g0/c-real-receipt.json
```

此脚本不访问数据库。Raw 唯一性及预期 episode 集合需 C 在真实持久层独立出具证据，不能用本地队列数量代替。契约服务的成功不得标记为真实 C 联调成功。

E 接线说明见 [INTEGRATION.md](INTEGRATION.md)。部署、真实跨机/IP、TLS、真人和真机负载验收由 E 统一安排。


共享采集方案（2026-10-21）：新平台任务先只读预检，实际开始前通过 prepare-recording 固定方案。完整 capture_snapshot 保存在本地录制、MCAP 元数据、HC LeRobot capture-context、ready descriptor 和队列；上传 manifest 只携带四项 capture_plan 引用。旧封存和旧队列不补写当前方案。无 annotation_task_id 的成功处理回执有效，数据可读、标注和训练交付分别显示。

隔离合同测试命令：`PYTHONPATH=.:upload_client:middleware:tests python -m pytest tests/test_shared_capture_plans.py -q`。测试使用合成 MCAP、可读 Parquet 和隔离 HTTP 服务；不证明生产平台、真实驱动或官方 LeRobot writer 已完成集成。
