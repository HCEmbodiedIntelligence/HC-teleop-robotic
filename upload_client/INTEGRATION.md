# 给 A/C/E 的挂接需求

A：保持 G0 schema；最终目录含关闭、验证、fsync 后原子发布的 marker。完成通知是本机 root + marker，不向 API 发绝对路径。D 默认扫描补偿，不要求 A 同步入队；导出或任务内容变更必须新 export_id，已排队路径不可覆盖。D 未修改 recording、runtime、server 或控制实现。A 的实际导出另需 A/D 共同验证，公共 synthetic fixture 不是 A 产品产物。

C：继续 robot-ingest/v1 严格 envelope、分片授权/complete/commit/GET；相同 robot/task/client_upload_id 内容一致返回原 upload。GET /uploads/{id}/processing 按 G0 proposed schema；机器人凭据只能读自己的 upload。逐段终态必须与 aggregate、quality、terminal、poll_after_seconds 一致。客户端保留成功段并区分 PARTIALLY_FAILED。已观察到开发中的 `POST /uploads/{id}:retry-processing {request_id: UUID}`，D 持久化 request_id 对账；此扩展需 C/E 确认交付。未消费 diagnostics 扩展，发现阶段无 episode 时显示聚合失败并提示平台排查。

C 真实联调所需：已启动的公开 API URL、外部可达签名 URL、与包冻结值匹配的合成 robot/task、凭据引用、服务端版本、真实 Worker 就绪；完成后出具同一 client_upload_id 的 Raw 数量=1 和两个预期源 episode 的持久化证据。不能用内存 service 或手动 apply_processing_result 代替。

E：安装独立 wheel 并管理 `run` 进程。首次部署默认 processing_endpoint=false；C 路由部署后设为 true。Web 只需在 app 装配处调用：

```python
from humanoid_manager.web.platform_upload_api import register_platform_upload_routes
register_platform_upload_routes(
    app,
    executable="/绝对路径/install/uploader/bin/humanoid-upload",
    config_path="/绝对路径/artifacts/upload-config.json",
)
```

页面地址 `/platform-upload`；E 在总入口加导航。模块提供 GET/PUT `/api/platform-upload/config`、GET `/api/platform-upload/queue`、POST `/api/platform-upload/queue/{client_upload_id}/retry`，body 可含 `resume: true` 或 `processing: true`。CLI 子进程执行队列操作，网络、hash 与 SQLite 不在 ROS 回调；Web 不导入 uploader 包。保留 manager 的认证、同源/CSRF 防护；上传配置/重试不应持有控制或采集公共操作锁。配置路径和可执行路径由部署代码固定，不能来自 HTTP 请求。禁止在 ROS callback 或 runtime.status() 同步调用 CLI。

目前 retry 要求先停独立 worker；否则返回 WORKER_ALREADY_RUNNING，不抢占活跃传输。E 可在后续进程管理入口提供 stop→retry→start，首版也可按 README 命令操作。不要在网页主线程等待控制循环停止来执行上传。

资源边界：D 的安装/build/log/queue/artifacts 独立，原 teleop_ws 与 U1 固定源码、配置及安装产物保持原样。开发未启动电机或部署 manager 公共页面。E 验收项：A 实际导出、C 真实基础设施、不同机器 IP/签名/TLS、真实控制/采集指标，以及 U5/U6 真人闭环。
