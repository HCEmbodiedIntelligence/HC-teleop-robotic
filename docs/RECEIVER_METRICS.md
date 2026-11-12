# 接收端遥测与延迟分位数

服务端原有 RTP 输出只代表本机传输调用完成。新增接收端自报指标独立标为 `receiver_*`，不能将其解释为已确认的物理显示或相机到头显端到端延迟。`receiver_end_to_end_ms`、`receiver_physical_display_ms` 在缺少同步测量时保持 `null`。

## 接入浏览器

先完成 WebRTC offer/answer，取 answer 的 `telemetry_session_id`，加载 `/static/receiver_metrics.js`，在实际接收视频的浏览器调用：

```javascript
const telemetry = TeleopReceiverMetrics.start({
  pc, video, sessionId: answer.telemetry_session_id,
  url: `/api/webrtc/receiver/${encodeURIComponent(cameraId)}`,
  onError: error => console.debug('receiver telemetry:', error.message)
});
// 关闭此接收会话时调用，不会自动关闭 pc：
telemetry.stop();
```

`pc` 是接收视频的 RTCPeerConnection，`video` 是播放该视频的 HTMLVideoElement。采集器每秒取一次 `getStats()`，只选择一个视频 inbound RTP 流。浏览器不支持的字段保持缺失，不能作为零值。自动收集支持浏览器的 `requestVideoFrameCallback` 回调；不支持时相应分位数保持未知。静态接收页由服务端集成提供。

`receive_to_render` 指浏览器上报 `expectedDisplayTime - receiveTime`，是接收至预计合成显示的估计，不是源相机到物理显示。`render_gap` 是视频帧呈现回调间隔；后台标签页限速也可能增加间隔。`processing` 来自浏览器 `processingDuration`。这些指标以浏览器单调时钟计算，不混用机器人 wall clock。

## 原生 Unity / PICO

实际工程位于 `/mnt/B81666081665C7C8/Users/maple/HC-Teleop`，本次已直接更新：

- `Assets/CameraFrameViewer.cs`：读取可选 `telemetry_session_id`，SDP answer 生效后开始当前相机遥测；复用现有每秒单次 GetStats，不新增并行 native stats 请求；关闭/重连时取消遥测。客户端 codec 偏好同时保留 `video/rtx`，避免头显自行过滤 RTX。
- `Assets/CameraFrameViewer.Telemetry.cs`：会话隔离、最多每秒一次 HTTP POST、三秒超时、单槽最新报告队列；当前请求未完成时不叠加请求，网络异常不影响视频/控制。`ClosePeerConnection()` 中停止协程、Abort/Dispose 请求并清空待发报告。

确认依据是工程内已安装的 `Library/PackageCache/com.unity.webrtc@3.0.0-pre.8/Runtime/Scripts/RTCStats.cs`：`RTCStats.Dict` 为公开字典，未定义 native stats 字段返回 `null`，而 typed getters 会返回默认零。适配器通过字典检查实际存在且已定义的有限数字，只发送已定义计数，秒转毫秒，不用默认零冒充缺失能力。新报告使用 `Stopwatch` 单调时钟并保证 `sample_ms` 递增；重复/倒退的同一 native stats 时间戳不重复上报。旧中间件未返回会话标识时仍可播放视频，仅关闭遥测。

此版本包的 `VideoStreamTrack.OnVideoReceived` 仅在首帧/尺寸变化时触发。Unity 相机渲染回调会重复渲染同一视频纹理，不能证明新视频帧已显示；所以原生客户端**不发送 `frames_rendered`、渲染间隔或物理显示时间**，仪表盘中的渲染 FPS 保持未知。真实接收/解码/FEC相关计数取决于设备提供的 native stats；PICO 若冻结 native counters，即使返回时间戳继续更新，也无法靠这些计数证明真实画面已冻结。当前应用自己的 FPS 文案不等于物理视频呈现测量。

本次没有启动 Unity、没有构建/安装 APK，没有提权、没有修改场景、原字体或发布清单。源码检查和 C# 语法解析通过不等于 Unity 编译或 Android 设备验收通过。原 `HC-Teleop.apk` SHA-256 为 `804cfe905a0b294c3c07f9a296af9c254b2b55447f29814d1c419aded6e28682`。

### 用户自行编译与设备验收

1. 使用工程 `ProjectSettings/ProjectVersion.txt` 所列 **Unity 2022.3.62f3c1** 打开上述工程，等待 Package Manager 解析，确认 WebRTC 为 **3.0.0-pre.8**；Console 必须无 C# 编译错误。不要升级插件来绕过报错。
2. 在 `Assets/CameraFrameViewer.cs` 及新增 partial 文件处确认 `reportReceiverMetrics` 默认启用。保持现有 Android/PICO 项目设置及主场景，先用编辑器或测试设备验证信令。
3. 用户决定构建时，通过工程已有菜单 **HC-Teleop → Build Release APK**（实现为 `Assets/Editor/HCTeleopReleaseBuild.cs`）。输出为 `Builds/HC-Teleop-v<当前版本>.apk`，不覆盖根目录 `HC-Teleop.apk`。该步骤由用户执行，本次未调用。
4. 用户选择测试头显后可自行 `adb install -r <上一步输出的APK绝对路径>`。启动中间件与头显，打开实际相机窗口；在机器人端读取 `/api/camera/status`，应看到该 stream 的 `receiver_reported=true`、`receiver_stale=false`、非空会话标识，支持设备上解码/抖动缓冲数值更新。不支持的字段应为 null，原生渲染 FPS 应为 null。
5. 重连同一路相机，确认会话标识变化且旧会话报告不被接受；多相机分别核对各自 ID，关闭一个窗口不能使其他窗口遥测停止。关闭应用/相机后五秒以上应 stale；再次开启应从新会话基线重新计算，不能沿用旧 FPS。
6. 临时让遥测 HTTP 返回失败或超时，确认最多三秒超时、持续单请求、视频/控制继续；恢复后无需重建视频连接即可恢复上报。先在静止、无机器人动作的场景验证，再做负载对照。
7. 使用受控视频源/外部高速相机校验真实显示和端到端延迟。如需判断 FEC，应另外记录 RTP 丢包时序/连续丢包长度；聚合 native loss counter 不能证明随机或突发丢包。

### RED / ULPFEC 接收能力与限制

Unity 接收端现从 `RTCRtpReceiver.GetCapabilities(TrackKind.Video)` 读取实际 native 接收能力，筛选时保留 H.264、VP8、RTX、RED 与 ULPFEC。正常编码偏好仍为 H.264 优先、VP8 后备；FEC 试验应由服务器明确选择 VP8。没有硬编码 payload type，没有向 SDP 注入本机未宣告的能力；设备最终 offer 未包含 `red` 和 `ulpfec` 时，发送端必须回退并说明原因。保留辅助 codec 不代表 FEC 已经工作，必须同时检查最终协商、真实纠错包输出及接收计数。

该工程包的 README 明确 `3.0.0-pre.8` 基于 **libwebrtc M116**；C# `RTCInboundRTPStreamStats` 中存在 `fecPacketsReceived`、`fecPacketsDiscarded`。适配器按原先的 Dict defined/null 规则映射为 `fec_packets_received`、`fec_packets_discarded` 累计计数；不支持时不发送。ARM64 Android 原生库也包含 ULPFEC receiver/generator 及对应统计标识，但静态字符串存在不能替代运行时能力与恢复测试。

**本版本应以 VP8 + RED/ULPFEC 作为 FEC 试验组合，不能把 NVENC/H.264 + NACK 宣称为已支持的 FEC 路径。** 与 M116 对应的 [libwebrtc branch-heads/5845 全链路测试](https://webrtc.googlesource.com/src/+/refs/branch-heads/5845/video/full_stack_tests.cc) 明确将 H.264 + ULPFEC 组合标为不支持并禁用了相应测试；[发送端实现](https://webrtc.googlesource.com/src/+/refs/branch-heads/5845/call/rtp_video_sender.cc) 在启用 NACK 且 codec 缺少 picture ID 时禁用 RED/ULPFEC，VP8/VP9 不受该条件限制。这是整个 libwebrtc 组合的兼容性限制，不能仅因 SDP 带有 FEC 名称就绕过。

[M116 接收路径](https://webrtc.googlesource.com/src/+/refs/branch-heads/5845/video/rtp_video_stream_receiver2.cc) 的 ULPFEC 构造和恢复入口本身没有简单的 H.264 拒绝分支；因此本次结论不是“所有 H.264 FEC 包绝对无法恢复”，而是“现有版本未提供可验收的 H.264 + NACK + ULPFEC 完整组合”。未改 C# 或原生解码器，不通过关闭 NACK 或升级 native 库来偷偷改变此限制。

设备验收时，应在 VP8 试验会话核对 offer/answer 的 RED、ULPFEC 和实际 payload type；确认 `fec_packets_received` 增加只是证明收到纠错包，不等于恢复了视频包。`fec_packets_discarded` 也不是恢复包计数。需要受控丢包对照和接收解码连续性数据，才能证明恢复收益；不能据此声称彻底消除卡顿。原有 APK 仍保持原哈希，需用户自行构建新版本才能在头显运行此源码改动。

## 服务端协议

`ReceiverMetrics.reset(session_id)` 开启新会话，`ingest(payload)` 接收当前会话报告，`snapshot()` 返回平铺 `receiver_*` 字段。旧会话拒绝，重复/倒序 sequence 返回 false；格式错误抛 ValueError。调用方仍须限制 HTTP 请求体大小，不对公网匿名开放写入口。

必需字段：

| 字段 | 含义 |
| --- | --- |
| `session_id` | offer 对应会话标识 |
| `sequence` | 从零递增的报告序号，最大 JS 安全整数 |
| `sample_ms` | 客户端单调时钟毫秒，必须递增 |
| `stats_id` | inbound RTP 统计对象 ID；切换流时必须变化 |

可选累计计数：`packets_received`、`packets_lost`、`fec_packets_received`、`fec_packets_discarded`、`frames_received`、`frames_decoded`、`frames_dropped`、`frames_rendered`、`freeze_count`、`freeze_duration_ms`、`jitter_buffer_delay_ms`、`jitter_buffer_emitted_count`、`decode_time_ms`。时间量单位均为毫秒。`packets_lost` 可因迟到包修正而下降，此时该区间不报告丢包率，不能伪造零丢包。

可选逐帧数组：`render_gap_ms`、`receive_to_render_ms`、`processing_ms`，每个报告每项最多 240 个有限非负样本，单样本不超过 60 秒。禁止布尔值、NaN、无穷大和超界值。

`frames_rendered` 为累计视频帧呈现回调计数，其差分生成 `receiver_render_fps`，不通过回调间隔倒数估算。服务端按计数差分计算 FPS、区间丢包、每帧平均解码和抖动缓冲时间。首个报告、SSRC/stat ID 切换、计数重置或超过一分钟的间隔不产生对应计数差分。保存最近 30 秒、最多 120 份报告；超过 5 秒未收到报告时 `receiver_stale=true`，即时数值回到 null，不展示旧数值为当前状态。

`receiver_freeze_count`、`receiver_freeze_duration_ms`、`receiver_frames_dropped` 是窗口内报告区间的增量总和，不是会话累计计数。窗口起止按报告收取时刻归属；边界区间不会插值。`receiver_loss_pct` 为窗口内丢包增量 /（接收增量＋丢包增量），不能反推出连续丢包长度。`receiver_loss_pattern` 明确为未知；FEC 决策需要额外 RTP 序列/丢包时间分布证据。

分位数采用 nearest-rank，无样本返回 null：

- 本机两秒窗口：`webrtc_submit_to_transport_p50_ms/p95_ms/p99_ms`、`webrtc_transport_p50_ms/p95_ms/p99_ms` 和 `webrtc_latency_sample_count`。
- 接收端三十秒窗口：`receiver_render_gap_p50_ms/p95_ms/p99_ms`、`receiver_receive_to_render_p50_ms/p95_ms/p99_ms`、`receiver_processing_p50_ms/p95_ms/p99_ms`。

所有接收端数据均为不可信客户端自报，`receiver_metrics_source=client_reported`；校验只能保护服务器资源与状态，不能证明客户端测量准确。

## 验证

```bash
PYTHONPATH=.deps:. /usr/bin/python3 -m unittest discover -s tests -p 'test_receiver_metrics.py'
PYTHONPATH=.deps:. /usr/bin/python3 -m unittest discover -s tests -p 'test_camera_metrics.py'
node tests/test_receiver_metrics.cjs
```

单元测试覆盖差分、流切换、会话/序号隔离、过期、缺失字段、有界存储、非法数值与分位数；JS mock 覆盖 getStats 映射、单位转换、呈现回调和停止清理。实际头显浏览器的字段支持及显示性能仍需设备验收。
