# 视频编码后端与码率反馈

摄像头配置的 `encoder_backend` 可设为 `x264`（默认）、`auto`、`nvenc` 或 `jetson`。

- `x264`：保留 CPU ultrafast/zerolatency、Baseline、无 B 帧、intra refresh 和 250 ms VBV。
- `auto`：仅当 device tree 识别为 Jetson 时优先尝试专用 GStreamer 管线，失败再尝试 NVENC，最后回退 x264；普通 NVIDIA 独显不会被当作 Jetson。
- `nvenc`：尝试 PyAV/FFmpeg 的 `h264_nvenc`，以第一张真实视频帧进行试编码；只有成功产出码流才标为 NVENC。缺少编码器、驱动、设备或低延迟参数不受支持时回退 x264，保留具体原因。运行期间 NVENC 编码失败同样回退并发送恢复关键帧。首次探测在编码 executor 内，避免阻塞服务事件循环。
- `jetson`：使用原生 `appsrc(I420) → nvvidconv → NVMM NV12 → nvv4l2h264enc → h264parse → appsink` 管线。第一张原始帧必须真正产生 Annex-B 码流并通过 SPS/PPS、Constrained Baseline/level ≤ 3.1 和 IDR 校验，才显示实际后端为 Jetson。管线创建、缺少插件/驱动、超时和运行期编码失败均释放管线并回退 x264，保留具体原因。

NVENC 使用 p1、ultra-low-latency、CBR、零 lookahead、禁用 B 帧和小 VBV。H.264 SDP profile/packetization 和头显解码兼容性仍需现场验证。当前 SDP 与编码器声明 level 3.1，配置会拒绝超出级别宏块/帧率上限或奇数宽高的 H.264 输出，例如支持 1280×720@30，不支持 1920×1080 或 1280×720@60；更高规格需要同时扩展协商与编码器级别。

## REMB 到实际编码预算

旧实现仅更新 `target_bitrate`，已打开编码器的实际码率没有变化。本环境 PyAV 17.1 / libx264 实测，给 live `codec.bit_rate` 赋值也未改变实际输出，不能只检查属性值。

当前策略在目标相对已配置码率变化至少 20%、距离上次配置至少 2 秒时，重建 x264/NVENC 编码器，实际更新 ABR 和 VBV。Jetson 插件如果声明 bitrate 可在 PLAYING 修改，则动态设置并读取确认；旧版插件未声明运行时可改时，也使用相同的限频重建策略。期间不断收到的反馈合并为最新目标，不逐包重建。分辨率变化立即重建。新上下文的恢复帧由小 VBV 约束，仍需要观察实际网络上的关键帧突发。2 秒为限频折中而非对所有无线环境的最优值。

仪表盘应区分反馈目标码率、已配置码率和实际输出码率；使用 `actual_backend`、`requested_backend`、`fallback_reason`、`configured_bitrate` 和 `reconfigurations` 排查回退与重配。

## 验证

在项目根目录运行：

```bash
PYTHONPATH=.deps:. /usr/bin/python3 -m unittest tests.test_encoder_backends tests.test_camera_streaming tests.test_jetson_h264 -v
```

码率回归测试编码真实运动噪声帧，先设置 2.5 Mbps 再降到 0.5 Mbps，比较稳态输出字节数，并验证只发生一次重配置。另测快速/小幅反馈、任务间后端隔离和硬编失败后的真实 x264 输出。

NVENC 成功路径必须在支持 NVENC 的机器上验收：确认实际后端没有回退，同时比较 CPU、编码耗时、冻结时长和头显端到端延迟。仅列出 FFmpeg 编码器名称不能证明硬件可用；本机软件测试通过也不能替代 GPU 和头显验收。

参考：[NVIDIA FFmpeg 硬件加速说明](https://docs.nvidia.com/video-technologies/video-codec-sdk/13.1/ffmpeg-with-nvidia-gpu/index.html)、[Jetson GStreamer 编码接口](https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)。

## Jetson 安装与实际健康探测

使用能够导入 JetPack 系统 GI 的 Python。需要与设备 L4T/JetPack 版本匹配的 `nvidia-l4t-gstreamer`，以及 `python3-gi`、`gir1.2-gstreamer-1.0`、`gir1.2-gst-plugins-base-1.0`、GStreamer base/bad 插件。检查 `gst-inspect-1.0 nvv4l2h264enc`、`gst-inspect-1.0 nvvidconv`，并确认容器或宿主机实际暴露编码设备。健康工具给出缺失项与安装方向，不会自动执行 sudo 或安装包。

Jetson Orin Nano 模块没有硬件视频编码器，安装 JetPack 不能补上硬件；应使用 x264。载板名称可能与所装模块不同，设备树用于区分平台，最终以实际编码是否成功为准；发现编码设备时仍允许实际探测。

```bash
PYTHONPATH=.deps:. /usr/bin/python3 tools/diagnostics/check_jetson_encoder.py
# 仅在真实 Jetson 和匹配 JetPack 上运行成功路径集成测试：
HC_TEST_JETSON_ENCODER=1 PYTHONPATH=.deps:. /usr/bin/python3 -m unittest tests.test_jetson_h264.RealJetsonTests -v
```

健康工具在有外层总超时的子进程中编码真实合成图像、请求恢复 IDR，并用 PyAV 软件解码全部输出帧，返回单个 JSON。成功退出码为 0，不可用/失败/超时为 1；本机非 Jetson 会明确报告 `hardware_available: false`，不会因为有桌面 NVIDIA GPU 就宣称 Jetson 硬编可用。

生产管线每次只允许一个输入帧在途，appsrc 一帧字节上限、非阻塞 push，appsink 最多保留两帧；首帧等待最多 3 秒，之后每帧最多 0.5 秒。超时后该实例立即失效并关闭，不能继续消费迟到帧。PLI/FIR 通过标准 force-key-unit 事件请求下一张 IDR，并验证实际返回的 NAL 类型。原始 I420 按 GstVideo.VideoInfo 的 plane stride/offset 打包，包括 642 等色度平面需要 padding 的偶数宽度；appsrc 字节上限使用实际布局 size。输入 PTS 保留为纳秒，输出 AU 必须匹配输入 PTS，以保持现有 RTP 90 kHz 时间线。连接结束或重配时管线进入 NULL，释放硬件资源。

底层驱动若在原生状态切换内卡死，Python 帧等待超时不能强行中断驱动；健康工具的外层子进程超时与现有独立视频进程退出机制提供隔离。此实现将 CPU 原始帧复制到 NVMM，并非摄像头到编码器零拷贝。

当前开发主机具有 GStreamer 1.20，但不是 Jetson，且没有 `nvvidconv` / `nvv4l2h264enc`；已验证缺失时的诊断、回退及 mock 管线契约。真实 Jetson 上的驱动、强制 IDR、码率响应、端到端延迟和头显兼容仍应执行上面的硬件探测与现场验收。

参考：[NVIDIA profile/level、码率、SPS/PPS 与 VBV 配置](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)、[GStreamer appsrc](https://gstreamer.freedesktop.org/documentation/app/appsrc.html)、[appsink 超时拉取](https://gstreamer.freedesktop.org/documentation/app/appsink.html)。

Jetson 输出约束为 Constrained Baseline，兼容当前 aiortc 的 Baseline/Constrained Baseline SDP 候选。新版 JetPack 支持 profile=1 时选用该模式；旧版使用 profile=0 时仍必须通过 caps 与 SPS constraint_set1 检查，否则明确回退，避免输出与 SDP 不一致的码流。Orin Nano 的硬件限制见 [NVIDIA 官方说明](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/SoftwareEncodeInOrinNano.html)。
