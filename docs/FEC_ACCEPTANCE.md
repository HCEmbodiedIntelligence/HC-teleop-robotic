# VP8 + RED/ULPFEC 配置与验收

已实现 RFC 2198 单块 RED 和 RFC 5109 ULPFEC level 0、16 位短掩码的发送链路。它在 RTP 加密前生成校验，经原有 DTLS/SRTP/ICE 发出；头显使用原生 libwebrtc 恢复，无需自行编写 C# 解码器。发送与接收链路不修改安装的 aiortc 依赖。

**本版只支持 VP8 + RED/ULPFEC。** 本项目头显使用 Unity WebRTC 3.0.0-pre.8 / libwebrtc M116，其[全链路测试](https://webrtc.googlesource.com/src/+/refs/branch-heads/5845/video/full_stack_tests.cc)明确把 H.264+ULPFEC 列为 unsupported；[发送端](https://webrtc.googlesource.com/src/+/refs/branch-heads/5845/call/rtp_video_sender.cc)也会因没有 picture ID 而禁用 H.264 的 NACK+ULPFEC。Jetson/NVENC 仍用于 H.264+RTX；本版无法在同一路上同时使用 Jetson H.264 硬编和 FEC。不会未经指示自动将 H.264 改为 VP8。

## 开关与协商

默认关闭。在 Dashboard 勾选 FEC 后，在需要保护的相机行明确选择 VP8，并保存相机配置；原生头显必须重新打包安装本次更新的源码。配置等价示例：

```yaml
camera:
  enabled: true
  codec: VP8
  fec:
    enabled: true
    group_size: 5
    media_bitrate_ratio: 0.8
  streams:
    - id: head
      topic: /observation/cameras/head/color/compressed
      codec: VP8
      enabled: true
```

已有 `streams[].codec: H264` 会覆盖全局 codec，必须修改实际行。单路 `streams[].fec` 可整对象覆盖全局 FEC 设置；没有显式覆盖则继承全局。`group_size` 为 1–10，`media_bitrate_ratio` 为 0.25–0.9。配置项不决定设备实际能力，最终以本会话状态为准。

只有远端 offer 中实际提供的 RED/ULPFEC payload type 才会出现在 answer 中。不修改全局 CODECS，不伪装 aiortc 拥有接收 FEC 的能力。原生头显通过 `RTCRtpReceiver.GetCapabilities` 保留实际能力；未提供 RED/ULPFEC 的旧客户端继续普通 VP8/RTX，显示 `peer_unsupported`。H.264 显示 `unsupported_codec` 并继续普通视频/RTX。兼容会话显示 `enabled`，仅表示发送路径已配置，不表示头显已经恢复丢包。

## 发送与带宽

每组最多 5 个媒体包（默认）附一个 XOR 校验包；帧结束时立即结算不足一组的数据，避免为了凑组额外等待。因此不是固定 20% 冗余：小帧单包时可接近 100%，另有 RTP/RED/ULPFEC/SRTP/网络头开销。同组丢一个媒体包且校验及其余媒体及时到达时可恢复；同组丢多个包不能保证恢复，丢失校验包也不能保证恢复。不会承诺消除卡顿。

校验和媒体共享主 SSRC 的连续序号空间；NACK 使用发送到网络的媒体序号查有界缓存，不重传校验包。若远端提供 apt=RED 的 RTX，则使用 RED RTX 和 FID SSRC；否则重发原 RED 媒体包。保留 PLI 等原有恢复机制。最大 plaintext RTP 校验包限制为 1400 字节，超出预算就跳过本组保护并计数，不制造未发送的序号空洞；隧道等更小 MTU 路径仍需单独检查。

收到 REMB 后，根据配置媒体份额及实际冗余开销给 VP8 预留 FEC 带宽。编码器有最低码率，若无法降到预算内，状态中 `minimum_bitrate_limited=true`，需要降低分辨率/FPS或关闭 FEC。没有 REMB 反馈时不能把配置份额当成已实施的链路带宽上限。算法不是自适应最佳比例控制器；当前份额按会话累计观测开销估算，应通过实际负载实验验证。

## 看哪些字段

访问 `/api/camera/status`，按实际相机 ID 查看：

- `webrtc_fec.state/reason`：`disabled`、`inactive`、`peer_unsupported`、`unsupported_codec` 或 `enabled`。
- `webrtc_fec.media_packets/fec_packets`：本会话发送媒体/校验包数；`fec_bytes/media_bytes`、`overhead_pct` 是 RTP 字节开销，不含外层网络头。
- `retransmissions/ignored_nacks/protection_skipped`：媒体修复、忽略请求和未保护分组数。
- `feedback_bitrate/media_target_bitrate/effective_media_ratio/minimum_bitrate_limited`：收到 REMB 后的实际媒体预算状态。
- `receiver_fec_packets_received/discarded`：头显有定义时上报的 30 秒窗口计数差分；没有该能力时为 null，不能伪造零。
- `recovery_verified`：保持 false。收到校验包不等于成功恢复包，不能凭该计数推断恢复数。

浏览器遥测同样读取 native `fecPacketsReceived/fecPacketsDiscarded`，未实现的浏览器字段保持未知。原生 Unity 渲染与物理显示指标仍未知。

## 设备验收

1. 先保留现有 APK，用 Unity 2022.3.62f3c1 和当前 WebRTC 包重新编译已改源码，按 `RECEIVER_METRICS.md` 安装测试 APK；本次没有执行 Unity 编译、打包或安装。
2. 无丢包环境先保持 VP8、FEC 关闭，确认持续视频和控制正常，记录基线。开启 FEC、重建连接，确认 `state=enabled`、校验包增长、画面持续更新，接收端支持的 FEC 计数增长。不要将接收计数称为恢复计数。
3. 使用**专用隔离测试网络/网络仿真器**，只针对测试视频链路注入可重复丢包。先验证同组单个媒体包丢失且校验保留，再验证同组双丢、校验丢失和较长突发。不要在机器人运动中给共享控制链路施加丢包；本项目不自动修改 qdisc、路由器或控制通信。
4. 对比相同 VP8、分辨率、FPS、视频内容、位置、交叉流量、总可用带宽下的 FEC 关/开。分别导出 `capture_media_metrics.py` 样本，用 `qos_compare.py` 比较冻结窗口、RTT、接收解码FPS、发送延迟及额外流量。分辨率相同不等于编码质量相同：FEC 预留带宽会降低媒体码率，需同时观察画质。
5. 切回 H.264+jetson，确认没有伪称 FEC 启用，明确显示兼容原因；关闭 FEC 后原有 H.264 硬编/RTX应正常。
6. 重连/多相机时，检查协商 payload type 与状态分别属于各自会话；关闭后停止统计和上报，不复用旧会话。

## 本地验证的范围

`tests/test_rtp_fec.py` 包含手算 RFC 字节向量、长度/CSRC/扩展/padding、回绕、单丢/多丢以及 GStreamer 原生 `rtpstorage ! rtpulpfecdec` 外部恢复互通测试。原生解码器恢复计数为 1，恢复包内容与源媒体相符，不是仅由本项目自行编码再自行解码。

`tests/test_fec_peer.py` 通过真实 ICE/DTLS/SRTP 收发 VP8 视频，测试侧接收 shim 主动丢一个媒体包、用校验恢复，然后实际解码检查图像。shim 仅供 aiortc 测试，生产头显使用原生 libwebrtc；它不是 PICO 已验收的证据。`test_fec_sender.py` 覆盖 RED RTX、NACK、RTCP SR 计数、序号回绕、MTU跳过和带宽预留。

标准参考：[RFC 2198](https://www.rfc-editor.org/rfc/rfc2198.html)、[RFC 5109](https://www.rfc-editor.org/rfc/rfc5109.html)。
