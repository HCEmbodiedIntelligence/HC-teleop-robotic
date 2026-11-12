# Wi-Fi 预检、DSCP 对照与 FEC 评估

工具不会更改 AP、路由、信道或系统 QoS。默认 Wi-Fi 预检读取扫描缓存，并在当前连接上读取两次 survey 计数；主动扫描仅在遥操作开始前显式启用。

## Wi-Fi 预检

`./run.sh`（middleware/teleop 模式）启动前默认运行缓存扫描与 survey 只读预检，总超时 20 秒；结果写到 `runtime/network_preflight/latest.json`。缺工具、权限不足或超时只告知，不阻止控制栈启动，也不自动切换信道。可用 `--no-wifi-preflight` 或 `HC_WIFI_PREFLIGHT=0` 跳过；多网卡通过 `HC_WIFI_INTERFACE=wlan0` 指定，候选带宽通过 `HC_WIFI_WIDTH=80` 指定。sim 模式跳过。启动脚本从不主动跨信道扫描。

在项目根目录运行：

```bash
python tools/diagnostics/wifi_preflight.py --interface wlan0 --width 80 > wifi-report.json
# 仅在遥操作开始前；主动扫描可能打断同网卡传输
python tools/diagnostics/wifi_preflight.py --interface wlan0 --width 80 --active-scan > wifi-report.json
```

需要系统安装 `iw`，驱动与权限允许读取对应数据；脚本不会调用 `sudo`。命令逐条限制为 12 秒，采样间隔默认 1 秒，可用 `--sample-seconds` 设置 0.1–10 秒。未指定接口时选取 `iw dev` 返回的首个接口，多网卡环境应显式指定。

报告包括当前链路、监管域、候选带宽组、附近 BSS 数、驱动支持时的 busy/active 差分和错误。空口忙率为 `100 × Δbusy / Δactive`，计数重置、没有新样本或驱动不支持时为 `null`。它包含本机业务占用，不应解释为纯粹的外部干扰。

当前实现保守列出 **5 GHz 的 20/40/80 MHz 候选组**，不为 2.4/6 GHz 或 160 MHz 生成建议；这些链路仍可返回当前状态和 survey。80 MHz 的 36/40/44/48 是同组，换主信道可能仍占用相同频谱。邻居带宽不一定可读，因此附近 BSS 计数使用保守的频率邻域，不能等价为真实竞争程度。

`suggested_trial_channels` 是低置信度试验候选，依据本机支持且未被禁用、非 DFS、非 no-IR 的分组和观测 BSS 数；不自动换信道。扫描缓存可能过时。没有扫描观测时不生成建议。DFS/no-IR 候选只展示，切换前仍需核对国家/地区、AP 和头显支持，并考虑雷达退避及 CAC。没有 `iw` 或无线网卡时输出 `no_wireless_interface` 和错误，不虚构拥堵率。

机器人通过网线接 AP 时，本机可能无无线测量能力；即使有扫描网卡，其位置也不代表头显位置，需要 AP/头显侧观测。

## DSCP A/B 采样

先使用无 DSCP 的配置记录基线，再通过项目媒体 QoS 配置启用标记、重建连接，记录处理组。例如在 camera 配置中设置 `dscp: null` 作为基线，处理组设置 `dscp: 34`（AF41，完整 TOS 字节为 `0x88`）；按部署方式重启服务使配置生效。报告同时列出采样中观察到的请求 DSCP 和 socket 标记状态，不会将标记成功宣称为 WMM 生效。两组固定相机分辨率、FPS、码率策略、头显位置和交叉流量，保持相同时长，交替多轮，避免把时段环境变化误判成 QoS 效果。标记发出不代表 AP 已映射到高优先队列；需结合抓包与 AP 队列统计检查。

```bash
python tools/diagnostics/capture_media_metrics.py --seconds 60 --output qos-off.json
# 按部署配置切换 DSCP 并重建连接后采样
python tools/diagnostics/capture_media_metrics.py --seconds 60 --output qos-on.json
python tools/diagnostics/qos_compare.py qos-off.json qos-on.json --camera-id main > qos-comparison.json
```

比较工具读取 `{ "samples": [...] }`，也支持直接样本数组。每个样本可以是单相机状态、`camera` 包装或带 `streams` 数组的 `/api/camera/status` 导出对象。报告按相机 ID 隔离，列出无法匹配的 ID。每组每相机只允许一个有效头显会话；若录制中发生重连导致多个会话，显式使用 `--baseline-session ID --treatment-session ID` 选择。两组会话 ID 可以不同，不会把不同相机或不同会话的指标混合。

报告的差值是“处理组 − 基线”：延迟、丢包和冻结通常越低越好，FPS 通常越高越好。缺失、非有限值和过期的头显报告不进入比较。头显 `receiver_freeze_duration_ms`、`receiver_freeze_count`、`receiver_frames_dropped` 是接收端滚动窗口增量，不能将重叠样本求和当成整场总量。每项统计的 p95/p99 是采样值分位数；窗口 p95 的采样分位数不等于所有帧的 p95。

头显渲染回调只代表浏览器侧呈现，物理显示和真实端到端延迟需要外部测量。服务端 RTP 输出不代表接收或显示。

## FEC 决策边界

工具不会启用 FEC，也不会假设一个校验包总能恢复丢失数据。有效 RTCP 反馈不足 30 个间隔时显示数据不足；有足量丢包反馈时仅建议继续采集 RTP 序号、连续丢包长度、接收冻结和带宽余量。RTCP 聚合丢包率不能区分孤立丢包与突发丢包，不能据此设定固定“5+1”策略。窗口无丢包也不证明网络永不丢包。

上游 aiortc 没有直接可开的 FEC；本项目现已实现独立 VP8 + RED/ULPFEC 发送适配层，由原生 libwebrtc 接收恢复。默认关闭，只有远端提供兼容 RED/ULPFEC 才启用；H.264 组合明确禁用。配置、额外带宽预算与验收见 [FEC 说明](FEC_ACCEPTANCE.md)。此对照工具不自动修改 FEC，也不能凭聚合丢包率决定最佳比例。

参考：[Linux Wireless ACS](https://wireless.docs.kernel.org/en/latest/en/users/documentation/acs.html)、[RFC 8325](https://www.rfc-editor.org/rfc/rfc8325.html)、[RFC 8837](https://www.rfc-editor.org/rfc/rfc8837.html)。
