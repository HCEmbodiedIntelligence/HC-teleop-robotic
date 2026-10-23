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

## 可重复的仿真测试脚本

从仓库根目录执行，使用 ROS Humble 的 `/usr/bin/python3`。脚本先检查仿真正在运行、
仿真域与真机域不同；VR 脚本还会拒绝已有的控制会话、录制、VR 输入和平台录制选择。
报告路径必须是新的。测试退出时关闭自己的会话、停止发包，并恢复临时录制配置。

两个独立终端分别模拟持续运行的程序与真实 VR UDP 通道：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/examples/dagger_program.py \
  --url http://172.168.200.30:7876 --log runtime/my-dagger/program.jsonl
```

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/examples/dagger_vr_takeover.py \
  --url http://172.168.200.30:7876 --cycles 5 --joint-recording \
  --report runtime/my-dagger/result.json
```

第一个脚本在 MANUAL 期间仍持续发布双臂 ServoJ，使用 Ctrl-C 结束。
第二个通过 PICO v2 UDP 位姿和右 Grip 接管，经过现有 VR 网关和遥操作链路，
并等待真实程序输出后才开始计数。日志包含候选命令、最终输出、反馈与控制权变化。

批量测试会自行启动和关闭程序模拟器，不要同时运行前面的两个脚本：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/examples/dagger_run_boundaries.py \
  --url http://172.168.200.30:7876 --report runtime/my-boundaries
```

| 脚本 | 覆盖内容 |
| --- | --- |
| `dagger_body_base_boundaries.py` | 普通模式底盘前后左右及斜向、头部带动腰部和底盘、松手保持；DAgger AUTO/MANUAL/HOLD 下辅助轴保持 |
| `dagger_command_boundaries.py` | 旧/未来/零时间戳、非法名称/维数/组、NaN/Inf/越限、旧控制权、绕过与回零入口 |
| `dagger_fault_boundaries.py` | Grip 滞回、10 次短接管、暂停松手、三处跟踪丢失、断流、重复序号/冻结时间戳、无效四元数、急停 |
| `dagger_aggressive.py` | 12 次竞争接管；VR 手柄位移峰峰值 24 cm、姿态范围约 70°、约 100 Hz 发包；程序幅度 0.3 rad |
| `dagger_startup_boundaries.py` | 准备时按住 Grip、就绪后重新松手；12 次交替开启两种录制模式，检查启动超时 |
| `dagger_concurrency_boundaries.py` | 8 请求并发启用、旧启停请求跨会话重放、旧 epoch、暂停/返回、退出/松手、急停/返回竞争和损坏 JSON |
| `dagger_wire_boundaries.py` | 损坏报文、旧协议、NaN/Inf、越界 Grip、100 m 不可达位姿与极端摇杆 |
| `dagger_full_session_boundaries.py` | 程序、人工、暂停混合完整会话；CRC、源时间、最终动作证据和 3 个接管区间回读 |
| `dagger_timing_boundaries.py` | 30 次松手：UDP 发包、控制权放行、首条程序最终输出及仿真反馈分别计时 |
| `dagger_check_recordings.py` | 回读纠正 MCAP 的 CRC、动作证据、序号、片段边界、来源时间间隔、接收年龄及反馈误差 |

例如 `--only command fault` 只运行指定批次。离线检查不启动机器人：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/examples/dagger_check_recordings.py runtime/dagger_preview/recordings \
  --sessions-from runtime/my-boundaries/aggressive/results.json \
  --output runtime/my-boundaries/recording-audit.json
```

## 2026-10-22 实测结果与限制

证据目录：`/home/hc_op/workspace/reports/dagger-two-scripts-20261022/`。
本轮使用 X1、Motion、隔离仿真域 175；真机域 14。最终会话 OFF，仿真保持运行，录制配置已恢复。

- `run2/verified.json`：5 次独立程序/VR 接管通过；匹配 4408 条最终输出，MANUAL 中程序持续发令，程序命令泄漏为 0，辅助轴位移为 0。
- `boundary1/command`、`boundary1/fault`、`boundary3/body_base`、`boundary3/aggressive`、`startup1`：最终 37 项检查通过，含 12 次激进接管和 12 次录制准备。
- `recording-audit.json`：38 份纠正 MCAP 通过；3292 条关节反馈、2895 条人工动作，动作证据与序号完整，质量均为 `ok`。反馈最大源间隔 21.17 ms、最大接收年龄 10.44 ms；动作最大源间隔 21.60 ms。
- 124 条动作未在片段内匹配到完全相等的后续反馈，其中 16 条之后已无反馈帧；下一帧反馈最大关节差约 0.0042 rad。完全匹配到的动作/反馈间隔最大 14.09 ms。该统计不能证明每条命令都被执行，不能作为真机电机延迟指标。
- 首轮录制准备曾出现一次 `teleop input timeout` 并安全进入 FAULT；后续身体测试及 12 次专门启动复测未复现，发送器未观察到超过 50 ms 的发包间隔。保留为间歇性问题线索，未通过调宽超时掩盖。

平台任务“DAgger 仿真边界与上传验证 20261022”使用专用采集方案。真实 MCAP 与采集包均已上传，
原文件及纠正侧车下载 SHA-256 与本地一致；采集包保留所有 680 条原消息和 90 条人工动作。
平台原始录制已就绪，等待切片；没有把这一状态标记成训练 Episode 质检通过。
MCAP 的 50 Hz 预览中，首条动作之前的 2 帧标签为空，之后 49 帧为布尔 `true`，没有用未来动作补填未知标签。

本次发现并修复：

1. 仿真配置错误指向空的上传队列目录，导致已连接平台看似丢失。恢复到原持久化 `runtime/upload_queue`，多次重启后使用原凭据验证成功，没有重新申请连接。
2. 平台 MCAP 数值映射拒绝 DAgger 布尔标签。现在仅为声明为 `bool` 的字段接受真正的布尔值，拒绝数字和字符串隐式转换。
3. 历史录制作为新上传生成采集包时复用旧目录，导致 `ARTIFACT_IDENTITY_CONFLICT`。新上传使用独立且可重试的目录，原包内容不变。
4. MCAP 原始录制准备完成但尚未切片时，本地误报缺少质检结论。现在显示原始录制已处理、需要到平台切片，同时保留质检待完成状态。

对应回归：平台采集方案测试 71 项通过、59 项因独立数据库等条件跳过；本地录制/上传交接 36 项通过；
受影响上传与采集包测试 51 项通过。另行尝试的旧 `test_client.py` 全组因仓库外 G0 示例响应文件缺失，
17 项未通过，见 `upload-regression.log`；没有用伪造响应冒充该组验收。

三维画面入口为 `http://172.168.200.30:7876/dashboard/#config`，勾选“显示三维仿真视图”。
无独立桌面窗口是当前 headless 仿真的启动方式。最终核验收到 15 帧、29 个关节的网页三维数据；
自动化 Firefox 无法创建 WebGL 上下文，因此没有完成浏览器像素渲染验收。
这些测试覆盖运动学仿真、关节/底盘控制与纠正数据链路，不包含真机动力学、碰撞效果、真实相机 MP4 编码或长时间相机负载。

## 追加边界与恢复时间检查

证据目录：`/home/hc_op/workspace/reports/dagger-extra-boundaries-20261022/`。
追加脚本可用 `dagger_run_boundaries.py --only concurrency wire full_session timing --report <新目录>` 顺序运行；
各脚本也支持 `--case <用例名>`。最终 21 组在线用例通过，包括一次独立的 30 次恢复计时；
失败尝试及原始 trace 均保留。控制/录制回归 69 项通过，上传/UDP 回归 71 项通过。
上传测试在本机真实 HTTP 连接上注入创建、授权、PUT、半包、完成和提交阶段断连，
覆盖 MCAP、LeRobot 和原始包三种格式；重启后保持同一上传身份，不重发服务端已确认的字节。
这组故障注入使用临时契约服务；实际平台上传验证见上一节。

本轮修复三类问题：旧请求重放会再次执行录制副作用（误停新会话或产生孤立录制）；
损坏 JSON 请求返回 500；录制结束事件入队失败却仍标为 `ok`。
失败的录制启停仍可在原会话、原 epoch 内显式重试。
另外为最终人工动作证据补充 `command_stamp_ns`，精确关联完整会话的原始动作头时间；
`stamp_ns` 仍表示 HC 最终发布时刻，纠正录制继续使用该发送边界，两个时间不能混同。

30 次 24 cm 手柄往复后的松手测量（同一主机时钟，单位 ms）：

| 区间 | 中位数 | P95 | 最大值 |
| --- | ---: | ---: | ---: |
| 系统识别松手 → 控制权放行 | 10.59 | 25.22 | 27.39 |
| 系统识别松手 → 第一条程序最终输出 | 26.88 | 42.56 | 44.18 |
| VR 模拟器发出松手包 → 第一条程序最终输出 | 34.35 | 45.78 | 46.13 |
| VR 模拟器发出松手包 → 仿真反馈发生变化 | 36.55 | 49.04 | 56.19 |

29/30 次在后续反馈中匹配到了完全相等的第一条程序目标；另一次只确认后续反馈已变化，
不能断言那一条离散目标被执行。反馈变化不代表回到原轨迹，也不是实际电机延迟。
断流、无效输入和急停进入 FAULT/ESTOP 后不会自行恢复 AUTO，需要明确恢复至 HOLD，再显式返回程序。

`timing-recording-audit.json`：30 份计时录制均为 `ok`，2072 条反馈、1797 条人工动作；
反馈最大源间隔 20.47 ms、最大接收年龄 10.23 ms，动作最大间隔 20.34 ms，无超过 50 ms 的间隔。
`full-session-final/results.json`：完整会话含 180 条人工动作、160 条程序动作、555 条反馈，
3 个接管区间及两种事件编码均验证；最大接收年龄 14.22 ms，命令源时间到最终发送最大 5.95 ms。

该轮仍保留两个未解决线索，不计作正常表现（后续修复见下一节）：

- `recording-audit.json` 的 15 份异常边界录制中，14 份为 `ok`，NaN 输入用例一份为 `warning`。
  该文件反馈存在 289.98 ms 间断，底盘约 299.87 ms；独立观察器同期有连续 30 帧、最大间隔 12.02 ms，
  问题已缩小到录制接收链路。文件的动作证据完整，但不能把数据完整性检查通过等同于连续采样合格。
- `time-check/full_session/results.json` 再次出现录制准备阶段 `teleop input timeout`，安全进入 FAULT。
  VR 发包无超过 50 ms 的间隔，网关处理队列丢弃了 16 个旧样本。之后新会话完整录制通过，
  但间歇性启动超时尚未定位修复，没有扩大超时阈值或自动清除故障。

最初完整会话检查器曾把 ROS CDR 事件当 JSON，并误把命令头时间与最终发送时间视作同一时间；
这些检查器错误已纠正，原失败输出保留在 `full-session*` 目录，最终结论以 `full-session-final` 为准。

## 录制连续性及准备阶段修复

证据目录：`/home/hc_op/workspace/reports/dagger-latency-fixes-20261022/`。

录制专用的 JointState、PoseStamped 等小型状态订阅使用 64 帧有界历史，
避免接收线程短暂停顿时被默认 5 帧队列覆盖。普通实时控制和图像订阅策略不变，
原始源时间、接收时间及数据新鲜度检查不变。真实 DDS 回归在连续发布 20 帧时暂停回调约 200 ms：
修复前关节和底盘都只收到最后 5 帧，修复后两者均收到全部 20 帧；见 `qos-before.log`、`qos-after.log`。

DAgger 开启后增加 `preparing_capture` 标志：录制进程、可用槽位和必要数据源就绪前保持 HOLD，
不允许开始程序或接管。就绪后必须观察到新的松手，准备期间的输入和延迟到达的旧松手都不能启用 Grip。
这解决了准备过程提前启动输入看门狗及旧输入越过准备边界的问题；
`preparation-before.log`、`readiness-cutoff-before.log` 保留确定性复现。
就绪后真正断流 301 ms 仍进入 FAULT，ESTOP/FAULT 不会被准备完成自动清除。

复测还复现了 `teleop: control permit timeout`：接收端对已经撤销的许可也计时，
准备阶段心跳延迟会被当成已放行许可过期。现在只有 grant 阶段存在需要续期的许可，
revoke 阶段始终关闭输出，新的 grant 仍检查身份与时间戳。
已放行许可的 300 ms 超时保持不变，复现见 `permit-repro-before.log`。

失败尝试保留在 `live/` 和 `final-live/`。并发退出/松手测试原来错误地要求退出请求必胜；
现在验证松手先改变 epoch 时旧请求必须返回 409，再对同一会话发送新的退出请求。
独立冷启动测试也补上了配置要求的显式安全恢复前置条件。
首轮录制中的一次 53.34 ms 人工动作间隔与独立最终输出观察器完全相同，60 条动作序号完整，
见 `source-gap-check.json`；这是源输出节奏间隔，不能算作录制漏帧。

最初的间歇性调度停顿本身在 40 次热启动、5 次冷启动基线中没有复现；
本次修复的是已经确定性复现的接收缓存和准备阶段状态问题，不能据此承诺任意负载下的实时性。

最终 `verified/` 九组 59 项在线检查全部通过，包含 12 次交替模式的录制准备、
12 次激进接管和 30 次恢复计时；`regression-latest.log` 为 84 项通过。
`recording-audit.json` 回读 80 份纠正录制，质量均为 `ok`，CRC、动作证据和序号完整：
4388 条关节反馈、4389 条底盘位姿、3844 条人工动作；三路均没有超过 50 ms 的源间隔。
关节最大间隔 20.13 ms、最大接收年龄 10.16 ms；底盘最大间隔 14.32 ms、最大接收年龄 5.75 ms。
完整会话另含 183 条人工动作、160 条程序动作和 3 个接管区间，最大接收年龄 11.59 ms，
关节及底盘最大源间隔均约 13.06 ms。

最终 30 次恢复计时（单位 ms）：

| 区间 | 中位数 | P95 | 最大值 |
| --- | ---: | ---: | ---: |
| 系统识别松手 → 控制权放行 | 8.50 | 14.17 | 20.28 |
| 系统识别松手 → 第一条程序最终输出 | 22.78 | 32.47 | 33.34 |
| VR 发出松手包 → 第一条程序最终输出 | 28.32 | 36.29 | 40.88 |
| VR 发出松手包 → 仿真反馈变化 | 33.12 | 41.65 | 46.80 |

30/30 次匹配到第一条程序目标的完全相等反馈。首轮同类测试仍曾观测到 92.62 ms 的
发包到程序输出尾延迟，保留在 `live/timing/results.json`；最终这一轮较快不代表尾延迟已被全面消除。
这些数值仍是同机运动学仿真结果，不代表真机电机时延或轨迹收敛时间。

`summary.json` 最终核验为 PASS：会话 OFF、录制配置已恢复、临时性能采样关闭，
重启后原平台凭据验证成功，两份此前真实上传仍已处理完成。
网页三维数据流收到 15 帧、29 个关节；浏览器实际像素渲染、真实相机长时间负载和真机验证仍不在本轮验收范围。
