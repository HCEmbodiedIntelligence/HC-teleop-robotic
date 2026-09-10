# Middleware

这一部分只负责产品中间件能力：

- Dashboard 前端和 WebSocket/WebRTC 接口；
- `config.yaml` 系统配置；
- 机器人/解算配置 ZIP 导入与切换；
- ROS 话题发现、状态监控、MCAP 数据录制与回放；
- VR UDP 网关和标准 ROS 话题接入。

录制订阅运行在独立进程、ROS Context 和专用执行器中，直接接收 raw CDR，只在录制开启时写入队列。默认配置优先使用 `/hc_teleop/.../compressed` 规范相机流，避免同时写入同源别名和未压缩回退图像。

服务、配置工具和运行监控入口位于本目录；Python 与前端实现位于 `core/`，系统配置为本目录的 `config.yaml`。网页导入的机器人配置统一保存到 `../adapters/robots/<配置ID>/`。

独立启动：

```bash
./middleware/start.sh
```

这是内部组件。正常使用从项目根目录运行 `./run.sh` 或 `./run.sh middleware`，先启动网页和网关，再在“系统配置 → PyBullet 仿真”基于当前已应用的机器人包独立启动仿真，可同时启动 IK/VR 控制。命令行完整仿真保留 `./run.sh sim`，真机遥操作使用 `./run.sh teleop`。停止通用栈不会停止相机、机械臂、底盘或腰部等外部硬件节点。

### VR Frame 标准话题

话题录制及系统配置的标准接口表包含 **VR Frame**：`/vrdata`，类型为
`std_msgs/msg/String`，`data` 是网关解码后的完整 VR 帧 JSON（位姿、跟踪状态、按键等）。
频率标准为目标 60 Hz、最低 30 Hz；即使尚未勾选录制，ROS bridge 也会独立监测消息频率。
勾选并保存录制配置后，该话题随正常 MCAP 录制流程写入；没有额外发布同一份 VR 数据的新话题。

### 多网卡自动发现

保持 `vr.listen_host: 0.0.0.0`，接收各网卡上的发现请求。Linux 网关利用
`IP_PKTINFO` 获取请求到达的接口和本机地址，再从同一接口/源地址单播回复，
避免默认路由、VPN/TUN 或另一张网卡改变发现响应的来源；非 Linux 保留普通 UDP 回复。
协议仍为 `PICO_DISCOVER_V1` → `PICO_RECEIVER_V1|<pose_port>`。
启动输出列出各网卡的 Dashboard 地址，不再用访问公网 DNS 的出口推断唯一地址。
网卡地址应按与 PICO 的实际连通性选择，不能仅凭 192.168/172/10 网段判断。
本改动不控制 PICO 官方投屏软件，也不改变 WebRTC ICE 的候选地址选择。

### 启动与退出生命周期

`run.sh`、`middleware/start.sh`、`adapters/start.sh` 均处理 HUP/INT/TERM。
独立组件由 `tools/runtime/process_supervisor.py` 监护：父启动脚本消失或组件退出时，
停止自身拥有的进程组，先 TERM，等待最多 3 秒后对残留组成员 KILL。
因此关闭终端、结束入口脚本或入口来不及执行 EXIT trap 时，组件也会自行退出。
不会通过 `pkill python` 等方式影响其他会话或外部硬件驱动。
录制中仍应优先正常停止录制再退出；超时强制退出不能保证文件完成正常收尾。
