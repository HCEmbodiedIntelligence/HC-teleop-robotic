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

这是内部组件。正常使用请从项目根目录运行 `./run.sh teleop`，一次启动本组件、Pinocchio IK 和通用遥操作控制；仿真使用 `./run.sh sim`。停止通用栈不会停止相机、机械臂、底盘或腰部等外部硬件节点。
