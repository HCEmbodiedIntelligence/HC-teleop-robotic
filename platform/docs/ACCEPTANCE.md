# 验收与复现

先运行 `bash platform/scripts/test.sh`。自动测试只使用隔离仿真、回环网络和独立 ROS Domain，不连接物理机器人。可选转换测试使用 `HC_CONVERTER_PYTHON` 指向安装器创建的独立环境。

## 已覆盖的软件验收

| 边界 | 可复现测试 |
| --- | --- |
| 控制状态机、无权来源、旧世代、反馈/输入期限、限位、Home | `platform/control/tests` |
| 真实 Unix IPC、身份与状态协议 | CTest `control_ipc` |
| PICO 协议、独占/释放、乱序/重复/冻结源时间 | `test_adapters_protocol.py`、`test_adapters_pipeline.py` |
| X1 实际 URDF、原生 FK/IK、身体/颈部、路径碰撞、仿真 stop 保持 | `test_adapters_model.py`、`test_adapters_simulation.py` |
| 真实子进程 VR → core → IK → PyBullet | `test_adapters_pipeline.py` |
| Motion 规划、TTL/epoch、取消、真实 X1 仿真执行 | `test_motion*.py` |
| 外骨骼名字/单位映射、时间戳和切源 | `test_exoskeleton.py` |
| 相机、软件/NVENC 编码、实际 WebRTC、ROS 元数据 | `test_adapters_media.py` |
| MCAP 真实写入、磁盘错误、崩溃、互斥与 DAgger | `test_data_service.py` |
| 分片上传、不确定网络结果、重试与取消 | `test_data_upload.py` |
| MCAP＋MP4 的真实 ffmpeg 产物 | `test_data_video.py` |
| 实际 LeRobot v3 写入、官方读取器、视频、缺失和命名重排 | `test_data_conversion.py`（独立可选环境） |
| 真实回放 worker、停止、加速、取消、旧租约不复用 | `test_replay*.py` |
| HTTP 认证、录制下载、配置失败保持旧版、成功换代仍停止 | `test_platform_api.py` |

测试证明对应软件路径成立。NVENC 测试在有可用 NVIDIA 编码器时运行；没有设备的机器应看到跳过或软件回退证据。实际验收报告应记录测试环境与跳过项，不能把 skip 当成 pass。

## X1 现场验收

由操作者在硬件驱动和物理保护正常的环境下执行。本分支默认仅授权右臂与右夹爪，初次测试不扩大组范围。

1. 启动 `HC_x1` 的驱动，核对 ROS Domain、名字、单位、正负方向与硬件 contract。启动新 runtime 后确认 `safety=stopped`，反馈是真实、持续更新的；不存在旧控制进程。
2. 先验证网页软件停止和驱动侧停止响应，再验证明确恢复后不发生自动回零/自动跟随。记录“软件已禁止输出”和“硬件实际停止”的区别与时间。
3. 右手柄 Grip 建参考，小幅移动、松开保持、再次按下重建。测试跟踪丢失、UDP 断流、冻结源时间、旧序号与恢复后的旧手势不复活。核对位姿映射、死区、缩放、关节及末端限制。
4. 验证右夹爪 Trigger 的开闭映射及手部实际范围；这不是逐手指 VR 手势追踪。确认单侧有效位不会给另一只手生成命令。
5. 回零核对仅作用于授权范围与 `home_joint_names`，速度/加速度受限，到位要求实际反馈稳定。几何检查开启时测试禁区拒绝、退出与恢复；几何模型需要与现场夹具相符。
6. 程序会话先运行静止/小幅目标，分别验证 DAgger 关闭、正常接管后交还、从暂停接管后回暂停。核对 owner 切换已完成，身体保持，旧程序命令被拒绝。
7. 分别录制普通 MCAP、仅纠正和完整 DAgger，会话结束后核对原始话题、单调时间、来源、有效位、接管区间、配置 ZIP 与错误/缺口。确认源断流不会产生合格标记。
8. 使用这次生成的数据进行观测回放，再在明确授权下尝试小幅驱动回放。验证配置与接口不匹配被拒绝，回放互斥、暂停、停止及结束后需要恢复。
9. 相机检查源话题、CameraInfo/TF/硬件标定来源、1–4 路头显投射、实际编码器和回退原因。视频显示不能作为控制链路畅通的唯一依据。
10. 右侧验收完成后，再逐项扩大左臂、腰部、底盘授权；分别记录方向、限幅、断流和停止结果。OpenArmX/HC_REALMAN 需要其独立真机适配与验收。

## 停止与恢复故障复现

仿真模式下可仅终止当前实例创建的 `ik/backend/vr/capture` 子进程，验证核心拒绝旧目标、暴露退出原因且不自动重新驱动。重启通过控制台或退出主进程后重新启动；不得使用全局 `pkill python` 影响外部驱动。

磁盘满、写盘进程退出、转换失败或上传网络断开应保留源数据并报告 PARTIAL/FAILED/重试状态。下载、回放、转换持有引用时删除应被拒绝。平台变更不能重写已冻结会话的任务和凭据归属。
