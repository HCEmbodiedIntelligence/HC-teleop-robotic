# main / humanoid 功能对照与合并范围

2026-10-22 核对的版本：main `692749f`，humanoid `f7f12c9`。本地原 main
`1545f2c` 之后的三个远端修复也包含在合并中。判断依据是实际代码与测试，不能仅用提交
是否出现在另一分支来判断功能是否存在：humanoid 曾复制 main 源码而没有保留相应祖先关系。

| 功能 | main | humanoid | 合并结果 |
|---|---|---|---|
| VR 网关、v23 解算、双臂与夹爪/手控制 | 有 | 有 | 保留，内置机器人默认仍为 v23 |
| X1 头显平移/偏航驱动速度底盘、腰部和颈部控制 | 有，包含较新的底盘改进 | 有基础控制，缺少 main 后续改进 | 保留 main 改进，并与 Motion 辅助关节路由共存 |
| Motion 的 IK/FK、MoveJ/MoveL、ServoJ/ServoP | 无此集成 | 有 | 合入，通过显式后端配置启用 |
| 统一机器人公共 API、关节映射、动态关节数、运行配置生成、示例客户端 | 无 | 有 | 合入；v23 保持原生关节名，Motion 使用公共名称 |
| VR / 外骨骼 / Motion 控制源 | VR、外骨骼 | 增加 Motion | 三种来源保留，切换后须重新使能 |
| 软件停止、硬件就绪与恢复 | 有 | 增加反馈时效、命令代际和硬件使能事件 | 合并；停止/切源使旧命令失效 |
| 碰撞保护 | v23 保护 | Motion 共享边界 | 按后端保留，Motion 不重复运行 v23 的机械臂过滤 |
| PyBullet、网页三维视图、运行模式与 ROS 域隔离 | 有较新的完整实现 | 部分能力落后于 main | 保留 main，统一模式支持内部状态/命令接口 |
| 机器人 ZIP 导入/切换及包内 middleware.yaml | 有 | 增加完整 API/Motion 资源导入导出 | 合并，归档包含两套配置，避免嵌套 ZIP |
| MCAP、LeRobot、双格式录制与任务绑定 | 有 | 有另一套录制实现 | 沿用 main 的 TopicRecorder / TaskRecorder |
| RGB MP4 + 无损深度原始包 | 有：video_bundle | 有：platform_raw | 沿用 main；不引入重复录制引擎 |
| 平台共享采集方案、本地/上传格式分离、上传分片续传与重试/取消、处理状态 | 有较新的完整实现 | 有较早上传功能 | 保留 main 的完整链路 |
| LeRobot 额外动作来源及独立有效性/时效信息 | 缺少显式 action_sources 配置 | 有 | 合入，同时保留 main 的冻结采集方案与时间轴约束 |
| 相机预览、深度显示和录制管理界面 | 有较新的界面 | 有预览和不同的数据工具界面 | 保留 main 界面 |
| 物理相机启动脚本、OpenArmX 相机安装模型 | 无相同集成 | 有 | 保留可选启动脚本、配置和模型；订阅仍由 middleware.yaml 配置 |
| 相机标定自动录制 | 无该实现 | 有 | 按用户要求不引入 |
| 独立持久录制作业、数据预览/标注/裁剪/审计/恢复工作台 | 无 humanoid 这套工作台 | 有 | 按用户要求不引入，交由数据平台 |

## 录制范围

main 原有 `middleware/core/video_bundle.py` 将 RGB 编码成独立 MP4，将深度消息保留在
`raw.mcap` 中。深度不转成有损视频，消息字节、log/publish 时间戳和 sequence 保留，源
MCAP 也保留。新增集成测试同时覆盖 `16UC1` 和 `32FC1`。这里的“无损”指深度数据；RGB
仍采用视频编码。

没有合入 humanoid 的 `RecordingService`、`core/capture/`、`data_tools.py`、
`data_quality.py` 或相机标定录制模块。main 原有任务录制、上传队列持久化、封存和故障恢复
仍保留，不因排除平台数据工作台而移除。

## 控制后端与子模块

默认启动继续使用 main 的 v23。要启用 Motion，先运行 `bash bootstrap_colcon.sh build`，
然后为整套进程设置 `HC_CONTROL_BACKEND=motion`，或设置机器人包中
`arm_teleop.yaml` 的 `control.backend: motion`。中间件、控制器、仿真、数据拆分和录制
元数据使用同一选择，不能只切换其中一个进程。启动方法见 [MOTION.md](MOTION.md)。

这两个目录是独立仓库中的 ROS 2 包，通过 Git 子模块嵌入 HC，并非复制进 HC 维护的源码：

| 子模块 | 职责 | 固定提交 |
|---|---|---|
| `src/humanoid_motion_interfaces` | 消息、服务和动作接口 | `c143d19a7585e026fe13c1772a212b992af1a71d` |
| `src/humanoid_motion_server` | Motion 运动服务及 SDK 集成 | `7e852fe1085ecba911a621423b6101fa7d4cd33a` |

HC 维护 `.gitmodules`、版本指针、机器人配置、构建与启动脚本。子模块源码修改应在各自
仓库提交，再更新 HC 的指针。新检出需要 `git submodule update --init`；构建脚本也会检查。

## 验证与本地修改

验证日志位于工作区 `reports/main-humanoid-20261022/`，包括构建、Python、前端及隔离
ROS 测试。测试不对实际硬件发送命令，也不重启部署中的服务。

- 主项目 Python：534 通过，7 跳过；包含 RGB MP4、两种深度编码字节/时间戳保真、
  采集方案、任务录制、上传、仿真、控制权限和 ZIP 兼容性。
- 上传模块不依赖旧外部工程的回归：88 通过；前端：60 通过。
- 真实软件链路配 fake 控制器：11 项通过（首轮 9 项，修复后两项复测通过），覆盖
  OpenArmX、X1、六轴模型、Move/Servo、VR、碰撞边界、回放与 HTTP 控制源。
- 两个 Motion 子模块构建成功。其测试报告 201 项中 1 失败、1 跳过：
  `test_sdk_cmake_provenance` 的独立 CMake 探针缺少本机 SDK 导出的
  `urdfdom::urdf_parser` 目标。未修改子模块或系统 SDK 来掩盖该限制。
- 完整旧上传测试集仍需要外部 G0 示例响应、wheelhouse 和旧 humanoid_manager Web
  挂钩；当前工作区不具备完整测试支架，不作为已通过的验收。官方 LeRobot writer/loader
  往返测试因可选环境未安装而跳过。未执行真机验证。

原工作区的未提交修改单独备份在同目录的 `workspace-before.tar.gz` 和
`workspace-before.patch`，合并提交与这些修改分开保留。本次处理本地 main，不自动推送。
