# 其他终端执行提示词

将下面整段复制到负责实现的终端。它指向完整实施方案，要求执行代码改造和软件验收。
本提示词及方案的编写本身没有实现新接口；代码执行从接收本提示词的终端开始。

```text
请在 /home/hc_op/workspace/teleop_ws/src 中实施 HC 统一机械臂接口，完成代码、配置、demo、文档和软件验收。

首先阅读：
1. /home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic/docs/UNIFIED_ROBOT_API_IMPLEMENTATION_PLAN.md
2. /home/hc_op/workspace/teleop_ws/src/HC-teleop-robotic/docs/ROBOT_PUBLIC_API.md
3. /home/hc_op/workspace/teleop_ws/reports/robot-api-20261015/RESULT.txt
4. 工作区和实际修改路径适用的 AGENTS.md。

第一份是目标方案，第二、三份是之前的实现和验证基线。以当前源码核对差异，不要把目标方案或历史测试结果当成已经实现/验证的新功能。

用户已明确的目标：
- 算法侧使用相同的话题/Action、消息格式和公共关节命名规则。允许不同机器人关节数不同。
- left_arm/right_arm 对外使用 left_joint_1…N、right_joint_1…N，不能暴露 OpenArmX、X1 的原生模型名作为必需输入。
- 机型差异在 HC 的 adapters/robots/<robot>/profile 内集中配置。
- 复用已有 URDF、arm_teleop.yaml 的关节分组、顺序和 initial_joints；在 profile 内补 robot_api.yaml 和公共映射逻辑，不能再维护一套重复的模型/回零真值。
- 主要用途是订阅状态、ServoJ、ServoP；MoveJ 用于回位和固定位置。
- VR 映射后的末端目标与外部 ServoP 走相同的位姿处理，最终与 ServoJ 共用关节伺服核心。内部不必绕公共话题，但不能保留两套不同的机械臂处理链路。
- 保留每个机器人的坐标约定，不做跨机器人同坐标/同物理位置对齐。

现有成果与工作边界：
- 当前 /control/hardware/enabled 已与 /control/teleop/enabled 分离。VR 禁用不能阻断 motion 控制。保留显式恢复、停止、source/permit epoch、反馈时效和旧目标拒绝。
- HC 已有 Motion 最终 RTC，优先复用 SdkMotionBackend::updateFinalJointTarget 和 command_pipeline，不另写重复 IK/伺服算法。
- OpenArmX 默认后端仍是 v23；当前公共状态/ServoJ 仍使用模型名；现有 demo 主要是 profile 驱动的订阅/MoveJ。它们都还不满足本次完整目标。
- 源码只改 teleop_ws/src 下当前项目，不改 /home/hc_op/workspace/ws_teleop/src 归档。
- src 包含多个 Git 仓库，HC 还包含子模块。分别保存初始 git status/diff，保留所有已有未提交修改与未跟踪内容，不 reset、不清理其他终端的文件。
- 本次完成软件实现和假硬件验证。不要操作真实机器人、CAN、线上域 14 或重启在线进程，不自动部署。

请按实施方案第 7 节顺序持续完成：

阶段 1：配置与契约。
建立统一的 profile 解析结果，引用已有分组/home，定义并验证公共到模型映射、顺序、单位、限位和驱动连接参数。
配置转换采用 q_model = scale*q_public + offset；速度、力矩和限位按方案转换。先查清已有驱动与 SDK 的单位转换，避免重复。
补齐 OpenArmX 和 X1 配置，处理导入导出、环境变量选择和各进程 profile 一致性。

阶段 2：运行时双向边界。
外部 ServoJ/MoveJ 公共名称转模型；反馈、分组状态、最终目标及 Action feedback/result 转公共名称。
隔离原始模型内部话题，公共通道不混入原生名字，不引入双发布者或反馈环路。
驱动启动实际消费 profile 连接参数；不能只增加配置而没有生效路径。
实现 /status/robot/info 的版本化 JSON 和晚加入订阅可用的 QoS。元数据包含组、公共名称/数量/单位/限位、端点、frame、Servo 参数和命名姿态。

阶段 3：共用 Servo/Move 执行。
两个机型的统一模式都启用 Motion。外部 ServoP 与 VR 输入走同一位姿处理，ServoJ/ServoP/MoveJ 进入同一最终关节 RTC。
公共程序输入和 VR 内部输入保留独立的来源检查：motion 只接受程序控制，vr 只接受 VR 输入。不能把两者混在无来源区分的回调里。
将机械臂共有的限位、反馈时效及已启用的碰撞检查放到共同路径。保留 VR 离合、腰部/头部/夹爪功能。
VR home 与程序 home 共用 MoveJ，解决当前 Move 只允许 motion 许可的问题时，不放开全部公共 Move 在 VR 下执行。
回位期间占用对应组，拒绝 Servo 插队，完成或取消后不续播旧目标。验证 VR 断流不会错误停止当前 motion 来源。

阶段 4：demo 与采集。
改造 examples/robot_api_demo.py，在线用法不需要 --profile、机器人品牌或厂商 SDK。
实现 describe、subscribe、连续 servo-j、连续 servo-p 和 home/固定姿态 MoveJ；在线获取名称、数量和本机 frame。
demo 不自动切源或恢复硬件。MoveJ 取消与 Servo 停流都要有正确处理。
同步 DatasetTopics、录制元数据、回放与外骨骼的名称边界。不要丢掉既有辅助关节，也不要把历史原生名回放静默当作新格式执行。

阶段 5：构建、集成测试与交付。
按方案 A1–A12 验收，至少覆盖 OpenArmX、X1、一个不同 DOF 测试模型。
使用实际 Motion/IK/RTC、实际适配代码和假硬件闭环。仅测试 YAML、mock 核心或函数调用次数不算完成 Servo/VR 整链路。
复测切源、停止、反馈丢失、显式恢复和旧目标拒绝，保留此前使能修复的回归。
修改 C++ 后构建受影响包，确认运行的是新产物。按需运行既有 Python/ROS/C++ 测试，检查 skip 和失败原因。
ROS 环境入口是 /home/hc_op/workspace/teleop_ws/setup_hc.bash，Humble 使用 /usr/bin/python3。
测试使用 localhost 和空闲隔离 ROS domain，现有集成测试使用 216；不要与其他测试/在线节点冲突。

执行方式：
- 开始时简要说明实际实施顺序，然后直接推进。无需重新询问用户是否开始，也不要只写计划后结束。
- 每阶段汇报完成内容和验证结果，必要的日常实现取舍自行完成。
- 若当前源码使推荐的内部节点组织不合适，可以做等价调整并记录理由；公共契约、复用现有配置、来源隔离和共享执行要求不变。
- 遇到依赖或环境问题先排查可解决原因；若确有无法完成的外部依赖，明确具体阻塞和已验证范围，不伪造通过结果。
- 定期保存实施进展，便于换上下文后继续，不因会话变长跳过后续阶段。
- 不要提交、推送或部署未经要求的代码；交付保留为可审阅的本地修改。

最终交付：
1. 实际实现结果和修改仓库/文件清单。
2. 最终公共/内部话题与名称、单位转换责任表。
3. 无需机型参数的通用 demo 命令和 profile 迁移/启动步骤。
4. 新报告放到 /home/hc_op/workspace/teleop_ws/reports/unified-robot-api-<执行时间>/，保存构建/测试命令、日志和 A1–A12 验收结论，不覆盖历史报告。
5. 更新 docs/ROBOT_PUBLIC_API.md、examples/README.md、有关硬件契约和实施状态；明确假硬件、真机、跨电脑 DDS、外部 X1 驱动的验证范围。

请持续完成到可复现的软件验收，不将“只统一了话题名”或“配置文件已补齐”当作完成。
```
