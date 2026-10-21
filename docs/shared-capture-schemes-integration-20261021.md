# 共享采集方案：主分支集成交接

2026-10-21。本文件补充机器人终端交付后的主分支集成修复；仅在隔离 worktree 和测试环境验证，未安装到业务机器人。

任务领取/预检只读；开始录制调用平台准备接口，按任务修订冻结方案和映射。metadata、capture-context、ready 描述、队列与上传引用保留同一快照，重启和普通重试不读取新的默认方案。

集成时修正了两处真实交接问题：

- LeRobot 的 timestamp、frame_index、episode_index、index、task_index 由 writer 产生，不要求设备订阅提供。写入前检查包含这些生成字段；封存前仍校验实际输出的 info，缺少真实字段不能通过。
- 当前平台在每个 Episode 返回 `annotation_status` 与 `data_readable`。客户端合同接收这些字段，并把 PENDING_CONFIGURATION/CREATED/APPROVED 映射为现有显示状态；只有所有 Episode 为 READY/PASS 且明确可读，才显示 READABLE。旧回执没有这些字段时显示 UNKNOWN，训练交付仍独立判断。

schema provenance 更新为 `hc-shared-capture-20261021.2`，摘要由合同测试检查。

主机器人回归 172 通过、7 跳过：3 项缺 rclpy、4 项缺官方 LeRobot writer 环境。实际平台连接票据、PostgreSQL 准备、快照、分片上传至 MinIO、Temporal 数据发布、无标注读取以及重启重试已通过。设备健康、ROS 输入及 writer 输出使用合成夹具；不是官方 writer 或真实硬件全链路验收，也未执行运动。

证据：[实际交接](/home/hc_op/workspace/reports/shared-capture-schemes-20261020/main-browser/robot-live-real.json)、[回归报告](/home/hc_op/workspace/reports/shared-capture-schemes-20261020/main-robot-final/python.xml)、[完整验收矩阵](/home/hc_op/workspace/reports/shared-capture-schemes-20261020/acceptance-matrix.md)、[发布回退](/home/hc_op/workspace/reports/shared-capture-schemes-20261020/release-and-rollback.md)。
