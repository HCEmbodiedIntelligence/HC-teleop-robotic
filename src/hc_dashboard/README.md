# hc_dashboard

Independent operator UI for the decomposed HC runtime. The ROS bridge only
subscribes to typed telemetry and calls the safety enable/reset services; it
never publishes command candidates or final actuator commands.

The browser receives a 4 Hz aggregate snapshot over WebSocket. High-rate ROS
messages remain inside the bridge and are reduced to stream frequency, age,
controller state and latest per-group values.

The console reuses the HTML structure and CSS from
`~/test2/HC-teleop-robotic/middleware/core/static` to match that Dashboard:

- **状态监控**: ROS status, control source, command/feedback rates, tracking,
  and joint positions in radians/degrees. Native diagnostics remain in an
  expandable panel below the joint monitor.
- **话题录制**: live HC topic telemetry. Recording controls are disabled because
  this Dashboard has no recording HTTP adapter.
- **数据集管理**: the reference layout with an explicit unavailable state;
  dataset import/delete/replay are not exposed by this Dashboard.
- **系统配置**: the running profile and standard interfaces, displayed read-only.

The JavaScript consumes the current `hc-dashboard/v1` WebSocket snapshot.
Stop calls `/api/v1/safety/enabled` with `false`; resume resets a latched fault
when necessary, then requests enable. Neither button publishes actuator targets.
Disconnected/stale telemetry is not shown as live; safety buttons are disabled
while disconnected or waiting for a response. Homing and source-switch buttons
are disabled because their former middleware APIs do not exist here.

```bash
./bootstrap_colcon.sh deps
./bootstrap_colcon.sh build
source install/setup.bash
ros2 run hc_dashboard dashboard --ros-args \
  -r __ns:=/robots/x1 \
  -p robot_id:=x1 -p profile:=x1 -p mode:=sim -p port:=7877
```

Open `http://127.0.0.1:7877/dashboard/`. The standard bringup launch starts the
dashboard by default; pass `start_dashboard:=false` for a control-only launch.

### Dashboard 参考界面适配

四个页面沿用旧版 Dashboard 的布局、配色、表格与弹窗样式，数据来自当前 ROS 2 控制链路。
`GET /api/v1/runtime` 提供已安装的机器人 profile、URDF 统计、运行参数和 ROS Graph；
`GET /api/v1/datasets` 只读列出 `dataset_root` 下的 `hc-dataset/v1` manifest，默认目录为
`~/.local/share/hc_teleop/datasets`，可通过 Dashboard ROS 参数覆盖。
话题选择和自定义规则仅保存在浏览器，不会启动录制。回零、源切换、录制、数据集导入/删除/回放、
profile 写入等尚无当前后端接口的操作保持禁用。离合状态当前未提供，不根据目标消息推测。

### ROS 2 标准监测接口

Dashboard 默认接口表及录制选择使用以下标准消息（前缀为 `/robots/<robot_id>/`）：

| 话题 | 类型 | 用途 |
|---|---|---|
| `state/joints` | `sensor_msgs/msg/JointState` | 原始实际关节反馈 |
| `standard/joint_targets` | `sensor_msgs/msg/JointState` | 后端关节候选目标 |
| `standard/joint_commands` | `sensor_msgs/msg/JointState` | 仲裁后的最终命令 |
| `standard/target_ee_pose/<group>` | `geometry_msgs/msg/PoseStamped` | 分机械臂末端目标 |
| `standard/actual_ee_pose/<group>` | `geometry_msgs/msg/PoseStamped` | 实际关节反馈 FK |

`standard/*` 由 Dashboard 启动后单向转换发布，属于观察接口，不能作为驱动控制入口。
关节消息逐组转发，保留关节名称和权威命令时间戳；位姿保留参考系和输入时间戳，
每臂独立话题，实际位姿仅在 `valid=true` 时发布。目标的姿态控制模式仍以内部消息为准。
内部自定义消息继续保存会话、序号、有效期及租约语义；页面不再将它们列作默认标准接口。
监控页新增末端目标和反馈的位置、四元数、参考系、末端坐标系及有效性显示。

### 与旧版 X1 sim 的 header / 维度对齐（2026-09-08 实测）

默认数组接口更新为 `standard/controller_target_ee_poses`、`standard/target_ee_poses`、
`standard/actual_ee_poses`（`PoseArray`），固定右臂、左臂顺序；原有分臂 `PoseStamped` 仍可使用。
控制器数组沿用 `generic_task_bases` 约定：右项在右臂基座、左项在左臂基座，
该字符串不是一个实际 TF 坐标系。可视化目标和实际反馈通过消息时间戳对应的 TF
转换至 `standard_pose_frame`（X1 默认 `zhi_Link`）；缺臂、反馈无效或缺 TF 时不发布数组。
保留原始输入时间戳，不用网页接收时间替换。四元数 q 与 -q 等价，不保证其分量符号与旧版相同。

`standard/joint_states` 按 profile 的 `standard_interfaces.feedback_joint_order` 排序，
X1 为旧仿真的 29 维顺序，保留原 header 及有效的 position/velocity/effort 数组。
缺少的关节不会填零，额外真实关节保留在末尾。
`standard/joint_targets` 和 `standard/joint_commands` 仅聚合同来源、同会话、同序号、
到达间隔不超过 50 ms 的完整双臂消息，顺序右 7 + 左 7，header 使用该批次较晚的权威时间戳。
不补造速度、力矩或夹爪命令；因此当前最终命令是 14 维，而旧版样本为 16 维（含 R_ban/L_ban）。
这些标准话题位于 `/robots/x1/standard/`，不会发布到正在运行的旧版 `/hc_teleop/` 控制话题。
