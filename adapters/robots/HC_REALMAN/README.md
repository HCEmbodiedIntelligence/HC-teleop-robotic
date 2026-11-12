# HC_REALMAN 导入包

此包用于 HC-teleop-robotic 的“系统配置”页面导入，包含双 ECO65-B、双 OmniPicker、龙门和相机机构。

- Profile ID：`HC_REALMAN`
- 左臂关节：`left_joint1` … `left_joint6`
- 右臂关节：`right_joint1` … `right_joint6`
- URDF 网格全部使用包内 STL 相对路径，可由网页 Three.js 查看器加载。
- OmniPicker 真机每侧按单一归一化开合量（0.0 开 ~ 1.0 闭）控制；URDF 已加入完整的四连杆机构 mimic 约束与对称镜像开合参数，三维视图与仿真均能正确闭合且不撕裂连杆。
- 原附件的碰撞网格与显示高模相同，包默认关闭遥操碰撞拦截；用于精确动力学前应另做低模 collision。

启动：在项目根目录运行 `./run.sh`，进入“系统配置”导入本 ZIP，应用 `HC_REALMAN`，再在 PyBullet 面板启动仿真并打开三维视图。
