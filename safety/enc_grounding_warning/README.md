# enc_grounding_warning

根据水深栅格、船舶状态和规划航线计算龙骨下净空与搁浅风险。

## 构建

```bash
colcon build --packages-select enc_grounding_warning_msgs enc_grounding_warning --symlink-install
source install/setup.bash
```

## 启动

仿真已运行时，启动预警节点：

```bash
ros2 launch enc_grounding_warning enc_grounding_warning.launch.py namespace:=usv_1 use_sim_time:=true
```

启动完整仿真和预警：

```bash
ros2 launch enc_grounding_warning full_sim_with_grounding_warning.launch.py
```

## 插件水深设置

在独立 `DepthSettingsPanel` 中设置半径和水深，用工具栏“添加水深点”单击地图即可立即应用，
每次单击后自动退出打点，不需要添加按钮或同步确认。地图以深红色圆环显示已应用的用户区域；
删除后恢复原始水深并移除对应圆环。用户区域只在本次运行内有效，重启后恢复
`depth_grid.yaml` 的原始水深，与浮标/障碍物面板添加的临时目标行为一致。
`GroundingWarningPanel` 只显示风险状态。具体操作见
[插件说明](../enc_grounding_warning_rviz/README.md#使用)。

`depth_provider_node` 提供 `safety/configure_depth` 服务，先校验并构建新栅格，成功后
切换内存数据并发布保留的 `safety/depth_settings` 消息。UKC、前瞻预警和航线水深节点
接收同一份设置；后加入的节点也能接收当前运行中的配置。用户区域不写入任何文件，
校验或构建失败时保留原有内存数据。

预设浅滩、地形起伏和 CSV 水深始终作为原始底图保留，不复制到用户区域列表。
每次编辑从原始矩阵叠加当前用户区域；区域重叠时取较浅的用户设置值，删除最后一个区域后完整恢复原矩阵。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
