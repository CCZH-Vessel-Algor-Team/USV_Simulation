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
删除后恢复原始水深并移除对应圆环，重启后自动恢复已保存结果。
`GroundingWarningPanel` 只显示风险状态。具体操作见
[插件说明](../enc_grounding_warning_rviz/README.md#使用)。

`depth_provider_node` 提供 `safety/configure_depth` 服务，先校验、构建新栅格并原子保存，
成功后再切换内存数据并发布保留的 `safety/depth_settings` 消息。UKC、前瞻预警和航线水深节点
接收同一份设置；后加入的节点也能接收最后保存的配置。磁盘写入失败时保留原有数据。
各节点启动时还会先从保存文件恢复设置，避免等待消息期间使用旧水深。

预设浅滩、地形起伏和 CSV 水深始终作为原始底图保留，不复制到用户区域列表。
每次编辑从原始矩阵叠加当前用户区域；区域重叠时取较浅的用户设置值，删除最后一个区域后完整恢复原矩阵。
用户设置保存为 v2，兼容读取 v1 并排除旧界面自动带入的预设区域。

默认保存位置按命名空间划分：
`$ROS_HOME/enc_grounding_warning/<namespace>/depth_settings.yaml`（`ROS_HOME` 默认 `~/.ros`）。
单独启动本包时，可用 `depth_settings_file:=/path/to/depth_settings.yaml` 显式指定，
launch 会将同一路径传给全部四个节点。损坏或不合法的保存文件会导致启动报错，
避免静默使用与已保存设置不同的水深。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
