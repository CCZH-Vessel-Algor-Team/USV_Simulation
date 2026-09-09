# usv_mmwave_sim

提供毫米波雷达 Gazebo 插件、4D 点云后处理和可选目标聚类。

## 构建

```bash
colcon build --packages-select usv_mmwave_sim --symlink-install
source install/setup.bash
```

## 启动

完整仿真中，在 `usv_sim_full` 配置中启用 `mmwave_radar` 后，系统会自动启动点云和聚类节点：

```bash
ros2 launch usv_sim_full main.launch.py
```

独立验证插件：

```bash
ros2 launch usv_mmwave_sim spawn_ego_mmwave_validation.launch.py
```

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
