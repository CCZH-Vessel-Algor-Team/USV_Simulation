# gz_maritime_radar_plugin

Gazebo 系统插件。将 GPU 激光雷达回波转换为海事雷达扇区数据。

## 构建

```bash
colcon build --packages-select gz_maritime_radar_plugin --symlink-install
source install/setup.bash
```

## 启动

本插件由启用海事雷达的 `usv_sim_full` 仿真加载：

```bash
ros2 launch usv_sim_full main.launch.py
```

插件路径由 `usv_sim_full` 设置，无独立 ROS 启动入口。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
