# vrx_ros

提供 VRX 仿真使用的 ROS 资源和接口。

## 构建

```bash
colcon build --packages-select vrx_ros --symlink-install
source install/setup.bash
```

## 启动

本包通常由 `vrx_gz` 或 `usv_sim_full` 使用。需要监视 Gazebo 仿真进程时可执行：

```bash
python3 $(ros2 pkg prefix vrx_ros)/share/vrx_ros/launch/monitor_sim.py
```

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
