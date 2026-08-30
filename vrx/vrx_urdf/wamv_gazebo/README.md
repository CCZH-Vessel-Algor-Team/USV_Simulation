# wamv_gazebo

提供将 WAM-V 模型接入 Gazebo 的动力学、传感器和配置模板。

## 构建

```bash
colcon build --packages-select wamv_gazebo --symlink-install
source install/setup.bash
```

## 使用

本包提供 Gazebo 模板和资源，无独立运行节点。通常由 `vrx_gz` 或 `usv_sim_full` 在启动仿真时使用。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
