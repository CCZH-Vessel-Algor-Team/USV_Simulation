# wamv_description

提供 WAM-V 的 URDF、Xacro、网格和模型资源。

## 构建

```bash
colcon build --packages-select wamv_description --symlink-install
source install/setup.bash
```

## 使用

本包提供模型资源，无独立运行节点。通常由 `wamv_gazebo`、`vrx_gz` 或 `usv_sim_full` 在生成或启动仿真时使用。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
