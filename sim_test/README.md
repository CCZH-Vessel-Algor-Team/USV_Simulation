# sim_test

检查 USV 仿真中的节点状态、预期话题和话题频率，并提供 `rqt_graph`、`rqt_image_view` 调试入口。

## 构建

```bash
colcon build --packages-select sim_test --symlink-install
source install/setup.bash
```

## 启动

先启动仿真，再执行：

```bash
ros2 run sim_test sim_monitor
```

使用自定义仿真配置时：

```bash
ros2 run sim_test sim_monitor --config /path/to/full_config.yaml
```

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
