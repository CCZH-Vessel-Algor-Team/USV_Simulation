# vrx_gz

提供 VRX 的 Gazebo Sim 世界、船舶生成和竞赛启动资源。

## 构建

```bash
colcon build --packages-select vrx_gz --symlink-install
source install/setup.bash
```

## 启动

```bash
# 启动 VRX 环境
ros2 launch vrx_gz vrx_environment.launch.py

# 生成船舶
ros2 launch vrx_gz spawn.launch.py
```

本项目通常通过 `usv_sim_full` 使用本包。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
