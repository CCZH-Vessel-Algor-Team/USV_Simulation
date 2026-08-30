# ground_truth_sim

生成运动目标真值，或读取 Gazebo 实体位姿生成真值，并发布 RViz Marker。

## 构建

```bash
colcon build --packages-select ground_truth_sim --symlink-install
source install/setup.bash
```

## 启动

```bash
# 独立真值模式
ros2 launch ground_truth_sim ground_truth_sim.launch.py

# Gazebo 真值桥接模式
ros2 launch ground_truth_sim gazebo_ground_truth.launch.py
```

两种模式不能同时发布 `/sim/ground_truth`。参数说明见 [PACKAGE_README.md](PACKAGE_README.md)。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
