# radar_gz_bridge

将 Gazebo 海事雷达扇区数据转换为 ROS 2 `marine_sensor_msgs/RadarSector` 消息。

## 构建

```bash
colcon build --packages-select radar_gz_bridge --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch radar_gz_bridge radar_bridge.launch.py \
  gz_topic:=/blueboat/radar/spokes \
  ros_topic:=/sensors/radar/nav/sector
```

在完整仿真中，`usv_sim_full` 会在海事雷达启用时按船启动本节点。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
