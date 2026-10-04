# usv_sim_full

读取 YAML 配置，统一启动 Gazebo 世界、船舶、桥接、传感器、场景和可选后处理组件。

## 构建

```bash
colcon build --packages-up-to usv_sim_full --symlink-install
source install/setup.bash
```

构建前应确保工作区已包含 `usv_interfaces`、VRX、`usv_mmwave_sim`、`radar_gz_bridge` 及需要使用的导航、感知包。

## 启动

```bash
# 完整仿真
ros2 launch usv_sim_full main.launch.py

# 完整仿真并启动 Nav2
ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py

# CCS 认证仿真
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py

# 指定配置
ros2 launch usv_sim_full main.launch.py config_path:=/path/to/full_config.yaml
```

主配置文件为 `config/full_config.yaml`。

Gazebo 服务器与GUI分别启动，避免合并启动时等待GUI握手；世界文件、物理步长和实时倍率保持配置原值。
`gz_headless:=true` 只启动服务器，传感器仍按配置运行。关闭GUI不会停止仿真服务器，应结束整个launch会话。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
