# USV 仿真

基于 ROS 2 Humble、Gazebo Sim 和 VRX 的无人水面艇仿真功能包集合。

本文档是目录入口，覆盖除 `third_party/` 外的功能包。外部依赖说明见 [third_party/README.md](third_party/README.md)。

## 构建

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/usv_simulation --ignore-src -r -y
colcon build --packages-up-to usv_sim_full --symlink-install
source install/setup.bash
```

完整仿真还依赖工作区中的 `usv_interfaces`、`usv_nav` 和 `usv_perception` 等包。

## 常用启动

```bash
# 完整仿真
ros2 launch usv_sim_full main.launch.py

# 完整仿真并启动 Nav2
ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py

# CCS 认证仿真
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py

# 使用指定配置
ros2 launch usv_sim_full main.launch.py config_path:=/path/to/full_config.yaml
```

## 功能包

| 功能包 | 作用 | 启动或使用 | 文档 |
| --- | --- | --- | --- |
| [`usv_sim_full`](usv_sim_full/README.md) | 统一启动世界、船舶、桥接、传感器和后处理组件。 | `ros2 launch usv_sim_full main.launch.py` | [架构](usv_sim_full/docs/ARCHITECTURE.md) · [数据流](usv_sim_full/docs/DATA_FLOW.md) · [变更](usv_sim_full/CHANGELOG.md) |
| [`ground_truth_sim`](ground_truth_sim/README.md) | 生成运动目标或 Gazebo 实体真值。 | `ros2 launch ground_truth_sim ground_truth_sim.launch.py` | [架构](ground_truth_sim/docs/ARCHITECTURE.md) · [数据流](ground_truth_sim/docs/DATA_FLOW.md) · [变更](ground_truth_sim/CHANGELOG.md) |
| [`usv_route_planner`](usv_route_planner/README.md) | 根据地图和任务航点生成候选航线。 | `ros2 launch usv_route_planner route_planner.launch.py use_sim_time:=true` | [架构](usv_route_planner/docs/ARCHITECTURE.md) · [数据流](usv_route_planner/docs/DATA_FLOW.md) · [变更](usv_route_planner/CHANGELOG.md) |
| [`gz_maritime_radar_plugin`](sensor_plugins/gz_maritime_radar_plugin/README.md) | 生成 Gazebo 海事雷达扇区数据。 | 由 `usv_sim_full` 在启用海事雷达时加载。 | [架构](sensor_plugins/gz_maritime_radar_plugin/docs/ARCHITECTURE.md) · [数据流](sensor_plugins/gz_maritime_radar_plugin/docs/DATA_FLOW.md) · [变更](sensor_plugins/gz_maritime_radar_plugin/CHANGELOG.md) |
| [`radar_gz_bridge`](sensor_plugins/radar_gz_bridge/README.md) | 将 Gazebo 雷达数据转换为 ROS `RadarSector`。 | `ros2 launch radar_gz_bridge radar_bridge.launch.py` | [架构](sensor_plugins/radar_gz_bridge/docs/ARCHITECTURE.md) · [数据流](sensor_plugins/radar_gz_bridge/docs/DATA_FLOW.md) · [变更](sensor_plugins/radar_gz_bridge/CHANGELOG.md) |
| [`usv_mmwave_sim`](sensor_plugins/usv_mmwave_sim/README.md) | 生成毫米波 4D 点云和可选目标列表。 | 启用毫米波配置后由 `usv_sim_full` 启动。 | [架构](sensor_plugins/usv_mmwave_sim/docs/ARCHITECTURE.md) · [数据流](sensor_plugins/usv_mmwave_sim/docs/DATA_FLOW.md) · [变更](sensor_plugins/usv_mmwave_sim/CHANGELOG.md) |
| [`env_panel`](env_panel/README.md) | 提供环境、目标船和风暴场 RViz 配置面板。 | 在 RViz 中加载相应面板。 | [架构](env_panel/docs/ARCHITECTURE.md) · [数据流](env_panel/docs/DATA_FLOW.md) · [变更](env_panel/CHANGELOG.md) |
| [`enc_grounding_warning_msgs`](safety/enc_grounding_warning_msgs/README.md) | 定义搁浅预警消息和服务。 | 接口包，无独立运行节点。 | [架构](safety/enc_grounding_warning_msgs/docs/ARCHITECTURE.md) · [数据流](safety/enc_grounding_warning_msgs/docs/DATA_FLOW.md) · [变更](safety/enc_grounding_warning_msgs/CHANGELOG.md) |
| [`enc_grounding_warning`](safety/enc_grounding_warning/README.md) | 计算龙骨下净空和搁浅风险。 | `ros2 launch enc_grounding_warning enc_grounding_warning.launch.py namespace:=usv_1 use_sim_time:=true` | [架构](safety/enc_grounding_warning/docs/ARCHITECTURE.md) · [数据流](safety/enc_grounding_warning/docs/DATA_FLOW.md) · [变更](safety/enc_grounding_warning/CHANGELOG.md) |
| [`enc_grounding_warning_rviz`](safety/enc_grounding_warning_rviz/README.md) | 提供搁浅预警 RViz 面板。 | 在 RViz 中加载 `GroundingWarningPanel`。 | [架构](safety/enc_grounding_warning_rviz/docs/ARCHITECTURE.md) · [数据流](safety/enc_grounding_warning_rviz/docs/DATA_FLOW.md) · [变更](safety/enc_grounding_warning_rviz/CHANGELOG.md) |
| [`sim_test`](sim_test/README.md) | 检查节点状态、预期话题和话题频率。 | 仿真运行后执行 `ros2 run sim_test sim_monitor`。 | [架构](sim_test/docs/ARCHITECTURE.md) · [数据流](sim_test/docs/DATA_FLOW.md) · [变更](sim_test/CHANGELOG.md) |
| [`vrx_ros`](vrx/vrx_ros/README.md) | 提供 VRX 的 ROS 接口和资源。 | 通常由 `vrx_gz` 或 `usv_sim_full` 使用。 | [架构](vrx/vrx_ros/docs/ARCHITECTURE.md) · [数据流](vrx/vrx_ros/docs/DATA_FLOW.md) · [变更](vrx/vrx_ros/CHANGELOG.md) |
| [`vrx_gz`](vrx/vrx_gz/README.md) | 提供 VRX 的 Gazebo 世界和生成入口。 | `ros2 launch vrx_gz vrx_environment.launch.py` | [架构](vrx/vrx_gz/docs/ARCHITECTURE.md) · [数据流](vrx/vrx_gz/docs/DATA_FLOW.md) · [变更](vrx/vrx_gz/CHANGELOG.md) |
| [`vrx_gazebo`](vrx/vrx_urdf/vrx_gazebo/README.md) | 生成 WAM-V 模型。 | `ros2 launch vrx_gazebo generate_wamv.launch.py` | [架构](vrx/vrx_urdf/vrx_gazebo/docs/ARCHITECTURE.md) · [数据流](vrx/vrx_urdf/vrx_gazebo/docs/DATA_FLOW.md) · [变更](vrx/vrx_urdf/vrx_gazebo/CHANGELOG.md) |
| [`wamv_description`](vrx/vrx_urdf/wamv_description/README.md) | 提供 WAM-V 描述和模型资源。 | 由下游包生成或加载。 | [架构](vrx/vrx_urdf/wamv_description/docs/ARCHITECTURE.md) · [数据流](vrx/vrx_urdf/wamv_description/docs/DATA_FLOW.md) · [变更](vrx/vrx_urdf/wamv_description/CHANGELOG.md) |
| [`wamv_gazebo`](vrx/vrx_urdf/wamv_gazebo/README.md) | 提供 WAM-V Gazebo 模板和配置。 | 由 `vrx_gz` 或 `usv_sim_full` 使用。 | [架构](vrx/vrx_urdf/wamv_gazebo/docs/ARCHITECTURE.md) · [数据流](vrx/vrx_urdf/wamv_gazebo/docs/DATA_FLOW.md) · [变更](vrx/vrx_urdf/wamv_gazebo/CHANGELOG.md) |

## 文档维护

每个功能包均在包目录内维护 `README.md`、`docs/ARCHITECTURE.md`、`docs/DATA_FLOW.md` 和 `CHANGELOG.md`。修改功能、配置或文档时，应同步更新该包 `CHANGELOG.md` 的 `Unreleased` 条目。
