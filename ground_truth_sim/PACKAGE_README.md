# ground_truth_sim

> 本文档是 `ground_truth_sim` 包级说明的单一事实源；目录下 `README.md` 仅保留导航与入口信息。

这个 ROS 2 Python 功能包用于在多传感器融合仿真中生成无人船（USV）的真值目标。核心节点会发布：

- `/sim/ground_truth`（`usv_interfaces/msg/GlobalTrackArray`）：权威的目标状态。
- `/sim/ground_truth_markers`（`visualization_msgs/msg/MarkerArray`）：供 RViz 展示的实时位置与预测轨迹图层。

仓库自带 `rviz/ground_truth_view.rviz`，开箱即可加载。视觉与毫米波传感器模拟节点位于独立包 `ground_truth_sensor_sim`（`src/usv_perception/ground_truth_sensor_sim`），便于单独复用或部署。

## 功能特性

- 基于 CTRV（Constant Turn Rate and Velocity）模型，以 50 Hz 推进状态，包含随机转向与衰减。
- 可选 **waypoint 固定航路模式**（`motion_mode: waypoint`）：配置 `fixed_targets[]`，目标沿航路点 **往返巡逻（ping-pong）**，仍发布 `/sim/ground_truth` 供 `ground_truth_sensor_sim` / 融合链路使用。
- 目标初始状态随机分布在可配置的环带区域内（CTRV 模式）。
- AIS 匹配目标与“暗船”使用不同颜色区分。
- 利用 CTRV 模型解析生成未来轨迹预览。
- RViz 中保留历史轨迹，便于对比过去运动与预测路径。

### Gazebo 实体权威真值（推荐全栈路径）

- 节点 `ground_truth_gazebo_entity_node`：`gazebo_visual: true` 时由 launch **单独**启动；内部 waypoint arc 驱动 cmd_vel，从 Gazebo `pose/info` 读回并发布 `/sim/ground_truth` 与 `/sim/ground_truth_markers`。
- 节点 `ground_truth_node`：仅 `gazebo_visual: false` 时使用（kinematic CTRV/waypoint，无 Gazebo 周邻实体）。
- 旧节点 `ground_truth_gazebo_models_node`（订阅 truth → mirror）已 **deprecated**，保留供回退。

### Gazebo Sim 桥接（legacy / 二选一）

- 节点 `gazebo_ground_truth_bridge_node`：由参数 `targets_json` 定义 Gazebo 模型名与 `track_id`，用与 `ground_truth_node` 相同的 CTRV 积分驱动 `SetEntityPose`，发布 `/sim/ground_truth` 与 `/sim/ground_truth_markers`；`GlobalTrack.source_model_name` 填入模型名。
- `reference_frame`：与 Gazebo 世界/桥接 TF 一致的坐标系（常用 `map`）；`message_frame_id`：真值消息头与 `ground_truth_sensor_sim` 一致的坐标系（默认 `base_link`），二者不同时通过 TF 将 `x,y,v` 与 Marker 变换到 `message_frame_id`。
- 碰撞：`collision_topic` 订阅 `ros_gz_interfaces/msg/Contacts`（需 `ros_gz_bridge` 从 gz 传感器桥接）；或 `collision_string_topic` 订阅 `std_msgs/String`，内容为单个 `model_name`。默认 `respawn_mode:=teleport` 在环带内重置状态并 `SetEntityPose`；`delete_clone` 会先 `DeleteEntity` 再以 `clone_name` 重生（需配置 `clone_source_name`）。
- 启动：`ros2 launch ground_truth_sim gazebo_ground_truth.launch.py bridge_params_file:=...`。**不要**与 `ground_truth_sim.launch.py` 中的 `ground_truth_node` 同时发布同一 `/sim/ground_truth` 话题。
- 桥接参数模板：[config/gazebo_ground_truth_bridge.yaml](config/gazebo_ground_truth_bridge.yaml)；Contacts 桥接示例说明：[config/ros_gz_bridge_contacts.example.yml](config/ros_gz_bridge_contacts.example.yml)。

## 构建与运行

```bash
# 在工作区根目录构建（假设已 source /opt/ros/<distro>/setup.bash）
cd <your_ros2_ws_root>
colcon build --packages-select usv_interfaces ground_truth_sim ground_truth_sensor_sim --symlink-install
source install/setup.bash

# 方案 A：一次性启动（TF + 节点 + RViz）
ros2 launch ground_truth_sim ground_truth_sim.launch.py

# 方案 B：分别控制各进程
ros2 run ground_truth_sim static_tf_broadcaster
ros2 run ground_truth_sim ground_truth_node
# 启动视觉检测模拟（ground_truth_sensor_sim 包）
ros2 run ground_truth_sensor_sim sim_vision_node

# 启动毫米波雷达模拟（ground_truth_sensor_sim 包）
ros2 run ground_truth_sensor_sim sim_mmwave_node
```

在 RViz 中查看：

```bash
source ~/usv_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix ground_truth_sim)/rviz/ground_truth_view.rviz
```

请确保固定坐标系设为 `map`（静态父坐标）或 `base_link`，并启用 “Targets (Markers)” 显示，即可看到彩色球体（当前位置）以及短期预测折线。

### 静态 TF 参数

`static_tf_broadcaster` 支持以下 ROS 2 参数：

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `parent_frame` | string | `map` | 推送到 TF 树的父坐标系（通常是 `map`） |
| `child_frame` | string | `base_link` | RViz 与话题使用的子坐标系 |
| `xyz` | double[3] | `[0,0,0]` | 父到子坐标平移 |
| `rpy` | double[3] | `[0,0,0]` | 父到子坐标的欧拉角（弧度） |
| `sensor_parent_frame` | string | `base_link` | 多传感器静态 TF 的父坐标系 |

节点会在广播 `parent_frame -> child_frame` 的同时，额外固定九个传感器坐标系（均以 `sensor_parent_frame` 为父，默认 `base_link`）：

| Frame | 平移 (x, y, z) m | Yaw (rad) |
| --- | --- | --- |
| `nav_radar_link` | (0.0, 0.0, 2.0) | 0.0 |
| `camera_front_link` | (0.25, 0.0, 1.15) | 0.0 |
| `camera_left_link` | (0.0, 0.25, 1.15) | 1.5708 |
| `camera_right_link` | (0.0, -0.25, 1.15) | -1.5708 |
| `camera_back_link` | (-0.25, 0.0, 1.15) | 3.1416 |
| `mmwave_front_link` | (0.25, 0.0, 1.0) | 0.0 |
| `mmwave_left_link` | (0.0, 0.25, 1.0) | 1.5708 |
| `mmwave_right_link` | (0.0, -0.25, 1.0) | -1.5708 |
| `mmwave_back_link` | (-0.25, 0.0, 1.0) | 3.1416 |

示例：将原点向东平移 5 米并以 `world` 为父坐标系，同时维持默认传感器 TF：

```bash
ros2 run ground_truth_sim static_tf_broadcaster --ros-args \
	-p parent_frame:=world -p child_frame:=base_link \
	-p xyz:="[5.0, 0.0, 0.0]"
```

### Launch 参数

使用 `ros2 launch ground_truth_sim ground_truth_sim.launch.py` 时，可配置以下参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `parent_frame` | `map` | 传递给 `static_tf_broadcaster` 的父坐标系 |
| `child_frame` | `base_link` | 传递给 `static_tf_broadcaster` 的子坐标系 |
| `use_rviz` | `true` | 是否自动启动 RViz |
| `rviz_config` | `<pkg_share>/rviz/ground_truth_view.rviz` | RViz 配置文件路径 |
| `start_vision_node` | `true` | 是否同时启动 `ground_truth_sensor_sim` 的视觉检测节点 |
| `start_ais_node` | `false` | 已废弃：`ground_truth_sensor_sim` 不提供 AIS 模拟节点 |
| `start_nav_radar_node` | `false` | 已废弃：`ground_truth_sensor_sim` 不提供导航雷达模拟节点 |
| `start_mmwave_node` | `true` | 是否同时启动 `ground_truth_sensor_sim` 的毫米波节点 |

### 节点参数

`ground_truth_node` 支持通过 ROS 参数在启动时调整仿真行为，可在 `ros2 launch` 命令后追加 `--ros-args -p 参数名:=值`（或使用 YAML 参数文件）。常用参数如下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `update_dt` | `0.02` | 主循环周期（秒），0.02 表示 50 Hz |
| `frame_id` | `base_link` | 发布在消息/Marker 中的坐标系 |
| `target_count` | `5` | 初始生成的目标数量 |
| `annulus_radius_min/max` | `50.0 / 500.0` | 目标初始位置环带半径范围（米） |
| `speed_min/max` | `2.0 / 12.0` | 目标初始速度范围（m/s） |
| `size_width_min/max` | `2.0 / 10.0` | 目标宽度范围（米） |
| `size_length_min/max` | `5.0 / 50.0` | 目标长度范围（米） |
| `size_height_min/max` | `2.0 / 15.0` | 目标高度范围（米） |
| `ais_match_probability` | `0.4` | 初始化为 AIS 匹配目标的概率，其余视为暗目标 |
| `omega_noise_std` | `0.005` | 角速度随机游走的标准差（rad/s） |
| `omega_decay` | `0.99` | 每周期对角速度的衰减系数 |
| `omega_limit` | `0.1` | 角速度绝对值上限（rad/s） |
| `prediction_horizon` | `5.0` | 未来轨迹的预览时长（秒） |
| `prediction_dt` | `0.25` | 未来轨迹的离散间隔（秒） |
| `history_max_points` | `500` | 在 RViz 中保留的历史轨迹点数 |
| `rng_seed` | `-1` | 伪随机数种子，`-1` 表示随机种子 |
| `motion_mode` | `ctrv` | `ctrv` 随机 CTRV；`waypoint` 固定航路点往返 |
| `fixed_targets` | `[]` | waypoint 模式目标列表（见下文） |
| `waypoint_arrival_threshold_m` | `0.5` | 到点判定半径（米） |

#### waypoint 模式示例（`full_config` → `scenario.ground_truth_sim`）

```yaml
ground_truth_sim:
  enabled: true
  motion_mode: waypoint
  frame_id: map
  fixed_targets:
    - track_id: 1
      speed: 4.0
      loop: true
      waypoints:
        - [40.0, -20.0]
        - [80.0, 20.0]
    - track_id: 2
      speed: 3.5
      loop: true
      is_ais_matched: false
      waypoints:
        - [-30.0, 10.0]
        - [10.0, 50.0]
```

隔离测试（`ground_truth_test.launch.py`，仅 Gazebo + 真值 + RViz）：

```bash
ros2 launch usv_sim_full ground_truth_test.launch.py \
  config_path:=$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/config/full_config.yaml
```

航路点 waypoint 模式可改用 `config/three_vision_one_mmwave/full_config.yaml`。话题健康检查：

```bash
bash $(ros2 pkg prefix ground_truth_sim)/share/ground_truth_sim/scripts/verify_sim_health.sh
```

示例：一次性调整目标数量和速度范围：

```bash
ros2 launch ground_truth_sim ground_truth_sim.launch.py --ros-args \
	-p target_count:=8 -p speed_min:=4.0 -p speed_max:=15.0
```

### 参数 YAML

默认参数文件位于 `config/ground_truth_params.yaml`。`ground_truth_sim.launch.py` 会通过 `params_file` 参数自动加载它，因此只需修改该 YAML 即可影响下一次启动。若希望在不同场景间切换，也可以提供自定义文件：

```bash
ros2 launch ground_truth_sim ground_truth_sim.launch.py \
	params_file:=/path/to/my_ground_truth_params.yaml
```

YAML 结构遵循 ROS 2 约定：

```yaml
ground_truth_node:
  ros__parameters:
    target_count: 5
    # ... 其它参数
```


## 话题说明

| 话题 | 类型 | 描述 |
| --- | --- | --- |
| `/sim/ground_truth` | `usv_interfaces/msg/GlobalTrackArray` | 所有仿真目标的真值状态 |
| `/sim/ground_truth_markers` | `visualization_msgs/msg/MarkerArray` | RViz 覆盖层（实体权威模式下与 Gazebo pose 一致） |
| `/vision/*/detections` | `usv_interfaces/msg/VisionDetectionArray` | 模拟摄像头输出的目标检测（由 `ground_truth_sensor_sim` 发布） |
| `/vision/*/markers` | `visualization_msgs/msg/MarkerArray` | 视觉检测在 RViz 中的可视化（由 `ground_truth_sensor_sim` 发布） |
| `/mmwave/*/targets` | `usv_interfaces/msg/MmwaveTargetArray` | 毫米波雷达模拟输出（由 `ground_truth_sensor_sim` 发布） |
| `/mmwave/*/markers` | `visualization_msgs/msg/MarkerArray` | 毫米波目标可视化（由 `ground_truth_sensor_sim` 发布） |

## 参数配置

绝大多数场景只需编辑 `config/ground_truth_params.yaml` 或在 launch 命令中指定 `params_file` 即可。若需引入新的自定义参数，请在 `ground_truth_sim/ground_truth_node.py` 中添加 `declare_parameter` 及对应逻辑，然后同步更新 YAML 和 README。

## 许可证

Apache-2.0
