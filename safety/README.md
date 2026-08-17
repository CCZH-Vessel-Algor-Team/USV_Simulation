# USV Safety（仿真搁浅预警）

本目录是仿真版电子海图搁浅预警的独立实现，不修改任何现有功能包。

## 功能简介

本功能包基于仿真水深矩阵，提供**当前 UKC 监控、前瞻搁浅告警、航线校核与可视化**能力：

- 发布船体附近稠密水深栅格与当前 UKC 状态；
- 订阅 Nav2 航线，自动计算前方搁浅风险并输出告警；
- 提供航线校核服务，返回首个危险点与最小 UKC；
- 发布航线走廊水深栅格与彩色 Marker，并配套 RViz 面板。

详细功能、模块组成与核心公式见 [功能说明书](docs/FUNCTIONAL_SPEC.md)。

## 文档

- [功能说明书](docs/FUNCTIONAL_SPEC.md)：当前功能包的具体功能、组成、数据流与核心公式
- [使用说明](docs/USAGE.md)：构建、启动、配置、全仿真接入
- [接口说明](docs/INTERFACES.md)：话题、消息、服务、枚举、接入示例

## 包

| 包 | 说明 |
|----|------|
| `enc_grounding_warning_msgs` | DepthGrid / UKCState / GroundingAlert / GroundingRiskGrid / RouteDepthProfile / RouteCheck |
| `enc_grounding_warning` | SimGridProvider、UKC 估算、前瞻告警、航线校核、launch、测试 |

## 快速启动（完整仿真）

```bash
colcon build --packages-select enc_grounding_warning_msgs enc_grounding_warning --symlink-install
source install/setup.bash
ros2 launch enc_grounding_warning full_sim_with_grounding_warning.launch.py
```

CCS 认证环境也已集成在仿真包中，启动 `usv_sim_full` 的 CCS launch 时会默认一起拉起安全节点：

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py
```

说明（`full_sim_with_grounding_warning.launch.py`）：默认 Include `usv_sim_full/nav2_sim_full_bringup.launch.py`
（当前工作区未安装 `dynamic_ship_manager_node`，因此不使用 three-vision mmwave bringup）。
`params_file` 为 Nav2 参数，`gw_params_file` 为搁浅预警自身参数，两者独立。
集成 launch 默认启动 RViz（`enable_rviz:=true`），使用
`usv_sim_full/rviz/default.rviz`，可直接用 2D Goal Pose 向 `/usv_1/goal_pose`
发布目标点。

为避免当前环境已知问题，集成 launch 做了以下兼容处理：

- Nav2 参数直接使用 `usv_sim_full/config/radar_nav2_param.yaml`
  （`GridBased=RRTStarPlanner`、`VORRTStar=VORRTStarPlanner`），
  与 humble-temp 组合保持一致；
- 启动时额外发布 identity `map -> usv_1/odom` 静态 TF，保证 map TF 对
  Nav2 readiness gate 与预警节点可见；
- VORRTStar 依赖 `vector_object_server`（`/get_avoidance_point` 等服务），
  集成 launch 的 `enable_keepout_filter` 默认已改为 `true`，验收时不要关闭；
- 设置 `RMW_FASTRTPS_USE_SHM=0` 与
  `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`，减少 Fast DDS 共享内存
  与 gz protobuf 兼容性报错。

> 若 `waypoint_follower` 报 `libwait_at_waypoint.so: file too short`，
> 说明 `build/nav2_waypoint_follower/libwait_at_waypoint.so` 是 0 字节坏产物，
> 需重新构建该包：
> `colcon build --packages-select nav2_waypoint_follower --symlink-install`

## 仅启动预警节点（仿真已在运行）

```bash
ros2 launch enc_grounding_warning enc_grounding_warning.launch.py namespace:=usv_1 use_sim_time:=true
```

## 默认话题

| Topic | 类型 |
|-------|------|
| `/usv_1/safety/depth_grid` | `enc_grounding_warning_msgs/DepthGrid` |
| `/usv_1/safety/ukc_state` | `enc_grounding_warning_msgs/UKCState` |
| `/usv_1/safety/grounding_alerts` | `enc_grounding_warning_msgs/GroundingAlert` |
| `/usv_1/safety/grounding_risk_grid` | `enc_grounding_warning_msgs/GroundingRiskGrid` |
| `/usv_1/safety/route_check` | `enc_grounding_warning_msgs/RouteCheck` |
| `/usv_1/safety/route_depth_grid` | `enc_grounding_warning_msgs/DepthGrid`（仅航线走廊） |
| `/usv_1/safety/route_depth_markers` | `visualization_msgs/MarkerArray`（彩色水深栅格） |
| `/usv_1/safety/grounding_markers` | `visualization_msgs/MarkerArray` |
| `/usv_1/safety/current_risk_marker` | `visualization_msgs/MarkerArray` |

## RViz 可视化

新增独立插件包 `enc_grounding_warning_rviz`：

- RViz 菜单：`Panels -> Add New Panel -> enc_grounding_warning_rviz/GroundingWarningPanel`
- Panel 显示：UKC、风险等级、告警信息、水深栅格统计
- Panel 顶部可修改 Namespace（默认 `usv_1`）

Marker 话题：

- `/usv_1/safety/current_risk_marker`：当前船位风险球 + UKC 文字
- `/usv_1/safety/grounding_markers`：航线风险点 + 首个危险点红球

在 RViz 中通过 `Add -> By topic` 添加这两个 MarkerArray 即可直接看到效果，
不需要修改原 RViz 布局文件。

## 测试

```bash
python3 -m pytest src/usv_simulation/safety/enc_grounding_warning/test -q
```

ROS 节点级闭环测试（无需 Gazebo/Nav2）：

```bash
ros2 launch enc_grounding_warning enc_grounding_warning.launch.py use_sim_time:=false &
sleep 3
ros2 run enc_grounding_warning integration_test
```
