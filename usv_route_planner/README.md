# usv_route_planner

基于 Map Server `/map` 快照生成最短和安全两条候选航线。规划器在局部 ROI
中计算 EDT，以 `collision_clearance_m` 为两条路线生成共同船体膨胀掩码；安全
路线额外应用硬净空和距离风险代价。人工选择后，节点使用最新 `/map` 重新验证
路线，并发送 `NavigateThroughPoses`。

## ROS 发行版要求

本节点必须使用与目标 Nav2 相同的 ROS 2 Humble 编译和运行。Humble 与 Jazzy
的 `nav2_msgs/action/NavigateThroughPoses` Result 定义不同，Jazzy Action Client
无法匹配 Humble Action Server。launch 会拒绝非 Humble 运行环境。

## ROS 接口

| 名称 | 类型 | 方向 |
| --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 订阅，transient-local |
| `/mission/waypoints` | `usv_interfaces/msg/WaypointList` | 订阅，首点为起点、末点为终点 |
| `/route_planner/candidates` | `usv_interfaces/msg/RouteCandidates` | 发布，两条 GPS/map 路线、request_id 和 plan_id |
| `/route_planner/selection` | `usv_interfaces/msg/RouteSelection` | 订阅，人工选择 request_id 和 plan_id |
| `/route_planner/cancel` | `std_srvs/srv/Trigger` | 服务，取消当前 Nav2 任务 |
| `/route_planner/shortest_path` | `nav_msgs/msg/Path` | 发布，RViz 调试 |
| `/route_planner/safest_path` | `nav_msgs/msg/Path` | 发布，RViz 调试 |
| `/route_planner/selected_path` | `nav_msgs/msg/Path` | 发布，选中路线的起点、关键拐点和终点 |
| `/route_planner/status` | `std_msgs/msg/String` | 发布，规划和 Nav2 执行状态 |
| `/usv_1/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Action 客户端 |

## 坐标基准

当前配置对应 `CN441122_enc_5km`：地图、Gazebo 世界和船舶 odom 共用
WGS84 datum `(34.692120, 119.481403)` 定义的局部 ENU 坐标：

```text
map_x = local ENU east
map_y = local ENU north
```

`map_projection: local_cartesian` 使用 GeographicLib 在 WGS84 与局部 ENU
之间双向转换。更换地图后必须从其 `.geo.json` 同步 datum，并从 YAML 同步
`expected_map_*` 几何签名，否则 GPS 路线会整体偏移或新地图会被规划器拒绝。
保留的 `utm_offset` 模式仅用于兼容旧 UTM 裁剪地图。

## 启动

```bash
ros2 launch usv_route_planner route_planner.launch.py use_sim_time:=true
```

请求示例：

```bash
ros2 topic pub --once /mission/waypoints usv_interfaces/msg/WaypointList \
  "{waypoints: [{latitude: 34.692120, longitude: 119.481403}, \
                {latitude: 34.697120, longitude: 119.486403}]}"
```

从候选结果复制当前 `request_id` 后选择路线。`plan_id=1` 表示最短路线，
`plan_id=2` 表示安全路线：

```bash
ros2 topic pub --once /route_planner/selection usv_interfaces/msg/RouteSelection \
  "{request_id: '<request_id>', plan_id: 2}"
```

候选消息中的 `gps_path.waypoints` 只包含起点、LOS 关键拐点和终点；
`map_path.poses` 保留按 `waypoint_spacing_m` 重采样的密集 map 路径。

收到选择后，节点用最新 `/map` 复核缓存路线，并将起点、LOS 关键拐点和终点
发布到 `/route_planner/selected_path`；其 Pose 数量与上位机收到的
`gps_path.waypoints` 一致。发送 `NavigateThroughPoses` 时是否包含起点由
`nav2_include_start_pose` 控制；当前配置为 `true`。联调时可配置
`execute_with_nav2: false`，只验证选路和路径准备而不调用 Nav2。

取消当前执行：

```bash
ros2 service call /route_planner/cancel std_srvs/srv/Trigger
```
