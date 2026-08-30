# 数据流输入输出

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/map` | `nav_msgs/msg/OccupancyGrid` | 用于路线规划和选择后的校验。 |
| 输入 | `/mission/waypoints` | `usv_interfaces/msg/WaypointList` | 首点为起点，末点为终点。 |
| 输出 | `/route_planner/candidates` | `usv_interfaces/msg/RouteCandidates` | 最短和安全候选路线。 |
| 输入 | `/route_planner/selection` | `usv_interfaces/msg/RouteSelection` | 人工选择的路线编号。 |
| 输出 | `/route_planner/selected_path` | `nav_msgs/msg/Path` | 选中路线的关键点。 |
| 输出 | `/route_planner/shortest_path`、`/route_planner/safest_path` | `nav_msgs/msg/Path` | RViz 调试路线。 |
| 输出 | `/route_planner/status` | `std_msgs/msg/String` | 规划和执行状态。 |
| 服务 | `/route_planner/cancel` | `std_srvs/srv/Trigger` | 取消当前 Nav2 任务。 |
| 输出 | `/usv_1/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 发送至 Nav2 的 action 目标。 |
