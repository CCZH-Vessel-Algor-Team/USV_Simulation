# 功能包架构

`route_planner_node` 是本包的唯一运行节点。它接收栅格地图和任务航点，在局部区域计算最短路线与安全路线；收到路线选择后重新校验，并调用 Nav2 的 `NavigateThroughPoses` action。

```mermaid
flowchart LR
    MAP[/map] --> NODE[route_planner_node]
    WP[/mission/waypoints] --> NODE
    NODE --> CANDIDATES[/route_planner/candidates]
    SELECT[/route_planner/selection] --> NODE
    NODE --> NAV[Nav2 NavigateThroughPoses]
```

配置文件为 `config/route_planner.yaml`，启动入口为 `launch/route_planner.launch.py`。
