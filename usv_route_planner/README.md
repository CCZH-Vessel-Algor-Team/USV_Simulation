# usv_route_planner

基于栅格地图和任务航点生成最短、安全两条候选航线；收到人工选择后向 Nav2 发送航线。

## 构建

```bash
colcon build --packages-select usv_route_planner --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch usv_route_planner route_planner.launch.py use_sim_time:=true
```

输入为 `/map` 和 `/mission/waypoints`，候选路线发布至 `/route_planner/candidates`。接口详情见 [docs/DATA_FLOW.md](docs/DATA_FLOW.md)。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
