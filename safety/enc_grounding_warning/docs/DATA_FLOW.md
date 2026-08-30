# 数据流输入输出

默认命名空间为 `usv_1` 时：

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/usv_1/state/vessel` | `usv_interfaces/msg/VesselState` | 船舶状态。 |
| 输入 | `/usv_1/plan` | `nav_msgs/msg/Path` | 规划航线。 |
| 输出 | `/usv_1/safety/depth_grid` | `DepthGrid` | 船体附近水深。 |
| 输出 | `/usv_1/safety/ukc_state` | `UKCState` | 龙骨下净空和风险。 |
| 输出 | `/usv_1/safety/grounding_alerts` | `GroundingAlert` | 搁浅告警。 |
| 输出 | `/usv_1/safety/grounding_risk_grid` | `GroundingRiskGrid` | 风险栅格。 |
| 输出 | `/usv_1/safety/route_depth_grid` | `DepthGrid` | 航线走廊水深。 |
| 服务 | `/usv_1/safety/route_check` | `RouteCheck` | 航线校核。 |
