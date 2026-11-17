# 数据流输入输出

默认命名空间为 `usv_1` 时：

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `/usv_1/state/vessel` | `usv_interfaces/msg/VesselState` | 船舶状态。 |
| 输入 | `/usv_1/plan` | `nav_msgs/msg/Path` | 规划航线。 |
| 输出 | `/usv_1/safety/depth_grid` | `DepthGrid` | 船体附近水深。 |
| 输出 | `/usv_1/safety/depth_region_markers` | `visualization_msgs/MarkerArray` | 用户区域细圆环，Reliable/Transient Local；原始预设不绘制，删除时清除。 |
| 输出 | `/usv_1/safety/ukc_state` | `UKCState` | 龙骨下净空和风险。 |
| 输出 | `/usv_1/safety/grounding_alerts` | `GroundingAlert` | 搁浅告警。 |
| 输出 | `/usv_1/safety/grounding_risk_grid` | `GroundingRiskGrid` | 风险栅格。 |
| 输出 | `/usv_1/safety/route_depth_grid` | `DepthGrid` | 航线走廊水深。 |
| 服务 | `/usv_1/safety/route_check` | `RouteCheck` | 航线校核。 |
| 服务 | `/usv_1/safety/configure_depth` | `ConfigureDepth` | 查询、保存分区水深或恢复启动配置。 |
| 内部同步 | `/usv_1/safety/depth_settings` | `DepthSettings` | 可靠、Transient Local，向各安全节点同步最新设置。 |

`ConfigureDepth.apply=false` 只读取当前设置。`apply=true` 且 `settings.enabled=true`
时校验并保存用户区域，叠加到原始水深上；基础水深字段只作为原始配置的元信息。
`enabled=false` 或用户区域为空时恢复原始矩阵，用户列表为空。删除操作提交剩余区域，
从原始矩阵重建以恢复被删除区域下方的水深；预设浅滩不进入用户列表。
响应 `success=false` 表示本次设置未成功保存；插件会显示原因。

`GroundingAlert.level=0` 表示已解除前瞻风险；更新水深后危险消失时会发送解除消息并删除危险点标记。
