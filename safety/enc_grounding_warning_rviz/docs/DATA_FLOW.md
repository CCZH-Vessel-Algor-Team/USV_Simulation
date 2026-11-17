# 数据流输入输出

| 组件 | 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 预警显示 | 输入 | `safety/ukc_state` | `UKCState` | 净空和风险等级。 |
| 预警显示 | 输入 | `safety/grounding_alerts` | `GroundingAlert` | 告警与解除。 |
| 预警显示 | 输入 | `safety/depth_grid` | `DepthGrid` | 水深统计。 |
| RViz 区域图层 | 输入 | `safety/depth_region_markers` | `visualization_msgs/msg/MarkerArray` | 可靠保留消息，只显示已应用用户区域的深红色圆环。 |
| 打点工具 | 输出 | `safety/depth_point` | `geometry_msgs/msg/PointStamped` | 单次发布 map 中的区域中心后自动退出。 |
| 水深设置 | 输入 | `safety/depth_point` | `geometry_msgs/msg/PointStamped` | 按当前半径和水深自动添加，无需确认。 |
| 水深设置 | 输入 | `safety/depth_settings` | `DepthSettings` | 可靠保留消息，更新当前已应用的用户区域列表。 |
| 水深设置 | 服务客户端 | `safety/configure_depth` | `ConfigureDepth` | 自动提交新增或删除，读取已保存区域。 |

安全类型来自 `enc_grounding_warning_msgs`。话题、服务按面板船舶命名空间解析，默认 `usv_1`。
列表只显示有效用户区域；原始预设是底图，不进入用户列表。
