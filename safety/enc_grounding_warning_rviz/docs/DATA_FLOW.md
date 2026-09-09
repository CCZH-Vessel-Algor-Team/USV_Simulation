# 数据流输入输出

| 方向 | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入 | `safety/ukc_state` | `enc_grounding_warning_msgs/msg/UKCState` | 显示龙骨下净空和风险等级。 |
| 输入 | `safety/grounding_alerts` | `enc_grounding_warning_msgs/msg/GroundingAlert` | 显示搁浅告警。 |
| 输入 | `safety/depth_grid` | `enc_grounding_warning_msgs/msg/DepthGrid` | 显示水深栅格统计。 |

话题相对于面板设置的命名空间解析；本包不发布控制命令。
