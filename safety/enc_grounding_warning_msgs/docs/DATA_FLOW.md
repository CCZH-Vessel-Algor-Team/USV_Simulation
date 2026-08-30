# 数据流输入输出

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `DepthGrid` | 消息 | 发布局部或航线走廊水深栅格。 |
| `UKCState` | 消息 | 发布龙骨下净空和风险状态。 |
| `GroundingAlert` | 消息 | 发布前方搁浅告警。 |
| `GroundingRiskGrid` | 消息 | 发布搁浅风险栅格。 |
| `RouteDepthProfile` | 消息 | 发布航线水深和风险剖面。 |
| `RouteCheck` | 服务 | 校核给定航线并返回风险结果。 |

本包不发布或订阅 ROS 话题。
