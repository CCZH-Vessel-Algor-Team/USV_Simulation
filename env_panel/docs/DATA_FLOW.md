# 数据流输入输出

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输出 | `/vrx/wind/velocity_cmd` | `geometry_msgs/msg/Vector3` | 世界坐标系风速。 |
| 输出 | `/ocean_current` | `geometry_msgs/msg/Vector3` | 世界坐标系水流速度。 |
| 输出 | `/vrx/wavefield/parameters` | `ros_gz_interfaces/msg/ParamVec` | 波浪方向、强度和周期。 |
| 输出 | `/vrx/environment/sun_light_cmd` | `ros_gz_interfaces/msg/Light` | 太阳光配置。 |

环境真值由 Gazebo 在 `/vrx/debug/wind/speed` 和 `/vrx/debug/wind/direction` 发布，本包不订阅这两个话题。
