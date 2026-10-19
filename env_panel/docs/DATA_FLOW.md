# 数据流输入输出

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输出 | `/vrx/wind/velocity_cmd` | `geometry_msgs/msg/Vector3` | 世界坐标系风速。 |
| 输出 | `/ocean_current` | `geometry_msgs/msg/Vector3` | 世界坐标系水流速度。 |
| 输出 | `/vrx/wavefield/parameters` | `ros_gz_interfaces/msg/ParamVec` | 波浪方向、强度和周期。 |
| 输出 | `/vrx/environment/sun_light_cmd` | `ros_gz_interfaces/msg/Light` | 太阳光配置。 |

环境真值由 Gazebo 在 `/vrx/debug/wind/speed` 和 `/vrx/debug/wind/direction` 发布，本包不订阅这两个话题。

## 雷达控制面板

| 方向 | 接口 | 类型 | 说明 |
| --- | --- | --- | --- |
| 输入/输出 | `/radar_control_node/get_parameters`、`set_parameters` | `rcl_interfaces` 参数服务 | 读取和设置雷达参数。 |
| 输入 | `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 同步雷达节点参数变化。 |
| 输出 | `/radar/set_power` | `std_srvs/srv/SetBool` | 雷达电源控制。 |
| 输出 | `/radar/set_transmit` | `std_srvs/srv/SetBool` | 发射/待机控制。 |
| 输出 | `/radar/query_all` | `std_srvs/srv/Trigger` | 查询雷达当前状态。 |
