# 数据流输入输出

| 组件 | 输入 | 输出 |
| --- | --- | --- |
| `mmwave_4d_cloud_node` | `ros_gz_bridge` 发布的 `points_gz` 点云。 | 带多普勒速度和雷达散射截面的 `points` 点云。 |
| `mmwave_cluster_node` | `points` 点云。 | `usv_interfaces/msg/MmwaveTargetArray` 目标列表。 |
| `FourDRadarPlugin` | Gazebo 传感器和模型状态。 | 独立验证用点云。 |

完整仿真中，话题前缀由 `usv_sim_full` 中的船名和传感器名生成。
