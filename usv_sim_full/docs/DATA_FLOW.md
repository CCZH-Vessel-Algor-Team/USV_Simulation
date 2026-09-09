# 数据流输入输出

| 组件 | 输入 | 输出 |
| --- | --- | --- |
| `session_manager` | `full_config.yaml`。 | 每艘船的 URDF、桥接配置、RViz 配置和障碍物布局。 |
| `infra_sim` | 世界名、资源路径和全局桥接配置。 | Gazebo 世界和全局 ROS-Gazebo 桥接。 |
| `robot_bringup` | 单船 URDF、桥接配置和生成位姿。 | Gazebo 船舶实体、TF、里程计和传感器 ROS 话题。 |
| 毫米波后处理 | `points_gz` 点云。 | 4D 点云和可选目标列表。 |
| 海事雷达后处理 | Gazebo 雷达扇区。 | `RadarSector` 和可选雷达地图。 |
| `cmd_vel_to_thruster` | Nav2 `/cmd_vel`。 | 推进器控制命令。 |

完整跨包数据流见 `usv_simulation` 中各功能包的 `docs/DATA_FLOW.md`。
