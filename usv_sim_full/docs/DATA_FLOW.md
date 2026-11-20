# 数据流输入输出

| 组件 | 输入 | 输出 |
| --- | --- | --- |
| `session_manager` | `full_config.yaml`。 | 每艘船的 URDF、桥接配置、RViz 配置和障碍物布局。 |
| `infra_sim` | 世界名、资源路径和全局桥接配置。 | Gazebo 世界和全局 ROS-Gazebo 桥接。 |
| `robot_bringup` | 单船 URDF、桥接配置和生成位姿。 | Gazebo 船舶实体、TF、里程计和传感器 ROS 话题。 |
| 毫米波后处理 | `points_gz` 点云。 | 4D 点云和可选目标列表。 |
| 海事雷达后处理 | Gazebo 雷达扇区。 | `RadarSector` 和可选雷达地图。 |
| `cmd_vel_to_thruster` | Nav2 `/cmd_vel`。 | 推进器控制命令。 |
| `dynamic_ship_manager` | RViz 点击/拖拽、目录/V2 服务、Gazebo 完整实测帧。 | 兼容 tracked 列表（逐船包络半径）和同帧 `/dynamic_ship/snapshot`（身份、运动、尺寸）。 |
| 普通仿真动态船 GT 桥 | `/dynamic_ship/snapshot`。 | `/sim/ground_truth/_src/dynamic_ships`，由既有 GT merger 统一发布，不另发公共真值。 |
| 可选 `usv_vessel_devices` | 每船附件配置、`/<船名>/devices/...` topic/service。 | Gazebo 灯材质、桅杆真实关节运动和设备状态，不经过推进控制链。 |

完整跨包数据流见 `usv_simulation` 中各功能包的 `docs/DATA_FLOW.md`。
