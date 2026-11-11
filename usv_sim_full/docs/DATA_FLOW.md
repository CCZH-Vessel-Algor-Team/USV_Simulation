# 数据流输入输出

| 组件 | 输入 | 输出 |
| --- | --- | --- |
| `session_manager` | `full_config.yaml`。 | 每艘船的 URDF、桥接配置、RViz 配置和障碍物布局。 |
| `infra_sim` | 世界名、资源路径和全局桥接配置。 | Gazebo 世界和全局 ROS-Gazebo 桥接。 |
| `robot_bringup` | 单船 URDF、桥接配置和生成位姿。 | Gazebo 船舶实体、TF、里程计和传感器 ROS 话题。 |
| 毫米波后处理 | `points_gz` 点云。 | 4D 点云和可选目标列表。 |
| 海事雷达后处理 | Gazebo 雷达扇区。 | `RadarSector` 和可选雷达地图。 |
| `cmd_vel_to_thruster` | v2 的 `control/actuator_command`；旧模式为 Nav2 `/cmd_vel`。 | 推进器控制命令；v2 中性绕过 PID。 |
| USV Test Goal | 地图位置与拖拽航向。 | `/<船>/autonomy/goal_pose`，只用于草稿。 |
| 原 Navigation 2（Managed Task） | G 工具单点、原多点草稿、Pause/Resume/Cancel、AutonomyState。 | 带当前 boot/revision 的 ManageMission；不绕过任务身份。 |
| USV BT v2 面板 | 航点表、当前 AutonomyState/GateState、按钮操作。 | 带当前 boot/revision 的 ManageMission、保护复位和 CCS 测试服务调用。 |
| simulation_controls | 显式 SetBool 测试服务。 | 仿真控制权及独立海事等待/定位故障/急停输入；不覆盖真实海事或常规判定主题。 |

完整跨包数据流见 `usv_simulation` 中各功能包的 `docs/DATA_FLOW.md`。
