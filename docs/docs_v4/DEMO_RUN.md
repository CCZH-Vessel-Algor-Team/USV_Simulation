# Demo 运行说明（docs_v4）

默认入口：**`nav2_sim_three_vision_mmwave_bringup.launch.py`**

## 启动前

```bash
cd <ws>
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 全栈启动

```bash
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py
```

## 常用 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `gz_headless` | `false` | `true` 时不启 Gazebo GUI |
| `enable_nav2` | `true` | `false` 时只跑仿真 + 感知链 |
| `launch_rviz` | `true` | 是否启动 RViz |
| `use_static_map_odom_tf` | `true` | 静态 `map→odom`（EKF 发布时再关） |
| `map_yaml` | 包内 `maps/sydney_map2.yaml` | Nav2 地图 |

地图路径已改为 **`get_package_share_directory('usv_sim_full')`** 下资源，无需本机绝对路径。

## 配置目录

三视觉 Demo 默认配置：

```text
usv_sim_full/config/three_vision_one_mmwave/
├── full_config.yaml      # 整机 / 世界 / 船名
├── sensor_config.yaml    # 传感器内参
└── ...
```

修改 `robot.name`、周邻船、`frame_id` 时须与 TF 文档一致，见 [`docs/sim_tf_tree.md`](../../../../docs/sim_tf_tree.md)。

## 数据流（简图）

```text
Gazebo 真值 → ground_truth_sim → ground_truth_sensor_sim
    → usv_late_fusion → convert_to_trackship → /tracked_ship
    → Nav2 COLREGS 层 → 推进器 / cmd_vel
```

## 冒烟（无显示）

```bash
ros2 launch usv_sim_full nav2_sim_three_vision_mmwave_bringup.launch.py \
  gz_headless:=true launch_rviz:=false
```

数分钟后检查：

```bash
ros2 topic hz /usv_1/tracked_ship
ros2 lifecycle get /controller_server
```

## 进一步阅读

- 架构全文：[`usv_sim_full/docs/nav2_sim_three_vision_mmwave_architecture.md`](../../usv_sim_full/docs/nav2_sim_three_vision_mmwave_architecture.md)
- TF：[`docs/sim_tf_tree.md`](../../../../docs/sim_tf_tree.md)
- Nav2 参数：launch 内 `default_radar_nav2_param_yaml` 及 `usv_nav` 内 COLREGS 配置
