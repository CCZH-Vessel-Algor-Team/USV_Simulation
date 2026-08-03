# usv_mmwave_sim（毫米波仿真一体化）

面向 **USV 感知融合** 的 Gazebo Sim（gz-sim）毫米波雷达仿真包。提供 ROS 2 后处理节点与可选 Gazebo 系统插件，与 [`usv_sim_full`](../../usv_sim_full/) 整机仿真配合使用。

> **整机默认集成方式**：`usv_sim_full` + URDF `gpu_ray` → `ros_gz_bridge` → 本包 `mmwave_4d_cloud_node` →（可选）`mmwave_cluster_node`。  
> **不强制**使用 `FourDRadarPlugin`。详细步骤见 [`docs/INTEGRATION_GUIDE_CN.md`](docs/INTEGRATION_GUIDE_CN.md)。

- **License:** Apache-2.0
- **ROS 2:** `rclcpp` / `rclpy`（可视化节点计划迁入）
- **仿真后端:** Gazebo Sim 8、`gz-plugin2`、`gz-math7`

---

## 功能概览

| 能力 | 说明 |
|------|------|
| 4D 点云（整机主链路） | `gpu_ray` 几何命中 + `mmwave_4d_cloud_node` 补 `doppler_velocity`、`rcs` |
| 点云聚类 | `mmwave_cluster_node`：3D DBSCAN → `usv_interfaces/MmwaveTargetArray` |
| 专用 Gazebo 插件 | `FourDRadarPlugin`：独立验证/对照，插件内直接发布五字段点云 |
| 动态目标插件 | `ReciprocatingTargetsPlugin`：供验证 world 往复运动目标（非整机必需） |

---

## 组件与源码路径

| 组件 | 路径 | 作用 |
|------|------|------|
| URDF 宏（gpu_ray） | [`usv_sim_full/description/urdf/sensor_macros.xacro`](../../usv_sim_full/description/urdf/sensor_macros.xacro) `mmwave_radar_macro` | Gazebo 几何探测（与 VLP16 同款 `gpu_ray`） |
| `mmwave_4d_cloud_node` | [`src/mmwave_4d_cloud_node.cpp`](src/mmwave_4d_cloud_node.cpp) | 订阅 `points_gz`，发布五字段 `points` |
| `mmwave_cluster_node` | [`src/mmwave_cluster_node.cpp`](src/mmwave_cluster_node.cpp) | 聚类 → `MmwaveTargetArray` |
| `FourDRadarPlugin` | [`src/4d_radar_plugin.cpp`](src/4d_radar_plugin.cpp) → `libusv_4d_radar_plugin.so` | **独立验证**（非整机主链路） |
| 验证 spawn | [`launch/spawn_ego_mmwave_validation.launch.py`](launch/spawn_ego_mmwave_validation.launch.py) | 在已运行 Gz 世界中插入插件探测体 |
| 聚类默认参数 | [`config/mmwave_cluster.yaml`](config/mmwave_cluster.yaml) | DBSCAN 等参数（整机需改 `input_topic`） |

---

## 依赖

**构建:** `ament_cmake`、`rclcpp`、`sensor_msgs`、`nav_msgs`、`tf2*`、`usv_interfaces`、`gz-cmake3`、`gz-plugin2`、`gz-sim8`、`gz-math7`

**运行:** `launch_ros`、`rclpy`、`visualization_msgs`（可视化节点迁入后）、`ros_gz_sim`（验证 launch）

`usv_sim_full` 已在 [`package.xml`](../../usv_sim_full/package.xml) 中声明 `<depend>usv_mmwave_sim</depend>`。

---

## 构建与安装

在工作区根目录：

```bash
colcon build --packages-select usv_interfaces usv_mmwave_sim usv_sim_full --symlink-install
source install/setup.bash
```

或仅编本包：

```bash
colcon build --packages-select usv_mmwave_sim --symlink-install
source install/setup.bash
```

安装产物：

| 类型 | 路径 |
|------|------|
| Gazebo 插件 | `install/usv_mmwave_sim/lib/libusv_4d_radar_plugin.so` |
| ROS 节点 | `install/usv_mmwave_sim/lib/usv_mmwave_sim/mmwave_4d_cloud_node`、`mmwave_cluster_node` |
| 配置 / launch | `install/usv_mmwave_sim/share/usv_mmwave_sim/{config,launch,docs}/` |

---

## 与 usv_sim_full 集成（摘要）

1. 在 [`full_config.yaml`](../../usv_sim_full/config/full_config.yaml) 的 `robot_N.sensors` 中配置 `type: mmwave_radar`。
2. 内参在 [`sensor_config.yaml`](../../usv_sim_full/config/sensor_config.yaml) 的 `mmwave.default`。
3. `session_manager` 生成 URDF（`gpu_ray`）与 bridge YAML（`…/points_gz`）。
4. `main.launch.py` 在配置含毫米波传感器时自动启动 **`mmwave_4d_cloud_node`** 与 **`mmwave_cluster_node`**，发布 `…/points` 与 `…/objects`。详见集成文档。

完整数据流、话题命名、launch 现状与排障：**[`docs/INTEGRATION_GUIDE_CN.md`](docs/INTEGRATION_GUIDE_CN.md)**。

---

## 快速验证

```bash
source install/setup.bash
ros2 launch usv_sim_full main.launch.py \
  config_path:=$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/config/full_config.yaml

# 另终端确认毫米波链路（main.launch 已自动启动 4d_cloud + cluster）
ros2 node list | grep mmwave
ros2 topic hz /usv_1/sensors/mmwave/mmwave_front/points
ros2 topic echo /usv_1/sensors/mmwave/mmwave_front/objects --once
```

### FourDRadarPlugin 独立验证

```bash
source install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(ros2 pkg prefix usv_mmwave_sim)/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
ros2 launch usv_mmwave_sim spawn_ego_mmwave_validation.launch.py world:=sydney_regatta_open_water
ros2 topic echo /mmwave_spawn_test/points --once
```

---

## 与海事导航雷达的区别

本包为 **毫米波（mmWave）** 仿真。`maritime_radar`（扇区/栅格）由 `gz_maritime_radar_plugin` + `radar_gz_bridge` + `gy_radar_driver` 处理，与本包无关。

---

## 备注

- 扩展验证资源（独立 world、`static_cluster_validation` launch、benchmark 脚本等）见同目录旁 **`usv_mmwave_sim (back_try)/`** 备份，尚未并入正式包安装路径。
- `mmw_objects_markers`（`MmwaveTargetArray` → RViz `MarkerArray`）计划从 `back_try` 迁入正式包。
