# CCS 方案 A 全部修改记录

> 历史方案：本方案记录旧的 UTM 裁剪地图对齐方式。当前
> `feature/ccs_world` 默认使用 `CN441122_enc_5km` 局部 ENU 地图、零位姿
> `map -> odom` 以及 LocalCartesian GPS 转换；以下非零 TF 和旧地图参数不再是
> 当前运行默认值。

## 1. 修改目标

本次修改解决以下问题：

1. 保留现有 EPSG:32650 UTM 裁剪海图和 route planner 坐标算法。
2. 将 Gazebo ENU 仿真坐标正确接入海图 `map` 坐标。
3. 保证系统中只有一个非零 `map -> usv_1/odom` 变换。
4. 让无人船在 Gazebo 水面和默认相机中可见。
5. 让 RViz 显示无人船模型、静态海图、global costmap 和船体 footprint。
6. 修正 Nav2 使用的实际船体碰撞轮廓。
7. 修正 ROS 2 Humble `navsat_transform_node` 参数。

## 2. 最终坐标定义

### 2.1 Gazebo ENU datum

文件：`worlds/ccs_open_water.sdf`

```text
latitude:  34.692120
longitude: 119.481403
elevation: 0.0 m
axes:      ENU
```

该 datum 与无人船初始目标 GPS 一致，因此无人船可以生成在 Gazebo
`(0, 0)`，不再生成在距离场景原点约 5 km 的位置。

### 2.2 海图坐标

```text
projection: EPSG:32650 / UTM Zone 50N
UTM reference: [736235.0, 3846381.0]
map resolution: 2.0 m/cell
map size: 2501 x 2001
map origin: [-10501.0, -6501.0, 0.0]
```

目标 GPS 的 UTM 坐标：

```text
easting:  727302.206702
northing: 3841703.729778
```

目标在局部海图中的坐标：

```text
map_x = 727302.206702 - 736235.0  = -8932.793298
map_y = 3841703.729778 - 3846381.0 = -4677.270222
```

UTM 子午线收敛角：

```text
1.412936707 degrees
0.0246604 radians
```

### 2.3 最终 TF

```text
map -> usv_1/odom
translation: [-8932.7933, -4677.2702, 0.0]
rotation RPY: [0.0, 0.0, 0.0246604]
```

`main.launch.py` 和 `enc_grounding_warning` 中原有的 identity
`map -> odom` 发布均被关闭，避免 TF 冲突。

## 3. 按文件记录的修改

### 3.1 `config/three_vision_one_mmwave/ccs_config.yaml`

修改场景说明中的 datum，并将无人船生成位置改为 Gazebo ENU 原点：

```yaml
robot_1:
  name: usv_1
  spawn_pose:
  - 0.0
  - 0.0
  - 0.5
  - 0.0
  - 0.0
  - 0.0
```

原配置把船生成在 `(-5003.776, 70.470)`。该位置虽然 GPS 正确，但已
超出默认相机和有限水面 visual 的有效区域。

### 3.2 `worlds/ccs_open_water.sdf`

修改 spherical coordinates：

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>34.692120</latitude_deg>
  <longitude_deg>119.481403</longitude_deg>
  <elevation>0.0</elevation>
  <heading_deg>0.0</heading_deg>
</spherical_coordinates>
```

将默认相机高度从 140 m 改为 40 m：

```xml
<camera_pose>0 0 40 0 1.2 0</camera_pose>
```

增加 Gazebo CameraTracking：

```xml
<plugin filename="CameraTracking" name="Camera Tracking">
  <follow_target>usv_1</follow_target>
  <follow_offset>-20 -20 12</follow_offset>
  <follow_pgain>0.05</follow_pgain>
</plugin>
```

Gazebo 启动后会从船后上方跟随 `usv_1`。在 Gazebo 窗口按 `Esc` 可以
解除跟随。

### 3.3 `launch/CCS_Certified_Simulation_Environment.launch.py`

基础 bringup 传入：

```text
use_static_map_odom_tf:=false
```

安全模块传入：

```text
publish_identity_map_odom_tf:=false
```

CCS wrapper 发布唯一的非零 TF，默认 launch 参数为：

```text
ccs_map_to_odom_x:   -8932.7933
ccs_map_to_odom_y:   -4677.2702
ccs_map_to_odom_yaw: 0.0246604
```

增加 Gazebo 自动跟随函数 `_gazebo_camera_follow()`。启动 8 秒后调用：

```bash
gz service \
  -s /gui/follow \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'data: "usv_1"'
```

新增 launch 参数：

```text
enable_gazebo_camera_follow:=true
```

不需要自动跟随时可使用：

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py \
  enable_gazebo_camera_follow:=false
```

### 3.4 `config/robot_localization_gps.yaml`

使用 ROS 2 Humble 支持的 navsat 参数：

```yaml
navsat_transform:
  ros__parameters:
    wait_for_datum: true
    use_local_cartesian: false
    datum: [34.692120, 119.481403, 0.0]
```

Humble 不使用以下旧参数名：

```text
use_utm
datum_latitude
datum_longitude
```

CCS 默认仍使用 `enable_robot_localization:=false`。显式启用 localization
时，上述 datum 配置才参与 `navsat_transform_node` 计算。

### 3.5 `config/radar_nav2_param.yaml`

global costmap 和 local costmap 均增加实际船体 footprint：

```yaml
footprint: "[[2.5, 1.0], [2.5, -1.0], [-2.5, -1.0], [-2.5, 1.0]]"
footprint_padding: 0.5
```

原配置未启用 footprint，Nav2 使用约 0.2 m 的默认圆形轮廓。修改后发布
的碰撞轮廓约为 6 m x 3 m，与无人船尺寸匹配。

需要注意，global costmap 不会把本船标记成静态障碍物。本船应通过
RobotModel 和 `/usv_1/global_costmap/published_footprint` 显示。

### 3.6 `rviz/three_vision_one_mmwave.rviz`

修正 RobotModel 描述话题：

```yaml
Description Source: Topic
Description Topic:
  Value: /usv_1/robot_description
```

新增静态海图显示：

```yaml
Class: rviz_default_plugins/Map
Name: Chart (static map)
Alpha: 1.0
Topic:
  Value: /map
  Durability Policy: Transient Local
```

静态海图使用完全不透明的 `map` 黑白配色。为避免透明叠加改变海图颜色，
Global Costmap 和雷达 occupancy map 显示项默认关闭，但仍保留在 RViz
Displays 列表中，可按需手动开启。

新增 global costmap 显示：

```yaml
Class: rviz_default_plugins/Map
Name: Global Costmap
Enabled: false
Topic:
  Value: /usv_1/global_costmap/costmap
```

新增红色船体轮廓显示：

```yaml
Class: rviz_default_plugins/Polygon
Name: USV Global Footprint
Color: 255; 25; 0
Topic:
  Value: /usv_1/global_costmap/published_footprint
```

默认 RViz 视角改为跟随船体：

```yaml
Distance: 30
Target Frame: usv_1/base_link
Focal Point:
  X: 0
  Y: 0
  Z: 0
```

### 3.7 `usv_sim_full/scripts/session_manager.py`

由 session manager 动态生成的 RViz 配置也增加 `/map` 显示，QoS 使用：

```text
Reliability: Reliable
Durability: Transient Local
```

这项修改覆盖不使用 `three_vision_one_mmwave.rviz` override 的其他启动
方式。

### 3.8 `/home/cat/cc_enc/src/usv_route_planner/config/route_planner.yaml`

route planner 的地图 geometry guard 修改为裁剪海图参数：

```yaml
expected_map_width: 2501
expected_map_height: 2001
expected_map_resolution: 2.0
expected_map_origin_x: -10501.0
expected_map_origin_y: -6501.0
```

UTM reference 保持不变：

```yaml
utm_zone: 50
utm_north: true
utm_reference_easting: 736235.0
utm_reference_northing: 3846381.0
```

### 3.9 地图文件

使用以下裁剪地图：

```text
maps/CN441122_aids_crop.yaml
maps/CN441122_aids_crop.pgm
```

地图参数：

```text
size: 2501 x 2001
resolution: 2.0 m/cell
origin: [-10501.0, -6501.0, 0.0]
```

目标位置对应：

```text
occupancy-grid cell: (784, 911)
PGM pixel: (784, 1089)
PGM value: 254 / free
```

## 4. 构建命令

### 4.1 Humble 仿真包

```bash
docker start usv_ws_test-dev-1
docker exec -it usv_ws_test-dev-1 bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-select usv_sim_full --symlink-install
source install/setup.bash
```

### 4.2 Jazzy route planner

```bash
cd /home/cat/cc_enc
source /opt/ros/jazzy/setup.bash
colcon build --packages-select usv_route_planner --symlink-install
source install/setup.bash
```

## 5. 启动命令

完整 CCS：

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py
```

不启动 RTSP 和动态目标 bridge 的验证模式：

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py \
  enable_camera_rtsp_streaming:=false \
  enable_map_rtsp_streamer:=false \
  enable_dynamic_ship_gt_bridge:=false
```

不要直接用 CCS 配置启动底层
`nav2_sim_three_vision_mmwave_bringup.launch.py`，除非同时提供相同的非零
`map -> odom`。默认由 CCS wrapper 负责该坐标对接。

## 6. 验证命令

```bash
ros2 topic echo /map --once --field info
ros2 topic echo /usv_1/odom --once --field pose.pose.position
ros2 topic echo /usv_1/sensors/gps/gps_sensor/data --once
ros2 topic echo /usv_1/global_costmap/published_footprint --once
ros2 run tf2_ros tf2_echo map usv_1/odom
ros2 run tf2_ros tf2_echo map usv_1/base_link
ros2 lifecycle get /usv_1/planner_server
gz topic -e -t /gui/currently_tracked
```

## 7. 实测结果

2026-08-19 在 `usv_ws_test-dev-1` 中重建并启动后：

```text
Gazebo spawn position: (0.0, 0.0, 0.5)
odom after wave settling: approximately (-0.2, -0.01)
GPS latitude: 34.692123
GPS longitude: 119.481395
map -> odom: (-8932.7933, -4677.2702), yaw 0.0246604 rad
planner_server: active
global footprint: approximately 6.0 m x 3.0 m
Gazebo camera tracking mode: FOLLOW
Gazebo camera target: usv_1
Gazebo camera offset: (-20, -20, 12)
```

RViz 已确认订阅：

```text
/usv_1/robot_description
/map
/usv_1/global_costmap/costmap
/usv_1/global_costmap/published_footprint
```

运行时仅有一套 CCS launch、Gazebo 和 RViz 实例。

## 8. 测试结果

```text
usv_sim_full: 0 tests, 0 failures
usv_route_planner: 40 tests, 0 failures
Python launch syntax: passed
YAML parsing: passed
SDF XML parsing: passed
Nav2 readiness gate: passed
```

## 9. 修改文件汇总

```text
src/usv_simulation/usv_sim_full/config/three_vision_one_mmwave/ccs_config.yaml
src/usv_simulation/usv_sim_full/config/radar_nav2_param.yaml
src/usv_simulation/usv_sim_full/config/robot_localization_gps.yaml
src/usv_simulation/usv_sim_full/launch/CCS_Certified_Simulation_Environment.launch.py
src/usv_simulation/usv_sim_full/rviz/three_vision_one_mmwave.rviz
src/usv_simulation/usv_sim_full/usv_sim_full/scripts/session_manager.py
src/usv_simulation/usv_sim_full/description/world/ccs_open_water.sdf
src/usv_simulation/usv_sim_full/docs/utm_enu_alignment_plan_a.md
/home/cat/cc_enc/src/usv_route_planner/config/route_planner.yaml
```

本汇总文档：

```text
src/usv_simulation/usv_sim_full/docs/ccs_plan_a_all_changes.md
```
