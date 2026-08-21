# CCS UTM Chart / ENU Simulation Alignment (Plan A)

## 1. Purpose

Plan A keeps the electronic chart and route planner in the existing local
UTM Zone 50N coordinate system while Gazebo continues to use the local ENU
frame defined by `ccs_open_water.sdf`.

The two frames are connected by one static `map -> usv_1/odom` transform.
This avoids reprojecting the S-57 chart and avoids changing the route planner
GPS conversion algorithm.

## 2. Coordinate definitions

CCS Gazebo datum (`worlds/ccs_open_water.sdf`):

```text
latitude:  34.692120
longitude: 119.481403
elevation: 0.0 m
axes:      ENU (x east, y north, z up)
```

Chart projection and local reference (`CN441122_aids_crop.metadata.json`):

```text
projection: EPSG:32650 (UTM Zone 50N)
map(0, 0) UTM: [736235.0, 3846381.0]
resolution: 2.0 m/cell
size: 2501 x 2001 cells
YAML origin: [-10501.0, -6501.0, 0.0]
```

The target initial GPS position is:

```text
longitude: 119.481403
latitude:  34.692120
```

The world datum is intentionally the initial GPS position, so its CCS ENU
position is:

```text
x: 0.0 m
y: 0.0 m
```

Keeping the boat and the finite VRX water mesh near the Gazebo origin avoids
an invisible boat 5 km away from the default camera and improves simulation
numerical stability. This does not change the boat's UTM/map coordinate.

## 3. map -> odom transform

The CCS datum is at UTM:

```text
[727302.206702, 3841703.729778]
```

Relative to the chart reference, the ENU/Gazebo origin is therefore:

```text
x = 727302.206702 - 736235.0  = -8932.793298
y = 3841703.729778 - 3846381.0 = -4677.270222
```

The UTM meridian convergence at the datum is approximately:

```text
yaw = +1.412936707 degrees = +0.0246604 radians
```

The transform used by TF is:

```text
map -> usv_1/odom
translation: [-8932.7933, -4677.2702, 0.0]
rotation RPY: [0.0, 0.0, 0.0246604]
```

Only one node may publish this transform. The identity transform in
`main.launch.py` and the second identity transform in the grounding warning
launch are disabled by launch arguments.

## 4. Implemented files

### `config/three_vision_one_mmwave/ccs_config.yaml`

The robot is spawned in the Gazebo ENU world frame:

```yaml
spawn_pose:
- 0.0
- 0.0
- 0.5
- 0.0
- 0.0
- 0.0
```

### `launch/CCS_Certified_Simulation_Environment.launch.py`

The CCS wrapper:

- forwards `use_static_map_odom_tf:=false` to disable the identity transform
  from `main.launch.py`;
- forwards `publish_identity_map_odom_tf:=false` to the grounding warning
  launch;
- publishes the single non-identity `map -> usv_1/odom` transform;
- exposes `ccs_map_to_odom_x`, `ccs_map_to_odom_y`, and
  `ccs_map_to_odom_yaw` as launch arguments;
- enables Gazebo vessel following after spawn by default; set
  `enable_gazebo_camera_follow:=false` to disable it.

### `config/robot_localization_gps.yaml`

Humble `navsat_transform_node` uses these parameter names:

```yaml
wait_for_datum: true
use_local_cartesian: false
datum: [34.692120, 119.481403, 0.0]
```

`wait_for_datum: true` selects the manual `datum` parameter. The older
`datum_latitude`, `datum_longitude`, and `use_utm` names are not valid for the
Humble implementation used by this workspace.

The default CCS launch keeps `enable_robot_localization:=false`; these
parameters take effect when localization is explicitly enabled.

### `/home/cat/cc_enc/src/usv_route_planner/config/route_planner.yaml`

The geometry guard is updated for the cropped chart:

```yaml
expected_map_width: 2501
expected_map_height: 2001
expected_map_resolution: 2.0
expected_map_origin_x: -10501.0
expected_map_origin_y: -6501.0
```

The existing UTM conversion remains unchanged:

```yaml
utm_zone: 50
utm_north: true
utm_reference_easting: 736235.0
utm_reference_northing: 3846381.0
```

### `usv_sim_full/scripts/session_manager.py`

Generated RViz sessions now include a `rviz_default_plugins/Map` display for
`/map`. Without this display, TF and Nav2 can be correct while the static chart
is invisible in RViz.

### `worlds/ccs_open_water.sdf`

The spherical datum is centered on the initial GPS position and the default
camera height is reduced from 140 m to 40 m. The boat and the finite water
visual are therefore both near the Gazebo origin and visible at startup. A
`CameraTracking` GUI plugin follows `usv_1` from offset `(-20, -20, 12)`, so
the vessel remains visible while navigating. Press `Esc` in Gazebo to release
the camera.

### `config/radar_nav2_param.yaml`

Both costmaps use the vessel footprint instead of Nav2's tiny default radius:

```yaml
footprint: "[[2.5, 1.0], [2.5, -1.0], [-2.5, -1.0], [-2.5, 1.0]]"
footprint_padding: 0.5
```

The published collision footprint is 6 m by 3 m after padding.

### `rviz/three_vision_one_mmwave.rviz`

The CCS RViz override now:

- reads the robot model from `/usv_1/robot_description`;
- displays `/map` and `/usv_1/global_costmap/costmap`;
- displays `/usv_1/global_costmap/published_footprint` in red;
- follows `usv_1/base_link` at a useful startup zoom level.

## 5. Build

Build the simulation package inside the Humble development container:

```bash
docker start usv_ws_test-dev-1
docker exec -it usv_ws_test-dev-1 bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --packages-select usv_sim_full --symlink-install
source install/setup.bash
```

Build the route planner in the cc_enc Jazzy workspace:

```bash
cd /home/cat/cc_enc
source /opt/ros/jazzy/setup.bash
colcon build --packages-select usv_route_planner --symlink-install
source install/setup.bash
```

## 6. Launch

Normal CCS launch:

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py
```

For coordinate verification without RTSP dependencies:

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py \
  enable_camera_rtsp_streaming:=false \
  enable_map_rtsp_streamer:=false \
  enable_dynamic_ship_gt_bridge:=false
```

Do not start `nav2_sim_three_vision_mmwave_bringup.launch.py` directly with
the CCS config unless the same non-identity transform is provided. The CCS
wrapper owns this frame alignment.

## 7. Verification commands

```bash
ros2 topic echo /map --once --field info
ros2 topic echo /usv_1/odom --once --field pose.pose.position
ros2 run tf2_ros tf2_echo map usv_1/odom
ros2 run tf2_ros tf2_echo map usv_1/base_link
ros2 topic echo /usv_1/sensors/gps/gps_sensor/data --once
ros2 lifecycle get /usv_1/planner_server
```

Expected values:

```text
/map:
  resolution = 2.0
  width = 2501
  height = 2001
  origin = (-10501.0, -6501.0)

/usv_1/odom:
  x ~= 0.0
  y ~= 0.0

map -> usv_1/odom:
  translation ~= (-8932.7933, -4677.2702)
  yaw ~= 0.0246604 rad

map -> usv_1/base_link:
  x ~= -8932
  y ~= -4677
```

## 8. Verification result (2026-08-19)

The implemented configuration was built and launched in
`usv_ws_test-dev-1`. The measured values were:

```text
map: 2501 x 2001 @ 2.0 m, origin (-10501.0, -6501.0)
Gazebo spawn position: (0.0, 0.0, 0.5)
odom position after wave settling: (-0.632, 0.022)
map -> odom: (-8932.793, -4677.270), yaw 1.413 deg
map -> base_link: (-8933.43, -4677.26)
GPS: latitude 34.69212303, longitude 119.48139492
global footprint: approximately 6.0 m x 3.0 m
planner_server: active
```

The target occupancy-grid cell is `(784, 911)`, corresponding to PGM pixel
`(784, 1089)` with value `254` (free). The measured GPS error was below 1 m,
and the map position was inside the cropped chart.

Only one `static_transform_publisher` was present for `map -> usv_1/odom`.

The Humble navsat parameters were also loaded in a standalone parameter
check:

```text
wait_for_datum: true
use_local_cartesian: false
datum: [34.692120, 119.481403, 0.0]
```

Automated test results:

```text
usv_sim_full: 0 tests, 0 failures
usv_route_planner: 40 tests, 0 failures
```

## 9. Accuracy and limits

The transform is a rigid approximation between the Gazebo local ENU tangent
frame and the UTM grid. Over the approximately 5 km operating area, the
remaining projection scale error is below the 2 m chart resolution. For much
larger areas, reprojecting the chart into a local ENU map should be evaluated.

## 10. Rollback

To restore the earlier off-origin configuration, set the world datum to
`(34.69149699539598, 119.53601222131164)`, spawn the boat at
`(-5003.776, 70.470)`, and restore the transform to
`(-3927.5745, -4621.6071, 0.0252032)`. This is not recommended because the
boat is outside the finite water visual and default camera view.
