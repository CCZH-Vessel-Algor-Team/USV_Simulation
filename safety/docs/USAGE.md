# USV Safety 使用说明

本目录为仿真版电子海图搁浅预警功能，包含三个包：

| 包 | 说明 |
|----|------|
| `enc_grounding_warning_msgs` | 搁浅预警相关的消息与服务定义 |
| `enc_grounding_warning` | 水深 Provider、UKC 估算、前瞻告警、航线校核、航线走廊可视化 |
| `enc_grounding_warning_rviz` | RViz 面板插件，展示 UKC / 风险 / 告警 / 水深统计 |

---

## 1. 快速开始

### 1.1 构建

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select \
  enc_grounding_warning_msgs \
  enc_grounding_warning \
  enc_grounding_warning_rviz \
  --symlink-install
source install/setup.bash
```

### 1.2 单元测试

```bash
python3 -m pytest -p no:anyio src/usv_simulation/safety/enc_grounding_warning/test -q
```

预期：`12 passed`。

### 1.3 ROS 节点级闭环测试（无需 Gazebo/Nav2）

终端 1：

```bash
ros2 launch enc_grounding_warning enc_grounding_warning.launch.py use_sim_time:=false
```

终端 2：

```bash
ros2 run enc_grounding_warning integration_test
```

预期输出：

```text
PASS: ukc_state received; alert level=3 ...
PASS: route_check OK: first_unsafe=0 ...
PASS: route_depth_grid received ...
PASS: route_depth_markers received ...
PASS: closed-loop integration test
```

### 1.4 全仿真启动

```bash
ros2 launch enc_grounding_warning full_sim_with_grounding_warning.launch.py
```

CCS 认证环境可从仿真包直接启动（`usv_sim_full` 已集成安全节点）：

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py
```

常用参数：

```bash
ros2 launch enc_grounding_warning full_sim_with_grounding_warning.launch.py \
  config_path:=/path/to/full_config.yaml \
  gz_headless:=true \
  auto_cleanup:=false \
  enable_rviz:=false \
  enable_gt_sensor_sim:=false \
  enable_late_fusion:=false \
  enable_convert_to_trackship:=false
```

注意：

- `enable_keepout_filter` 默认 `true`，VORRTStar 规划依赖 `vector_object_server`，请勿关闭；
- 无显示器环境使用 `gz_headless:=true`；
- 需要 RViz 交互时保持 `enable_rviz:=true`（默认）。

---

## 2. 节点与 Launch

### 2.1 节点

| 节点 | 功能 |
|------|------|
| `depth_provider_node` | 加载水深矩阵，发布船体附近的稠密 `DepthGrid` |
| `ukc_estimator_node` | 根据船位/船速计算当前 UKC，发布 `UKCState` 与风险栅格 |
| `grounding_warning_node` | 订阅 Nav2 `/plan`，做前瞻告警，提供航线校核服务 |
| `route_depth_publisher_node` | 发布航线走廊水深栅格与彩色 Marker |

### 2.2 `enc_grounding_warning.launch.py`

仅启动预警节点（适用于仿真已经运行的情况）：

```bash
ros2 launch enc_grounding_warning enc_grounding_warning.launch.py \
  namespace:=usv_1 \
  use_sim_time:=true
```

主要参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `namespace` | `usv_1` | ROS 命名空间 |
| `use_sim_time` | `true` | 是否使用仿真时钟 |
| `depth_grid_file` | `config/sim_depth_grid.yaml` | 水深矩阵配置 |
| `params_file` | `config/grounding_warning_params.yaml` | UKC/告警参数 |
| `enable_depth_provider` | `true` | 是否启动近场水深发布 |
| `enable_ukc` | `true` | 是否启动 UKC 估算 |
| `enable_grounding_warning` | `true` | 是否启动前瞻告警 |
| `enable_route_depth_publisher` | `true` | 是否启动航线走廊可视化 |

### 2.3 `full_sim_with_grounding_warning.launch.py`

一键启动“现有仿真 + Nav2 + 预警节点 + RViz”。

`usv_sim_full` 的
`CCS_Certified_Simulation_Environment.launch.py` 是另一条集成入口：
它在 CCS 三视觉/毫米波环境中默认 Include `enc_grounding_warning.launch.py`，
可通过 `enable_safety:=false` 关闭，或通过
`safety_depth_grid_file` / `safety_params_file` 覆盖水深与预警参数。

主要参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_path` | `usv_sim_full` 默认 full_config | 仿真整船配置 |
| `nav2_namespace` | `auto` | Nav2/机器人命名空间 |
| `gz_headless` | `false` | 是否无界面运行 Gazebo |
| `enable_rviz` | `true` | 是否启动 RViz |
| `enable_keepout_filter` | `true` | 是否启动 keepout/vector_object_server |
| `depth_grid_file` | `""` | 为空时使用包内默认水深配置 |
| `gw_params_file` | `config/grounding_warning_params.yaml` | 预警参数 |

---

## 3. 水深数据配置

### 3.1 默认合成水深 `sim_depth_grid.yaml`

支持：

- 平坦水深 + 正弦起伏；
- 多个圆形浅点（shoal）；
- 或直接读取 CSV 水深矩阵。

```yaml
depth_grid:
  frame_id: map
  origin_x: -2000.0
  origin_y: -2000.0
  resolution: 5.0
  width: 800
  height: 800
  mode: flat              # flat / file
  data_file: ""           # mode=file 时必填
  depth_m: 3.0
  undulation:
    enabled: true
    amplitude_m: 1.2
    wavelength_m: 50.0
    direction_deg: 30.0
  uncertainty_m: 0.1
  quality: 3              # CATZOC B
  shoals:
    - {x: 300.0, y: 250.0, radius: 50.0, depth_m: 0.5}
```

### 3.2 CSV 水深矩阵

`mode: file` 时，`data_file` 指向 CSV：

- 行列顺序：row-major，`index = y_index * width + x_index`；
- `x = origin_x + x_index * resolution`；
- `y = origin_y + y_index * resolution`；
- 无效水深使用 `-9999`；
- 每行用逗号分隔，行数 = `height`，列数 = `width`。

可用 `generate_depth_grid` 从 YAML 生成 CSV：

```bash
ros2 run enc_grounding_warning generate_depth_grid --config config/sim_depth_grid.yaml --output data/depth_grid.csv
```

---

## 4. 常用参数说明

`grounding_warning_params.yaml` 中 `ukc:` 下的关键参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `static_draft_m` | 0.5 | 静态吃水 |
| `length_m` / `beam_m` | 4.9 / 2.0 | 船长/船宽 |
| `block_coefficient` | 0.55 | 方形系数，用于 squat |
| `ukc_required_abs_m` | 0.3 | 最小绝对 UKC |
| `ukc_required_pct` | 0.10 | 按动态吃水百分比的最低 UKC |
| `water_level_m` | 0.0 | 水位（相对海图基准面） |
| `margin_safe_m` | 0.1 | 安全缓冲 |
| `hysteresis_m` | 0.1 | 告警清除迟滞 |
| `debounce_count` | 3 | 连续 N 拍才触发告警 |
| `lookahead_time_s` | 300 | 前瞻时间 |
| `lookahead_distance_m` | 1000 | 前瞻距离 |
| `sample_ds_m` | 2.0 | 航线采样步长 |
| `plan_immediate_min_interval_s` | 0.2 | 收到新 plan 后立即计算的限流间隔 |
| `route_corridor_half_width_m` | 10.0 | 航线走廊半宽 |
| `route_grid_resolution_m` | 1.0 | 航线走廊栅格分辨率 |

---

## 5. 常见问题

### 5.1 `ros2 topic echo` 提示消息类型 invalid

```bash
source install/setup.bash
ros2 daemon stop
ros2 daemon start
```

### 5.2 `/usv_1/plan` 为空

确认 Nav2 已 `Managed nodes are active`，并且 BT 使用 `VORRTStar` 时
`enable_keepout_filter:=true`（默认）。

### 5.3 提示 `libwait_at_waypoint.so: file too short`

重新构建：

```bash
colcon build --packages-select nav2_waypoint_follower --symlink-install
```
