# `usv_sim_full/config` 配置说明

**用户向速览**（环境 → 怎么跑 → 怎么配 → 话题）：[`docs/docs_v3/QUICK_START.md`](../../docs/docs_v3/QUICK_START.md)。

本目录为 **`usv_sim_full` 包内随包安装的资源**（`setup.py` 中 `data_files` 安装 `config/*`）。  
部分 YAML 由 **固定 launch** 直接读取；**整机仿真**路径下，更多产物由 **`session_manager`** 按 `full_config` 在**会话目录**（如 `/tmp/usv_sim_sessions/...`）生成，不在本目录重复列出，见文末「会话产物」。

---

## 文件一览与消费方

| 文件 | 内容摘要 | 谁读 / 谁传 |
|------|----------|-------------|
| **`full_config.yaml`** | **整机主配置**：`environment`、`robot_1`/`robot_2`/…、`sensors`、`obstacles`、`scenario`、`visualization`；可选每船 `enable_env_dynamics` / `env_dynamics`；`sensor_config_path` 指向传感器内参 YAML。 | **入口**：`main.launch.py` 默认 `config_path`；`sensor_tune.launch.py`、`nav2_sim_full_bringup.launch.py`、`test_hull.launch.py` 等以默认或 `config_path:=` 引用；毫米波最小场景对 `main.launch.py` 传 `config_path:=.../mmwave_sydney_minimal.yaml`。**核心消费者**：`ros2 run usv_sim_full session_manager --config-path`（子进程，由上述 launch 触发）；`scenario_manager_node`（参数 `config_path`）；`launch_config_helpers.primary_robot_name` 等。 |
| **`full_config.reference.yaml`** | **同结构的带注释参考**：逐块说明各键含义与消费者；**默认不被 launch 加载**。 | 人工查阅；修改默认行为请编辑 `full_config.yaml`（或自定义 `config_path`）。若与代码不一致以 `session_manager` / `main.launch.py` 为准。 |
| **`sensor_config.yaml`** | **传感器内参/默认值**：`lidar`/`camera`/`imu`/`gps`/`mmwave` 等数值，供 xacro 与 `mmwave_4d_cloud_node` 默认参数使用。 | **`session_manager.py`**：按 `full_config` 顶栏 `sensor_config_path` 解析路径，拷贝到会话目录 `sensor_config.yaml` 并参与 URDF/桥接生成。**`main.launch.py`**：`launch_config_helpers.load_mmwave_sensor_defaults()` 读 `mmwave.default`。**`description/urdf/sensor_params.xacro`**：`load_yaml('$(find usv_sim_full)/config/sensor_config.yaml')`（开发时直连包内文件；完整会话以 session 内副本为准）。 |
| **`mmwave_sydney_minimal.yaml`** | **毫米波最小场景**：`sydney_regatta`、精简障碍/场景、特定 `spawn_pose`、默认关 RViz；结构与 `full_config` 兼容。 | **`main.launch.py`**：`config_path:=$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/config/mmwave_sydney_minimal.yaml`（或源码树等价路径）。 |
| **`certi_senario.yaml`** | **认证仿真基底**：单船 `robot_1`、全传感器、`sydney_regatta_open_water`、默认开 RViz；无 `ground_truth_sim`。 | 由 **`certifi_launch.launch.py`** 作 `base_config`；与 **`certificate_case/*.yaml`** 经 **`tools/merge_certi_config.py`** 合并为 `config/generated/*.merged.yaml` 再喂给 `main.launch.py`。 |
| **`certificate_case/*.yaml`** | **会遇语义**（`encounter`）：局面类型、DCPA、本船/目标速度航向。 | **`merge_certi_config.py`**；launch 参数 `case_config:=`。 |
| **`generated/*.merged.yaml`** | 合并产物（勿手改）；含 `scenario.dynamic_obstacles` 与 `certificate_runtime`。 | **`certifi_launch.launch.py`** → `main.launch.py` 的 `config_path`。 |
| **`robot_localization_gps.yaml`** | **robot_localization**：`ekf_filter_node`、`navsat_transform` 等滤波与坐标约定（与仿真 GPS/IMU/odom 配合）。 | **`main.launch.py`**：`localization_params_file` 的默认值；经 **`robot_bringup.launch.py`** 在 `enable_robot_localization:=true` 时加载。**`nav2_sim_full_bringup.launch.py`**：声明默认并转发给 `main.launch.py`（`localization_params_file`）。 |
| **`radar_nav2_param.yaml`** | **Nav2 栈参数**：代价地图、规划器、`static_layer` 等；运行中常被改写 `map_topic`。 | **`nav2_sim_full_bringup.launch.py`**：`params_file` 默认值（源码树 `gy_radar_driver-main/config/...` 优先，否则本文件，再退回 `gy_radar_driver` 包）。**`nav2_thruster_bringup.launch.py`**：自身默认解析链同上；**OpaqueFunction** 内读入并写入 `/{namespace}/map/navradar/occupancy_grid` 后再 **Include `nav2_bringup/navigation_launch.py`**。 |
| **`global_bridge.yaml`** | **全局 Gz↔ROS 桥**：当前主要为 **`/clock`**（`GZ_TO_ROS`）。 | **`launch/components/infra_sim.launch.py`**：`ros_gz_bridge/parameter_bridge` 的 `config_file`（硬编码 `share/usv_sim_full/config/global_bridge.yaml`）。 |
| **`tf_tune.rviz`** | **RViz2 布局**：无 Gazebo 时查看 URDF/TF/关节。 | **`sensor_tune.launch.py`**：`rviz2 -d` 固定指向本文件。 |

---

## 与 launch 的对应关系（简图）

```text
full_config.yaml (或 mmwave_sydney_minimal.yaml 等)
        │
        ▼
session_manager ──► 会话目录：URDF、bridge_config*.yaml、session.rviz、obstacle_layout.json、sensor_config 副本 …
        ▲
        │ config_path
main.launch.py ◄──── nav2_sim_full_bringup / sensor_tune（仅 session+URDF）/ test_hull（另逻辑）；毫米波最小场景为 main + `config_path:=.../mmwave_sydney_minimal.yaml`

infra_sim.launch.py ──► global_bridge.yaml（与 main 并行，不读 full_config）

robot_bringup.launch.py ◄── main 传入每船参数；可选加载 robot_localization_gps.yaml

nav2_sim_full_bringup ──► radar_nav2_param.yaml ──► nav2_thruster_bringup（改写后）──► navigation_launch.py

sensor_tune.launch.py ──► full_config.yaml + session_manager + tf_tune.rviz
```

---

## 会话产物（不在 `config/` 目录内）

以下由 **`session_manager`** 写入**临时/会话目录**，**不是**本文件夹下的静态文件，但都与 **`full_config`（或你传入的 `config_path`）** 一致解读：

| 产物 | 用途 |
|------|------|
| `source_config.yaml` | 本次会话选用的整机 YAML 副本 |
| `sensor_config.yaml` | 传感器内参副本（路径来自 `sensor_config_path`） |
| `bridge_config[_<船名>].yaml` | 每船 `ros_gz_bridge` 规则 |
| `session.rviz` | 本会话 RViz 配置（`main` 可能还会追加显示块） |
| `obstacle_layout.json` | 静态障碍布局，**`obstacle_spawner`** 使用 |
| 合并后的 **URDF** | `robot_bringup` spawn 使用 |

---

## 维护提示

- 改 **`full_config.yaml` 结构**（如新增顶层键）时，同步检查 **`session_manager.py`** 与 **`launch_config_helpers.py`**。
- 改 **Nav2 行为**优先改 **`radar_nav2_param.yaml`**，并确认 **`nav2_thruster_bringup`** 里对 `static_layer.map_topic` 的注入仍与海事雷达建图话题一致。
- **`sensor_config.yaml`** 与 **`full_config` 里 `sensor_config_path`** 不一致时，以 **full_config 指定文件** 为准进入会话；包内 `sensor_params.xacro` 直连路径仅作模板/默认开发用。
