# `usv_sim_full/launch` 说明笔记

**用户向速览**（环境 → 怎么跑 → 怎么配 → 话题）：[`docs/docs_v3/QUICK_START.md`](../../docs/docs_v3/QUICK_START.md)。

本目录为 **`usv_sim_full` 包内仿真与导航相关的启动入口**；部分文件会 `Include` `components/` 下的组装件，或再 `Include` 其他包（如 `ros_gz_sim`、`gy_radar_driver`、`nav2_bringup`、`usv_mmwave_sim`）。

---

## 根目录：`*.launch.py`

| 文件 | 用途摘要 |
|------|-----------|
| **`main.launch.py`** | **唯一全量仿真主入口**：`session_manager` 解析多船 → **`infra_sim`** → 每船静态 **`map`→`{sanitized}/odom`**（可关）→ **逐船 `robot_bringup`**（海事雷达桥按传感器块开关）；若 **`scenario.ground_truth_sim.enabled`** 则启动 **`scenario_ground_truth_node`**（map 系 CTRV 真值）；若另设 **`gazebo_visual: true`** 再启动 **`scenario_ground_truth_gazebo_models`**（Gazebo 柱状实体跟随真值）；可选 **RViz**（`default.rviz` / 会话 RViz 含 `/sim/ground_truth_markers`）；毫米波、**`usv_sim_wrapper`**、**`scenario_manager_node`** 等见前文。Launch 参数：`config_path`、`enable_robot_localization`、`localization_params_file`、`localization_start_delay`、`use_static_map_odom_tf`。 |
| **`nav2_sim_full_bringup.launch.py`** | **仿真 + Nav2 一键 bringup**：可选启动前 `pkill` 清理残留进程；`Include` **`main.launch.py`** 拉起仿真；延时后再 `Include` **`nav2_thruster_bringup.launch.py`**。`namespace` 默认 `auto` 时从 `full_config` 解析主船名。 |
| **`nav2_thruster_bringup.launch.py`** | **仅导航与控制桥**（假定仿真已在跑）：`tf_namespace_relay`、动态改写 Nav2 代价地图 `map_topic` 指向 `/<ns>/map/navradar/occupancy_grid` 后 `Include` **`nav2_bringup/navigation_launch.py`**，并启动 **`cmd_vel_to_thruster`**。通常由 `nav2_sim_full_bringup` 调用，也可单独对接已运行的仿真。 |
| **`sensor_tune.launch.py`** | **无 Gazebo 的 TF/URDF 调参**：`session_manager` + 按 `robot_index` 选船 URDF，将 `model://` 转回 `package://` 后启动 `robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`（`tf_tune.rviz`）。用于关节/连杆可视化调试，**不含仿真与桥接**。 |

**毫米波相关（原独立 launch 已移除）**

- **整机仿真中启用毫米波**：在 `full_config.yaml`（或自定义 `config_path`）的 `robot_N.sensors` 中配置 `type: mmwave_radar`，由 **`main.launch.py`** 自动拉起毫米波后处理节点。
- **在已运行仿真中 spawn 毫米波验证体**：请直接调用 **`usv_mmwave_sim`** 包内 **`spawn_ego_mmwave_validation.launch.py`**（参数见该文件内 `DeclareLaunchArgument`），不再经本目录封装。

---

## 子目录：`components/*.launch.py`

| 文件 | 用途摘要 |
|------|-----------|
| **`infra_sim.launch.py`** | **基础设施层**：设置 `GZ_SIM_RESOURCE_PATH` / `GAZEBO_MODEL_PATH` / `GZ_SIM_SYSTEM_PLUGIN_PATH`，按 `world_name` 选择 `vrx_gz/worlds` 下 `.sdf`/`.world`，**`Include` → `ros_gz_sim/gz_sim.launch.py`**；并启动全局 **`ros_gz_bridge`**（`global_bridge.yaml`）。 |
| **`robot_bringup.launch.py`** | **单船容器**：命名空间下 spawn 实体、`ros_gz_bridge`、传感器桥、可选 maritime 雷达桥、`odom_tf_broadcaster`、定位相关节点等。由 **`main.launch.py`** 引用。 |
| **`visualization.launch.py`** | **仅 RViz2**：参数 `rviz_config_path` 传入配置文件路径。被 **`main.launch.py`** 等引用。 |

---

## `main.launch.py`：启动流程（mermaid）

以下为 **`OpaqueFunction(launch_setup)` 内实际执行顺序**（`launch_items` 的追加顺序）。ROS 2 launch 中同列表内多项会并行调度，图中自上而下表示 **代码组装顺序** 与 **数据依赖**；虚线表示 **Launch 参数** 或 **YAML 驱动**。

```mermaid
flowchart TB
  subgraph LARGS["① generate_launch_description：DeclareLaunchArgument"]
    P_cfg["config_path"]
    P_rl["enable_robot_localization"]
    P_loc["localization_params_file"]
    P_ldelay["localization_start_delay"]
    P_tf["use_static_map_odom_tf"]
  end

  subgraph SETUP["② launch_setup：读配置 + session_manager（阻塞子进程）"]
    RD["读取 config_path YAML → world_name / visualization.launch_rviz"]
    SCAN["iter_all_block_sensors → radar_processing_enabled / mmwave_enabled"]
    SM["ros2 run usv_sim_full session_manager --config-path"]
    PARSE["stdout JSON → rviz_config_path / obstacle_layout_path"]
    MERGE["resolve_session_robots + ship_config_blocks"]
    RVIZPATCH["向 rviz_config_path 追加 Map + Front Camera 显示块"]
    RD --> SCAN
    SCAN --> SM
    SM --> PARSE
    PARSE --> MERGE
    MERGE --> RVIZPATCH
  end

  subgraph ITEMS["③ 追加到 launch_items（组内顺序；组与组之间为代码追加顺序）"]
    INF["Include infra_sim.launch.py · world_name"]
    MAP["每船 static_transform_publisher<br/>map → sanitized/odom · 条件 use_static_map_odom_tf"]
    RB["每船 Include robot_bringup.launch.py"]
    MM["可选：mmwave_enabled 时<br/>每船每毫米波 mmwave_4d_cloud_node + mmwave_cluster_node"]
    VIZ["可选：launch_rviz 时<br/>Include visualization.launch.py"]
    SCN["Node scenario_manager_node"]
    WRAP["每船 usv_sim_wrapper"]
    ENV["可选：每船 enable_env_dynamics<br/>usv_env_dynamics"]
    GY["可选：海事雷达船<br/>Include radar_controller.launch.py"]
    INF --> MAP
    MAP --> RB
    RB --> MMQ{"mmwave_enabled ?"}
    MMQ -->|是| MM
    MMQ -->|否| VIZQ
    MM --> VIZQ{"launch_rviz ?"}
    VIZQ -->|是| VIZ
    VIZQ -->|否| SCN
    VIZ --> SCN
    SCN --> WRAP
    WRAP --> ENV
    ENV --> GY
  end

  LARGS --> SETUP
  SETUP --> ITEMS

  P_cfg -.-> RD
  P_cfg -.-> SM
  P_rl -.-> RB
  P_loc -.-> RB
  P_ldelay -.-> RB
  P_tf -.-> MAP
```

### Nav2 报 `map` 不存在 / costmap 等 TF 超时

仿真使用 **`/clock`（`use_sim_time`）** 时，`main.launch.py` 里 **`map`→`{robot}/odom` 的 `static_transform_publisher` 也必须 `use_sim_time: true`**。否则静态 TF 用墙钟打戳，Nav2 用仿真时刻查 TF，会出现 **`Invalid frame ID "map"`** 或 **`Timed out waiting for transform ... to map`**（`tf2_echo` 也可能先报错再变好）。**`nav2_sim_full_bringup`** 默认 **`nav2_start_delay:=15`**，在 spawn/桥接稍慢时可再加大。

### `robot_bringup` 每船传入要点（摘要）

| 参数来源 | 说明 |
|----------|------|
| `session_manager` / `resolve_session_robots` | `robot_name`、`urdf_path`、`bridge_yaml_path`、`spawn_pose` |
| YAML `ship_config_blocks` | `enable_maritime_radar_bridge`、`radar_sensor_name`、`radar_ros_topic` |
| 索引 `idx` | `enable_obstacle_spawner` 仅首船 `true`；`create_entity_delay` 错开 spawn |
| 固定/全局 | `obstacle_layout_path`、`gz_world_name`（= `world_name`）、`use_sim_time:=true` |
| Launch 参数 | `enable_robot_localization`、`localization_params_file`、`localization_start_delay` |

### 职责边界：`scenario_manager_node` 与 `robot_bringup` 是否「抢同一套 robot 参数」？

**并不矛盾，配置对象不同：**

| 组件 | 读的 YAML / 段落 | 实际做什么 |
|------|------------------|------------|
| **`robot_bringup.launch.py`** | 间接依赖 `session_manager` 已生成的 **URDF、桥接 YAML、障碍布局**；Launch 再传入每船的 `robot_name`、位姿、海事雷达桥开关等 | **本船（ego）**：在 Gz 里 spawn 模型、起 `parameter_bridge`（含 session 里写的各传感器话题）、可选 **`radar_gz_bridge`**（海事扫描 → ROS 扇区）、`odom`/TF/定位等 **仿真与桥接层** |
| **`scenario_manager_node`** | 仅 **`scenario.dynamic_obstacles`**（以及 `environment.world_name` 用于 spawn 服务名） | **场景里的动态障碍**（如巡逻艇）：生成简单 SDF、spawn、`cmd_vel` 环路、为障碍再起小型桥；**不读 `robot_1.sensors`，不改本船 URDF/海事或毫米波参数** |

因此流程图里二者相邻，只是 **launch 列表顺序** 如此，不是「两个节点都在配置同一艘船的 maritime/mmwave」。

### `NodeShared::RecvSrvRequest() error sending response: Host unreachable`

出自 **Gazebo Sim 的 gz-transport**（TCP 回包到服务调用方失败），常见于：

- **多机/容器与宿主机混跑**：`gz service` 子进程或另一终端里的客户端连到了**错误的 discovery 端点**（例如仍指向旧容器 IP）。同一台机器上跑仿真时，尽量 **统一终端**、同一 `ROS_DOMAIN_ID`，并避免残留 **`GZ_IP` / `IGN_IP`** 指向不可达地址。
- **防火墙或网络分区**：阻断 gz-transport 选用的回连端口。
- **多实例冲突**：两台 `gz sim` 或错误 `GZ_PARTITION` 导致服务注册混乱。

排查：只保留一个仿真实例；`unset GZ_IP IGN_IP` 后重开 shell；在**与 launch 同一环境**下执行 `gz service -l` 确认世界服务存在。

开启 **`gazebo_visual`** 时，若 spawn/remove 频繁失败，多为 **多个 `ros_gz_sim create` 与 `gz service remove` 同时打 gz-transport**。`ground_truth_gazebo_models_node` 已对 **create / remove / 可选 set_pose（drift 修正）** 子进程 **全局互斥**，并 **保留 create 用临时 SDF 至节点退出**（避免 Gazebo 异步读文件时路径已删）。周邻船运动由 **VelocityControl + cmd_vel** 驱动，不再每帧 `set_pose_vector`。

### 海事雷达 vs 毫米波：同级传感器，为何拆在 `robot_bringup` 与 `main`？

在 **`full_config` 里二者同级**（都在某船的 `sensors` 列表），但 **仿真管线长度不同**，代码按层拆开：

- **海事雷达**  
  - **Gz 插件 → spokes**；**`robot_bringup`** 里可选启动 **`radar_gz_bridge`**，把 Gz 侧数据变成 ROS 扇区话题（与 session 生成的桥接配置同一层级，属于「本船必连仿真」）。  
  - **扇区 → 点云 / OccupancyGrid** 在 **`gy_radar_driver`** 的 **`radar_controller`**，依赖重、参数多，放在 **`main.launch.py` 里按船 Include**，避免让通用 `robot_bringup` 强依赖该包。

- **毫米波**  
  - Gz 侧 **`gpu_ray` + 桥接** 同样由 session 的桥接 YAML 在 **`robot_bringup` 的 `parameter_bridge`** 里带上。  
  - **`points_gz` → 带 RCS 等字段的最终点云** 由 **`usv_mmwave_sim::mmwave_4d_cloud_node`** 完成；**`…/points` → `…/objects`（MmwaveTargetArray）** 由 **`mmwave_cluster_node`** 完成，属于 **独立包的后处理**；与海事侧「桥在 bringup、算法在 main」的拆分理由一致。

若未来希望 **结构上更对称**，可以把 `mmwave_4d_cloud_node` / `radar_controller` 的启动挪进 `robot_bringup`（用条件 Include），但会加深 `robot_bringup` 对 `usv_mmwave_sim`、`gy_radar_driver` 的耦合；当前做法是 **bringup = 实体 + 通用桥；main = 会话编排 + 可选重型后处理**。

### 与 Nav2 全量 bringup 图的关系

**`nav2_sim_full_bringup`** 仅向 `main.launch.py` 转发 `config_path`、`enable_robot_localization`、`localization_params_file`、`use_static_map_odom_tf`；**不转发** `localization_start_delay`（`main` 内用声明的默认值）。

---

## `nav2_sim_full_bringup.launch.py`：联动与参数（初版 mermaid）

以下为 **第一版** 示意图，便于后续按实际调参习惯再改。执行顺序自上而下；实线箭头表示 **Include / 启动动作**，虚线表示 **参数传递**（Launch 参数名与代码一致）。

```mermaid
flowchart TB
  subgraph ARGS["① 本文件 DeclareLaunchArgument（均可被命令行覆盖）"]
    A_cfg["config_path → 默认 share/config/full_config.yaml"]
    A_ns["namespace → 默认 auto"]
    A_pf["params_file → 默认 radar_nav2_param.yaml 解析链"]
    A_simt["use_sim_time → 默认 true"]
    A_delay["nav2_start_delay → 默认 10.0 s"]
    A_autoc["auto_cleanup → 默认 true"]
    A_shm["cleanup_fastdds_shm → 默认 false"]
    A_rl["enable_robot_localization → 默认 false"]
    A_loc["localization_params_file → 默认 robot_localization_gps.yaml"]
    A_tf["use_static_map_odom_tf → 默认 true"]
  end

  LOG["② LogInfo：Starting simulation bringup from config_path"]

  subgraph CLEAN["③ OpaqueFunction：prelaunch_cleanup"]
    C1{"auto_cleanup == true ?"}
    C1 -->|是| PK["pkill -9 -f 匹配 main/nav2/gz/桥接/EKF/…"]
    C1 -->|否| C2
    PK --> C2{"cleanup_fastdds_shm == true ?"}
    C2 -->|是| SHM["rm -rf /dev/shm/fastrtps*"]
    C2 -->|否| CEND[结束]
    SHM --> CEND
  end

  subgraph MAIN["④ Include → main.launch.py（与 Nav2 链并行启动、无 Timer）"]
    M["main.launch.py 全量仿真"]
  end

  subgraph DELAY["⑤ OpaqueFunction + TimerAction → thruster bringup"]
    RNS["解析 namespace：空或 auto → 读 config_path YAML 的 primary_robot_name；失败则 usv_1"]
    T["TimerAction period = nav2_start_delay"]
    NT["Include → nav2_thruster_bringup.launch.py"]
    RNS --> T
    T --> NT
  end

  subgraph NVT["⑥ nav2_thruster_bringup 内并行/顺序启动的节点"]
    REL["Node tf_namespace_relay"]
    OFN["OpaqueFunction：读 params_file"]
    PATCH["改 YAML：static_layer.map_topic → /{ns}/map/navradar/occupancy_grid"]
    TMP["临时 params yaml"]
    NAV2["PushRosNamespace + Include navigation_launch.py"]
    THR["cmd_vel_to_thruster"]
    OFN --> PATCH --> TMP --> NAV2
  end

  ARGS --> LOG
  LOG --> CLEAN
  CLEAN --> M
  CLEAN --> RNS
  A_cfg -.-> M
  A_rl -.-> M
  A_loc -.-> M
  A_tf -.-> M
  A_cfg -.-> RNS
  A_ns -.-> RNS
  A_pf -.-> NT
  A_simt -.-> NT
  A_delay -.-> T
  A_autoc -.-> C1
  A_shm -.-> C2
  NT --> REL
  NT --> OFN
  NT --> THR
```

### 参数传递对照（与上图呼应）

| 来源 | 目标 | 传递的 launch 参数 / 行为 |
|------|------|---------------------------|
| `nav2_sim_full_bringup` | `main.launch.py` | `config_path`、`enable_robot_localization`、`localization_params_file`、`use_static_map_odom_tf` |
| `nav2_sim_full_bringup` | `nav2_thruster_bringup.launch.py`（经 Timer） | `namespace`（解析后）、`params_file`、`use_sim_time` |
| `nav2_sim_full_bringup` | （不传给 main） | **`localization_start_delay`** 未转发；`main` 侧使用该参数默认值。 |
| `nav2_thruster_bringup` | `navigation_launch.py` | `namespace`、`params_file`（实为**改写 map_topic 后的临时文件**）、`use_sim_time` |

> **初版说明**：`params_file` 在 `nav2_sim_full_bringup` 与 `nav2_thruster_bringup` 中各自有一套「源码树 → usv_sim_full 安装 → gy_radar_driver」的默认路径解析，二者默认应对齐；若只改其一的命令行 `params_file:=`，以 bringup 传给 thruster 的为准。

---

## `ground_truth_sim` 与 Nav2 / P3D / Gazebo 实体（备忘）

- **`overrides.ground_truth_enabled`**（xacro）：URDF 内 **Gazebo P3D 真值里程计**，与 ROS 包 **`ground_truth_sim`** 无关。
- **`scenario.ground_truth_sim.enabled`**：启动 **`scenario_ground_truth_node`**，在 **map** 系发布 **`/sim/ground_truth`** 与 **`/sim/ground_truth_markers`**；环带圆心取 **`reference_robot`**。与 **Nav2** 通常 **可并存**（本船仍由 Nav2/推进驱动时）。RViz 中 Marker 命名空间为 **`target_pose` / `target_path` / `target_history`**；插件面板里那句 *“Displays visualization_msgs::MarkerArray…”* 为 **插件说明**，非报错。
- **`scenario.ground_truth_sim.gazebo_visual`**：启动 **`ground_truth_gazebo_models_node`**，订阅同一真值话题，在 **`environment.world_name`** 对应世界中 **`ros_gz_sim create`** 生成实体（box/cylinder/mesh_profile），SDF 内嵌 **VelocityControl**，周期向 **`/model/gt_ctrv_*/cmd_vel`** 发布 Twist（经 **ros_gz_bridge**）。可选 **`position_resync_enabled`** 在 drift 超阈值时偶发 **`set_pose`**。旧 teleport 实现见 **`ground_truth_gazebo_models_node_teleport.py`**。
- **Nav2 代价地图**：当前**未**将 CTRV 真值接入局部/全局 costmap；若需规划避障「看见」虚拟目标，后续再定（如 `PointCloud2` + `voxel_layer` 等）。
- 若将来某船改为 **本船位姿仅由仿真绝对真值写出**（如 `SetEntityPose` 驱动本船），则应 **关闭** 该船的 Nav2/推进闭环，避免双重控制。

---

## 典型启动命令（备忘）

```bash
source install/setup.bash
ros2 launch usv_sim_full main.launch.py
ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py
# 自定义配置示例：
# ros2 launch usv_sim_full main.launch.py config_path:=/path/to/my.yaml
```

参数以各文件内 `DeclareLaunchArgument` 为准；`config_path` 多为核心开关。
