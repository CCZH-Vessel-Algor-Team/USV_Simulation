# VRX 环境参数控制

`env_panel` 提供 RViz2 环境控制面板，用于设置仿真中的风、水流、波浪和太阳光亮度。

面板只负责发布环境参数，不直接向船体施加外力。ROS 话题经 `ros_gz_bridge` 转发到 Gazebo，最终由 Gazebo 插件计算船体受力。

## 实现结构

```text
VrxEnvPanel
  |
  +-- /vrx/wind/velocity_cmd
  |       -> ros_gz_bridge
  |       -> vrx::USVWind
  |       -> 相对风力和偏航力矩
  |
  +-- /ocean_current
  |       -> ros_gz_bridge
  |       -> gz::sim::systems::Hydrodynamics
  |       -> 相对水流、阻力、附加质量和科氏力
  |
  +-- /vrx/wavefield/parameters
  |       -> ros_gz_bridge
  |       -> VRX Wavefield
  |       -> vrx::Surface
  |       -> 船体浮力和波浪响应
  |
  +-- /vrx/environment/sun_light_cmd
          -> ros_gz_bridge
          -> /world/<world_name>/light_config
          -> Gazebo UserCommands
          -> sun 方向光
```

旧的 `usv_env_dynamics` 不再由 `main.launch.py` 启动，避免 ROS 简化外力和 Gazebo 环境力重复作用。

## RViz 面板

启动仿真并等待 RViz 完全打开，然后手动加载面板：

```text
Panels -> Add New Panel -> env_panel/VrxEnvPanel
```

当前不在默认 RViz 配置中自动加载该面板。自动恢复插件会在部分 RViz2 Humble 环境中引发段错误，表现为 `rviz2` 以 `exit code -11` 退出。

面板提供以下参数：

| 参数 | 单位 | 说明 |
| --- | --- | --- |
| Wind Speed | m/s | 世界坐标系中的风速大小 |
| Wind Direction | deg | 风吹向的方向，0 度为 `+X`，90 度为 `+Y` |
| Wave Gain | 无量纲 | PMS 强度控制，范围 `0–1`；Panel 同时换算并发布有效波高 Hs |
| Wave Period | s | 波浪周期，范围 `1–20 s` |
| Wave Direction | deg | 波浪传播方向；船首朝 `+X` 时，`90 deg` 为横浪 |
| Current X | m/s | 世界坐标系 `X` 方向水流速度，范围 `-2–2 m/s` |
| Current Y | m/s | 世界坐标系 `Y` 方向水流速度，范围 `-2–2 m/s` |
| Sun Brightness | % | `sun` 方向光亮度，范围 `0–100%`；不修改场景 Ambient |

面板以 10 Hz 更新风浪流参数，并对参数变化做速率限制。太阳光滑块停止变化约 `200 ms` 后发布最终配置，避免拖动时持续处理 Light 命令。

`Calm Water` 只将风、波浪和水流逐步降为零，不改变光照。`Reset Defaults` 会同时把太阳光亮度恢复到 `100%`。

### 风浪流联动

启用 `Couple Wind to Wave Gain and Current` 后，Wave Gain、波向和水流由风自动计算，Wave Period 仍可独立设置：

```text
current_speed = 0.03 * wind_speed
wave_gain = clamp((0.11 * wind_speed^2/g) / 3, 0, 1)
```

水流和波浪方向与风向一致。联动启用时，Wave Gain、Wave Direction 和水流滑块不可单独调整。关闭联动后，Wind Speed、Wave Gain、Wave Period 和 Wave Direction 相互独立，可以同时生效。

参数过渡速率经过限制：风速 `0.5 m/s²`、Wave Gain `0.03/s`、Wave Period `0.1 s/s`、水流分量 `0.1 m/s²`。参数变化时立即发布；稳定后每秒重发一次当前波场，保证后生成的动态船能够同步环境状态。

面板默认开启风浪流联动。需要独立调整波浪和水流时，先关闭 `Couple Wind to Wave Gain and Current`。横摇测试可在船首朝 `+X` 时设置 `Wave Direction=90 deg`、`Wave Period=3–4 s`。长周期顺浪对小型船体各浮力点的高度差较小，稳态主要表现为升沉，横摇不会持续明显。

## ROS 与 Gazebo 话题

### 左右舷吃水节点

`hull_draft_publisher` 随 `usv_sim_full` 的每艘船自动启动，订阅船体里程计并按
`Surface` 插件的四个浮力采样点计算静水吃水。波浪瞬时水面高度暂未纳入计算。

以 `usv_1` 为例：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/usv_1/hull_draft` | `usv_interfaces/msg/HullDraft` | 保留原始四点与两舷均值，并新增 `average_draft`、`trim`、`heel` |

默认浮力点为前后 `x=[0.6,-1.4]`、左右 `y=[1.03,-1.03]`，浮体半径为 `0.213 m`。
模型参数变化时，可通过 `front_x`、`aft_x`、`port_y`、`starboard_y`、`hull_radius`
和 `fluid_level` 覆盖。`trim` 正值表示艏部更深，`heel` 正值表示左舷更深。

桥接配置位于：

```text
usv_sim_full/config/global_bridge.yaml
```

### 控制话题

| ROS 话题 | ROS 类型 | Gazebo 类型 | 方向 |
| --- | --- | --- | --- |
| `/vrx/wind/velocity_cmd` | `geometry_msgs/msg/Vector3` | `gz.msgs.Vector3d` | ROS -> Gazebo |
| `/ocean_current` | `geometry_msgs/msg/Vector3` | `gz.msgs.Vector3d` | ROS -> Gazebo |
| `/vrx/wavefield/parameters` | `ros_gz_interfaces/msg/ParamVec` | `gz.msgs.Param` | ROS -> Gazebo |
| `/vrx/environment/sun_light_cmd` | `ros_gz_interfaces/msg/Light` | `gz.msgs.Light` | ROS -> Gazebo |

太阳光的 Gazebo 目标话题是 `/world/<world_name>/light_config`。`infra_sim.launch.py` 会读取所选 SDF 内部的 `<world name>`，启动专用单向 bridge，并将 ROS 侧话题重映射为固定的 `/vrx/environment/sun_light_cmd`。

面板发布名为 `sun` 的完整方向光配置，通过 `intensity` 设置亮度。Gazebo 的 Sensors 渲染路径可能打印 `Could not find visual for entity: 0`，基础设施 launch 只过滤这一条终端日志，其他 Gazebo 错误仍会显示。

风命令是世界坐标速度向量：

```text
x = speed * cos(direction)
y = speed * sin(direction)
z = 0
```

水流命令也是世界坐标速度向量。波浪消息包含：

```text
direction  # 弧度
gain       # PMS 波谱增益
period     # 秒
steepness  # 当前固定为 0
significant_height  # 有效波高 Hs，单位米
```

`/ocean_current` 是全局 Gazebo 环境输入。不要绕过面板向该话题发布 `NaN`、`Inf` 或超出仿真范围的数值；Gazebo 官方 Hydrodynamics 插件不会自行过滤非有限输入。


### 风真值话题

`USVWind` 计算完成后发布实际风状态：

| Gazebo / ROS 话题 | Gazebo 类型 | ROS 类型 | 方向 |
| --- | --- | --- | --- |
| `/vrx/debug/wind/speed` | `gz.msgs.Float` | `std_msgs/msg/Float32` | Gazebo -> ROS |
| `/vrx/debug/wind/direction` | `gz.msgs.Float` | `std_msgs/msg/Float32` | Gazebo -> ROS |

这两个话题是输出真值，不是控制输入。

## Gazebo 物理插件

相关插件配置位于：

```text
usv_sim_full/description/urdf/wamv_no_battery.urdf.xacro
```

### 风

每艘船加载一个 `vrx::USVWind`。插件订阅 `/vrx/wind/velocity_cmd`，根据世界风速和船体速度计算相对风：

```text
relative_wind = wind_velocity - ship_velocity
```

插件根据相对风计算纵向力、横向力和偏航力矩。动态风输入支持运行中修改，风速为零时保留最近一次有效风向。

### 水流

原来的 `vrx::SimpleHydrodynamics` 已替换为 Gazebo 官方插件：

```text
gz::sim::systems::Hydrodynamics
```

插件订阅 `/ocean_current`，使用船体速度与水流速度之差计算水动力：

```text
relative_water_velocity = ship_velocity - current_velocity
```

当前水动力使用线性阻尼和二次阻尼。现有正值水动力参数在 xacro 中转换为官方插件使用的负 SNAME 导数。配置中的附加质量参数目前均为零，因此关闭了插件附加质量和对应科氏项，避免 DART 在零时间步或仿真重置时产生非有限力。

PMS 的有效波高由 `significant_height` 控制。Period 只改变频率和波长，不再自动改变有效波高。Gain 仍保留作为 RViz 控制量，Panel 按当前 Period 将 Gain 映射为 Hs。运行时更新会在世界原点保持中心波分量的相位连续，物理 Surface 和 WaveVisual 使用同一参考相位。

`phase` 现在参与物理和视觉波高计算。运行中修改 Period 或方向时，Wavefield 会为每个波分量分别补偿相位，而不是使用新的频率直接乘绝对仿真时间。

同一船体只加载一套 `Hydrodynamics` 和一个包含四个浮力采样点的 `Surface`，避免左右插件在不同物理步切换波场导致横摇冲量。合并后使用两倍等效排水长度，保持与原左右双船体相同的总浮力。

### 波浪

左右船体的 `vrx::Surface` 订阅 Gazebo 话题 `/vrx/wavefield/parameters`。Wavefield 根据 `gain`、`period` 和 `direction` 更新波场，Surface 再根据船体采样点计算浮力。

world 中的固定波浪参数仅用于初始化。面板加载后，以面板持续发布的数据为运行时参数。

### 动态目标船

`dynamic_ship_manager_node` 生成的目标船也加载 `Surface`、`Hydrodynamics` 和 `USVWind`。原来的 Gazebo `VelocityControl` 会直接锁定六自由度速度，环境力无法改变船体状态；当前改为 `vrx::TargetShipController`，使用有限水平推力和偏航力矩跟踪 `/model/<name>/cmd_vel`。

该控制器不控制升沉、横摇和纵摇，因此波浪可以改变姿态；风和水流会与航迹控制器共同作用，造成可见漂移和航向误差。目标船采用约 `3500 kg` 的简化 10 米船模型，等效最大排水质量约 `4750 kg`。

掉头控制使用 Gazebo 实际 yaw 做航向闭环，航向误差超过约 `8.6 deg` 时禁止纵向推进。控制器采用航向 PD 和 `120 kN·m` 最大偏航力矩；即使管理器的等待时间结束，船首没有对准前也不会直接向前冲。控制命令超过 1 秒未更新时，推进自动归零。

每个 `Surface` 以模型进入物理循环的时刻作为波浪渐入起点，使用 Wavefield 的启动包络逐步施加波浪。静水浮力从第一步生效，动态插船不会因为绝对仿真时间较大而瞬间承受完整横浪。

`/dynamic_ship/tracked_ships` 目前仍是管理器按参考航迹计算的计划状态，不是 Gazebo 受扰后的实际位姿。需要环境真值时应读取 Gazebo 模型 pose；该话题暂时继续用于确定性的 COLREGS 规划输入。

## 编译

修改涉及 RViz 插件、VRX Gazebo 插件和船体描述，需要重新编译三个包：

```bash
rm -rf \
  build/vrx_gz install/vrx_gz \
  build/env_panel install/env_panel \
  build/usv_sim_full install/usv_sim_full

colcon build \
  --packages-select vrx_gz env_panel usv_sim_full \
  --symlink-install

source install/setup.bash
```

启动前确认当前工作空间排在其他 VRX 工作空间前面：

```bash
ros2 pkg prefix vrx_gz
printenv GZ_SIM_SYSTEM_PLUGIN_PATH
```

`ros2 pkg prefix vrx_gz` 应指向本工作空间的 `install/vrx_gz`。如果仍指向其他 VRX 工作空间，Gazebo 可能加载不支持动态风命令的旧版 `libUSVWind.so`。

## 运行检查

检查 ROS 控制话题：

```bash
ros2 topic echo /vrx/wind/velocity_cmd
ros2 topic echo /ocean_current
ros2 topic echo /vrx/wavefield/parameters
```

检查 Gazebo 是否收到参数：

```bash
gz topic -e -t /vrx/wind/velocity_cmd
gz topic -e -t /ocean_current
gz topic -e -t /vrx/wavefield/parameters
```

检查风真值：

```bash
gz topic -e -t /vrx/debug/wind/speed
gz topic -e -t /vrx/debug/wind/direction

ros2 topic echo /vrx/debug/wind/speed
ros2 topic echo /vrx/debug/wind/direction
```

检查旧外力节点没有运行：

```bash
ros2 node list | grep usv_env_dynamics
ros2 node list | grep environment_state_publisher
```

正常情况下，两条命令均无输出。

## 常见问题

### ROS 话题有数据，但 Gazebo 没有数据

检查全局 bridge 是否启动：

```bash
ros2 node list | grep bridge
gz topic -l | grep -E 'wind/velocity_cmd|ocean_current|wavefield/parameters'
```

### 风真值一直为零

确认 RViz 面板已经加载，并检查 `/vrx/wind/velocity_cmd` 是否为非零向量。`/vrx/debug/wind/speed` 是 `USVWind` 的输出，不接受外部设置。

### 水流话题变化，但船不受影响

检查生成的船体描述是否包含：

```text
gz::sim::systems::Hydrodynamics
```

同时确认没有继续启动 `usv_env_dynamics`，也没有同时加载旧的 `vrx::SimpleHydrodynamics`。

### RViz 启动时闪退

不要把 `env_panel/VrxEnvPanel` 写入默认 RViz 配置自动加载。先启动 RViz，再从 `Panels` 菜单手动添加。

### 水面消失或 Gazebo 报 DART NaN

旧实现可能在风速为零且启用联动时发布 `period=0`，导致 Wavefield 产生 NaN。当前版本将周期限制在 `1–20 s`，并在 Panel、Wavefield、Surface 三层检查非有限值。出现过 NaN 后必须完全重启 Gazebo，已经污染的 DART 刚体状态不能通过重新设置参数恢复。
