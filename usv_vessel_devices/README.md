# 船灯、可放倒桅杆与推进器起翘

> 推进器浸没及起翘力矩模型仍在验证中。4 ms 固定船体联调已通过，但波浪、整船导航及命令超时保护尚未完成验收。

ROS 2 Humble / Gazebo Harmonic 可选附件。控制代码、Gazebo 插件、模型、消息和服务均在本包；不改变导航或现有传感器挂载。推进器起翘单独启用，旧配置保持原推进结构。

## 功能

- 默认 16 盏灯独立开关：桅杆上的 12 盏灯随桅杆放倒，船顶四角另有左红右绿四灯固定在船体上。
- 桅杆整体默认放大至原 CAD 的两倍：总高约 1.196 m，底座、杆体、横杆、灯具及相对安装坐标均等比例缩放；船上安装基准不变。
- 桅杆底部铰接，`0 rad` 竖起，默认 `pi/2 rad` 向船尾放倒，支持中间角度。
- 灯采用自发光材质，不照亮周围环境；不包含闪烁或航行灯法规认证。
- 初始灯全部关闭、桅杆竖起。安装位置为 M5 船模的近似值，可配置。
- 可选 CUSTOM 双舷外机起翘：左右独立或同时控制，默认 0～75°，真实关节、限速和反馈；推力按螺旋桨浸水面积衰减，角度本身不阻断推进命令；相机、雷达、GPS 等不随桅杆运动。
- 默认使用用户提供 CAD 转换后的桅杆和闭盖航行灯；移除被替换的全部 11 套旧灯几何，保留托盘和支架。固定底座不随桅杆放倒，灯光只改变灯罩，不改变外壳。

## 构建与启动

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_vessel_devices --symlink-install
source install/setup.bash
# 独立测试台，不需要导航和感知模块
ros2 launch usv_vessel_devices demo.launch.py
# 无 GUI，仍渲染观察相机（需要可用的 EGL/OpenGL）
ros2 launch usv_vessel_devices demo.launch.py headless:=true
```

完整仿真通过 `robot_N.vessel_devices` 接入：

```yaml
robot_1:
  name: usv_1
  xacro_template: wamv_no_battery.urdf.xacro
  # 保留原有其他配置
  vessel_devices:
    enabled: true
    mast_xyz: [-0.35, 0.0, 1.55]
    mast_use_cad: true
    mast_length: 1.1957382058263922
    lamp_use_cad: true
    lamp_scale: 1.0
    lights_on_mast: true
    mast_lamp_scale: 0.15
    extra_lights_enabled: true
    roof_lights_enabled: true
    # 可只覆盖个别新增灯，其余保留默认红/绿。
    extra_light_colors:
      aft_lower: red
      aft_middle: green
      aft_upper: red
    mast_max_angle: 1.5707963267948966
    mast_speed: 0.3
```

构建 `colcon build --packages-up-to usv_sim_full --symlink-install` 并重新 source 后，使用原入口 `ros2 launch usv_sim_full main.launch.py`。主配置 `full_config.yaml` 已为 `usv_1` 启用。旧配置缺少此块或 `enabled: false` 时，不生成附件或设备控制接口。目前完整仿真仅接入本地 `wamv_no_battery.urdf.xacro` 模板。

### 从整机入口启动

`ros2 launch usv_bringup system.launch.py` 默认经过 `system_sim.yaml` 读取的是
`usv_bringup/config/simulation/ccs_config.yaml`，**不会合并** `usv_sim_full/config/full_config.yaml`。
因此必须在实际场景的 `robot_1` 中启用 `vessel_devices`；整机默认 CCS 场景已补上该开关。
使用其他 `system_config_file` 或 `config_path` 时，同样检查它最终指向的场景配置。

```bash
colcon build --packages-select usv_vessel_devices usv_sim_full usv_bringup --symlink-install
source install/setup.bash
# 先结束旧仿真，再重新启动；已生成的旧船体不会热加载新附件。
ros2 launch usv_bringup system.launch.py
```

在调用开关服务前，可用 `ros2 service list | grep /usv_1/devices/` 检查接口是否已创建。
若没有任何设备服务，先核对实际场景配置、`ros2 pkg prefix usv_sim_full`、
`ros2 pkg prefix usv_vessel_devices`，以及 Gazebo 的 `libVesselDevices.so` 加载日志；
仅重复调用服务不会使未启用附件的旧模型自动加载插件。

完整默认值见 [config/devices.yaml](config/devices.yaml)。坐标相对 `base_link`，单位米；角度、速度为 rad、rad/s。每船可独立设置，不修改会话目录中的生成文件。

### 推进器起翘

整机入口实际读取的 `usv_bringup/config/simulation/ccs_config.yaml` 已在
`robot_1.vessel_devices` 启用 `thruster_tilt_enabled: true`，其余配置默认关闭。
最大角度、速度、容差和力矩分别由 `thruster_tilt_max_angle`、`thruster_tilt_speed`、
`thruster_tilt_tolerance`、`thruster_tilt_effort` 设置，要求 `thruster_config: CUSTOM`。
起翘配置和 ROS topic/service 的硬上限均为 75°，物理关节求解间隙不会扩展到该上限之外。修改后须重新生成船模并重启仿真，不能只 Reset 世界。
`0` 为放下，正角向船尾起翘；铰链采用原转向安装基准，位置零点不变。
运动使用有限力矩闭环控制，并补偿重力及实际水推力对起翘关节的力矩；不重置模型位置。
原 `demo.launch.py` 仍是灯/桅杆测试台，推进器起翘随整机 CCS 船模生成。

```bash
ros2 service call /usv_1/devices/thrusters/both/tilt/set_position \
  usv_vessel_devices/srv/SetThrusterTilt '{position: 1.3089969389957472}'
ros2 topic echo /usv_1/devices/thrusters/state
```

放下改为 `position: 0.0`，单侧将 `both` 改为 `left` 或 `right`。
同路径的 `/cmd` topic 使用 `std_msgs/msg/Float64`，独立状态消息为 `ThrusterTiltState`，
不改变原 `DeviceState`。推力/转向 ROS 话题和桥接保留；起翘过程中允许带推力调角。
`thrust_enabled` 表示推进物理实体可用，与起翘角度无关；`applied_thrust` 是限幅后的命令，
不代表实际水推力。可订阅 `/usv_1/devices/thrusters/physics`，每侧依次为命令、
目标桨转速、浸没率、有效水推力、起翘力矩和力矩是否饱和，先左后右。
桨完全出水时水推力为零但桨仍转动；部分浸水按圆盘浸没面积近似，忽略空化、通气和空气推力。
实际输出受原生插件兼容的 -1000～8529.75 N 限幅。此模型不替代硬件急停。
完整操作、限制与主入口配置链见
[`usv_bringup/docs/THRUSTER_TILT.md`](../../usv_bringup/docs/THRUSTER_TILT.md)。

## CAD 模型、位置与尺寸

模型文件及来源记录在 [description/meshes](description/meshes/README.md)。网格保存原始米制尺寸，模型默认按 XYZ 两倍加载，不依赖 Downloads 中的原文件，也不需要在运行机器安装 CAD 软件。

- **四盏红绿灯**：采用第二个 CAD 中的闭盖灯具，不加载旁边的开盖展示模型。默认安装在活动桅杆的两层横杆，位置来自 CAD 托盘，自动随 `mast_length` 缩放。原尺寸灯具约宽 0.152 m、高 0.200 m；为适配小托盘，默认安装缩放为 `lamp_scale × mast_lamp_scale × mast_length / 0.5978691`，其中 `mast_lamp_scale` 默认 0.15。原高桅杆上的实际灯高约 0.030 m，桅杆放大到 1.2 m 后灯高约 0.060 m。外壳为深色，灯罩受独立红/绿开关控制。
- **桅杆**：原高约 0.598 m，默认总高约 1.196 m。`mast_xyz` 是 CAD 底板的安装基准，`mast_length` 为包含固定底座的总高，改变它会对 CAD 模型做等比例缩放，不是只拉伸 Z 轴。虚拟放倒轴位于固定支架上沿，默认距安装基准约 0.242 m。顶部白灯使用 CAD 本身的灯罩，随动且不再叠加球形灯。
- **物理近似**：底座采用简化盒碰撞，活动杆体采用简化圆柱碰撞；名义质量保持不变，惯量随几何更新，并非按密度将质量放大八倍。该 CAD 未定义活动铰链，当前虚拟关节不代表工程机构验证。不移动现有传感器。
- **材质差异**：STEP 未保留 JPG 预览中的碳纤维/玻璃渲染贴图，仿真使用深色灯座、金属桅杆和自发光灯罩，不模拟玻璃折射或光学认证效果。

可将 `lamp_use_cad: false` 切回球形灯；`mast_use_cad: false` 切回圆柱桅杆。原先的 1 m 简化桅杆可用：

```yaml
vessel_devices:
  enabled: true
  mast_use_cad: false
  mast_length: 1.0
  mast_radius: 0.025
  lamp_use_cad: false
  lamp_radius: 0.065
```

简化模式下 `mast_xyz` 仍为底部铰链坐标，`roof_z` 为球形灯中心，`lamp_radius` 控制球形灯半径。

### 两层横杆的灯名与位置

为兼容既有 ROS 调用，仍保留原来的 `front/rear` 名称，它们现在表示上/下层：

| 灯名 | 默认安装位置 | 颜色 |
| --- | --- | --- |
| `left_front` | 左上层横杆 | 红 |
| `right_front` | 右上层横杆 | 绿 |
| `left_rear` | 左下层横杆 | 红 |
| `right_rear` | 右下层横杆 | 绿 |
| `mast` | 桅杆顶部，保留原白灯 | 白 |
| `right_lower` | 右侧最下层 | 绿 |
| `aft_lower` / `aft_middle` / `aft_upper` | 后侧中线，由低到高 | 红 / 绿 / 红 |
| `forward_lower` / `forward_middle` / `forward_upper` | 前侧中线，由低到高 | 红 / 绿 / 红 |

原尺寸时两层托盘相对活动铰链的坐标约为 `(0, ±0.100196, 0.240871)` 和
`(0, ±0.100196, 0.150695)` 米。改变 `mast_length` 会等比例缩放托盘位置及四盏灯，
无需重新修改四个 XYZ；`lamp_scale` 或 `mast_lamp_scale` 可单独微调红绿灯尺寸，不改变顶部白灯。
过大的灯具可能跨层重叠，需要用户自行保持合理比例。

设置 `lights_on_mast: false` 可将原四灯切回船顶安装；使用
`front_x/rear_x/left_y/right_y/roof_z`，灯具缩放仅为 `lamp_scale`。
`mast_use_cad: false` 时也自动回退船顶布局，因为简化圆柱没有 CAD 横杆。

### 新增七盏灯

新增灯始终挂在 CAD 活动桅杆上，不受四灯的 `lights_on_mast` 船顶布局开关影响。
`extra_light_colors` 按上述新增灯名配置 `red` 或 `green`，可只覆盖部分；颜色在生成模型时确定，运行时独立控制亮灭。
中线灯额外使用 0.8 的灯具适配比例，避免外壳与杆体相交，右下灯使用 1.0。
设置 `extra_lights_enabled: false` 时不生成这七灯，船顶四灯不受影响；关闭 CAD 桅杆时退回原五灯布局。旧装饰灯不会恢复。

### 船顶四角灯

保留全部桅杆灯，另在船顶四角添加四个独立灯具，命名为：

| 灯名 | 位置（相对 `base_link`，米） | 颜色 |
| --- | --- | --- |
| `roof_left_front` | `(0.20, 0.75, 1.55)` | 红 |
| `roof_left_rear` | `(-1.15, 0.75, 1.55)` | 红 |
| `roof_right_front` | `(0.20, -0.80, 1.55)` | 绿 |
| `roof_right_rear` | `(-1.15, -0.80, 1.55)` | 绿 |

坐标分别由 `front_x/rear_x/left_y/right_y/roof_z` 控制。CAD 灯具底部安装于该坐标，
使用原闭盖灯模型和 `lamp_scale`，默认宽约 0.152 m、高约 0.200 m；球形回退时该坐标为球心。
四灯固定在 `base_link`，不随桅杆放倒，也不随 `mast_length` 或 `mast_lamp_scale` 缩放。
`roof_lights_enabled: false` 可仅关闭这组新增灯，恢复此前的十二灯模型。
若 `lights_on_mast: false` 或 `mast_use_cad: false`，原四灯已占用船顶四角，自动省略新增 `roof_*`，避免重叠。

## ROS 接口

`<灯名>` 见上表，默认共 16 个。原有 topic/service 路径不变，新增灯使用相同命名规则。

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/usv_1/devices/lights/<灯名>/cmd` | `std_msgs/msg/Bool` | 开关 topic |
| `/usv_1/devices/lights/<灯名>/set` | `std_srvs/srv/SetBool` | 开关服务 |
| `/usv_1/devices/mast/cmd` | `std_msgs/msg/Float64` | 目标角度 topic |
| `/usv_1/devices/mast/set_position` | `usv_vessel_devices/srv/SetMastPosition` | 目标角度服务 |
| `/usv_1/devices/state` | `usv_vessel_devices/msg/DeviceState` | 10 Hz 墙钟，可靠、transient-local |

```bash
# 开左前红灯
ros2 service call /usv_1/devices/lights/left_front/set std_srvs/srv/SetBool '{data: true}'
# 开白色桅灯
ros2 topic pub --once /usv_1/devices/lights/mast/cmd std_msgs/msg/Bool '{data: true}'
# 开新增后侧下层红灯；关闭改为 data: false
ros2 service call /usv_1/devices/lights/aft_lower/set std_srvs/srv/SetBool '{data: true}'
# 开船顶左前红灯；关闭改为 data: false
ros2 service call /usv_1/devices/lights/roof_left_front/set std_srvs/srv/SetBool '{data: true}'
# 放倒；竖起用 position: 0.0
ros2 service call /usv_1/devices/mast/set_position usv_vessel_devices/srv/SetMastPosition '{position: 1.5707963267948966}'
ros2 topic echo /usv_1/devices/state
```

服务成功表示命令接受，不表示完成；等待 `mast_at_target: true` 并检查 `mast_position`。超限、NaN、Inf 或尚未就绪时拒绝命令；topic 拒绝通过日志提示。多个入口采用最后接受的目标。

`ready` 表示所有已配置灯的可视实体和桅杆物理角度反馈可用；`lights_on` 是仿真端已应用材质开关，不是光照测量。消息字段不变，前五项顺序仍为 `left_front, left_rear, right_front, right_rear, mast`，之后为新增七个桅杆灯，最后追加上述四个 `roof_*` 灯。不能用数组最后一项定位白灯，应通过 `light_names` 查找。时间戳为仿真时间，暂停时不增长。暂停时服务仍接受命令，运动须恢复仿真；世界重置后全部关灯、目标归零。

## 验证

```bash
colcon test --packages-select usv_vessel_devices
colcon test-result --verbose
```

测试覆盖十六灯颜色/顺序、船顶四灯固定不动与独立亮灭、配置覆盖与校验、旧几何移除、整体两倍缩放、固定关节保留、桅杆轴、URDF 转 SDF、目标校验和限速。完整仿真回归测试位于 `usv_sim_full/test/test_vessel_devices.py`。独立测试世界的 Gazebo `/vessel_devices/observer/image` 用于核对相机中的亮灭。

可选真实联调（需要 Gazebo Python transport/msgs 绑定、NumPy 和可用的 EGL 渲染）：

```bash
USV_VESSEL_DEVICES_INTEGRATION=1 colcon test --packages-select usv_vessel_devices
```

联调自行启动隔离的 Gazebo partition 与 ROS domain，验证相机像素、GUI ECM 状态、双船隔离、实际放倒/回升、角度与速度限制、重力保持、暂停、重置和模型移除，不控制已运行的仿真。普通测试默认跳过此项。

完整船体动态 TF 使用原有关节桥接；外部定位模式由已有模型 TF 发布方消费相同关节状态，本包不发布第二套 TF。

架构见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)，数据流见 [docs/DATA_FLOW.md](docs/DATA_FLOW.md)。
