# usv_sim_full

读取 YAML 配置，统一启动 Gazebo 世界、船舶、桥接、传感器、场景和可选后处理组件。

## 构建

```bash
colcon build --packages-up-to usv_sim_full --symlink-install
source install/setup.bash
```

构建前应确保工作区已包含 `usv_interfaces`、VRX、`usv_mmwave_sim`、`radar_gz_bridge` 及需要使用的导航、感知包。

## 启动

```bash
# 完整仿真
ros2 launch usv_sim_full main.launch.py

# 完整仿真并启动 Nav2
ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py

# CCS 认证仿真
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py

# 指定配置
ros2 launch usv_sim_full main.launch.py config_path:=/path/to/full_config.yaml
```

主配置文件为 `config/full_config.yaml`。

### CCS 高公岛码头

`usv_bringup` 主入口使用的 `ccs_open_water_crop` 已接入高公岛港口、实体接岸堤路和局部坡道，
统一港口与近岸山体材质，保留绿色岛屿主体、水面及其他场景设置。新增堤路改变局部岸线，
仿真及 PX4 SITL profile 使用对应的派生导航图；原 ENC 海图和实船配置保留。
港口外观、碰撞和接岸参数以当前世界与模型配置为准。
护坡石、12 个浮桥构件和 14 艘装饰船已加入简化静态碰撞；外观与导航地图保持不变。

### CCS 海上光伏场与陆地风机

CCS 世界东南海域的 12 台风机已替换为固定桩式光伏场；4 台陆地风机保留原位置和转动。
光伏场以 119.49527°E、34.67997°N 为中心，占地范围 1 km × 1 km。Gazebo 保留桩柱碰撞；
默认全局地图不标光伏，另保留带桩柱标记的备用地图。
光伏场参数、桩柱碰撞和备用地图以当前场景与地图配置为准。

### CCS 田湾跨海大桥

按 `CN441122.000` 海图位置布置田湾跨海大桥，建至 5 km 地图南边缘。模型保留
125 m 主跨、70 m 辅通航跨、引桥、栏杆、6 车道标线和 4 个航标；路面和墩形参照照片拟合。
桥在 Gazebo 中有碰撞，但**默认全局地图不加入大桥**，用于测试局部感知/避障。
大桥的海图定位、跨径和碰撞以当前模型配置为准。

### 船灯与可放倒桅杆

主配置已为 `usv_1` 设置 `vessel_devices.enabled: true`，默认桅杆整体两倍尺寸（高约 1.196 m），十二个桅杆灯随其竖起/放倒，另增船顶四角左红右绿 `roof_*` 四灯固定在船体，十六灯独立控制。中线等新增桅杆灯颜色可用 `extra_light_colors` 逐灯覆盖；`roof_lights_enabled: false` 仅关闭新增四个角灯。现有传感器和推进器不改。

接口、位置参数、操作示例和独立测试入口见 [usv_vessel_devices](../usv_vessel_devices/README.md)。旧配置缺少 `vessel_devices` 块或设置 `enabled: false` 时保持旧船模；目前支持本地 `wamv_no_battery.urdf.xacro` 模板。

### RViz导航目标

CCS使用 **Nav2 Goal** 工具（快捷键 `g`）向唯一的 **Navigation 2** 面板输入本船导航目标。
固定坐标系使用 `map`，等待面板显示Navigation active后操作：

- 单点导航：在普通模式下选择Nav2 Goal，按住左键拖拽位置和艏向，释放后发送NavigateToPose。
- 多点导航：先点击 **Waypoint / Nav Through Poses Mode**，再用Nav2 Goal依次拖拽各目标；
  累积过程中不发送导航任务，`Nav2 Waypoints`显示所选航点。点击 **Start Nav Through Poses**
  发送NavigateThroughPoses；**Start Waypoint Following**对应逐点到达的FollowWaypoints。
- 导航中点击 **Cancel** 取消当前任务；**Cancel Accumulation**退出选点模式。

目标通过面板的Nav2 Action客户端发送，使用RViz所在的 `usv_1` 命名空间。
**添加动态船**用于下述目标船生成，不用于设置本船导航目标。

### RViz生成动态目标船

在 **DynamicShipConfig** 面板“船类型”点击 **仿真船** 或 **虚拟船** 按钮，
然后使用唯一的工具栏 **添加动态船**（快捷键 `d`）。切换仅影响后续新建目标，不修改已有目标：

- **单击**：按下位置作为生成点，使用面板“默认航向”（北向 0°，顺时针增加）。
- **拖拽**：按下位置作为生成点，箭头指定初始船艏方向，松开后添加；拖拽长度不影响速度或航行距离。
- **取消**：右键取消本次放置；`Esc` 退出工具。只在有效松开时发送一次创建请求。
- 默认连续添加；小于 6 像素的移动视为单击，避免鼠标抖动误设航向。

固定坐标系必须使用 `map`，沿用 Gazebo world XY 与 map 对齐的前提。单击向
`/dynamic_ship/spawn_point` 发布 PointStamped，拖拽向 `/dynamic_ship/spawn_pose` 发布
PoseWithCovarianceStamped；两条路径均拒绝非法坐标系与非有限值，位姿路径还拒绝零四元数。
速度、船型和半程距离都取自 DynamicShipConfig，拖拽航向不受默认航向覆盖。
管理器不再订阅通用 `/clicked_point`，CCS 工具栏不再使用 Publish Point / 2D Pose Estimate 添加动态船。
`spawn_pose_topic` 是启动只读参数，覆盖时须同时调整工具的 `Topic`。
请求协方差、Z 和时间戳不作为目标观测。

程序化添加通过 `/dynamic_ship/spawn`（`usv_interfaces/srv/SpawnDynamicShip`），设置后续默认值通过
`/dynamic_ship/set_config`（`usv_interfaces/srv/SetDynamicShipConfig`）。服务名和类型名不变，
但字段以 `model_id` 代替 `shape`，旧客户端须更新并重新构建。`/dynamic_ship/models` 列出船型；
`legacy_10m`、`debug_box`、`debug_cylinder` 对应原有的默认船和调试形状。生成请求的
`virtual_ship: true` 只创建航迹；`mesh_profile` 可选，用来覆盖采用网格配置的仿真船。
`ros2 launch usv_sim_full dynamic_targets.launch.py` 可单独运行虚拟后端，不启动 Gazebo。

**添加风暴**（`s`）与面板选中的虚拟动态船也采用同样的单击／拖拽交互，操作备注位于各自面板最下方：

- 风暴单击使用面板“默认漂移方向”，拖拽箭头指定实际漂移方向；漂移速度为 0 时仍静止。
  半径、漂移速度、有效时长和网格间距继续取自风暴面板。
- 面板选择“虚拟船”时，同一个“添加动态船”按钮创建无 Gazebo 的目标；单击使用默认航向，拖拽指定船艏方向。
- 拖拽方向只用于本次新建目标，不修改面板默认值或已存在目标。

风暴单击话题为 `/storm_field/clicked_point`，拖拽为 `/storm_field/spawn_pose`；
虚拟船单击为 `/dynamic_ship/virtual_clicked_point`，拖拽为 `/dynamic_ship/virtual_spawn_pose`。
两种拖拽话题均使用 PoseWithCovarianceStamped；所有工具共用现有交互逻辑。

`/storm_field/set_config` 更新后续新建storm的默认配置；已经生成的storm保留各自的半径、漂移和有效期。

`use_sim_time` 在合并参数后显式应用到各Nav2节点和costmap，并传给 `cmd_vel_to_thruster`。
仿真默认使用 `/clock`；需要系统时间时可传 `use_sim_time:=false`，不改变世界的实时倍率。

Gazebo 服务器与GUI分别启动，避免合并启动时等待GUI握手；世界文件、物理步长和实时倍率保持配置原值。
`gz_headless:=true` 只启动服务器，传感器仍按配置运行。关闭GUI不会停止仿真服务器，应结束整个launch会话。

动态目标船直接启动实际 `parameter_bridge` 可执行文件并持有其子进程句柄。
删除确认后同步退休控制发布器，以 SIGINT / SIGTERM / SIGKILL 逐级停止 bridge，每级最多等待 2 秒回收；
失败的发布器或进程句柄保留重试，单船失败不影响其他船清理，零活动船时仍定时重试，退出时再清理。
创建前必须由场景清单确认名称不存在，名称已存在或清单不可用时拒绝发送创建请求；
发送后必须在成功读取的清单中找到精确名称才确认创建，`create` 退出码为 0 本身不代表成功。
创建失败返回失败，已发送但尚未观察到实体的请求保留为不控制、不发布跟踪的隔离注册，
即使多次清单均无此名称也继续保留，防止迟到创建失去管理；随后观察到实体时重试回收。
Gazebo 删除的 Boolean 成功回复仅表示排队，须由场景清单确认实体不存在才释放注册；
创建和删除命令返回后，另用单调墙钟最多等待约 2 秒确认精确名称出现或消失，每次查询之间
休眠约 50 毫秒，查询超时受剩余期限约束；这些等待均在锁外执行，超期继续保留待确认注册。
已观察到的实体若被外部或延迟删除，经两次成功的缺失清单读取后回收注册。
清单查询使用有界 `gz service /world/<world>/scene/info` 子进程，将同步请求与高频位姿回调隔离；
使用 protobuf 文本解析仅读取根模型名称，进程启动、等待及解析超过剩余期限或回复无效均视为查询失败。
位姿订阅仍使用原 transport 节点；绑定不可用时拒绝创建并保留未确认注册。
控制定时器独立于阻塞的创建、删除和资源清理回调；保留
运动控制、实测艏向及低通速度滤波时间常数 0.3 秒。跟踪时间戳采用下述实测快照契约。
spawn/delete/clear/config 服务、点击订阅和场景查询定时器共用独立的互斥 `lifecycle_group`，
保留操作串行性。默认组留给 TimeSource 的 `/clock` 等节点回调，控制仍使用 `control_group`；
两个 executor 线程使生命周期等待期间的时钟更新与控制/跟踪发布能够继续执行。

## CCS standalone COLREGS

此接线要求 Nav2 提供下述独立 TS 快照、避让点和 barrier 服务及参数，运行时须使用接口一致的
消息、服务和二进制 overlay。当前接口验证基线为 Nav2
`e58faf3012cbbecb7f0e23252de92daf6899f1d8`；后续包含相同接口和参数能力的分支可沿用此入口与配置。
复用现有 `ts_subsystem.launch.py` 启动独立的 TS manager、avoidance point 和 barrier 节点，
经 `/processed_ts_list`、`/get_avoidance_point`、`/get_barrier_lines` 接入 VO-RRT*。
Barrier 使用 avoidance 返回的同一 snapshot UUID 和 header；这是 standalone 接口，不是 Server 接口，
也不包含归档 P 实验的 prediction 字段。

CCS/三视觉毫米波导航入口及独立 `ts_subsystem.launch.py` 默认加载
`config/ts_subsystem_asymmetric.yaml`，无需额外指定 `ts_params_file`；该参数仍可显式覆盖配置。

可选的对称配置 `config/ts_subsystem.yaml` 是 **CCS 参数与几何迁移，不保证旧版轨迹等价**。
保持配置 OS 半径 15m、TS 半径 5m、威胁 TCPA horizon 40s；威胁判定倍率为2，避让倍率为1.5。
按当前半径和20m，判定区间为40m、避让计算半径为30m，使避让域小于判定域，降低刚避让就离开判定域的情况。
AP 扩展距离为 40m，即原 `(15+5)*2`；第一段 barrier 为 `5+15=20m`，第三段为 999m。
新扩展距离是独立参数，仅对当前 5m TS 重现旧公式；40s horizon 是威胁筛选窗口。

使用默认非对称航向平滑（D）profile：

```bash
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py \
  auto_cleanup:=false cleanup_fastdds_shm:=false
```

`ts_params_file` 独立于 Nav2 的 `params_file`。非对称 profile 同样保持 OS 15m、判定/避让倍率2/1.5、horizon 40s、
AP 扩展 40m；启用不对称增益 1.0/0.15、速度容差 ±0.3m/s、5 个区间样本（另检查不在网格上的实测速度），
barrier 第三段 8m、lateral margin 0.3m（第一段 5.3m）。`heading_smoothing_alpha=0.5` 是对称模式
备用增益，不与 D 增益串联。可选对称 profile 为 alpha 1、速度容差 0、不启用 D。
两份配置的输入/OS/processed 超时均为 1s，请求位置容差为 3m。

参数迁移：manager 的 `frequency/ts_timeout/tcpa_horizon/safety_factor` 分别变为
`update_frequency/track_list_timeout/threat_tcpa_horizon/threat_radius_scale`；avoidance 使用
`avoidance_radius_scale/point_extension_distance`，OS 半径由 processed snapshot 共享；barrier 使用
`closing_segment_length/lateral_margin`，不再声明自己的 `os_radius`。
只有 `avoidance_radius_scale` 与 `point_extension_distance` 可运行时更新；其余上述算法参数为启动只读。

### 动态目标快照契约

#### 普通场景与认证场景的发布归属

- `scenario_manager_node` 默认不发布跟踪列表；认证配置由 `merge_certi_config.py` 显式启用，
  默认发布到 `/certificate_case/tracked_ships`。普通 CCS 的 `/dynamic_ship/tracked_ships`
  由动态船管理器／tracked merger 提供完整场景，不能另接一个只包含认证目标的完整列表发布者。
- 普通 CCS 动态船 GT 桥读取含逐船尺寸的 `/dynamic_ship/snapshot`。浮标栈开启时，最终
  `/sim/ground_truth` 由浮标栈的 `ground_truth_track_merger` 发布；浮标关闭时，
  由 `ccs_ground_truth_track_merger` 汇总各个 `_src` 输入。关闭动态船 GT 桥不关闭该备用 merger，
  认证 GT 仍可使用它。
- `ccs_certificate_encounter.launch.py` 沿用认证专用话题到 `_src/scenario` 的桥接。
  旧 `certifi_launch.launch.py` 同样读取生成配置中的 `tracked_ships_topic`：浮标开启时输出到
  `_src/scenario`，浮标关闭时直接输出最终 GT，避免和已有 merger 同时发布。
- 这项接线隔离不将两种目标列表直接混入导航输入，也不改变认证场景已有的导航数据源选择。

#### 实测快照与失效处理

- 唯一 Gazebo 位姿订阅读取 `/world/<world>/dynamic_pose/info`。只接受精确根模型名，
  `model::link` 不会覆盖根位姿。累积反馈供原控制使用，最新完整 FRAME 单独用于发布完整性判断。
- `/dynamic_ship/tracked_ships` 和 `/dynamic_ship/snapshot` 同批输出仿真船和虚拟船。
  仿真船使用实测 XY、艏向和世界坐标系低通速度；虚拟船使用自身计算的航迹。混合时保留 Gazebo
  原始帧时间戳，只有虚拟船时可用 ROS 时钟发布，不等待 Gazebo。
- 所有仿真船必须在同一新鲜帧中，且没有 pending/quarantine，才能发布整批。
  每艘仿真船需两次递增观测建立速度；缺失、过期、非法位姿或未完成 bootstrap 时整批停止发布。
  仿真船清空后以成员变更后的新鲜帧确认；位姿缺失不等于实体删除。独立虚拟后端清空后直接发布空列表。
  Gazebo 成员变更边界保留所有时间戳准入检查通过的帧的最大时间（含窗口内未来帧和未保留在待处理槽中的帧），新帧必须
  严格越过该边界。已删除根模型还须在边界后的帧中确认缺席，才解除对应的发布阻塞。
  最终发布按 registry → cache 顺序短暂加锁，重查成员版本和当前帧身份，并提升已到时的待处理帧；
  读取最终 ROS 时间后再次检查 reset Event，避免构造期间的新帧或时钟回退绕过发布门控。
- 默认 merger 为单输入透明转发（要求非空且符合 `frame_id` 的 frame，绝不重标坐标系）。
  可选多输入模式必须收到所有配置输入的完全相同 header；重复输入、自反馈环、重复/冲突 UUID
  均拒绝，不会把局部缓存并成伪完整快照。
- 重复/乱序观测不修改滤波历史；长间隙重新 bootstrap。未来帧只保留一个待处理槽，
  不推进已接受时间水位，时钟追上后先提升旧待处理帧，避免正常 `/clock` 偏差导致饥饿。
  未来时间差最多为现有 `pose_feedback_timeout`（默认 1s）；更远的帧在写入待处理槽或已观察时间
  上界之前丢弃，防止时钟重置后的旧时期迟到消息污染成员变更边界。时钟严重落后时停止发布，
  直到时钟追上且收到符合时间窗口的新帧。
- ROS 时钟回退回调只发出重置信号；控制线程清空位姿缓存/水位并重置速度历史，保留实体与
  隔离注册。merger 同样清空跨时钟时期的缓存。暂停沿用仿真时间语义，不增加墙钟超时。
- 停止发布依赖 TS 子系统现有 1s 输入契约使旧快照失效；没有新增立即失效的 wire 消息。
  消费端应区分 `ProcessedTSList.valid=false` 与合法空场景的 `NO_THREAT`。
- **要求完整新 launch 会话。** 注册表仅在内存中；只重启 manager 不能恢复仍存活模型的所有权，
  不可用这种方式判定空场景或恢复会话。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
