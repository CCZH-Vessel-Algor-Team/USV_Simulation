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

### RViz导航目标

CCS 入口默认 `autonomy_v2:=true`，在现有 `three_vision_one_mmwave.rviz` 显示底稿上生成
增量加入 **USV BT v2** 页签及 **USV Test Goal** 工具（快捷键 `u`）。原 Navigation 2、G 工具、
显示、视角与保存布局保留；v2 下原单点/多点和 Pause/Resume/Cancel 通过受管任务接口执行。
固定坐标系使用 `map`：

- 等待 Nav2/readiness、任务状态及真实 FCU 状态就绪；默认观察 `/usv1/cc/state`，仅已连接、已解锁且 OFFBOARD/GUIDED 时获得自主权。纯模拟权限按钮需显式选择下述测试配置。
- 用 USV Test Goal 在地图上点击并拖拽设航向，航点进入表格，此时不运动。
- 在航点页选择 REQUIRED（逐点确认）或 VIA（连续经过点），点击 **新建导航任务**。
- 顶部常驻 **NAVIGATION / HOLD / DOCK / UNDOCK** 同级模式按钮；NAVIGATION 接续进入 HOLD 前的导航，不重建路线或清零预算。
- 使用常用页 **暂停、继续、结束** 控制生命周期，进度由服务端持久化。
- 新面板分常用、航点、模式、测试、诊断五页；暂存任务接续位于模式页。
- **DOCK/UNDOCK** 后端当前返回 501，不破坏当前任务，按钮不虚假显示已切换。
- 测试页和按钮始终保留。真实 FCU 模式禁用“仿真自主权/仿真人工接管”，保留海事等待、定位失效、急停等仿真故障注入与保护复位；合成控制权仅在显式模拟权限配置下提供。

面板根据当前状态自动填写 boot/revision，请求通过 `/<船>/autonomy/manage` 下达。
202 是管理确认，不是物理任务完成。默认 `authority_source:=cc_state`、
`enable_autonomy_test_controls:=true`。该开关控制故障注入，不再等同于模拟控制权；真实控制权适配器与模拟权限发布者仍不可同时启用。

```bash
# 真实状态监测：BT 在 /usv_1，FCU 聚合状态在 /usv1，允许显式桥接命名空间。
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py \
  left_crossing_stand_on:=true \
  authority_source:=cc_state fcu_state_topic:=/usv1/cc/state \
  fcu_state_clock:=system enable_autonomy_test_controls:=true

# 仅用于不连接真实 FCU 的合成权限/故障测试。
ros2 launch usv_sim_full CCS_Certified_Simulation_Environment.launch.py \
  authority_source:=simulation enable_autonomy_test_controls:=true
```

`authority_source`、`fcu_state_topic`、`fcu_state_clock` 通过 CCS → 三传感器 bringup →
thruster bringup → autonomy 显式传递。消息类型是 `cc_msgs/USVState`，不是 `mavros_msgs/State`。
`authority_source=cc_state` 时，故障节点以只读参数 `simulate_authority=false` 启动，只提供三类
故障服务，不创建 `control/authority` 发布者、`autonomy/test/authority` 或 `control/request_manual`。
RViz 设置 `Simulation Tests=true`、`Simulation Authority=false`，前两个权限按钮显示但禁用。
`enable_autonomy_test_controls=false` 可完全禁用故障注入；测试页仍保留并显示禁用原因。
只有 `authority_source=simulation` 且启用测试时才设置 `simulate_authority=true`，用于纯模拟权限测试。
测试急停使用独立 `autonomy/test_emergency`，在门控内与真实 `safety/estop` 取或，解除测试输入不能
覆盖真实急停，也不清除既有保护锁存。该测试输入仅在只读 `enable_simulation_tests` 启用时订阅。
输入头默认为系统时间，适配器按该时间检查来源新鲜度，并以单调时钟检测停更/重放；输出
`/<船>/control/authority` 使用 BT 所在 ROS 时钟（可为仿真时钟）。输入确实使用同一 ROS 仿真
时钟时才配置 `fcu_state_clock:=ros`。默认输入超时 `state_timeout=1.0` 秒，可通过 autonomy 参数文件配置。
MANUAL/POSCTL、未解锁、断连或超时均不放行自主运动；是否自动恢复任务仍受
`automatic_authority_return=false` 等既有生命周期策略约束。

适配器只观察、不解锁、不切模式。禁用模拟控制后，模拟 `control/request_manual` 服务也不再存在；
实际接管应通过遥控器/飞控既有路径操作，不能把“请求人工接管”当作已经完成切换。
此配置仅更换状态/控制权来源，执行后端仍是本仿真的受管推进器桥，**不自动改为 PX4 输出后端**。
源码更改需重新构建并在受控停机后重启，不能与旧模拟权限发布者并行运行。

本次验证：197 项 Python 回归与 30 项通信/控制隔离测试通过。随后已将 `usv_autonomy`、
`usv_sim_full`、`cc_ctrl_interface` 正式安装到容器 `/workspace/install`；新终端只需加载
`/workspace/install/setup.bash`，无需再加载 `/tmp/opencode` 的测试安装。安装不自动重启节点；
控制/估计进程也需使用新版，以启用上游 MAVROS 状态超时保护。
只读现场探针已验证系统时间输入到仿真时间输出的转换；
真实接管、模式切换和执行后端验收仍需操作员在安全条件下完成。
态势监测在 CCS 中默认 5 Hz，与 v2 数据时效相匹配。

HOLD 在区域内持续监测，越外半径后返航，进入内半径就撤权并退出该次导航，随后继续监测并允许再次返航。
正常返航不消耗重试次数；持续时间 0 表示不限时。CCS 可用 `python3 scripts/verify_ccs_hold.py --cycles 3`
在仿真船上注入位移并验收连续返航，脚本结束时停止自建任务、恢复原权限。

配置实际经过 `ccs_bt_v2_*.rviz` 与 main.launch 的 `usv_rviz_*.rviz` 运行时副本，原底稿不改写。
若显式 `autonomy_v2:=false`，Navigation 2 / Nav2 Goal 使用原直接 Action 流程。
**2D Pose Estimate**和**Publish Point**用于下述目标船生成，不用于设置本船导航目标。

### RViz生成动态目标船

CCS的 **2D Pose Estimate** 工具发送到专用 `/dynamic_ship/spawn_pose`：点击选择位置，拖拽箭头选择
TS初始艏向。固定坐标系使用 `map`，沿用Gazebo world XY与map对齐的前提；非法坐标系、非有限值和
零四元数会被拒绝。速度、形状、half-distance仍取自DynamicShipConfig，面板Heading不覆盖拖拽方向。
原 **Publish Point** 入口继续保留，并继续使用面板Heading；spawn/delete/config服务也保持原样。
管理器的 `spawn_pose_topic` 是启动只读参数，覆盖时须同时调整RViz工具的Topic。
Pose Estimate的协方差与请求时间仅属于编辑请求，不作为目标观测；跟踪发布仍等待Gazebo实测完整帧。

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
控制定时器独立于阻塞的创建、删除和资源清理回调；保留原 `/clicked_point` 操作、
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
- CCS 浮标栈开启时，动态船 GT 桥读取 `/_internal`，最终 `/sim/ground_truth` 由浮标栈的
  `ground_truth_track_merger` 发布；浮标关闭时，桥读取 `/dynamic_ship/tracked_ships`，
  由 `ccs_ground_truth_track_merger` 汇总各个 `_src` 输入。关闭动态船 GT 桥不关闭该备用 merger，
  认证 GT 仍可使用它。
- `ccs_certificate_encounter.launch.py` 沿用认证专用话题到 `_src/scenario` 的桥接。
  旧 `certifi_launch.launch.py` 同样读取生成配置中的 `tracked_ships_topic`：浮标开启时输出到
  `_src/scenario`，浮标关闭时直接输出最终 GT，避免和已有 merger 同时发布。
- 这项接线隔离不将两种目标列表直接混入导航输入，也不改变认证场景已有的导航数据源选择。

#### 实测快照与失效处理

- 唯一 Gazebo 位姿订阅读取 `/world/<world>/dynamic_pose/info`。只接受精确根模型名，
  `model::link` 不会覆盖根位姿。累积反馈供原控制使用，最新完整 FRAME 单独用于发布完整性判断。
- `/dynamic_ship/tracked_ships` 默认只包含本 manager 注册的动态船。Pose 使用实测 XY/艏向，
  linear twist 使用世界坐标系低通速度；假定 Gazebo world XY 与 `map` 对齐。header 精确保留
  Gazebo 秒/纳秒，禁止用接收时间补戳。不会发布开环推算的跟踪状态。
- 所有注册目标必须在同一新鲜帧中，且没有 pending/quarantine，才能发布整批。
  每个目标需两次递增观测建立速度；缺失、过期、非法位姿或未完成 bootstrap 时整批停止发布。
  空列表只表示已知零注册且无 pending，并且收到成员变更后的新鲜帧；位姿缺失不等于实体删除。
  成员变更边界保留所有时间戳准入检查通过的帧的最大时间（含窗口内未来帧和未保留在待处理槽中的帧），新帧必须
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
