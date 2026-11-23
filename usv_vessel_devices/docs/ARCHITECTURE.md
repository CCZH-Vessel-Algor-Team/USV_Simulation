# 架构

同一 `ament_cmake` 包生成 ROS 接口并构建灯/桅杆的 `libVesselDevices.so` 和可选起翘的 `libThrusterTilt.so`。模型宏包含到船体 URDF，或使用独立固定测试台，无需修改 VRX 子模块。

每个模型的插件创建自己的 ROS context、带船名命名空间的节点和 executor 线程。模型销毁时取消 executor、join 线程并关闭自己的 context，不影响其他船或全局 ROS context。回调只更新互斥锁保护的目标状态，不访问 ECM。

- `Configure`：读取参数，创建 ROS 接口。
- `PreUpdate`：按模型查找实体，更新灯 `Material` 和 `VisualCmd`。位置误差生成限速速度目标，速度 PI 输出力矩（上限 100 Nm），积分项保持桅杆抵抗重力；真实关节具有角度和速度限制，不通过重置位置模拟运动。
- `PostUpdate`：读取物理角度、角速度、仿真时间，更新状态快照。
- ROS 墙钟 timer：发布快照，暂停期间可查询状态及调用服务。
- `Reset`：全部关灯、桅杆目标归零，等待世界物理重置。

默认十二盏桅杆灯都固定在活动 mast_link：四盏红绿灯使用两层横杆原有托盘，顶部白灯保持原位，另加右下灯和前后两组三层中线灯。另有四个 `roof_*` 灯固定于 base_link 的船顶四角，左红右绿，不随桅杆运动或缩放。默认共十六灯，前十二项顺序不变，四个 roof 名称追加在后。原 front/rear 接口名称分别表示上/下层，不改 ROS 协议。`lights_on_mast: false` 将原四灯移回 base_link，此时省略新增 roof 灯以避免重叠；关闭 CAD 桅杆或 `extra_lights_enabled` 则不生成新增七个桅杆灯。每个灯的固定关节均保留。每个灯 link 中命名为 `lamp` 的 visual 才受控制，插件同时识别 sdformat 转换后的名称后缀，明确排除 `housing`，而不是任意选择第一个 visual。

CAD 模式新增固定在船上的 mast_base_link，在其支架上沿设置虚拟 mast_joint，活动杆体和顶部灯罩绕轴放倒。三块桅杆网格和两块灯具网格均已是米制；转换工具及原始 CAD 不参与运行。灯具选用闭盖展示状态。碰撞/惯量使用简化几何体和名义质量，CAD 来源、坐标变换及虚拟铰链假设记录在 description/meshes/cad_assets.json。附件增加少量质量和惯量，启用后不承诺与旧船体物理参数完全一致。传感器挂载不变；未启用起翘时推进器结构不变。

转换时移除两层横杆旧灯的 24 个实体和四个开放曲面壳，保留托盘/横杆，记录四个安装基准。新灯底部放在基准上，尺寸按 `lamp_scale * mast_lamp_scale * CAD缩放` 适配，避免与原灯叠加或因桅杆放大而失去对齐。

其余七套旧灯移除 36 个实体、三个开放曲面壳，记录逐灯安装位置与尺寸系数。Xacro 从同一有序列表递归生成模型和插件项，保留原五项作为状态数组前缀；插件接受后续唯一且有效的 ROS 灯名。颜色可按灯覆盖红/绿，默认两组中线灯由低到高红绿红。默认 CAD 缩放为 XYZ 两倍，包含底座、碰撞和灯具坐标，安装基准和名义质量不变。

`usv_sim_full/session_manager` 仅在启用时校验并生成每船 YAML，本地船模包含附件宏。插件路径由环境钩子和 `infra_sim` 接入，无需新增设备桥接。

## 可选推进器起翘

`thruster_tilt_enabled` 默认 false，整机 `usv_bringup/config/simulation/ccs_config.yaml`
显式启用。只适配本地 CUSTOM 双发动机模型。`custom_thrusters.xacro` 在原转向关节上游
插入每侧 0.2 kg 的 `*_tilt_link` / `*_tilt_joint`，转轴沿船体局部 +Y，正角将向下的
桨叶向船尾抬起。零角的发动机与螺旋桨位置、原转向轴/角度范围/话题均保留。
安装点沿用原推进器坐标，不另外移动船顶灯、桅杆或传感器。

独立 `ThrusterTilt` 插件使用同样的每模型私有 ROS context/executor 生命周期，另发
`ThrusterTiltState`，不修改既有 `DeviceState` 的类型定义或 ABI。左右/同时命令入口
只修改目标缓存；物理线程计算限速、限加速度的角度参考和有限力矩闭环控制，
以前馈补偿重力及推进器水推力矩。不会收拢物理限位或通过重置位置模拟动画。
URDF effort 是仿真力矩限值，不代表真实液压压力。软件目标严格校验
`[0, max_angle]`，反馈不裁剪、不伪造角度。

原推力 ROS→Gazebo 桥接不变。起翘插件直接消费推力，独立设置桨转速，按桨盘
浸没面积和当地波浪水面计算水推力并沿桨轴施加；原生 Thruster 不再重复施力。
不根据起翘角度断推。完全出水时水推力为零，桨仍可转动；非有限输入按零处理。
浸没面积是工程近似，不模拟空化、通气或空气推力。

独立物理联调读取 bringup 主入口的真实 CCS 场景，固定两艘实际船模并隔离 domain / partition。
检查实际 link 位移、推进角速度反馈、转向兼容、浸没力、暂停/重置
和模型删除，不启动有全局清理逻辑的主 launch，不操作运行中的主仿真。
