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
控制定时器独立于阻塞的创建、删除和资源清理回调，原 `/clicked_point` 操作、物理运动、
滤波位姿反馈及跟踪消息时间戳保持原语义。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
