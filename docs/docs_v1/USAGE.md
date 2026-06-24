
# usv_sim_full 使用指南（分模块）

本指南面向具备 ROS2 基础的工程师，分模块说明如何在本仓库上构建、启动并验证仿真系统，侧重于 `usv_sim_full` 功能包的使用。

目录
- 前置条件（依赖与环境）
- 构建（colcon）
- 启动流程（launch 组件说明）
- 控制与接口（teleop、话题说明）
- 桥接（ros_gz_bridge）
- 可视化（RViz）
- 验证步骤（最小验证套件）

---

## 一、前置条件

- 操作系统：Linux（推荐 Ubuntu 24.04）
- ROS 发行版：**Humble**（见 [docs_v4/QUICK_START.md](../docs_v4/QUICK_START.md)）
- Gazebo：**Harmonic**（`gz sim`，经 `ros_gz_bridge`）
- Python：3.8+（用于脚本）
- 推荐工具：colcon、rviz2、gz 命令行工具

安装提示（最小）：
```bash
# 安装系统依赖（示例）
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions \
  ros-<distro>-ros-base rviz2
# 安装 ros_gz_bridge 等按项目 README 指示
```

## 二、构建

在仓库根或工作区根下运行：

```bash
# 推荐先清理旧的构建产物
rm -rf build install log
colcon build --packages-select usv_sim_full
source install/setup.bash
```

注意：完整构建所有包可能需要更多依赖，若使用 Docker 镜像请参照 `notes/QUICK_START.md`。

## 三、启动流程（分模块说明）

本项目采用组件化 launch：基础设施（infra_sim）、机器人 bringup（robot_bringup）与可视化（visualization）。主入口通常是 `main.launch.py`，它会组合这些组件。

- infra_sim：启动 Gazebo（或 headless server）以及全局 bridge（/clock 等基础话题）。
- robot_bringup：调用 `session_manager` 生成 URDF、bridge 配置，并启动机器人专属的桥接节点和相关驱动。
- visualization：启动 RViz，加载默认 rviz 配置。

示例启动：

```bash
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'
```

## 四、控制与接口

- 主要固定帧：`usv_sim_full/base_link`（RViz 与多数显示项假定此为 Fixed Frame）
- 主要话题示例：
  - 推进器控制：`/usv_sim_full/thrusters/left/thrust`, `/usv_sim_full/thrusters/right/thrust` （兼容 `/wamv/...` 命名空间）
  - 里程计：`/model/usv_sim_full/odometry`（桥接生成）
  - IMU：`/usv_sim_full/sensors/imu/data`
  - Lidar：`/usv_sim_full/sensors/lidar/front/points`

遥控示例（键盘 teleop）：

```bash
python3 src/usv_sim_full/scripts/dual_thruster_teleop_incre.py
```

或直接使用 `ros2 topic pub` 发布控制消息进行测试：

```bash
ros2 topic pub /usv_sim_full/thrusters/left/thrust std_msgs/msg/Float64 "data: 100.0"
```

## 五、桥接（ros_gz_bridge）

核心点：项目使用 `session_manager.py` 动态生成 bridge 配置（YAML），并为机器人传感器与控制指令生成映射。启动流程中会读取该配置并启动 `parameter_bridge` 节点。

验证桥接（详见 `BRIDGE_VALIDATION.md`）：
- 检查 bridge 配置文件位置（通常在 `logs/session_*/bridge_config.yaml` 或 `install` 后的 share 目录）
- 确认 `/clock` 被桥接（GZ_TO_ROS），并确认至少一个 thruster 话题从 ROS 被转发到 Gazebo joint cmd 话题（ROS_TO_GZ）

## 六、可视化（RViz）

已包含 rviz 配置：`src/usv_sim_full/rviz/usv_sim_full.rviz`。

加载方法：
```bash
rviz2 -d "$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/rviz/default.rviz"
```

Fixed Frame：`usv_sim_full/base_link`。如 RViz 报错找不到 frame，可先运行系统并用 `ros2 topic echo /tf_static` / `ros2 node list` 检查 TF 发布状态。

## 七、最小验证套件（发布前检查）

1. `colcon build --packages-select usv_sim_full` 能成功
2. `ros2 launch usv_sim_full main.launch.py ...` 启动无致命错误
3. `ros2 node list | grep bridge` 显示 bridge 节点
4. `ros2 topic list | grep thrusters` 能看到 thruster 话题
5. 使用 `ros2 topic pub` 向 thruster 发送测试命令，同时用 `gz topic -l` / `gz topic -e` 验证 Gazebo 侧接收

## 八、文件位置参考

- 包根： `src/usv_sim_full/`
- 主要资源： `src/usv_sim_full/description/`、`src/usv_sim_full/worlds/`、`src/usv_sim_full/launch/`、`src/usv_sim_full/rviz/`、`src/usv_sim_full/scripts/`
- 文档： `docs/`（本目录）与 `notes/`（技术笔记）

## 九、后续建议

- 在 CI 中加入 lint / 安装验证步骤
- 提供 `scripts/verify_v3.sh` 做自动化的最小验证

---
本指南旨在快速上手与验证。如果需要我可以把这些命令包装为脚本并在 CI 中运行（需要你确认 CI runner 环境）。

