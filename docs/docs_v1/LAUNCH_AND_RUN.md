
# 启动与运行（详细）

本章详细说明各个 launch 组件的职责与常用参数，帮助在不同运行模式（带 GUI / headless / Docker）下正确启动。

## Launch 组件说明

- `infra_sim.launch.py`：
  - 启动 Gazebo（可选 headless 模式），启动全局 `parameter_bridge`（/clock 等基础话题）。
  - 常用参数：`use_sim_time`、`gz_server`/`gz_client` 选择、`world` 文件路径。

- `robot_bringup.launch.py`：
  - 调用 `session_manager` 生成 URDF、bridge config 与 rviz 配置，然后启动机器人专属的桥接节点与控制节点。
  - 常用参数：`config_path`（指向 full_config.yaml）、`robot_name`、`use_sim_time`。

- `visualization.launch.py`：
  - 启动 RViz 并加载默认 rviz 配置文件（`src/usv_sim_full/rviz/usv_sim_full.rviz`）。

## 常见运行模式

1. 带 GUI（本地）
```bash
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'
```

2. Headless（CI / Server）
```bash
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml' gz_headless:=true
```

3. Docker 容器（示例）
 - 在 Dockerfile 中确保安装必要的 rviz 插件和 Xvfb（若需要 GUI 转发）。

## 调试与日志

- Bridge 日志：检查 `ros2 run ros_gz_bridge parameter_bridge --ros-args --log-level debug` 的输出。
- Gazebo 日志：在启动日志中搜索 plugin 加载或模型解析错误（纹理缺失、mesh 未找到等）。

