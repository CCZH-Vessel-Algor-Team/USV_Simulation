# ground_truth_sim 文档导航

该目录包含 `ground_truth_sim` ROS 2 包本体及联调脚本。文档职责收敛如下：

- **`PACKAGE_README.md`**：包级单一事实源（节点、参数、话题、launch、运行方式）。
- **`scripts/verify_sim_health.sh`**：真值与传感器模拟话题的健康检查（需先启动相关节点）。

## 快速入口

- 包级详细说明：[`PACKAGE_README.md`](./PACKAGE_README.md)
- 关键启动文件：
  - `launch/ground_truth_sim.launch.py`（独立真值模式）
  - `launch/gazebo_ground_truth.launch.py`（Gazebo 桥接模式）
  - `usv_sim_full/launch/ground_truth_test.launch.py`（仅 Gazebo + 真值 + RViz 隔离测试）
- 关键参数：
  - `config/ground_truth_params.yaml`
  - `config/gazebo_ground_truth_bridge.yaml`

整机场景中的 `scenario.ground_truth_sim` 字段见 [`docs/docs_v4/main_launch.md`](../docs/docs_v4/main_launch.md) 附录。

## 工作区相关包

本包通常与以下包协同运行：

- `ground_truth_sensor_sim`：消费 `/sim/ground_truth` 生成视觉与毫米波模拟输出（位于 `src/usv_perception/ground_truth_sensor_sim`）。
- `usv_interfaces`：提供统一消息定义与 topic 常量。

建议在工作区根目录统一构建：

```bash
colcon build --packages-select usv_interfaces ground_truth_sim ground_truth_sensor_sim --symlink-install
```

## 说明

- 历史文档中出现的 `src/sim/...` 路径已不再使用；请以当前目录结构为准（`src/usv_simulation/ground_truth_sim/...`）。
- 为避免同话题重复发布，`ground_truth_node` 与 `gazebo_ground_truth_bridge_node` 不应同时发布同一个 `/sim/ground_truth`。
- 原 `docs/` 下融合联调分册已移除；验证流程以 `PACKAGE_README.md` 与 `scripts/verify_sim_health.sh` 为准。
