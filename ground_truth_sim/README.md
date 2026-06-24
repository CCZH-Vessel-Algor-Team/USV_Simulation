# ground_truth_sim 文档导航

该目录当前包含 `ground_truth_sim` 包本体，以及与其强相关的联调说明与脚本。为避免重复，文档职责收敛如下：

- `PACKAGE_README.md`：包级单一事实源（节点、参数、话题、launch、运行方式）。
- `docs/sim_validation_guide.md`：仿真与融合联调的验证流程。
- `docs/usv_fusion_new_guide.md`：`usv_fusion_new` 流程说明。
- `docs/fusion_test_guide.md`：历史融合测试说明（保留作兼容参考）。

## 快速入口

- 包级详细说明：[`PACKAGE_README.md`](./PACKAGE_README.md)
- 关键启动文件：
  - `launch/ground_truth_sim.launch.py`（独立真值模式）
  - `launch/gazebo_ground_truth.launch.py`（Gazebo 桥接模式）
- 关键参数：
  - `config/ground_truth_params.yaml`
  - `config/gazebo_ground_truth_bridge.yaml`

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
