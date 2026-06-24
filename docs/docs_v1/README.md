
# USV_Simulation — 文档索引（docs_v1）

本目录包含基于 `usv_sim_full` 功能包的分模块使用指南，目标读者是对 ROS 2 / Gazebo 有一定基础的同事，能够快速在本仓库上复现仿真环境并运行示例。

**其他文档层级**：

- **用户向（推荐优先）**：[`docs_v2/QUICK_START.md`](../docs_v2/QUICK_START.md)
- **架构与包职责**：[`docs_v3/仿真仓库结构说明.md`](../docs_v3/仿真仓库结构说明.md)
- **工作区总览**：仓库根 [`README.md`](../../../../README.md)、[`src/usv_simulation/README.md`](../../README.md)

推荐阅读顺序（本目录内）：
1. [QUICK_START.md](./QUICK_START.md) — 5 分钟快速体验（最小上手流程）
2. [USAGE.md](./USAGE.md) — 分模块完整使用指南（构建 / 启动 / 验证）
3. [LAUNCH_AND_RUN.md](./LAUNCH_AND_RUN.md) — 各 launch 组件说明与运行模式（GUI / headless / Docker）
4. [RVIZ.md](./RVIZ.md) — RViz 配置与调试（如何加载与常见问题）
5. [BRIDGE_VALIDATION.md](./BRIDGE_VALIDATION.md) — ros_gz_bridge 验证步骤（生成的 bridge_config.yaml 与运行验证）
6. [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) — 常见问题与快速修复（Gazebo / RViz / bridge 常见故障）

本目录文件会随着 V3 清理与 CI 增强逐步更新。

