# docs_v4 — 用户文档索引

**当前正式 Demo 线**：Ubuntu 22.04 + **ROS 2 Humble** + **Gazebo Harmonic** + 自建 **[`usv_nav`](../../../usv_nav/README.md)**。

| 文档 | 说明 |
|------|------|
| [QUICK_START.md](QUICK_START.md) | **推荐入口**：环境、统一 install 构建、Demo launch、常见问题 |
| [BUILD.md](BUILD.md) | 构建细节、`--packages-up-to`、nav 写入 `install/` |
| [DEMO_RUN.md](DEMO_RUN.md) | 三视觉 + 毫米波 + Nav2 全栈参数与验证 |
| [main_launch.md](main_launch.md) | **`main.launch.py` 使用说明**：Launch 参数、常用 YAML 字段、生成逻辑、典型场景 |
| [custom_config.md](custom_config.md) | **自建配置包**：复制 YAML、目录布局、`config_path` 启动 |
| [certifi_launch.md](certifi_launch.md) | 认证会遇仿真：`certifi_launch` 流程、案例 YAML 联动 |

**历史文档**（仅供参考，矩阵可能过时）：

- [archive/docs_v3/QUICK_START.md](../archive/docs_v3/QUICK_START.md) — 含旧版 Docker 表述
- [archive/](archive/) — v1 / v2 / v3 完整存档

工作区级说明：

- [docs/sim_tf_tree.md](../../../../docs/sim_tf_tree.md) — 仿真 TF
- [docs/USV_GIT_ARCHITECTURE.md](../../../../docs/USV_GIT_ARCHITECTURE.md) — Git 多仓
- [docs/RELEASE_VERSIONING.md](../../../../docs/RELEASE_VERSIONING.md) — 版本与子模块 SHA
