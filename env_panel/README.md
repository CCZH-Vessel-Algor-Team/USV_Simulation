# env_panel

提供 RViz 环境、目标船和风暴场配置面板；环境面板用于设置风、水流、波浪和太阳光。

## 构建

```bash
colcon build --packages-select env_panel --symlink-install
source install/setup.bash
```

## 启动

先启动 `usv_sim_full` 仿真，再在 RViz 中选择：

```text
Panels -> Add New Panel -> env_panel/VrxEnvPanel
```

面板只发布环境控制话题，由 Gazebo 插件计算船体受力。
目标船和风暴场面板可分别选择 `env_panel/DynamicShipConfig` 与 `env_panel/StormFieldConfig`。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
