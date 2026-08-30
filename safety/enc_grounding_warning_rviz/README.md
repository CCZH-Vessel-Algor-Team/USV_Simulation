# enc_grounding_warning_rviz

提供搁浅预警 RViz 面板，显示龙骨下净空、风险等级和告警信息。

## 构建

```bash
colcon build --packages-select enc_grounding_warning_rviz --symlink-install
source install/setup.bash
```

## 启动

先启动 `enc_grounding_warning` 和 RViz，再在 RViz 中选择：

```text
Panels -> Add New Panel -> enc_grounding_warning_rviz/GroundingWarningPanel
```

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
