# enc_grounding_warning_msgs

定义仿真搁浅预警使用的 ROS 2 消息和服务接口。

## 构建

```bash
colcon build --packages-select enc_grounding_warning_msgs --symlink-install
source install/setup.bash
```

## 使用

本包为接口包，无独立运行节点。构建后由 `enc_grounding_warning` 和 RViz 插件使用。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
