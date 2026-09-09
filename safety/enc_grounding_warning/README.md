# enc_grounding_warning

根据水深栅格、船舶状态和规划航线计算龙骨下净空与搁浅风险。

## 构建

```bash
colcon build --packages-select enc_grounding_warning_msgs enc_grounding_warning --symlink-install
source install/setup.bash
```

## 启动

仿真已运行时，启动预警节点：

```bash
ros2 launch enc_grounding_warning enc_grounding_warning.launch.py namespace:=usv_1 use_sim_time:=true
```

启动完整仿真和预警：

```bash
ros2 launch enc_grounding_warning full_sim_with_grounding_warning.launch.py
```

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
