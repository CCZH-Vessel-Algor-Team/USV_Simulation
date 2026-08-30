# vrx_gazebo

提供 WAM-V 模型生成所需的 Gazebo 资源和启动文件。

## 构建

```bash
colcon build --packages-select vrx_gazebo --symlink-install
source install/setup.bash
```

## 启动

```bash
# 根据组件和推进器配置生成 WAM-V
ros2 launch vrx_gazebo generate_wamv.launch.py

# 在 RViz 中查看生成的模型
ros2 launch vrx_gazebo rviz.launch.py
```

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
- [变更记录](CHANGELOG.md)
