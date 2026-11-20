# 动态船与风暴回归测试

此集合针对当前版本的操作和接口，后续修改动态船、风暴或 RViz 面板后运行。

| 文件 | 防止退化的行为 |
| --- | --- |
| `test_dynamic_ship_placement.py` | 单击默认航向、拖拽四向航向、仿真/虚拟分流、非法输入拒绝、虚拟船添加与删除/清空。 |
| `test_dynamic_ship_deletion.py` | Gazebo 删除确认、外部删除检测、查询失败不误删、失败后保留注册。 |
| `test_virtual_dynamic_ship.py` | 虚拟船不创建 Gazebo 发布器、往返运动、掉头期间零速度。 |
| `test_pose_feedback.py` | 当前纳秒时间戳的反馈有效期、速度计算、乱序忽略及重置。 |
| `test_storm_field_manager_node.py` | 漂移方向、默认配置隔离、天气/航迹/Marker 一致性、过期和删除/清空。 |
| `../../env_panel/test/test_target_list.cpp` | 中文与转义名称、选中状态保留、非法列表、控件销毁时取消排队更新。 |
| `../../env_panel/test/test_panel_interaction.cpp` | 两个互斥类型按钮、切换状态、底部备注、单一动态船添加工具。 |

在工作区根目录构建并运行（Bash）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select env_panel --symlink-install --cmake-args -DBUILD_TESTING=ON
bash src/usv_simulation/usv_sim_full/test/run_dynamic_storm.sh
```

Python 测试隔离外部仿真调用；Qt 测试使用 offscreen 模式。此集合不启动 Gazebo，不能代替真实仿真中的鼠标交互与实体回归。
