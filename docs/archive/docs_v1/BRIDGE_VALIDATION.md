
# ros_gz_bridge 验证步骤

此文档列出快速验证 ros_gz_bridge 与 session_manager 生成桥接配置的步骤。

## 预备
- 确保仿真已启动并且 `session_manager` 已运行（在 robot_bringup 或 main.launch 中）。

## 步骤

1. 查找并打开生成的 bridge 配置（YAML）
   - 位置示例：`logs/session_*/bridge_config.yaml` 或 `install/.../share/usv_sim_full/config/bridge_config.yaml`（视启动流程而定）

2. 检查关键映射
   - `/clock` 是否被桥接为 GZ_TO_ROS
   - thrusters 是否包含 ROS_TO_GZ 的映射，例如：

```yaml
- ros_topic_name: "/usv_sim_full/thrusters/left/thrust"
  gz_topic_name: "/model/usv_sim_full/joint/left_engine_propeller_joint/cmd_vel"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: ROS_TO_GZ
```

3. 检查 bridge 节点状态
```bash
ros2 node list | grep bridge
ros2 param list | grep bridge
```

4. 实际消息流验证
   - 在 ROS 端发布测试命令：
   ```bash
   ros2 topic pub /usv_sim_full/thrusters/left/thrust std_msgs/msg/Float64 "data: 100.0"
   ```
   - 在 Gazebo 端查看对应 joint 话题是否收到：
   ```bash
   gz topic -e -t "/model/usv_sim_full/joint/left_engine_propeller_joint/cmd_vel"
   ```

5. 常见故障与修复
   - Bridge 报错 "Did you forget to start the discovery service?"：延迟启动 bridge 或在 launch 中增加重试/延时逻辑
   - 话题未转发：检查配置中话题名称、类型是否匹配，确认命名空间与 sanitized_bridge_ns 的值

````
