
# 常见问题与快速修复

本页收集在开发与 Docker 调试过程中遇到的常见问题与快速解决方案。

## 1. Gazebo 无法创建实体 / create 请求超时

- 现象：`ros_gz_sim create` 卡住或报 "create 请求世界名超时"
- 可能原因：Discovery 服务未就绪、网络命名或 world 名称解析问题
- 修复建议：
  - 确认 Gazebo 已启动并在正确的 Partition/World 下运行
  - 延迟或重试 bridge 的启动
  - 在 Docker 中确保主机网络与 container 网络配置允许通信

## 2. RViz 插件加载错误（容器）

- 现象：RViz 报 plugin 加载失败或界面空白
- 修复建议：
  - 在 Docker 镜像中安装必要的 rviz 插件包
  - 或使用 X11 转发 / Xvfb 提供虚拟帧缓冲

## 3. 纹理或 mesh 加载失败

- 现象：Gazebo 控制台中出现 "failed to create drawable" 或纹理找不到
- 修复建议：
  - 确认 `model://` 路径所指向的资源在 `description/models/` 中存在
  - 确保 `GZ_SIM_RESOURCE_PATH` 或 `GAZEBO_MODEL_PATH` 包含该路径

## 4. 推进器不响应 ROS 话题

- 现象：ROS 侧话题有消息，但 Gazebo 的 joint 没收到命令
- 修复建议：
  - 验证 bridge 配置是否包含对应的映射（见 `BRIDGE_VALIDATION.md`）
  - 检查 message 类型（std_msgs/msg/Float64 与 gz.msgs.Double 的对应）
  - 检查命名空间是否被 sanitized（session_manager 可能将命名空间替换）

## 5. parameter_bridge 启动顺序问题

- 现象：启动早期有关于 discovery 的警告，后续 bridge 成功
- 修复建议：
  - 在 launch 中添加短延时（或在 bridge 节点加重试逻辑）

## 额外调试命令
```bash
# 查看所有话题
ros2 topic list
# 查看 Gazebo 端话题
gz topic -l
# 检查 bridge 节点
ros2 node list | grep bridge
```

````
