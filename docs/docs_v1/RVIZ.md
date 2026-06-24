````markdown
# RViz 使用与调试

本文件介绍如何使用项目提供的 rviz 配置并排查常见的 RViz 问题。

## 默认配置

- 文件：`src/usv_sim_full/rviz/usv_sim_full.rviz`
- Fixed Frame：`usv_sim_full/base_link`
- 默认显示项：Grid、RobotModel（/robot_description）、TF、Odometry、PointCloud2、IMU

## 加载配置

```bash
rviz2 -d "$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/rviz/default.rviz"
```

或在 launch 中通过 `visualization.launch.py` 自动加载（若 launch 已配置）。

## 常见问题与排查

1. RViz 报错找不到 Fixed Frame
   - 确认 TF 被发布：`ros2 topic echo /tf` 或 `ros2 topic echo /tf_static`
   - 检查节点是否已发布 base_link：`ros2 node list` 与 `ros2 topic list`

2. 视觉元素缺失或空白
   - 确认相关话题有消息：`ros2 topic echo /usv_sim_full/sensors/lidar/front/points`
   - 如果是 RobotModel 空白，确认 `/robot_description` 参数被正确设置（xacro 是否正确编译并发布到参数服务器）。

3. RViz 插件加载失败（容器中常见）
   - 在 Docker 中运行需安装对应 rviz 插件包或使用 host 的 GUI 转发（X11 / Xvfb）。

## 推荐调试命令

```bash
ros2 node list
ros2 topic list | grep usv_sim_full
ros2 topic echo /usv_sim_full/sensors/imu/data
gz topic -l | grep usv_sim_full
```

````
