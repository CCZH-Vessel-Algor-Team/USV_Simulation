# USV Simulation Health Monitor (`sim_test`)

`sim_test` 是一个用于 USV_ROS 仿真的自动化可视化状态自检工具。主要用于排查仿真启动后（如 `ros2 launch usv_sim_full main.launch.py`）模块加载不全、传感器（Camera/Lidar 等）无数据、QoS 配置不匹配等问题。

## 功能特性

1. **配置自动发现**：通过解析 `usv_sim_full/config/full_config.yaml`（或者通过终端参数传入），自动计算出当前仿真世界预期应该存在的 Nodes 和 Topics。
2. **节点存活检测 (Node Health)**：面板左侧实时展示关键节点（如 `ros_gz_bridge`、`usv_sim_wrapper`、`radar_controller` 等）是否真的启动成功。
3. **话题频率与数据流监听 (Topic Hz Monitor)**：
   * 自动探测并动态订阅 ROS 2 网络中出现的预期 Topics。
   * 本质等效于运行了无数个 `ros2 topic hz` 进程，实时显示真实的数据发送频率。
   * 如果频率严重低于期望值，会标为黄色 🟡 报警；如果卡死无数据，会亮红灯 🔴。
4. **快捷调试**：一键打开 `rqt_graph` 查看节点连接拓扑；一键打开 `rqt_image_view` 排查摄像头是否真的产出图像流，以此判断是 RViz 问题还是后端无图。

## 依赖要求

环境要求：
* ROS 2 (rclpy)
* PyQt5 (用于 GUI 界面的构建)
* pyyaml (用于读取配置文件)

可以通过以下命令安装可能缺失的依赖：
```bash
pip3 install PyQt5 pyyaml
```

## 编译方法

在工作空间根目录下执行该包的局部编译：
```bash
cd <ws>
colcon build --packages-select sim_test
source install/setup.bash
```

## 怎么跑? 🏃

### 1. 启动完整的仿真节点

打开一个新的终端启动仿真：
```bash
cd <ws>
source install/setup.bash
ros2 launch usv_sim_full main.launch.py
```

### 2. 启动自检工具 (Monitor)

在另一个终端执行：
```bash
cd <ws>
source install/setup.bash
ros2 run sim_test sim_monitor
```

> **注意**：脚本默认会去读 `usv_sim_full` 中安装的默认 `full_config.yaml` 文件。如果你使用的是自定义存放在其它位置的文件，你可以传入路径参数：
> ```bash
> ros2 run sim_test sim_monitor --config /path/to/your/custom_config.yaml
> ```

## 当“Camera 没有图像”时，如何通过此工具排查？

如果在 RViz 中看不到摄像头图像，请立刻打开 `sim_monitor`：
1. **看节点状态**：左侧 `bridge` 相关的节点是否亮绿灯？如果没绿亮，说明你的 `ros_gz_bridge` 组件死掉了。
2. **看 Topic HZ 状态**：
   * 找到列表中的 `.../camera/front/image_raw` （取决于你 yaml 里的相机名字配置）。
   * 观察 `Actual Hz`：如果不为 0（比如 > 10Hz），**说明 Gazebo 和 Bridge 都在正常干活。无图像纯粹是因为 RViz 这里 QoS 不对！**（你应该检查 RViz 中的对应 Image 显示策略，将 Reliability 从 Reliable 手动切到 Best Effort ）
   * 如果 `Actual Hz = 0.0`：**说明桥接失败或者是 Gazebo 物理挂了**。
3. **点 `rqt_image_view`**：直接在弹出的基础 ROS `rqt_image_view` 选择该话题查看有无画面（rqt会自适应去接管各类 QoS）。 

---
Author: MurphyChen的AI编程助手
Date: 2026.03
