# USV 多传感器融合节点测试指南

本文档介绍了在 `ground_truth_sim` 工作区中，如何完整启动多传感器仿真并测试基于 卡尔曼滤波 (EKF) 与 匈牙利算法的 `usv_fusion` 融合节点。 

## 1. 环境准备

目前 Python 库 `filterpy` 在最新的 `numpy 2.x` 与 `scipy 1.14+` 版本中存在兼容性问题（会抛出无法导入 `Inf` 等报错）。在测试开始前，请务必在终端中降级并安装兼容的科学计算库：

```bash
pip install "numpy<2.0.0" "scipy<1.14.0" filterpy
```

## 2. 编译工作区

确保你在工作区的根目录下，并重新编译所有依赖包，以确保刚刚添加的 `usv_fusion` 被系统正确识别。

```bash
cd <ws>
colcon build
```

## 3. 测试流程

此测试需要开启两个终端窗口：**终端 1** 用于运行真值与多传感器仿真模拟系统，**终端 2** 用于运行我们的数据融合节点。

### 终端 1：启动基础仿真系统

该步骤会启动目标真值随机器、4 个视觉摄像头模拟节点、4 个毫米波雷达模拟节点、TF 坐标变换树，并自动打开 RViz。

```bash
cd <ws>
source install/setup.bash

# 直接通过 launch 文件启动基础仿真环境（参数文件默认由安装目录解析）
ros2 launch ground_truth_sim ground_truth_sim.launch.py

> **注意：** 如果你的运行环境没有图形界面（例如 SSH 远程、无头服务器），上述命令可能会因为启动 RViz 失败而崩溃。在这种情况下，请使用下面的命令在无图形界面模式下启动：
> ```bash
> ros2 launch ground_truth_sim ground_truth_sim.launch.py use_rviz:=false
> ```
```

此时你应该能在弹出的 RViz 界面中看到带有视场角扇形、且各种传感器模拟结果（绿色圆柱体、蓝色球体）不断跳动的观测画面。

### 终端 2：启动数据融合节点

新开一个终端，加载工作区环境并启动 `usv_fusion`。

```bash
cd <ws>
source install/setup.bash

# 运行 Python 融合节点
ros2 run usv_fusion fusion_node
```

当节点成功启动后，如果传感器输入到了有效数据，它会在后台自动进行时间同步与数据关联匹配。

## 4. RViz 可视化验证

由于 RViz 是由终端 1 的默认配置启动的，融合节点的输出 Marker 默认并没有加入画面，需要手动在 RViz 中添加：

1. 在 RViz 左侧的 Displays 面板，点击左下角的 **`Add`** 按钮。
2. 选择 **`By topic`** 标签页。
3. 找到 **/usv_fusion /track_markers** 话题，选择其下的 `MarkerArray` 并点击 OK。
4. **效果确认**：你能看到 RViz 画面中出现了 **橙色的实心球体** 代表融合后的追踪航迹，球体上方还悬浮着白色的文字标记，显示该航迹的 **唯一追踪 ID** 以及目前的 **融合速度 (V)**。

## 5. 参数与话题调试

- **查看原始输出话题**：`ros2 topic echo /usv_fusion/tracks` （格式为 `usv_interfaces/msg/GlobalTrackArray`）。
- **观测 TF 树**：融合节点依赖于 `base_link`。可以通过 `ros2 run tf2_tools view_frames` 检查传感器（如 `sim_vision_left`、`sim_mmwave_front` 等）到中心 `base_link` 的 TF 树是否完整。