# USV 跨进程分层多传感器融合系统 (usv_fusion_new) 测试与运行指南

本文档提供了如何编译、配置和测试完整的 USV 分层多传感器（视觉+毫米波雷达+导航雷达）融合追踪系统（`usv_fusion_new`）。此系统的核心特点是通过在操作系统层拉起多个子进程来完全绕过 CPython 的全局解释器锁（GIL），利用 ROS 2 的 DDS 进行高效异步串联，实现分布式的局部追踪 (Local Tracking) 与汇聚式的全局目标级关联 (Global Hub T2T)。

## 1. 软件架构概览
该架构包含四个层级，完全由 Python 纯核心计算实现，零依赖第三方繁重框架（仅有 NumPy & SciPy）：
1. **Level 1 (Local Fusion Nodes)**: 4个独立的、针对不同方向（前、后、左、右）的独立节点。各自利用 Extended Kalman Filter (EKF) 在传感器的局部感知原点进行独立的坐标对齐与 Track 维护。
2. **Level 2 (Global Fusion Node)**: 唯一全局汇聚中心（Hub），将从 Level 1 传入的 4 方向局部 `Track`，结合从导航雷达（Nav Radar）传入的 `Track`，基于全局 `base_link` 和匈牙利算法进行跨视野盲区（Cross-FOV）和跨模态（T2T）轨迹身份重分配和交接 (Handover) ，维护唯一性 ID 组。
3. **Level 3 (Visualization Node)**: Rviz2 可视化标记生成节点。它将枯燥的全局 4x4 （$x, y, v_x, v_y$）数组转化为紫色的3D发光球、青色的速度预见箭头以及常驻历史文本。
4. **Level 4**: RViz2 数据展示容器。

---

## 2. 编译流程 (Build Instructions)

确保你处于正确的 ROS 2 工作空间根目录（`<ws>/`，包含 `src/` 的 colcon 工作区）下：

### 2.1 依赖安装与更新
该功能包利用了 `scipy` 做代价分配，`filterpy` （或其他纯实现）做 EKF：
```bash
pip3 install numpy scipy filterpy
```

### 2.2 构建功能包
因为这是一个带有额外资源的纯 `ament_python` 项目（包含 `launch` 和 `rviz`）：
```bash
# 1. 切换到工作空间根目录
cd <ws>

# 2. 编译指定的工程包，允许符号链接便于开发调试（免除重复编译）
colcon build --packages-select usv_fusion_new --symlink-install

# 3. 重新源化环境路径
source install/setup.bash
```
*提示: 如果你要编译整个工作空间的所有模拟与感知节点，可以不带 `--packages-select` 参数运行 `colcon build --symlink-install`.*

---

## 3. 测试与运行流程 (Testing Procedure)

测试框架可以通过原有的 ROS 2 Launch 体系实现一键无缝拉起，内部会自动配置进程管理和参数。

### 3.1 一键拉起系统
在 sourced 过 `install/setup.bash` 的终端窗口中，运行以下命令。该命令会自动拉出：4 个方向节点、1 个全局节点、1 个标记转化节点、以及配置好融合 Marker 的 **RViz**。

```bash
ros2 launch usv_fusion_new fusion_system.launch.py
```
**期望现象 / Expected Behavior:**
由于这是脱离 GIL 的多进程 launch，终端的输出将极为密集。系统会被划分为具有不同进程 PID (Process ID) 的 log 输出。RViz 将在数秒后自动弹出。

### 3.2 注入模拟测试数据验证
为了激活融合节点，需要向系统灌入观测数据或运行地面真实环境仿真节点（你的 `sim` 包）。
打开一个**新的终端**，加载环境并启动原有的全量模拟节点，或者发包（请确保话题发布存在才能在融合端打印接受到数据的回显）：
```bash
cd <ws>
source install/setup.bash

# 替换为你实际的仿真数据发送 Launch / 袋子 (Rosbag) / 独立发送节点脚本
ros2 launch ground_truth_sim ground_truth_sim.launch.py
# （或者如果有别的脚本例如 run_full_sim.sh 也是可以的，例如 ./scripts/run_full_sim.sh）
```

### 3.3 验证与诊断 (Verification)

验证流程分为 **系统架构检查** 和 **融合效果检查**：

1. **检查节点集群存活 (Terminal 3):**
   ```bash
   ros2 node list
   ```
   **期望返回值:** 你应该看到至少这些系统新节点：
   `/local_fusion_front`, `/local_fusion_rear`, `/local_fusion_left`, `/local_fusion_right`, `/global_fusion_node`, `/global_viz_node`。

2. **检查话题联通拓扑 (Terminal 3):**
   ```bash
   ros2 topic list
   ```
   **期望返回值**应包括各方向的输出 `Track` 以及融合后的结果：
   - `/fusion/front/local_tracks`
   - `/fusion/rear/local_tracks`
   - `/fusion/global_tracks` (最终结果级)
   - `/fusion/global_markers` (渲染级)

3. **从 RViz 肉眼验收效果:**
   - 在弹出的 RViz 面板中，检查 `Global Fusion Markers` 插件。
   - 寻找地图坐标系 (`base_link`) 下是否出现了**紫色实心小球**，小球上延伸出蓝色的表示速度走向的坐标轴，上方浮动有大号的 `ID: xx` 白色文本标签。当重叠的多个传感器提供同一个区域内的雷达和视觉目标时，该 ID 不应来回跳动（闪烁），系统应能计算滑动窗口并进行无缝平滑交接。

---

## 4. 架构特殊设计注意事项 (Design Notes)
- `global_fusion_node.py` 中的 `threading.Lock()` 在多进程向其注入数据时**极度重要**。因为它是 T2T 的全局重排序域，如果不加锁或者抛弃负向 Delta time (`dt < 0`)，全局卡尔曼推演会出现崩溃性预测错误。此逻辑已被妥善约束处理。
- 采用局部全分布式独立进行卡尔曼预测的方案，规避了全局维度直接构建极庞大的协方差计算的耗时。全局中心仅作为简单的 $4 \times 4$ `Track-As-Measurement` 高度置信映射和跨视野的缓冲分配池。