# 自动驾驶仿真数据源 (Sim Packages) 校验流程与验证指南

为了确保整个传感器融合系统能够获得稳健、准确的前端数据，我们第一步需要验证底层的仿真功能包（包含真值生成器 `ground_truth_sim` 以及传感器模拟器 `ground_truth_sensor_sim`）是否正常工作。

本指南将带你从下至上排查测试这些模拟数据节点及其话题的流通情况。

---

## 1. 编译与环境准备
在执行任何校验之前，请确保仿真功能包已正确编译并刷新了环境变量：
```bash
# 切换到工作空间根目录
cd <ws>

# 编译相关仿真包
colcon build --packages-select usv_interfaces ground_truth_sim ground_truth_sensor_sim --symlink-install

# 刷新环境
source install/setup.bash
```

---

## 2. 启动仿真系统

在测试之前，请新建一个终端并拉起包含“全量传感器模拟”的 launch 文件：
*(Launch 位于 `ground_truth_sensor_sim` 或 `ground_truth_sim`；三视觉 + 前向毫米波示例)*

```bash
ros2 launch ground_truth_sim ground_truth_sim.launch.py
# 或仅传感器节点（需已有 /sim/ground_truth）：
ros2 launch ground_truth_sensor_sim three_sensor_sim.launch.py
```
*(也可直接执行 `./scripts/run_multi_sensor_sim.sh`。)*

**期望现象**：在此终端中，你应当看到多个节点成功拉起的日志，包括 `ground_truth_node`, `sim_vision_front`, `sim_mmwave_front` 等。且不能有报错异常退出（crash）。

---

## 3. 校验流程与命令行字典

请**再新建一个终端**（注意每次都要执行 `source install/setup.bash`），通过以下指令层层校验验证结果。

### 3.1 检查仿真节点组是否存活
**执行命令：**
```bash
ros2 node list
```

**正常输出应包含：**
* `/ground_truth_node` (生成场景绝对真值)
* `/static_tf_broadcaster` (如果存在，负责发布仿真传感器的固定坐标系)
* `/sim_vision_front`, `/sim_vision_left`, `/sim_vision_right`, `/sim_vision_rear` (4个视觉模拟节点)
* `/sim_mmwave_front`, `/sim_mmwave_left`, `/sim_mmwave_right`, `/sim_mmwave_rear` (4个毫米波模拟节点)

---

### 3.2 检查 Ground Truth (场景真值) 输出
整个系统由 Ground Truth 提供最原始的上帝视角坐标数据。它应以 50Hz 左右的频率向外广播。

**执行命令：**
```bash
ros2 topic hz /sim/ground_truth
```
**正常输出**：持续打印类似 `average rate: 50.000` (Hz) 的信息。

**执行命令：**查看真值具体内容
```bash
ros2 topic echo /sim/ground_truth --once
```
**正常输出**：能看到包含 `tracks` 数组的数据结构，输出模拟容器的真值，例如 `x`, `y`, `vx`, `vy`。如果此时数组内部有正确的数字，说明**真值生成组件完全正常**。

---

### 3.3 检查传感器视野裁剪与加噪 (ground_truth_sensor_sim) 输出

每个方向的视觉和毫米波雷达模拟节点会监听 `ground_truth`，根据自身的安装朝向（Yaw）、视野相角（FOV, 如视觉 90°，雷达 120°）以及采样盲区对真值进行遮挡裁剪和加噪。

**校验视觉数据 (以 Front 为例)：**
```bash
ros2 topic hz /vision/front/detections
```
**正常输出**：因配置档通常要求视觉按 25Hz 频率输出，你应当看到类似 `average rate: 25.000` (如果此方向当前没有目标进入视野，`tracks` 数组可能为空，但话题本身必须以设定频率被挂载发布！)。

**检查具体数据：**
```bash
ros2 topic echo /vision/front/detections --once
```
* **验证点**：如果你看到内部的 `tracks` 中的坐标数据包含少量小数震荡飘移，说明其距离噪声 (Distance noise) 与角度噪声 (Angle noise) 代码模拟注入成功。并且它的 `frame_id` 应该是 `camera_front_link`。

**校验毫米波数据 (以 Front 为例)：**
```bash
ros2 topic hz /mmwave/front/targets
```
**正常输出**：按配置文件，雷达被定为 15Hz 输出。期望看到 `average rate: 15.000` 左右。

---

### 3.4 检查 TF 树 (坐标系拓扑) 断言
传感器发出的所有数据都需要有坐标系支撑才能被后续的 `usv_fusion_new` 转换到 `base_link` 完成融合。

**执行命令：**
```bash
ros2 run tf2_ros tf2_echo base_link camera_front_link
ros2 run tf2_ros tf2_echo base_link mmwave_front_link
```

**正常输出**：应该能持续刷出 Translation（平移）和 Rotation（旋转 四元数）。如果是 `Exception: "base_link" passed to lookupTransform argument target_frame does not exist` 则代表你的 TF 树发送异常（这通常会导致下游融合节点的 TF 监听彻底死锁）。

---

### 3.5 RViz 纯数据层渲染检查
除了通过命令行查看日志，我们最好直接观察模拟器生成的 Marker 映射：

**执行命令：**
```bash
rviz2
```
然后在左侧面板执行以下检查：
1. Global Options -> Fixed Frame 改为 **`base_link`**
2. 添加 `MarkerArray`，Topic 订阅为 `/sim/ground_truth_markers`（这些通常是你设定的真值目标物体，比如白色的半透明圆柱体）。
3. 添加 `MarkerArray`，Topic 订阅为 `/vision/front/markers` 或者 `/mmwave/front/markers`。
4. **验证结果**：你应该能很直观地看到有带颜色的小方块或球在真值目标周边跟随，如果超出该传感器的视野圆锥范围，这些带颜色的块就会自动消失（代表脱落被滤除）。

---

## 4. 故障排查附录
一旦前面的检查中出现了任何一个“没有输出”、“频率不对”、“找不到节点”的情况，请遵循以下流程自检：
1. **找不到相关 Topic**：
   - 检查启动的 Launch 是否为 `ground_truth_sensor_sim three_sensor_sim.launch.py` 或 `ground_truth_sim ground_truth_sim.launch.py`。
2. **Hz 显示无输出 (no new messages)**：
   - 检查节点的 yaml 参数配置：可能 `ground_truth_node` 配置的最近距离太大导致视野内无目标，或更新的 dt 被定为了异常值导致内部陷入死循环。
3. **坐标系异常/TF报错**：
   - 确保 `static_tf_broadcaster` 被正确拉起，它是整个空间框架系赖以存在的脊骨。通常它附着在 `ground_truth` 核心层一起发布。

---

## 5. 融合系统 (Level 1 & Level 2) 链路验证

在确认底层感知数据（如 `/vision/*/detections` 和 `/mmwave/*/targets`）已经健康输出后，我们要进一步验证多传感器融合模块是否正常运转。这包含了 **Level 1 (四大方向局部融合)** 与 **Level 2 (全局 Track-to-Track 汇聚)**。

**前提条件：**
保持第一步中启动仿真节点的终端运行不动，新开**第二个终端**，拉起融合调度框架：
```bash
cd <ws>
source install/setup.bash
ros2 launch usv_fusion_new fusion_system.launch.py
```

### 5.1 验证融合节点群存活状态
新开**第三个终端**（注意每次都要 source 环境），检查各级进程守护情况：
```bash
ros2 node list | grep fusion
```
**正常输出**：你能看到完全解耦部署的树状进程组：
* `/local_fusion_front`
* `/local_fusion_left`
* `/local_fusion_right`
* `/local_fusion_rear`
* `/global_fusion_node`
* `/global_viz_node`

### 5.2 验证 Level 1 (局部感知域) 滤波器输出
四大局部融合节点会同时订阅对应方向的相机与雷达数据，使用 `ApproximateTimeSynchronizer` 进行时间包对齐，并在 `base_link` 局部坐标下做 Extended Kalman Filter 推演。

**执行命令 (以 Front 节点为例)：**
```bash
ros2 topic hz /fusion/local/front_tracks
```
*(若要验证其他盲区，可将 front 替换为 left / right / rear)*

**正常输出**：只要视野内有目标，应该会有稳定的频率回显（因依赖相机数据的更新被触发，通常在 15~25 Hz 左右浮动）。

### 5.3 验证 Level 2 (全局认知去重域) T2T 汇聚输出
全局中心网络 (`global_fusion_node`) 使用非阻塞并发锁（`threading.Lock`），异步接收全方向带来的局部 Track，进行匈牙利代价关联与多视野重叠帧过滤，输出最终的 `Global Tracks`。

**执行心跳验证命令：**
```bash
ros2 topic hz /fusion/global_tracks
```
**正常输出**：持续产生高度稳定的数据流频率（代表主进程多线程推演循环没有卡死）。

**执行数据内容验证命令：**
```bash
ros2 topic echo /fusion/global_tracks --once
```
**正常输出**：应该看到内含的 `tracks` 数组元素。你会看到高度降维且清洗过的数据：唯一的 `x, y, vx, vy`，以及最重要的 — 被打上的唯一身份标签 `track_id`。

### 5.4 在 RViz2 中进行可视化系统终极验收
由于 `fusion_system.launch.py` 会在 Level 4 自动拉起配置好的 RViz2 并载入 `fusion_view.rviz`，你可以直接进行图形化终极验收：

1. **界面检查**：在 RViz 左侧 Displays 面板，应该存在 `Global Fusion Markers` 并高亮打勾（监听的话题为 `/fusion/global_markers`）。
2. **验收标准 1（位置贴合与过滤）**：三维空间中应当出现代表融合结果的 **紫色实心球**（位置）和 **青色箭头**（速度矢量）。这些紫色球必须稳稳地依附在白色半透明的假想真值目标上，几乎没有发散或飞飞出的情况。
3. **验收标准 2（盲区交接不闪变 ID）**：请仔细观察当仿真目标从“前方视野”穿越到“左方视野”的边缘盲区重叠带时，**观察目标上的浮空文本 `ID: XX`。如果数字保持平滑一致，没有突然变成新数值疯狂闪跳，就说明 Level 2 的纯数学 T2T (跨进程 Track-to-Track) 交接分配极其完美，滑动窗口保护生效！**