# enc_grounding_warning_rviz

两个独立面板：`GroundingWarningPanel` 只显示 UKC、风险、告警和水深统计；
`DepthSettingsPanel` 设置打点区域的半径和水深，并列出当前用户添加的区域。

## 构建

```bash
colcon build --packages-select enc_grounding_warning_msgs enc_grounding_warning enc_grounding_warning_rviz --symlink-install
source install/setup.bash
```

## 使用

默认 `three_vision_one_mmwave.rviz` 已加载两个面板和“添加水深点”工具。
使用自己的 RViz 布局时，通过 Panels → Add New Panel 添加 `DepthSettingsPanel`，
工具栏通过添加工具选择 `enc_grounding_warning_rviz/DepthPointTool`。

1. 在“水深设置”面板填写区域半径和水深，单位“米”显示在输入框右侧。
2. 选择工具栏“添加水深点”（快捷键 W），在地图上单击，区域立即应用并自动保存。
3. 每次打点后自动退出到默认地图工具，无需同步或确认；添加下一个区域时再次选择工具或按 W。右键或 Esc 可取消取点。
4. 在列表中选中区域，点击“删除选中区域”立即删除并恢复原来的水深；可多选或 Ctrl+A 批量删除。
5. 点击“删除全部区域”可一键清空当前船舶的用户区域，无需选中或再次确认；原始水深恢复，全部区域圆环同时移除。

列表只显示已经成功应用的用户区域，初始为空，不会列出原始预设浅滩。
半径和水深用于下一次打点；修改已有区域时删除后重新打点。
每个点保留接收时的半径和水深。保存失败时不会将失败区域显示为已应用，
删除失败时原区域保留在列表中。

## 设置区域显示

已应用的用户区域在地图上显示深红色圆环（线宽 0.3 米），圆环半径与设置一致，只显示边界，不填充、不添加立体标记。
原始预设浅滩和其他水域不会在这个图层中额外绘制；删除区域后相应圆环消失。
图层由安全节点发布，关闭设置面板后继续显示，重启后自动恢复。

默认 RViz 配置已启用“自定义水深区域”图层。使用自定义布局时，添加 `MarkerArray`，
话题设为 `/usv_1/safety/depth_region_markers`，Reliability 设为 Reliable，
Durability 设为 Transient Local，可立即显示当前保存的区域。其他船舶命名空间需调整话题前缀。

## 原始水深与删除恢复

用户区域叠加在原始 YAML/CSV 水深之上，区域外的地形起伏、预设浅滩和未知数据完整保留。
用户区域内使用设置的水深，既可比原来浅，也可比原来深；多个用户区域重叠时取较浅的设置值。
删除区域后，重叠位置保留其他有效用户区域；没有其他用户区域覆盖的位置恢复原始水深。
删除全部用户区域后，整张水深矩阵与原始配置一致，不需要单独点击恢复按钮。

地图 Fixed Frame 必须是 `map`。半径大于 0，水深范围 0–11000 米，最多 128 个区域，
仅在原始水深栅格覆盖范围内生效。区域直径为半径的两倍。

## 保存与旧版兼容

设置由安全节点保存在 `$ROS_HOME/enc_grounding_warning/usv_1/depth_settings.yaml`，
`ROS_HOME` 默认 `~/.ros`。不同船舶命名空间分别保存；重启后自动恢复用户区域和删除结果。
旧版 v1 保存文件仍可读取，旧界面自动带入的原始预设区域会从用户列表中排除；新编辑保存为 v2。
原始数据文件始终保留，不依赖保存 RViz 布局。

首次加载本次更新需要重启 RViz 和安全节点；之后打点与删除均无需重启。

## 文档

- [功能包架构](docs/ARCHITECTURE.md)
- [数据流输入输出](docs/DATA_FLOW.md)
