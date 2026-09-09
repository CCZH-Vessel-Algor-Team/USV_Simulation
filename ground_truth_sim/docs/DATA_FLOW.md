# 数据流输入输出

| 模式 | 输入 | 输出 | 说明 |
| --- | --- | --- | --- |
| 独立运动学 | `ground_truth_params.yaml` 中的 CTRV 或航路点参数。 | `/sim/ground_truth`，`usv_interfaces/msg/GlobalTrackArray`。 | 生成目标状态。 |
| Gazebo 实体 | Gazebo 实体位姿和可选碰撞信息。 | `/sim/ground_truth`，`/sim/ground_truth_markers`。 | 发布实体真值和可视化标记。 |
| 静态 TF | 父、子坐标系及位姿参数。 | `/tf_static`。 | 提供目标和传感器坐标系。 |

`ground_truth_sensor_sim` 为工作区外部包，可订阅 `/sim/ground_truth` 生成视觉和毫米波模拟数据。
