# 功能包架构

本包提供独立运动学真值和 Gazebo 实体真值两种模式。

```mermaid
flowchart LR
    CFG[参数文件] --> KIN[ground_truth_node]
    GZ[Gazebo 实体位姿] --> ENTITY[ground_truth_gazebo_entity_node]
    KIN --> TRACKS[/sim/ground_truth]
    ENTITY --> TRACKS
    TRACKS --> MARKERS[/sim/ground_truth_markers]
```

- `ground_truth_node`：按 CTRV 或固定航路点生成目标。
- `ground_truth_gazebo_entity_node`：读取 Gazebo 实体位姿并发布真值。
- `gazebo_ground_truth_bridge_node`：旧版 Gazebo 桥接节点，仅用于兼容已有配置。
- `static_tf_broadcaster`：发布静态 TF。

同一时刻只能选择一种真值发布模式。
