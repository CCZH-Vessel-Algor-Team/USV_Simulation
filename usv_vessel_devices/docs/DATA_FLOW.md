# 数据流

```text
robot_N.vessel_devices -> session_manager -> 每船 YAML -> Xacro -> Gazebo 模型
usv_bringup/system.launch -> system_sim.yaml -> simulation/ccs_config.yaml -> 上述配置链
ROS Bool / SetBool -------------> 灯目标 -> PreUpdate -> Material + VisualCmd
ROS Float64 / SetMastPosition ---> 桅杆目标 -> 限速速度目标 -> 力矩伺服 -> 物理关节
Gazebo 关节位置/速度 -----------> PostUpdate -> DeviceState（10 Hz 墙钟）
Gazebo Material / VisualCmd ----> SceneBroadcaster / Sensors -> GUI / 相机
Gazebo JointStatePublisher -----> 原有 ros_gz_bridge -> 模型 TF 发布方
ROS Float64 / SetThrusterTilt --> 起翘目标 -> 限速力矩闭环 -> 起翘物理关节
原推力 ROS topic -> 原桥接 -> 桨转速命令
原推力 + 桨姿态 + 波浪水位 --> 浸没率 -> 有效水推力 -> Gazebo 物理力
重力和有效推力矩 -------------> 起翘闭环前馈
起翘物理反馈 ------------------> ThrusterTiltState（独立 10 Hz 墙钟状态）
推进与起翘诊断 ----------------> thrusters/physics（先左后右）
```

设备接口在 `/<船名>/devices` 下，不使用 `/wamv` 历史别名，保留原推进器公开话题。灯名与开关数组按同一索引对应；起翘名称与其独立状态数组也按索引对应。启用附件时，关节桥接不依赖 `enable_telemetry`，不新增第二套动态 TF。
