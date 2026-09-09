# 数据流输入输出

| 输入 | 输出 | 说明 |
| --- | --- | --- |
| Gazebo 经 `ros_gz_bridge` 转换的仿真话题。 | VRX ROS 接口。 | 供 VRX 上层组件使用。 |
| Gazebo 仿真进程。 | 进程状态。 | 由 `monitor_sim.py` 检查。 |

具体话题取决于启动的 VRX 场景和桥接配置。
