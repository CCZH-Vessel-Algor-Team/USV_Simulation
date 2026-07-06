
# 快速开始（5 分钟体验）

按下列步骤在一个干净的工作区中快速体验仿真：

1. 准备环境
```bash
# 进入工作区
cd <your_ros2_workspace>
# 确认仓库位于 src/ 下
ls src/USV_Simulation
```

2. 构建并 source
```bash
rm -rf build install log
colcon build --packages-select usv_sim_full
source install/setup.bash
```

3. 启动仿真
```bash
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'
```

4. 打开 RViz（可选）
```bash
rviz2 -d "$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full/rviz/default.rviz"
```

5. 控制机器人
```bash
python3 src/usv_sim_full/scripts/dual_thruster_teleop_incre.py
```

验证点：
- 使用 `ros2 node list` 确认 bridge 节点运行
- 使用 `ros2 topic list | grep thrusters` 查看推进器话题
- 使用 `gz topic -l` 确认 Gazebo 话题存在

如果遇到问题，请参阅 `TROUBLESHOOTING.md`。
````
