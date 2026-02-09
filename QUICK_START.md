# 🚀 USV_Simulation 快速入门指南

## ⏱️ 5分钟快速体验

### 第1步：环境检查 (1分钟)
```bash
# 确认ROS 2环境
echo $ROS_DISTRO  # 应显示 "humble"

# 进入工作空间
cd ./
ls src/USV_Simulation/  # 确认项目文件存在
```

### 第2步：构建项目 (2分钟)
```bash
# 清理并构建
rm -rf build install log
colcon build 

# 源设置环境
source install/setup.bash
```

### 第3步：启动仿真 (1分钟)
```bash
# 终端1：启动主系统
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'
```

### 第4步：控制机器人 (1分钟)
```bash
# 终端2：启动键盘控制
cd ./
source install/setup.bash
python3 src/usv_sim_full/scripts/dual_thruster_teleop_incre.py
```

## 🎮 控制说明

**键盘控制**：

## 🔍 验证系统运行

### 检查核心组件
```bash
# 查看运行节点
ros2 node list

# 查看传感器话题
ros2 topic list | grep sensors

# 实时查看传感器数据
ros2 topic echo /sensors/imu/data
```

## 📚 进阶学习路径

| 阶段 | 目标 | 推荐文档 |
|------|------|----------|
| 🔰 基础使用 | 熟悉基本操作 | 当前文档 |
| ⚙️ 配置定制 | 修改机器人参数 | [主项目README](README.md#配置说明) |
| 🏗️ 系统理解 | 掌握架构原理 | [技术架构详解](src/usv_sim_full/README.md) |
| 🛠️ 开发扩展 | 自定义功能开发 | [开发指南](src/usv_sim_full/README.md#开发指南) |

## 🐛 常见问题快速解决

### 系统启动问题
```bash
# 问题：包找不到
colcon build --packages-select usv_sim_full vrx_gz vrx_gazebo
source install/setup.bash

# 问题：Gazebo窗口不显示
echo $DISPLAY  # 确认有图形界面支持
```

### 控制响应问题
```bash
# 问题：控制无响应
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 100.0"  # 测试话题
```

### 传感器数据问题
```bash
# 问题：传感器数据为空
ros2 topic list | grep sensors  # 检查话题是否存在
ros2 node list                  # 检查节点状态
```

## 💡 实用技巧

### 快速重启
```bash
# 一键清理并重启
pkill -f "ros2 launch" && pkill -f "gz sim" && pkill -f "rviz2"
# 然后重新运行启动命令
```

### 多配置切换
```bash
# 使用不同配置文件
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/minimal_config.yaml'
```

*需要深入了解？请查看[完整文档](README.md)*