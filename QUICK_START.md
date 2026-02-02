# 🚀 USV_Simulation 快速入门指南

## 5分钟快速体验

### 第一步：环境检查

```bash
# 检查ROS 2环境
echo $ROS_DISTRO  # 应显示 "humble"

# 检查工作空间
cd /home/cczh/USV_ROS
ls src/USV_Simulation/  # 确认项目文件存在
```

### 第二步：构建项目

```bash
# 清理并重新构建
rm -rf build install log
colcon build --packages-select usv_sim_full vrx_gz vrx_gazebo wamv_gazebo wamv_description

# 源设置环境
source install/setup.bash
```

### 第三步：启动仿真

```bash
# 终端1：启动主仿真系统
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'
```

### 第四步：控制机器人

```bash
# 终端2：启动键盘控制
cd /home/cczh/USV_ROS
source install/setup.bash
python3 src/usv_sim_full/scripts/dual_thruster_teleop_incre.py
```

**控制说明：**
- `W/S`：左推进器前进/后退
- `A/D`：左推进器左转/右转  
- `↑/↓`：右推进器前进/后退
- `←/→`：右推进器左转/右转
- `空格`：紧急停止

## 🎯 常用操作示例

### 1. 查看传感器数据

```bash
# 查看激光雷达数据
ros2 topic echo /sensors/lidar/front/points

# 查看摄像头图像
ros2 topic echo /sensors/camera/front/image_raw

# 查看IMU数据
ros2 topic echo /sensors/imu/data
```

### 2. 修改物理参数

编辑 `src/usv_sim_full/config/full_config.yaml`：
```yaml
robot:
  overrides:
    mass: 200.0  # 增加质量到200kg
```

### 3. 切换仿真环境

```yaml
simulation:
  world_name: "sydney_regatta"  # 或其他VRX环境
```

## 🛠️ 常见问题快速解决

### Q: 启动时报错找不到包？
```bash
# 重新构建相关包
colcon build --packages-select usv_sim_full vrx_gz vrx_gazebo
source install/setup.bash
```

### Q: Gazebo窗口不显示？
```bash
# 检查是否有图形界面支持
echo $DISPLAY  # 应该有值
```

### Q: 传感器数据为空？
```bash
# 检查话题是否存在
ros2 topic list | grep sensors
```

## 📚 进阶学习路径

1. **基础操作** → 熟悉键盘控制和基本传感器使用
2. **配置定制** → 修改YAML配置文件调整机器人参数  
3. **控制算法** → 开发自定义控制器替代键盘控制
4. **系统集成** → 集成导航、路径规划等功能

---
*需要更多帮助？请查看完整的 README.md 文档*