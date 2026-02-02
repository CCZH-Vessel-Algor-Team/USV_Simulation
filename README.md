# USV_Simulation - 无人水面航行器仿真平台

[![ROS 2](https://img.shields.io/badge/ROS-2_Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-orange.svg)](https://gazebosim.org/)
[![VRX](https://img.shields.io/badge/VRX-Competition-green.svg)](https://github.com/osrf/vrx)
[![License](https://img.shields.io/badge/license-Apache_2.0-blue.svg)](LICENSE)

基于 **ROS 2 + Gazebo Garden + VRX** 的无人水面航行器（USV）高保真度仿真平台，专为WAM-V等水面无人船的研发、测试和竞赛训练而设计。

## 🎯 项目特色

- **一体化仿真解决方案**：集成了物理仿真、传感器模拟、控制系统和可视化监控
- **高度可配置**：通过YAML文件灵活定义机器人参数、传感器配置和环境设置
- **模块化架构**：采用组件化设计，便于扩展新功能和自定义配置
- **VRX竞赛兼容**：支持VRX标准竞赛环境和任务场景
- **多控制方式**：支持键盘和手柄双重控制模式

## 🚀 核心功能

### 🌊 物理仿真
- 基于Gazebo Garden的高精度物理引擎
- 真实的水动力学模拟（浮力、阻力、波浪影响）
- 可配置的质量、惯性矩和物理参数
- 支持多种船体构型（WAM-V标准配置）

### 🎮 控制系统
- **双推进器控制**：独立控制左右推进器
- **键盘控制**：WASD控制左侧推进器，方向键控制右侧推进器
- **手柄控制**：支持游戏手柄实时操控
- **急停功能**：空格键一键停止所有动作

### 📡 传感器套件
- **激光雷达**：360°环境感知，点云数据输出
- **摄像头**：RGB图像采集，支持相机内参配置
- **IMU**：惯性测量单元，提供姿态和加速度数据
- **GPS**：全球定位系统，提供经纬度坐标
- **里程计**：实时位姿和速度信息

### 🎨 可视化监控
- **RViz集成**：3D可视化界面显示机器人状态
- **传感器数据显示**：实时查看各传感器数据流
- **TF变换树**：坐标系关系可视化
- **轨迹跟踪**：运动路径记录和显示

### 🌍 环境系统
- **VRX标准场景**：支持sydney_regatta等竞赛环境
- **程序化障碍物生成**：支持随机和固定布局
- **动态天气模拟**：波浪、风力等环境因素
- **多世界切换**：轻松切换不同的仿真环境

## 🏗️ 系统架构

```
USV_Simulation/
├── src/
│   └── usv_sim_full/           # 主控功能包
│       ├── launch/             # 启动文件
│       │   ├── main.launch.py          # 主启动协调器
│       │   └── components/             # 组件启动文件
│       │       ├── infra_sim.launch.py # 基础设施仿真
│       │       ├── robot_bringup.launch.py # 机器人系统
│       │       └── visualization.launch.py # 可视化界面
│       ├── config/             # 配置文件
│       │   └── full_config.yaml        # 主配置文件
│       ├── scripts/            # 核心脚本
│       │   ├── session_manager.py      # 会话管理器
│       │   └── dual_thruster_teleop_incre.py # 双推进器控制
│       └── templates/          # URDF模板
│           └── wamv_no_battery.urdf.xacro  # 无电池WAM-V模板
├── install/                    # 构建输出目录
└── logs/                      # 仿真会话日志
```

## 📦 技术栈

| 组件 | 版本 | 用途 |
|------|------|------|
| **ROS 2** | Humble Hawksbill | 机器人操作系统框架 |
| **Gazebo** | Garden (Harmonic) | 物理仿真引擎 |
| **VRX** | 最新版 | 海上机器人竞赛框架 |
| **Python** | 3.10+ | 脚本开发语言 |
| **URDF/Xacro** | - | 机器人建模语言 |

## 🛠️ 快速开始

### 环境准备

```bash
# 1. 安装ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 2. 安装Gazebo和相关依赖
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev
sudo apt install ros-humble-rviz-visual-tools

# 3. 安装构建工具
sudo apt install python3-colcon-common-extensions python3-rosdep

# 4. 配置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 构建项目

```bash
# 进入工作空间
cd /home/cczh/USV_ROS

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建项目
colcon build --packages-select \
  usv_sim_full \
  vrx_gz \
  vrx_gazebo \
  wamv_description \
  wamv_gazebo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 源设置环境
source install/setup.bash
```

### 运行仿真

```bash
# 启动主仿真系统
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'

# 另开终端启动键盘控制
cd /home/cczh/USV_ROS
source install/setup.bash
python3 src/usv_sim_full/scripts/dual_thruster_teleop_incre.py
```

## ⚙️ 配置说明

### 主配置文件 (full_config.yaml)

```yaml
robot:
  # 物理参数配置
  overrides:
    mass: 180.0           # 质量(kg)
    inertia: [100.0, 100.0, 200.0]  # 惯性矩阵[kg·m²]
    visual_mesh: "custom_ship.stl"  # 自定义船体外观
  
  # 推进器配置
  thruster_config: "H"    # H(标准后推) / T(X型) / X(十字型)
  
  # 传感器启用状态
  sensors:
    lidars:
      - name: "front_lidar"
        enabled: true
        xyz: [1.0, 0.0, 1.5]  # 安装位置[x,y,z]

simulation:
  world_name: "sydney_regatta"  # 仿真世界
  obstacles:
    fixed:                    # 固定障碍物
      - type: "buoy_start"
        position: [10.0, 5.0, 0.0]
```

### 关键配置项

| 参数 | 类型 | 描述 |
|------|------|------|
| `mass` | float | 机器人总质量(kg) |
| `inertia` | array[3] | 绕XYZ轴的转动惯量 |
| `thruster_config` | string | 推进器布局配置 |
| `world_name` | string | Gazebo世界名称 |
| `sensors.*.enabled` | bool | 传感器启用开关 |

## 🎯 使用示例

### 1. 基础仿真测试

```bash
# 启动最小配置仿真
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/minimal_config.yaml'
```

### 2. 自定义传感器配置

```yaml
# 在配置文件中添加自定义传感器
robot:
  sensors:
    cameras:
      - name: "custom_camera"
        enabled: true
        xyz: [0.5, 0.0, 2.0]
        rpy: [0.0, 0.2, 0.0]  # 俯仰角20度
        topic: "/custom/camera/image_raw"
```

### 3. 物理参数调试

```yaml
# 调整船体物理特性进行测试
robot:
  overrides:
    mass: 200.0              # 增加质量测试稳定性
    xU: 150.0               # 调整X轴线性阻尼
    yV: 120.0               # 调整Y轴线性阻尼
```

## 📊 数据接口

### 发布的话题 (Published Topics)

| 话题 | 类型 | 描述 |
|------|------|------|
| `/sensors/lidar/front/points` | sensor_msgs/PointCloud2 | 激光雷达点云数据 |
| `/sensors/camera/front/image_raw` | sensor_msgs/Image | 前置摄像头图像 |
| `/sensors/imu/data` | sensor_msgs/Imu | IMU传感器数据 |
| `/sensors/gps/data` | sensor_msgs/NavSatFix | GPS定位数据 |
| `/model/wamv/odometry` | nav_msgs/Odometry | 机器人里程计 |
| `/wamv/thrusters/*/thrust` | std_msgs/Float64 | 推进器推力指令 |

### 订阅的话题 (Subscribed Topics)

| 话题 | 类型 | 描述 |
|------|------|------|
| `/wamv/thrusters/*/thrust` | std_msgs/Float64 | 推进器推力控制 |
| `/wamv/thrusters/*/pos` | std_msgs/Float64 | 推进器角度控制 |

## 🔧 开发指南

### 添加新传感器

1. **修改配置文件**
```yaml
robot:
  sensors:
    custom_sensors:
      - name: "new_sensor"
        enabled: true
        # 添加传感器特有参数
```

2. **更新会话管理器**
```python
# 在session_manager.py中添加传感器处理逻辑
def generate_custom_sensor(sensor_config):
    # 生成传感器Xacro宏定义
    pass
```

### 扩展控制算法

```python
# 自定义控制器示例
class CustomController:
    def __init__(self):
        self.publisher = rospy.Publisher('/wamv/thrusters/left/thrust', Float64, queue_size=10)
    
    def control_callback(self, sensor_data):
        # 实现自定义控制逻辑
        thrust_command = self.calculate_thrust(sensor_data)
        self.publisher.publish(thrust_command)
```

## 🐛 常见问题

### Q: Gazebo无法找到模型文件？
**A:** 确保正确设置了Gazebo资源路径：
```bash
export GZ_SIM_RESOURCE_PATH="/path/to/wamv_description/models:$GZ_SIM_RESOURCE_PATH"
```

### Q: 传感器数据不更新？
**A:** 检查以下几点：
1. 确认传感器在配置文件中已启用
2. 查看对应话题是否存在：`ros2 topic list`
3. 检查传感器桥接是否正常：`ros2 node list`

### Q: 控制响应延迟？
**A:** 优化建议：
1. 降低Gazebo仿真步长
2. 调整控制器发布频率
3. 检查系统资源使用情况

## 🤝 贡献指南

欢迎提交Issue和Pull Request来改进项目！

### 开发流程
1. Fork项目仓库
2. 创建功能分支：`git checkout -b feature/new-feature`
3. 提交更改：`git commit -am 'Add new feature'`
4. 推送到分支：`git push origin feature/new-feature`
5. 创建Pull Request

## 📄 许可证

本项目采用Apache 2.0许可证，详情请参见[LICENSE](LICENSE)文件。

## 🙏 致谢

- [VRX项目](https://github.com/osrf/vrx) - 提供海上机器人竞赛框架
- [Gazebo社区](https://gazebosim.org/) - 强大的物理仿真引擎
- [ROS 2团队](https://www.ros.org/) - 优秀的机器人操作系统

---
*Made with ❤️ for autonomous marine robotics*