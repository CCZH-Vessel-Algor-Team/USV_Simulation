# USV仿真系统 - 无人水面航行器仿真平台

这是一个基于ROS 2 + Gazebo Garden + VRX的无人船（USV）仿真环境启动系统，支持通过YAML配置驱动的灵活仿真流程。它为水面无人船（如WAM-V）提供完整的建模、控制、传感器配置与可视化能力。

## 🚀 项目特点

- **模块化架构**：采用组件化设计，易于扩展和维护
- **配置驱动**：通过YAML配置文件灵活定制仿真环境
- **完整传感器套件**：支持激光雷达、摄像头、IMU、GPS等多种传感器
- **实时控制**：支持键盘和手柄控制
- **可视化界面**：集成RViz可视化工具

## 📋 系统要求

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo Garden (或更高版本)
- Python 3.10+
- CMake 3.8+

## 🛠️ 环境依赖安装

### ROS 2 Humble Hawksbill
```bash
# 添加仓库
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# 添加ROS仓库密钥
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# 添加ROS仓库
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-dev
sudo apt install ros-humble-rviz-visual-tools
```

### 设置ROS环境
```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 其他依赖
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo apt install ruby-dev  # 用于Gazebo Ruby脚本
```

## 🚀 快速开始

### 1. 编译项目
```bash
cd ~/simulation/vrx_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译相关包
colcon build --packages-select \
  vrx_gazebo \
  vrx_ros \
  vrx_gz \
  wamv_description \
  wamv_gazebo \
  usv_sim_full \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 源环境
source install/setup.bash
```

### 2. 启动仿真

#### 启动主仿真系统
```bash
# 启动完整仿真系统
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'

# 或者启动船体测试环境（用于验证船体参数）
ros2 launch usv_sim_full test_hull.launch.py
```

#### 启动VRX竞赛仿真
```bash
# 启动VRX世界
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# 启动配套的RViz界面
ros2 launch vrx_gazebo rviz.launch.py
```

### 3. 控制方式

#### 键盘控制
在另一个终端中运行：
```bash
cd ~/simulation/vrx_ws
source install/setup.bash
python3 dual_thruster_teleop_incre.py
```

使用键盘控制船只：
- 左侧推进器 (WASD):
  - W: 左侧推力 +
  - S: 左侧推力 -
  - A: 左侧角度 +
  - D: 左侧角度 -
- 右侧推进器 (方向键):
  - ↑: 右侧推力 +
  - ↓: 右侧推力 -
  - ←: 右侧角度 +
  - →: 右侧角度 -
- 空格键: 急停（所有状态归零）
- Q键: 仅重置角度
- E键: 仅重置推力

#### 手柄控制
```bash
# 启动手柄控制
ros2 launch vrx_gz usv_joy_teleop.py
```

## 🔧 配置说明

主要配置文件位于 `src/usv_sim_full/config/full_config.yaml`，包括：

- **机器人配置**: 定义船体名称、物理参数、传感器等
- **环境配置**: 指定仿真世界和Gazebo启动参数
- **传感器配置**: 定义激光雷达、摄像头、IMU、GPS等传感器参数
- **可视化配置**: 控制RViz是否启动及遥测数据

## 📁 项目结构

```
simulation/
├── dual_thruster_teleop.py          # 双推进器手柄控制脚本
├── dual_thruster_teleop_incre.py    # 双推进器增量式键盘控制脚本
├── src/                             # 源代码目录
│   ├── generate.py                  # 生成脚本
│   ├── generate_new.py              # 新版生成脚本
│   ├── usv_sim_full/                # 完整版无人船仿真系统
│   │   ├── config/                  # 配置文件
│   │   ├── launch/                  # 启动文件
│   │   ├── scripts/                 # 脚本文件
│   │   └── ...
│   ├── vrx/                         # VRX相关包
│   └── 使用指南.md                   # 详细使用指南
```

## 🛠️ 自定义开发

### 添加新传感器
1. 修改`templates/sensor_macros.xacro`添加传感器宏定义
2. 更新`session_manager.py`中的传感器处理逻辑
3. 在配置文件中添加新传感器的配置
4. 重新构建项目

### 添加新世界
1. 将`.sdf`世界文件放在`test_env`目录
2. 更新`config/full_config.yaml`中的`world_name`
3. 重新启动仿真

## 🔍 故障排除

### 编译错误
如果遇到`Could not find a package configuration file provided by "gz-sim7"`错误，安装Gazebo Garden:
```bash
curl -fsSL https://gazebosim.jfrog.io/gazebosim/gazebo_signing.key | sudo gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://gazebosim.jfrog.io/gazebosim/debian-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null
sudo apt update
sudo apt install gz-harmonic
```

### 传感器数据问题
检查传感器话题是否有数据:
```bash
# 查看所有话题
ros2 topic list

# 检查特定话题是否有数据
ros2 topic echo /sensors/imu/data
```

## 🤝 贡献

欢迎提交Issue和Pull Request来帮助改进这个项目！

## 📄 许可证

该项目使用MIT许可证 - 查看[LICENSE](LICENSE)文件了解详情。