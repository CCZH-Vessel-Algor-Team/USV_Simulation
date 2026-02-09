# USV Sim Full - 完整版无人船仿真系统

基于 ROS 2 + Gazebo Garden + VRX 的完整版无人船仿真系统，支持深度船体定制和程序化场景生成。

## 🚀 系统特性

### ✅ 模块化架构
- **Robot Bringup**: 封装机器人相关节点（状态发布、实体生成、传感器桥接）
- **Infra Sim**: 仅负责环境基础设施（Gazebo仿真、全局桥接）
- **Visualization**: 专门负责RViz可视化
- **Main Coordinator**: 组装各组件的协调器

### ✅ 深度船体定制
- 通过YAML配置船体Mesh路径、质量和惯性矩阵
- 支持动态URDF生成，无需重新编译
- 可定制外观和物理属性

### ✅ 程序化场景生成
- 支持"固定列表"和"随机区域生成"两种障碍物模式
- 自动生成圆柱或方体障碍物并加载到Gazebo
- 可配置数量、大小、颜色等参数

### ✅ 船体测试环境
- 提供简化水面仿真环境用于船体参数验证
- 可快速测试外观、碰撞体积、重力和浮力效果
- 无需启动完整复杂仿真

### ✅ 高度可扩展
- 为路径规划（Nav2）和定位（EKF）预留扩展点
- 支持多机协同仿真
- 传感器配置完全可定制

## 🏗️ 系统架构

``` mermaid
graph TD
    A[YAML配置文件] --> B[Session Manager]
    B --> C[动态URDF生成]
    B --> D[传感器桥接配置]
    B --> E[RViz配置]
    B --> F[障碍物布局JSON]
    
    G[Main Launch] --> H[Infra Sim]
    G --> I[Robot Bringup]
    G --> J[Visualization]
    
    C --> I
    D --> I
    F --> I
    E --> J
    
    H --> K[Gazebo Sim]
    I --> L[Robot State Publisher]
    I --> M[Create Entity]
    I --> N[Sensor Bridge]
    I --> O[Obstacle Spawner]
    J --> P[RViz2]
    
    K --> Q[Gazebo World]
    L --> R[TF Tree]
    M --> S[WAM-V Model]
    N --> T[Sensor Topics]
    O --> U[Dynamic Obstacles]
    P --> V[Visualization]
    
    R --> V
    S --> Q
    T --> V
    U --> Q
```

## 安装和构建

```
# 进入仓库根目录（或你的 ROS2 工作空间中包含本项目的目录）
cd ./

# 构建功能包
colcon build --packages-select usv_sim_full

# 源环境
source install/setup.bash
```

## ⚙️ 配置详解

### 机器人配置 (`config/full_config.yaml`)

```yaml
robot:
  name: "wamv_custom"
  # 基础模板
  xacro_template: "wamv_gazebo.urdf.xacro"
  spawn_pose: [0, 0, 0.5, 0, 0, 0]
  
  # 物理与外观覆盖
  overrides:
    visual_mesh: "package://my_custom_package/meshes/new_hull.dae"
    mass: 180.0
    inertia: [100.0, 100.0, 200.0]

obstacles:
  # 随机生成区域
  random_areas:
    - name: "zone_A"
      type: "cylinder"
      count: 10
      center: [15.0, 5.0]
      radius: 10.0
      size_range: [0.3, 0.8]
      color: "Red"
      
  # 固定障碍物列表
  fixed_list:
    - name: "buoy_start"
      type: "cylinder"
      pose: [5.0, 0.0, 0.0]
      size: [0.5, 1.0]
      color: "Green"
```

### 支持的传感器类型

- `vlp16`: VLP-16激光雷达（点云传感器）
- `camera`: RGB摄像头（图像传感器）

## 🚀 快速开始

### 1. 构建项目

```bash
cd ./
colcon build --packages-select usv_sim_full
source install/setup.bash
```

### 2. 启动仿真

```bash
# 启动完整仿真系统
ros2 launch usv_sim_full main.launch.py

# 启动船体测试环境（用于验证船体参数）
ros2 launch usv_sim_full test_hull.launch.py
```

### 3. 自定义配置

修改 `config/full_config.yaml`（位于 `src/usv_sim_full/config/full_config.yaml`）文件来自定义船体和场景：

1. 修改 `robot.overrides` 来定制船体物理/外观
2. 调整 `obstacles` 来配置场景中的障碍物
3. 重新启动仿真以应用更改

## 🧪 船体测试环境

我们提供了一个专门的船体测试环境，用于验证船体参数：

```bash
# 启动船体测试环境
ros2 launch usv_sim_full test_hull.launch.py
```

此环境包含：
- 简化的水面环境（`test_env/simple_water.sdf`）
- 仅加载一个船体模型
- 包含RViz可视化
- 可快速测试船体外观、碰撞体积、重力和浮力交互

非常适合：
- 验证船体外观修改
- 测试物理参数（质量和惯性）
- 验证浮力和重力平衡
- 快速迭代船体设计

### 仿真组件

启动后将运行以下组件：

1. **Gazebo仿真环境**：加载指定的世界地图
2. **Robot State Publisher**：发布机器人TF变换
3. **Gazebo实体创建**：将生成的机器人模型添加到仿真中
4. **ROS-GZ桥接**：传输传感器数据和控制指令
5. **传感器桥接**：为每个配置的传感器创建数据传输通道
6. **遥测桥接**：传输里程计、关节状态和位姿数据
7. **RViz可视化**：动态配置的可视化界面

### 查看传感器数据

启动仿真后，可以通过以下命令查看传感器数据：

```bash
# 查看激光雷达点云数据
ros2 topic echo /sensors/lidar/front/points sensor_msgs/msg/PointCloud2

# 查看摄像头图像数据
ros2 topic echo /sensors/camera/front/image_raw sensor_msgs/msg/Image

# 查看里程计数据
ros2 topic echo /model/wamv/odometry nav_msgs/msg/Odometry
```

## Launch架构详解

### 1. `robot_bringup.launch.py` - 机器人容器
封装了所有与特定机器人相关的节点：
- Robot State Publisher：加载URDF模型
- Spawn Entity：使用指定姿态生成机器人模型
- Sensor Bridge：加载传感器桥接配置
- **预留扩展点**：为Nav2路径规划和EKF定位预留接口

### 2. `infra_sim.launch.py` - 基础设施仿真
仅负责环境基础设施：
- 启动Gazebo仿真环境
- 启动全局桥接（仅包含/clock等系统级话题）

### 3. `visualization.launch.py` - 可视化组件
- 负责启动RViz2
- 接收RViz配置文件路径参数

### 4. `main.launch.py` - 协调器
组装上述模块：
- 运行会话管理器获取配置路径
- 包含基础设施仿真组件
- 包含机器人启动组件（传入URDF和桥接配置路径）
- 包含可视化组件（传入RViz配置路径）

## 📁 目录结构

```
usv_sim_full/
├── config/                 # 配置文件
│   └── full_config.yaml    # 主配置文件
├── launch/                 # 启动文件
│   ├── main.launch.py      # 主启动文件
│   ├── test_hull.launch.py # 船体测试启动文件
│   └── components/         # 组件启动文件
│       ├── infra_sim.launch.py
│       ├── robot_bringup.launch.py
│       └── visualization.launch.py
├── scripts/                # 脚本文件
│   ├── session_manager.py  # 会话管理器
│   └── obstacle_spawner.py # 障碍物生成器
├── templates/              # 模板文件
│   ├── hull_macro.xacro    # 船体宏定义
│   └── sensor_macros.xacro # 传感器宏定义
├── test_env/               # 测试环境文件
│   └── simple_water.sdf    # 简化水面环境
└── logs/                   # 会话日志（运行时生成）
```

## 工作原理

1. **Session Manager**：读取YAML配置，生成临时文件
2. **Xacro Overlay**：创建覆盖层Xacro，将传感器添加到基础模型
3. **URDF编译**：将Xacro编译为最终的URDF模型
4. **桥接配置**：根据传感器配置生成ROS-GZ桥接映射
5. **RViz配置**：根据传感器配置动态生成RViz显示项
6. **模块化启动**：各组件独立启动，实现配置与启动逻辑的分离

## 自定义扩展

### 添加新传感器类型

1. 在 `templates/sensor_macros.xacro` 中添加新的传感器宏定义
2. 在 `session_manager.py` 中添加对该传感器类型的处理逻辑
3. 在配置文件中使用新类型

### 修改仿真环境

更改 `config/full_config.yaml` 中的 `world_name` 字段，指定不同的仿真环境。

### 扩展机器人功能

利用 `robot_bringup.launch.py` 中预留的扩展点，可以轻松添加：
- Nav2路径规划节点
- EKF定位节点
- 自定义控制节点

## 故障排除

- **传感器未显示**：检查配置文件中传感器的父链接名称是否正确（应为`wamv/base_link`）
- **桥接错误**：检查生成的桥接配置文件格式是否正确
- **模型加载失败**：确保WAM-V相关包已正确安装和编译
- **WAM-V模型未出现在仿真中**：确保使用了最新版本的代码，我们已修复了模型加载问题，现在使用`ros_gz_sim`的`create`节点正确加载模型
- **创建实体失败**：查看create节点的日志输出，确保URDF文件路径有效且格式正确
- **RViz中未显示传感器数据**：确认传感器话题在RViz中已正确配置，且桥接正常工作

## 🤝 贡献

欢迎提交 Issue 和 Pull Request 来帮助改进这个项目！

## 📄 许可证

该项目使用 MIT 许可证 - 查看 `LICENSE`（位于 `src/usv_sim_full/LICENSE`）文件了解详情。

# usv_sim_full - 无人水面航行器主控功能包

## 📦 包概述

`usv_sim_full`是USV仿真平台的核心协调包，负责整个仿真系统的启动、配置管理和运行时控制。该包实现了基于YAML配置的动态系统配置功能。

## 🏗️ 包结构详解

```
usv_sim_full/
├── launch/                     # 系统启动文件
│   ├── main.launch.py          # 主启动协调器（入口点）
│   └── components/             # 子系统组件
│       ├── infra_sim.launch.py     # 基础设施仿真组件
│       ├── robot_bringup.launch.py # 机器人系统组件  
│       └── visualization.launch.py # 可视化组件
├── config/                     # 配置文件
│   └── full_config.yaml        # 完整配置示例
├── scripts/                    # 核心执行脚本
│   ├── session_manager.py      # 会话管理器（核心引擎）
│   ├── dual_thruster_teleop_incre.py # 双推进器遥控器
│   └── load_robot_description.py   # 机器人描述加载器
├── templates/                  # URDF模板文件
│   └── wamv_no_battery.urdf.xacro  # 无电池WAM-V模板
├── logs/                       # 运行时日志（自动生成）
│   └── session_*               # 会话日志目录
└── package.xml                 # ROS 2包描述文件
```

## 🎯 核心组件功能

### 1. 主启动协调器 (main.launch.py)

**职责**：作为系统的单一入口点，负责：
- 解析用户提供的YAML配置文件
- 调用会话管理器生成运行时配置
- 按依赖顺序启动各个子组件
- 管理整个系统的生命周期

**使用方法**：
```bash
ros2 launch usv_sim_full main.launch.py config_path:='<path_to_config.yaml>'
```

### 2. 会话管理器 (session_manager.py) ⭐

**职责**：动态配置管理系统的核心引擎，负责：
- 解析和验证YAML配置文件
- 生成传感器Xacro叠加层定义
- 编译最终的URDF机器人描述
- 创建Gazebo-Ros桥接配置
- 生成RViz可视化配置文件
- 管理会话的创建、维护和清理

**核心方法**：
```
def create_session(config_path): 
    """创建新的仿真会话，返回会话信息字典"""
    pass

def generate_sensors_overlay(config_data, session_dir): 
    """根据配置生成传感器Xacro定义"""
    pass

def compile_xacro_to_urdf(xacro_input, config_data, session_dir): 
    """编译Xacro模板为最终URDF"""
    pass

def generate_bridge_config(config_data): 
    """生成传感器数据桥接配置"""
    pass

def generate_rviz_config(config_data, session_dir): 
    """生成RViz可视化配置"""
    pass
```

### 3. 子系统组件启动器

#### 基础设施仿真组件 (infra_sim.launch.py)
- 启动Gazebo仿真环境和服务
- 配置Gazebo资源搜索路径
- 建立全局时钟同步桥接
- 加载指定的世界场景

#### 机器人系统组件 (robot_bringup.launch.py)  
- 发布机器人状态描述信息
- 在Gazebo中创建机器人实体
- 建立传感器数据转发桥接
- 启动障碍物生成服务

#### 可视化组件 (visualization.launch.py)
- 启动RViz可视化界面
- 加载动态生成的配置文件
- 显示机器人模型和传感器数据

## ⚙️ 配置系统详解

### 配置文件结构规范

```
# 配置文件必须包含的基本结构
robot:
  # 必需字段
  xacro_template: string        # URDF模板文件名
  thruster_config: string       # 推进器布局配置(H/T/X)
  
  # 可选物理参数覆盖
  overrides:
    mass: float                 # 质量(kg)
    inertia: [ixx, iyy, izz]    # 惯性矩阵
    visual_mesh: string         # 自定义外观网格
  
  # 传感器配置（支持多种类型）
  sensors:
    lidars: []                  # 激光雷达数组
    cameras: []                 # 摄像头数组  
    imus: []                    # IMU传感器数组
    gps_sensors: []             # GPS传感器数组

simulation:
  # 仿真环境配置
  world_name: string            # Gazebo世界名称
  obstacles:                    # 障碍物配置
    fixed: []                   # 固定障碍物
    random: {}                  # 随机障碍物参数
```

### 配置项详细说明

| 配置项 | 类型 | 必需 | 默认值 | 描述 |
|--------|------|------|--------|------|
| `robot.xacro_template` | string | 是 | - | URDF模板文件名 |
| `robot.thruster_config` | string | 是 | "H" | 推进器布局(H/T/X) |
| `robot.overrides.mass` | float | 否 | 模板默认值 | 机器人总质量(kg) |
| `robot.sensors.*.enabled` | bool | 是 | false | 传感器启用状态 |
| `simulation.world_name` | string | 是 | "empty" | Gazebo世界名称 |

## 🔄 系统工作流程

```
用户启动 → main.launch.py
    ↓
解析配置 → session_manager.create_session()
    ↓
生成会话 → 创建临时配置文件和目录
    ↓
启动基础设施 → infra_sim.launch.py (Gazebo环境)
    ↓
启动机器人系统 → robot_bringup.launch.py (实体创建)
    ↓
启动可视化 → visualization.launch.py (RViz界面)
    ↓
系统运行 ← 各组件协同工作
```

## 🛠️ 开发扩展指南

### 添加新型传感器支持

1. **配置定义扩展**
```
# 在配置文件中定义新传感器类型
robot:
  sensors:
    sonar_arrays:
      - name: "forward_sonar"
        enabled: true
        xyz: [1.0, 0.0, -0.3]
        parameters:
          beam_count: 48
          frequency: 200000
```

2. **会话管理器扩展**
```
# 在session_manager.py中添加处理逻辑
def generate_sonar_sensor(sensor_config):
    """生成声纳传感器的Xacro宏定义"""
    sonar_xacro = f'''
    <xacro:macro name="sonar_macro" params="name parent_link xyz rpy">
        <xacro:sonar_sensor name="${{name}}" parent_link="${{parent_link}}" 
                          xyz="${{xyz}}" rpy="${{rpy}}"
                          beam_count="{sensor_config['parameters']['beam_count']}"/>
    </xacro:macro>
    '''
    return sonar_xacro
```

### 自定义控制算法集成

```
# 创建自定义控制器类
class PIDController:
    def __init__(self, kp=1.0, ki=0.1, kd=0.01):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.previous_error = 0.0
    
    def compute(self, error, dt):
        """计算PID控制输出"""
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

# 在会话管理器中集成
def integrate_custom_controller(controller_config):
    """集成用户自定义控制器"""
    controller_type = controller_config.get('type', 'default')
    if controller_type == 'pid':
        return PIDController(**controller_config.get('parameters', {}))
```

## 📊 性能监控与调试

### 会话日志结构

每次运行都会在`logs/`目录下创建独立的会话文件夹：
```
session_YYYYMMDD_HHMMSS/
├── source_config.yaml      # 用户原始配置备份
├── final_robot.urdf        # 生成的最终URDF文件
├── bridge_config.yaml      # 传感器桥接配置
├── session.rviz            # RViz配置文件
├── obstacle_layout.json    # 障碍物布局定义
└── session.log             # 会话运行日志
```

### 调试工具集

```bash
# 查看系统状态
ros2 node list              # 列出所有运行节点
ros2 topic list             # 列出所有话题
ros2 service list           # 列出所有服务

# 监控特定数据流
ros2 topic hz /sensors/lidar/front/points    # 查看话题频率
ros2 topic echo /model/wamv/odometry        # 实时查看数据

# 系统性能分析
ros2 doctor                 # ROS 2系统诊断
top                         # 系统资源监控
```

## 🔧 故障排除指南

### 常见配置错误

1. **YAML语法错误**
```bash
# 验证YAML文件语法
python3 -c "import yaml; yaml.safe_load(open('<config_file>'))"
```

2. **URDF编译失败**
```bash
# 手动测试Xacro编译
ros2 run xacro xacro templates/wamv_no_battery.urdf.xacro --inorder
```

3. **传感器桥接异常**
```bash
# 检查桥接配置
cat logs/session_*/bridge_config.yaml
ros2 run ros_gz_bridge parameter_bridge --help
```

### 性能优化建议

1. **减少不必要的传感器**：只启用需要的传感器以提高仿真效率
2. **调整Gazebo参数**：适当降低仿真精度换取更高帧率
3. **优化控制器频率**：平衡控制精度和计算负载

## 📈 API参考文档

### SessionManager类
```
class SessionManager:
    def __init__(self):
        """初始化会话管理器"""
        pass
    
    def create_session(self, config_path: str) -> dict:
        """
        创建新的仿真会话
        
        Args:
            config_path: 用户配置文件路径
            
        Returns:
            dict: 包含会话路径和配置文件信息的字典
        """
        pass
    
    def cleanup_session(self, session_path: str):
        """
        清理会话资源
        
        Args:
            session_path: 会话目录路径
        """
        pass
```

### 配置验证器
```python
def validate_config(config_data: dict) -> bool:
    """
    验证配置文件的完整性和有效性
    
    Args:
        config_data: 解析后的配置数据
        
    Returns:
        bool: 配置是否有效
    """
    pass
```

---
*有关项目整体架构和其他组件包的信息，请参见[主项目README](../../README.md)*
