import os

# 定义功能包名称
PKG_NAME = "usv_interfaces"

# 定义文件内容结构
files = {}

# ==========================================
# 1. package.xml (增加 ament_cmake_python 依赖)
# ==========================================
files[f"{PKG_NAME}/package.xml"] = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{PKG_NAME}</name>
  <version>0.1.0</version>
  <description>Standard interfaces and constants for USV control system</description>
  <maintainer email="developer@usv.com">USV Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend> <!-- 新增: 支持Python模块安装 -->
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>action_msgs</depend>
  <depend>sensor_msgs</depend> <!-- 新增: 明确依赖 sensor_msgs -->
  <depend>vision_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

# ==========================================
# 2. CMakeLists.txt (增加 Python 安装指令)
# ==========================================
files[f"{PKG_NAME}/CMakeLists.txt"] = f"""cmake_minimum_required(VERSION 3.8)
project({PKG_NAME})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # 新增
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 定义消息文件
set(MSG_FILES
  msg/MmwRadarObject.msg
  msg/NavRadarObject.msg
  msg/VisualObject.msg
  msg/AisData.msg
  msg/ControlDeviation.msg
  msg/OperationMode.msg
  msg/VesselState.msg
)

set(SRV_FILES
  srv/ControlDevice.srv
)

set(ACTION_FILES
  action/ExecuteMission.action
)

# 生成 ROS 接口代码
rosidl_generate_interfaces(${{PROJECT_NAME}}
  ${{MSG_FILES}}
  ${{SRV_FILES}}
  ${{ACTION_FILES}}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# 1. 安装 C++ 头文件 (用于 C++ 节点引用常量)
install(DIRECTORY include/
  DESTINATION include
)

# 2. 安装 Python 模块 (用于 Python 节点引用常量)
ament_python_install_package(${{PROJECT_NAME}})
install(DIRECTORY ${{PROJECT_NAME}}
  DESTINATION ${{CMAKE_INSTALL_PREFIX}}/${{PYTHON_INSTALL_DIR}}
)

# 导出依赖
ament_export_dependencies(std_msgs geometry_msgs builtin_interfaces)
ament_export_include_directories(include)

ament_package()
"""

# ==========================================
# 3. C++ 常量头文件
# ==========================================
files[f"{PKG_NAME}/include/{PKG_NAME}/topics.hpp"] = """#ifndef USV_INTERFACES__TOPICS_HPP_
#define USV_INTERFACES__TOPICS_HPP_

namespace usv_interfaces {

    // 坐标系
    constexpr char FRAME_BASE_LINK[] = "base_link";
    constexpr char FRAME_MAP[]       = "map";
    constexpr char FRAME_ODOM[]      = "odom";
    constexpr char FRAME_GPS[]       = "gps_link";
    constexpr char FRAME_IMU[]       = "imu_link";
    constexpr char FRAME_LIDAR[]     = "lidar_link";

    // 原始传感器 (Raw Sensors)
    constexpr char TOPIC_SENSOR_GPS[]        = "/sensors/gps/fix";           // sensor_msgs/NavSatFix
    constexpr char TOPIC_SENSOR_IMU[]        = "/sensors/imu/data";          // sensor_msgs/Imu
    constexpr char TOPIC_SENSOR_MMW_RAW[]    = "/sensors/radar/mmw/points";  // sensor_msgs/PointCloud2
    constexpr char TOPIC_SENSOR_NAV_IMAGE[]  = "/sensors/radar/nav/image";   // sensor_msgs/CompressedImage
    constexpr char TOPIC_SENSOR_CAMERA[]     = "/sensors/camera/image_raw";  // sensor_msgs/Image

    // 感知结果 (Perception Results)
    constexpr char TOPIC_PERCEPTION_MMW[]    = "/perception/radar/mmw/objects";
    constexpr char TOPIC_PERCEPTION_NAV[]    = "/perception/radar/nav/objects";
    constexpr char TOPIC_PERCEPTION_VISUAL[] = "/perception/camera/objects";
    constexpr char TOPIC_PERCEPTION_AIS[]    = "/perception/ais/data";

    // 状态与控制 (State & Control)
    constexpr char TOPIC_VESSEL_STATE[]      = "/usv/state/vessel";
    constexpr char TOPIC_CONTROL_DEVIATION[] = "/usv/control/deviation";
    constexpr char TOPIC_CONTROL_MODE[]      = "/usv/control/mode";
    constexpr char TOPIC_CMD_VEL[]           = "/cmd_vel";

} // namespace usv_interfaces

#endif // USV_INTERFACES__TOPICS_HPP_
"""

# ==========================================
# 4. Python 常量模块 (__init__.py 和 topics.py)
# ==========================================

files[f"{PKG_NAME}/{PKG_NAME}/__init__.py"] = ""

files[f"{PKG_NAME}/{PKG_NAME}/topics.py"] = """# 此文件与 topics.hpp 保持同步
# Python 节点使用方式: from usv_interfaces.topics import *

# ==========================================
# 坐标系名称 (Frame IDs)
# ==========================================
FRAME_BASE_LINK = "base_link"      # 船体中心
FRAME_MAP       = "map"            # 全局地图
FRAME_ODOM      = "odom"           # 里程计坐标系
FRAME_GPS       = "gps_link"
FRAME_IMU       = "imu_link"
FRAME_LIDAR     = "lidar_link"

# ==========================================
# 传感器话题 (Sensors)
# ==========================================
TOPIC_SENSOR_GPS        = "/sensors/gps/fix"           # Type: sensor_msgs/NavSatFix
TOPIC_SENSOR_IMU        = "/sensors/imu/data"          # Type: sensor_msgs/Imu
TOPIC_SENSOR_MMW_RAW    = "/sensors/radar/mmw/points"  # Type: sensor_msgs/PointCloud2
TOPIC_SENSOR_NAV_IMAGE  = "/sensors/radar/nav/image"   # Type: sensor_msgs/CompressedImage
TOPIC_SENSOR_CAMERA     = "/sensors/camera/image_raw"  # Type: sensor_msgs/Image

# ==========================================
# 感知结果话题 (Perception)
# ==========================================
TOPIC_PERCEPTION_MMW    = "/perception/radar/mmw/objects"    # Type: usv_interfaces/MmwRadarObject
TOPIC_PERCEPTION_NAV    = "/perception/radar/nav/objects"    # Type: usv_interfaces/NavRadarObject
TOPIC_PERCEPTION_VISUAL = "/perception/camera/objects"       # Type: usv_interfaces/VisualObject
TOPIC_PERCEPTION_AIS    = "/perception/ais/data"             # Type: usv_interfaces/AisData

# ==========================================
# 状态与控制话题 (State & Control)
# ==========================================
TOPIC_VESSEL_STATE      = "/usv/state/vessel"        # Type: usv_interfaces/VesselState
TOPIC_CONTROL_DEVIATION = "/usv/control/deviation"   # Type: usv_interfaces/ControlDeviation
TOPIC_CONTROL_MODE      = "/usv/control/mode"        # Type: usv_interfaces/OperationMode
TOPIC_CMD_VEL           = "/cmd_vel"                 # Type: geometry_msgs/Twist
"""

# ==========================================
# 5. 特殊需求：在msg文件夹中说明原始数据类型
# ==========================================
files[f"{PKG_NAME}/msg/StandardDataTypes.md"] = """# 原始传感器数据类型对照表

为了保持与 ROS 2 生态的兼容性，本系统中的**原始传感器数据**直接使用 ROS 2 标准消息 (`sensor_msgs`)，不进行自定义封装。

## 数据流向参考

| 数据源 | 话题名称 (Code Constant) | 消息类型 (ROS Type) | 说明 |
| :--- | :--- | :--- | :--- |
| **GPS/GNSS** | `TOPIC_SENSOR_GPS` | `sensor_msgs/NavSatFix` | 包含经纬度、海拔及协方差 |
| **IMU** | `TOPIC_SENSOR_IMU` | `sensor_msgs/Imu` | 包含四元数姿态、角速度、线加速度 |
| **毫米波雷达(Raw)** | `TOPIC_SENSOR_MMW_RAW` | `sensor_msgs/PointCloud2` | 包含 x, y, z, velocity, intensity 字段 |
| **导航雷达(Raw)** | `TOPIC_SENSOR_NAV_IMAGE` | `sensor_msgs/CompressedImage` | 极坐标回波图像 (JPEG/PNG) |
| **摄像头(Raw)** | `TOPIC_SENSOR_CAMERA` | `sensor_msgs/Image` | 原始 RGB 图像数据 |
| **2D 扫描** | - | `sensor_msgs/LaserScan` | 若有单线雷达或虚拟扫描使用此格式 |

**注意**：自定义的 `.msg` 文件（如 `MmwRadarObject.msg`）仅用于描述经过感知算法处理后的**目标结果**。
"""

# ==========================================
# 6. Message Files (.msg)
# ==========================================

files[f"{PKG_NAME}/msg/MmwRadarObject.msg"] = """std_msgs/Header header
uint32 id               # 目标ID
geometry_msgs/Point position # 相对本船的位置 (x, y, z)
geometry_msgs/Vector3 velocity # 相对速度 (vx, vy, vz)
float32 range           # 距离 (m)
float32 azimuth         # 方位角 (rad)
float32 rcs             # 雷达截面积/强度
"""

files[f"{PKG_NAME}/msg/NavRadarObject.msg"] = """std_msgs/Header header
uint32 id

# === 绝对运动信息 ===
float32 sog                 # 对地速度 (m/s)
float32 cog                 # 对地航向 (rad)

# === 相对关系 ===
float32 range               # 距离本船 (m)
float32 relative_bearing    # 相对本船船头的方位角 (rad)
geometry_msgs/Point position # 在地图坐标系下的估算位置

# === 属性 ===
float32 area                # 目标面积 (m^2)
bool is_moving              # 是否运动
"""

files[f"{PKG_NAME}/msg/VisualObject.msg"] = """std_msgs/Header header
uint32 id
string class_name           # 类别名

# === 识别框 Bounding Box ===
uint32 bbox_x               # 左上角 x (pixel)
uint32 bbox_y               # 左上角 y (pixel)
uint32 bbox_w               # 宽 (pixel)
uint32 bbox_h               # 高 (pixel)

# === 空间关系 ===
float32 relative_bearing    # 相对角度中心 (rad)
float32 confidence          # 置信度 (0.0 - 1.0)
"""

files[f"{PKG_NAME}/msg/AisData.msg"] = """std_msgs/Header header
uint32 mmsi                 # 水上移动业务标识码

# === 时间戳 ===
builtin_interfaces/Time timestamp_utc # AIS报文原始生成时间

float64 latitude
float64 longitude
float32 sog                 # m/s
float32 cog                 # rad
float32 heading             # rad
uint8 navigational_status   # 航行状态枚举
"""

files[f"{PKG_NAME}/msg/ControlDeviation.msg"] = """std_msgs/Header header

float32 cross_track_error   # 循迹误差 (偏航距离 m, 左正右负)
float32 heading_error       # 艏向误差 (rad)
float32 distance_to_goal    # 距离当前局部目标点的距离 (m)
float32 target_speed        # 期望航速 (m/s)
float32 current_speed       # 实际航速 (m/s)
"""

files[f"{PKG_NAME}/msg/OperationMode.msg"] = """uint8 MODE_MANUAL        = 0  # 手动遥控
uint8 MODE_AUTO_HEADING  = 1  # 自动航向/巡线
uint8 MODE_SMART_PATH    = 2  # 智能避障巡线
uint8 MODE_AUTO_NAV      = 3  # 自主点对点导航

std_msgs/Header header
uint8 mode                    # 当前模式
"""

files[f"{PKG_NAME}/msg/VesselState.msg"] = """std_msgs/Header header

# === 1. 地理位置 (来源: GPS/GNSS) ===
float64 latitude            # 纬度
float64 longitude           # 经度
float32 altitude            # 海拔

# === 2. 局部运动状态 (来源: 里程计/IMU融合) ===
# 包含位置(x,y,z) 和 姿态(x,y,z,w 四元数)
geometry_msgs/Pose pose

# 包含线速度(vx, vy, vz) 和 角速度(wx, wy, wz)
geometry_msgs/Twist velocity

# === 3. 欧拉角 (方便调试与UI显示) ===
float32 roll                # 横滚 (rad)
float32 pitch               # 俯仰 (rad)
float32 yaw                 # 航向 (rad)

# === 4. 设备健康状态 ===
float32 battery_voltage     # 电池电压 (V)
float32 battery_percentage  # 电量 (%)
bool leak_detected          # 漏水报警 (True=漏水)
float32 cpu_temperature     # 核心温度 (C)
"""

# ==========================================
# 7. Service & Action
# ==========================================

files[f"{PKG_NAME}/srv/ControlDevice.srv"] = """# === Request ===
string device_id    # 设备ID (例如: "nav_light_left", "sampler_pump")
string command      # 指令 (例如: "ON", "OFF", "SET_LEVEL")
float32 value       # 数值 (例如: 亮度 0-100)

---
# === Response ===
bool success        # 执行是否成功
string message      # 错误信息或执行结果描述
"""

files[f"{PKG_NAME}/action/ExecuteMission.action"] = """# === Goal (请求) ===
string mission_file_name      # 任务航点文件路径/名称
string forbidden_zone_file    # 禁航区描述文件路径/名称
bool loop_execution           # 是否循环执行

---
# === Result (结果) ===
bool success
string message                # 比如 "Mission Complete"

---
# === Feedback (反馈) ===
string current_task_name      # 当前阶段名称
uint32 current_waypoint_index # 当前目标索引
float32 mission_progress      # 总进度 %
string status_text            # 状态机描述
"""

# ==========================================
# 执行生成
# ==========================================
def create_package():
    print(f"Start creating package: {PKG_NAME} ...")
    
    for filepath, content in files.items():
        directory = os.path.dirname(filepath)
        if directory and not os.path.exists(directory):
            os.makedirs(directory)
            print(f"Created directory: {directory}")
            
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"Generated file: {filepath}")

    print("\n" + "="*50)
    print(f"Package '{PKG_NAME}' generated successfully!")
    print("Next steps:")
    print("1. Delete this script.")
    print(f"2. Run: colcon build --packages-select {PKG_NAME}")
    print("3. Source environment: . install/setup.bash")
    print("4. Python Usage: from usv_interfaces.topics import *")
    print("="*50)

if __name__ == "__main__":
    create_package()
