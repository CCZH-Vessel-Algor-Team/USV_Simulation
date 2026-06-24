# Radar Gazebo Bridge

将 Gazebo 海事雷达数据 (`gz.msgs.Float_V`) 转换为 ROS2 标准消息 (`marine_sensor_msgs/RadarSector`)。

## 依赖

```bash
# ROS2 Humble
sudo apt install ros-humble-ros-gz

# marine_sensor_msgs
cd ~/your_ws/src
git clone -b ros2 https://github.com/apl-ocean-engineering/marine_msgs.git
colcon build --packages-select marine_sensor_msgs
```

## 编译

```bash
cd ~/your_ws/src
# 复制 radar_gz_bridge 到这里

cd ~/your_ws
colcon build --packages-select radar_gz_bridge
source install/setup.bash
```

## 使用

### 方式 1: 直接运行

```bash
ros2 run radar_gz_bridge radar_gz_bridge \
  --ros-args \
  -p gz_topic:=/blueboat/radar/spokes \
  -p ros_topic:=/sensors/radar/nav/sector \
  -p frame_id:=nav_radar_link
```

### 方式 2: 使用 Launch 文件

```bash
ros2 launch radar_gz_bridge radar_bridge.launch.py \
  gz_topic:=/blueboat/radar/spokes \
  ros_topic:=/sensors/radar/nav/sector
```

### 方式 3: 使用配置文件

```bash
ros2 run radar_gz_bridge radar_gz_bridge \
  --ros-args --params-file config/radar_bridge.yaml
```

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gz_topic` | `/blueboat/radar/spokes` | Gazebo 雷达话题 |
| `ros_topic` | `/sensors/radar/nav/sector` | ROS2 输出话题 |
| `frame_id` | `nav_radar_link` | TF 坐标系 |
| `range_min` | `5.0` | 最小距离 (m) |
| `range_max` | `500.0` | 最大距离 (m) |
| `rotation_period` | `2.5` | 旋转周期 (秒) |

## 消息格式

### 输入: gz.msgs.Float_V

```
data[0]: 当前角度 (rad)
data[1]: 角度分辨率 (rad)
data[2]: 距离分辨率 (m)
data[3...N]: 强度值 (dB)
```

### 输出: marine_sensor_msgs/RadarSector

```yaml
header:
  stamp: <当前时间>
  frame_id: "radar_antenna_link"
angle_start: <角度 rad>
angle_increment: <角度分辨率 rad>
time_increment: {sec: 0, nanosec: 0}
scan_time: {sec: 2, nanosec: 500000000}  # 旋转周期
range_min: 5.0
range_max: 500.0
intensities:
  - echoes: [0, 15, 14, 0, ...]  # 强度 [0,15]
```

## 验证

```bash
# 终端 1: 运行 Gazebo 仿真
gz sim -r your_world.sdf

# 终端 2: 运行桥接
ros2 launch radar_gz_bridge radar_bridge.launch.py

# 终端 3: 查看数据
ros2 topic echo /sensors/radar/nav/sector

# 查看话题频率
ros2 topic hz /sensors/radar/nav/sector
```

## 数据流

```
GPU LiDAR (250Hz)
     ↓
MaritimeRadarPlugin (Gazebo)
     ↓
gz.msgs.Float_V (/blueboat/radar/spokes)
     ↓
radar_gz_bridge (ROS2 节点)
     ↓
marine_sensor_msgs/RadarSector (/sensors/radar/nav/sector)
     ↓
ROS2 应用
```
