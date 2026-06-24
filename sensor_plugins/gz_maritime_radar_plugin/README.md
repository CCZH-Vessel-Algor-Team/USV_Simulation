# Maritime Radar Gazebo Plugin

独立的海事雷达仿真 Gazebo 插件适用于Gazebo Garden版本，编译后生成 `libMaritimeRadarPlugin.so`，可直接在任何 SDF 模型中引用。
## 仿真机理
通过仿真旋转式激光雷达并设定数据格式来仿真海事雷达。

## 编译

```bash
cd gz_maritime_radar_plugin
mkdir build && cd build
cmake ..
make -j4
```

编译后得到: `build/libMaritimeRadarPlugin.so`

## 使用

### 1. 设置环境变量

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/path/to/build
```

### 2. 在 SDF 模型中引用

```xml
<model name="my_boat">
  <!-- 你的模型定义 -->
  

    <!-- ==================== 海事雷达 ==================== -->
    
    <!-- 雷达基座 (固定在传感器板上) -->
    <link name="radar_base_link">
      <pose>0 0 0.15 0 0 0</pose>  <!-- 安装在传感器板上方 -->
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="radar_base_visual">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <diffuse>0.2 0.2 0.2 1</diffuse>
          <ambient>0.1 0.1 0.1 1</ambient>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
      <collision name="radar_base_collision">
        <geometry>
          <cylinder>
            <radius>0.08</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="radar_base_joint" type="fixed">
      <parent>sensor_plate_link</parent>
      <child>radar_base_link</child>
    </joint>

    <!-- 雷达天线 (旋转部分) -->
    <link name="radar_antenna_link">
      <pose>0 0 0.2 0 0 0</pose>  <!-- 基座上方 -->
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>0.005</ixx>
          <iyy>0.02</iyy>
          <izz>0.02</izz>
        </inertia>
      </inertial>
      
      <!-- 天线外壳 (雷达罩) -->
      <visual name="radar_dome_visual">
        <geometry>
          <box>
            <size>0.4 0.1 0.06</size>
          </box>
        </geometry>
        <material>
          <diffuse>0.9 0.9 0.9 1</diffuse>
          <ambient>0.8 0.8 0.8 1</ambient>
          <specular>0.9 0.9 0.9 1</specular>
          <pbr>
            <metal>
              <metalness>0.3</metalness>
              <roughness>0.4</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      
      <collision name="radar_antenna_collision">
        <geometry>
          <box>
            <size>0.4 0.1 0.06</size>
          </box>
        </geometry>
      </collision>

      <!-- GPU LiDAR 传感器 (用于雷达仿真) -->
      <sensor name="radar_lidar" type="gpu_lidar">
        <pose>0.15 0 0 0 0 0</pose>  <!-- 天线前端 -->
        <topic>/blueboat/radar/lidar</topic>
        <update_rate>250</update_rate>
        <always_on>true</always_on>
        <visualize>false</visualize>
        <ray>
          <visibility_mask>0xFFFFFFFD</visibility_mask>
          <scan>
            <horizontal>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </horizontal>
            <vertical>
              <samples>128</samples>
              <resolution>1</resolution>
              <min_angle>0.003</min_angle>  <!-- 水平面往上 -->
              <max_angle>0.15</max_angle>   <!-- 约 +9° -->
            </vertical>
          </scan>
          <range>
            <min>1.0</min>
            <max>500.0</max>  <!-- 小型船用雷达，500m 足够 -->
            <resolution>0.1</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.05</stddev>
          </noise>
        </ray>
      </sensor>
    </link>

    <!-- 雷达旋转关节 -->
    <joint name="radar_joint" type="revolute">
      <parent>radar_base_link</parent>
      <child>radar_antenna_link</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-1e16</lower>
          <upper>1e16</upper>
        </limit>
        <dynamics>
          <damping>0.001</damping>
        </dynamics>
      </axis>
    </joint>

    <!-- 雷达旋转控制器: 48 RPM = -5.03 rad/s (典型小型雷达转速) -->
    <plugin filename="gz-sim-joint-controller-system"
            name="gz::sim::systems::JointController">
      <joint_name>radar_joint</joint_name>
      <initial_velocity>-5.03</initial_velocity>
    </plugin>

    <!-- 海事雷达数据处理插件 -->
    <plugin filename="MaritimeRadarPlugin" name="maritime_radar">
      <joint_name>radar_joint</joint_name>
      <lidar_topic>/blueboat/radar/lidar</lidar_topic>
      <radar_topic>/blueboat/radar/spokes</radar_topic>
      <angular_resolution>0.00306796</angular_resolution>  <!-- ~1.44° -->
      <linear_resolution>1.0</linear_resolution>           <!-- 2m/bin -->
      <min_range>5.0</min_range>
      <max_range>500.0</max_range>
      <!-- 新增：最小仰角过滤，消除水面杂波 -->
      <min_elevation_angle>-0.08</min_elevation_angle>  <!-- 约 -1.15°，可根据需要调整 -->
    </plugin>

    <!-- ==================== 海事雷达结束 ==================== -->

</model>
```

## SDF 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `joint_name` | string | (必需) | 雷达旋转关节名称 |
| `lidar_topic` | string | /model_name/lidar | GPU LiDAR 输入话题 |
| `radar_topic` | string | /model_name/radar/spokes | 雷达数据输出话题 |
| `angular_resolution` | double | 0.0251327 | 角度分辨率 (rad), 约 1.44° |
| `linear_resolution` | double | 0.75 | 距离分辨率 (m) |
| `min_range` | double | 1.0 | 最小检测距离 (m) |
| `max_range` | double | 1500.0 | 最大检测距离 (m) |

## 输出消息格式

发布 `gz.msgs.Float_V` 消息:

```
data[0]: 当前角度 (rad)
data[1]: 角度分辨率 (rad)
data[2]: 距离分辨率 (m)
data[3...N]: 各距离 bin 的强度值 (dB)
```

## 测试

```bash
# 终端1: 启动 Gazebo
gz sim -r models/maritime_radar/model.sdf

# 终端2: 监听雷达数据
gz topic -e -t /maritime_radar/radar/spokes
```

## 完整示例模型

参考 `models/maritime_radar/model.sdf`，包含:
- 基座 + 旋转天线
- GPU LiDAR 传感器 (250Hz, 256 垂直采样)
- 关节控制器 (60 RPM)
- 海事雷达插件

