# 传感器宏修改与添加操作指南

根据 `usv_sim_full` 当前的架构设计，机器人的传感器采用的是 **“配置驱动 + 动态聚合”** 机制。每次启动仿真时，后端的 `session_manager.py` 会读取 `full_config.yaml` 中配置的参数，然后主动去 `sensor_macros.xacro` 里拉取对应的传感器宏，自动生成模型结构(URDF)，最后配套生成 Gazebo 与 ROS 2 的话题桥接文件 (Bridge Configuration)。

下面为您详细说明两种经常遇到的应用场景：**如何修改现有的传感器属性** 以及 **如何从零加入一种新类别传感器**。

---

## 场景一：修改现有传感器宏的大致尺寸/底层参数（例如摄像头）

如果您仅仅是想调整目前已有传感器的特性（例如摄像头的视场角、分辨率、IMU的噪声方差，或是更改它们的外观和碰撞体积），非常简单，只需要改一处模板文件即可：

### 1. 修改底层 Xacro 宏模板
**所在文件：** `src/usv_sim_full/templates/sensor_macros.xacro`

找到代表目标传感器的宏定义代码（如 `<xacro:macro name="camera_macro" ...>` ）。
修改其中的 `<sensor>` 标签内的 Gazebo 配置。例如，我们要把相机的纵横比调宽：

```xml
<!-- 找到该段落并修改下面标记的值 -->
<sensor name="${name}" type="camera">
  <camera name="${name}">
    <!-- 修改FOV视场角 -->
    <horizontal_fov>1.5</horizontal_fov>
    <image>
      <!-- 修改分辨率 -->
      <width>1920</width>
      <height>1080</height>
    </image>
    ...
  </camera>
</sensor>
```

### 2. 使修改生效
保存文件后，在当前工作空间(Workspace)重新编译以将模板更新到安装层（如果配置了 symlink 安装偶尔可以跳过这步，但为了安全起见建议编译）：
```bash
colcon build --packages-select usv_sim_full
source install/setup.bash
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml'
```
系统会重新构建 session 并生成最新的摄像头模型。

---

## 场景二：添加一种全新的传感器类别（以“声纳 Sonar”为例）

如果系统尚未支持该类别（比如之前只有 lidar、camera、imu、gps），要想将其完全纳入这套动态生成框架并与 ROS 2 实现话题互通，需要按**顺序修改 4 个核心文件**完成闭环。

### 第一步：在模板中设计底层 Xacro 宏
**所在文件：** `src/usv_sim_full/templates/sensor_macros.xacro`

在这里描述这颗新传感器的长什么样(`visual`)、体积多大(`collision`)，并向 Gazebo 声明传感器插件。
```xml
  <!-- 全新的声纳宏 Sonar Macro -->
  <xacro:macro name="sonar_macro" params="name parent_link:='base_link' xyz:='0 0 0' rpy:='0 0 0' topic:=/sonar/range">
    <!-- 1. 定义物理Link和几何体 -->
    <link name="${name}_link">
      <visual>
        <geometry><cylinder radius="0.05" length="0.1"/></geometry>
      </visual>
      <!-- 可以补充 inertial 和 collision -->
    </link>

    <!-- 2. 定义Fixed Joint将其吸附在给定的parent_link上 -->
    <joint name="${name}_joint" type="fixed">
      <parent link="${parent_link}"/>
      <child link="${name}_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <!-- 3. 定义Gazebo标签激活Sensor -->
    <gazebo reference="${name}_link">
      <!-- 注意type需要使用Gazebo Harmonic支持的类型 -->
      <sensor name="${name}" type="gpu_lidar"> <!-- (由于Gazebo兼容性，声线常使用极窄的lidar模拟) -->
        <topic>${topic}</topic>
        <!-- 此处填入传感器的细分参数 -->
      </sensor>
    </gazebo>
  </xacro:macro>
```

### 第二步：在 YAML 配置文件中开放使用者接口
**所在文件：** `src/usv_sim_full/config/full_config.yaml`

在 `sensors:` 节点下新增对应的配置列表区，方便日后只改 YAML 就能增删设备。
```yaml
sensors:
  # ... 其他已有传感器 ...
  sonars:
    - name: "front_sonar"
      type: "sonar"
      parent_link: "base_link"
      xyz: [1.0, 0.0, -0.4]  # 挂在水下
      rpy: [0.0, 0.0, 0.0]
      topic: "/sensors/sonar/data"
      enabled: true
```

### 第三步：让生成脚本 (Session Manager) 知道如何提取 YAML 生成 Xacro
**所在文件：** `src/usv_sim_full/scripts/session_manager.py` -> `generate_sensors_overlay(...)` 函式

仿照原有的解析片段（如 cameras的循环），将刚才配的字典转换为 Xacro 文本：
```python
    # 处理声纳 (Sonars)
    sonars = sensors_config.get('sonars', [])
    for sonar in sonars:
        if not sonar.get('enabled', True):
            continue
            
        sensor_name = sonar['name']
        parent_link = sonar['parent_link']
        xyz = ' '.join(map(str, sonar['xyz']))
        rpy = ' '.join(map(str, sonar['rpy']))
        topic = sonar.get('topic', f'/sensors/sonar/{sensor_name}/data')
        
        # 将参数压入要生成的Xacro字符串中，引入我们在第一步中设计的sonar_macro
        topic_ns = topic.lstrip('/')
        sensors_content += f'''
    <xacro:sonar_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}"/>
'''
```

### 第四步：补充该新传感器的数据桥接 (ROS-GZ Bridge)
**所在文件：** `src/usv_sim_full/scripts/session_manager.py` -> `generate_bridge_config(...)` 函式

为了能用 ROS 2 指令（`ros2 topic echo`）听到传感器发出的数据，必须在这配置 Topic Bridge 映射表：

```python
    # 在这个函数内遍历 YAML，把 GZ 世界里产生的数据对接到 ROS 2 对应的消息格式上：
    
    sonars = sensors_config.get('sonars', [])
    for sonar in sonars:
        if not sonar.get('enabled', True):
            continue
            
        snsr_name = sonar['name']
        topic = sonar.get('topic', f'/sensors/sonar/{snsr_name}/data')

        # 添加该声纳的桥接块
        bridges.append({
            "ros_topic_name": f"/{sanitized_bridge_ns}{topic}",
            # 注意：大部分Gazebo传感器的原始主题在 /world/[世界名]/model/[模型名]/sensor/[传感器名]... 结构下
            "gz_topic_name": f"/world/{world_name}/model/{sanitized_bridge_ns}/link/{(sanitized_bridge_ns) + '_' + snsr_name}_link/sensor/{sanitized_bridge_ns}_{snsr_name}/scan",
            
            # 您需要确保这两个报文类型相互匹配，否则Bridge桥接器会报错罢工
            "ros_type_name": "sensor_msgs/msg/LaserScan", 
            "gz_type_name": "gz.msgs.LaserScan",
            "direction": "GZ_TO_ROS"
        })
```

完成以上4步后，编译包 `colcon build --packages-select usv_sim_full` 并启动即可验证。
系统会自动把您在 YAML 中的配置渲染为带有声纳物理组件环境，并把桥接器拉起，输出 ROS2 Topic！