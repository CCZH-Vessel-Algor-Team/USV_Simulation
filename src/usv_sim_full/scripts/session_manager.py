#!/usr/bin/env python3
"""
*****************************************************************************************
*  Copyright (C) 2026 MurphyChen, All Rights Reserved                                   *
*                                                                                       *
*  @brief    会话管理器，根据YAML配置生成传感器配置、桥接配置和机器人描述              *
*  @author   MurphyChen                                                               *
*  @version  1.0.0                                                                      *
*  @date     2026.1.14                                                                *
*****************************************************************************************
"""

import os
import yaml
import json
import shutil
from datetime import datetime
import subprocess
from ament_index_python.packages import get_package_share_directory
import math
import random


def create_session(config_path):
    """
    创建一个新的会话，包括日志目录和配置快照
    
    Args:
        config_path (str): 用户配置文件路径
        
    Returns:
        dict: 包含session_path、urdf_path、bridge_yaml_path和rviz_config_path的字典
    """
    # 获取当前时间戳
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 获取脚本所在目录的父目录（包目录）
    package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # 创建会话日志目录
    session_dir = os.path.join(package_dir, "logs", f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)
    
    # 复制用户配置到会话目录
    source_config_path = os.path.join(session_dir, "source_config.yaml")
    shutil.copy2(config_path, source_config_path)
    
    # 读取配置文件
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # 生成传感器叠加层Xacro
    sensors_overlay_path = generate_sensors_overlay(config_data, session_dir)
    
    # 生成根Xacro文件
    root_xacro_path = generate_root_xacro(config_data, session_dir, sensors_overlay_path)
    
    # 编译Xacro为URDF - 使用物理参数覆盖
    urdf_path = compile_xacro_to_urdf(root_xacro_path, config_data, session_dir)
    
    # 生成桥接配置
    bridge_config = generate_bridge_config(config_data)
    
    # 保存桥接配置到会话目录
    bridge_yaml_path = os.path.join(session_dir, "bridge_config.yaml")
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_config, f)
    
    # 生成RViz配置
    rviz_config_path = generate_rviz_config(config_data, session_dir)
    
    # 生成障碍物布局
    obstacle_layout_path = generate_obstacles(config_data, session_dir)
    
    # 返回路径信息
    result = {
        "session_path": session_dir,
        "urdf_path": urdf_path,
        "bridge_yaml_path": bridge_yaml_path,
        "rviz_config_path": rviz_config_path,
        "obstacle_layout_path": obstacle_layout_path
    }
    
    return result


def generate_sensors_overlay(config_data, session_dir):
    """
    根据传感器配置生成传感器叠加层Xacro文件
    
    Args:
        config_data (dict): 配置数据
        session_dir (str): 会话目录
        
    Returns:
        str: 生成的传感器叠加层Xacro文件路径
    """
    sensors_overlay_path = os.path.join(session_dir, "generated_sensors.xacro")
    
    # 读取模板文件
    template_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                 "templates", "sensor_macros.xacro")
    
    with open(template_path, 'r') as f:
        template_content = f.read()
    
    # 只提取宏定义部分，不包括xml和robot标签
    lines = template_content.split('\n')
    inside_macro = False
    current_macro = []
    macros_content = []  # 添加缺失的变量定义
    
    for line in lines:
        stripped_line = line.strip()
        if stripped_line.startswith('<xacro:macro'):
            inside_macro = True
            current_macro = [line]
        elif inside_macro:
            current_macro.append(line)
            if stripped_line == '</xacro:macro>':
                # 结束宏定义
                macros_content.extend(current_macro)
                inside_macro = False
    
    # 生成传感器链接 - 注意不要重复XML声明
    sensors_content = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
    
    # 添加提取的宏定义
    for line in macros_content:
        sensors_content += line + '\n'
    
    sensors_content += "\n"
    
    # 添加传感器链接
    sensors_config = config_data.get('sensors', {})
    
    # 处理激光雷达
    lidars = sensors_config.get('lidars', [])
    for lidar in lidars:
        if not lidar.get('enabled', True):
            continue
            
        sensor_name = lidar['name']
        parent_link = lidar['parent_link']
        xyz = ' '.join(map(str, lidar['xyz']))
        rpy = ' '.join(map(str, lidar['rpy']))
        topic = lidar.get('topic', f'/sensors/lidar/{sensor_name}/points')
        
        sensors_content += f'''
  <xacro:vlp16_macro name="{sensor_name}" parent_link="wamv/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="{topic}"/>
'''
    
    # 处理摄像头
    cameras = sensors_config.get('cameras', [])
    for cam in cameras:
        if not cam.get('enabled', True):
            continue
            
        sensor_name = cam['name']
        parent_link = cam['parent_link']
        xyz = ' '.join(map(str, cam['xyz']))
        rpy = ' '.join(map(str, cam['rpy']))
        topic = cam.get('topic', f'/sensors/camera/{sensor_name}/image_raw')
        info_topic = cam.get('info_topic', f'/sensors/camera/{sensor_name}/camera_info')
        
        sensors_content += f'''
  <xacro:camera_macro name="{sensor_name}" parent_link="wamv/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="{topic}" info_topic="{info_topic}"/>
'''
    
    # 处理IMU
    imus = sensors_config.get('imus', [])
    for imu in imus:
        if not imu.get('enabled', True):
            continue
            
        sensor_name = imu['name']
        parent_link = imu['parent_link']
        xyz = ' '.join(map(str, imu['xyz']))
        rpy = ' '.join(map(str, imu['rpy']))
        topic = imu.get('topic', '/sensors/imu/data')
        update_rate = imu.get('update_rate', 100)
        
        sensors_content += f'''
  <xacro:imu_macro name="{sensor_name}" parent_link="wamv/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="{topic}" update_rate="{update_rate}"/>
'''
    
    # 处理GPS
    gps_sensors = sensors_config.get('gps_sensors', [])
    for gps in gps_sensors:
        if not gps.get('enabled', True):
            continue
            
        sensor_name = gps['name']
        parent_link = gps['parent_link']
        xyz = ' '.join(map(str, gps['xyz']))
        rpy = ' '.join(map(str, gps['rpy']))
        topic = gps.get('topic', '/sensors/gps/data')
        update_rate = gps.get('update_rate', 20)
        
        sensors_content += f'''
  <xacro:gps_macro name="{sensor_name}" parent_link="wamv/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="{topic}" update_rate="{update_rate}"/>
'''

    sensors_content += '</robot>\n'
    
    # 写入传感器叠加层文件
    with open(sensors_overlay_path, 'w') as f:
        f.write(sensors_content)
    
    return sensors_overlay_path


def generate_root_xacro(config_data, session_dir, sensors_overlay_path):
    """
    不再生成独立的root xacro，仅返回基础模板路径和参数信息。
    现在由 compile_xacro_to_urdf 直接处理模板选择。
    """
    return None  # 后续由 compile_xacro_to_urdf 处理


def compile_xacro_to_urdf(root_xacro_path, config_data, session_dir):
    """
    使用 ros2 run xacro xacro 编译最终URDF，支持选择不同模板（如无电池版）和推进器配置
    现在也包含传感器配置
    
    Args:
        root_xacro_path (str): 已废弃，不使用
        config_data (dict): 配置数据
        session_dir (str): 会话目录
        
    Returns:
        str: 生成的URDF文件路径
    """
    urdf_path = os.path.join(session_dir, "final_robot.urdf")
    robot_config = config_data.get('robot', {})
    
    # 获取要使用的xacro模板名称
    xacro_template = robot_config.get('xacro_template', 'wamv_gazebo.urdf.xacro')
    
    # 判断是否使用本地模板（例如无电池版本）
    if xacro_template == 'wamv_no_battery.urdf.xacro':
        # 使用本地模板文件
        local_template_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            'templates', 
            'wamv_no_battery.urdf.xacro'
        )
        if not os.path.exists(local_template_path):
            raise FileNotFoundError(f"Template not found: {local_template_path}")
        xacro_input = local_template_path
    else:
        # 使用本地description目录中的xacro文件
        usv_sim_path = get_package_share_directory('usv_sim_full')
        xacro_input = os.path.join(usv_sim_path, 'description', 'urdf', xacro_template)
        if not os.path.exists(xacro_input):
            # 回退到原来的查找方式
            package_name = robot_config.get('package_name', 'wamv_gazebo')
            xacro_relative_path = robot_config.get('xacro_relative_path', f'urdf/{xacro_template}')
            try:
                pkg_path = get_package_share_directory(package_name)
                xacro_input = os.path.join(pkg_path, xacro_relative_path)
            except Exception:
                xacro_input = f"/home/cczh/simulation/vrx_ws/install/{package_name}/share/{package_name}/{xacro_relative_path}"

    # 构建xacro命令
    cmd = [
        'ros2', 'run', 'xacro', 'xacro',
        xacro_input,
        f'name:=wamv'
    ]

    # 添加物理参数覆盖
    overrides = robot_config.get('overrides', {})
    for key, value in overrides.items():
        if key == 'inertia' and isinstance(value, list):
            cmd.extend([
                f'ixx:={value[0]}',
                f'iyy:={value[1]}',
                f'izz:={value[2]}'
            ])
        elif isinstance(value, (int, float)):
            cmd.append(f'{key}:={value}')
        elif isinstance(value, str):
            cmd.append(f'{key}:="{value}"')

    # 添加浮力和水动力参数覆盖
    buoyancy_params = robot_config.get('buoyancy_params', {})
    for key, value in buoyancy_params.items():
        if isinstance(value, (int, float)):
            cmd.append(f'{key}:={value}')
        elif isinstance(value, str):
            cmd.append(f'{key}:="{value}"')

    # 添加推进器配置
    thruster_config = robot_config.get('thruster_config', 'H')  # 默认为 'H' 配置
    cmd.append(f'thruster_config:={thruster_config}')

    # 如果是自定义推进器配置，则添加具体的位置参数
    if thruster_config == 'CUSTOM':
        thruster_positions = overrides.get('thruster_positions', {})
        for pos_key, pos_value in thruster_positions.items():
            if isinstance(pos_value, (int, float)):
                cmd.append(f'{pos_key}:={pos_value}')

    # 检查是否有传感器叠加层文件，如果有则添加它们
    sensors_overlay_path = os.path.join(session_dir, "generated_sensors.xacro")
    if os.path.exists(sensors_overlay_path):
        # 读取传感器文件内容，如果不为空则添加参数
        with open(sensors_overlay_path, 'r') as f:
            sensor_content = f.read().strip()
            if sensor_content and "<xacro:" in sensor_content:  # 检查是否有传感器定义
                cmd.append('has_extra_sensors:=true')
                cmd.append(f'extra_sensors:={sensors_overlay_path}')
    else:
        # 即使没有传感器，也要显式设置has_extra_sensors为false
        cmd.append('has_extra_sensors:=false')

    # 输出路径
    cmd.extend(['-o', urdf_path])

    # 执行命令
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"Failed to compile Xacro to URDF:\nCommand: {' '.join(cmd)}\nError: {result.stderr}")

    # 后处理：将package://协议替换为model://协议以便Gazebo能正确找到模型
    post_process_urdf_for_gazebo(urdf_path)

    print(f"Generated URDF at: {urdf_path}")
    return urdf_path


def post_process_urdf_for_gazebo(urdf_path):
    """
    后处理URDF文件，将package://协议的网格路径替换为正确的model://协议路径
    这样Gazebo就能正确找到模型文件
    
    Args:
        urdf_path (str): URDF文件路径
    """
    # 读取URDF文件
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # 替换package://wamv_description/models/为model://
    # 这样Gazebo就能在模型路径中找到文件
    # 例如: package://wamv_description/models/WAM-V-Base/mesh/M5_body.dae
    # 变为: model://WAM-V-Base/mesh/M5_body.dae
    modified_content = urdf_content.replace(
        'package://wamv_description/models/', 
        'model://'
    )
    
    # 特别处理PNG纹理文件路径
    # 将 model://WAM-V-Base/mesh/WAM-V_Albedo.png 等替换为相对路径
    # 或者使用正确的model://路径格式
    import re
    
    # 匹配PNG纹理文件引用并替换为相对路径
    png_pattern = r'model://([^/]+)/mesh/([^/]+\.png)'
    def replace_png_path(match):
        model_name = match.group(1)
        texture_name = match.group(2)
        # 使用相对路径或者从wamv_description包中正确引用
        return f'model://{model_name}/mesh/{texture_name}'
    
    modified_content = re.sub(png_pattern, replace_png_path, modified_content)
    
    # 写回文件
    with open(urdf_path, 'w') as f:
        f.write(modified_content)
    
    print(f"Post-processed URDF for Gazebo compatibility")


def generate_bridge_config(config_data):
    """
    根据传感器配置生成桥接配置
    
    Args:
        config_data (dict): 配置数据
        
    Returns:
        list: 桥接配置（YAML序列格式）
    """
    # 桥接配置必须是列表格式
    bridges = [
        {
            "ros_topic_name": "/clock",
            "gz_topic_name": "/clock",
            "ros_type_name": "rosgraph_msgs/msg/Clock",
            "gz_type_name": "gz.msgs.Clock",
            "direction": "GZ_TO_ROS"
        }
    ]
    
    # 添加遥测配置
    if config_data.get('visualization', {}).get('enable_telemetry', True):
        # 添加里程计桥接
        bridges.append({
            "ros_topic_name": "/model/wamv/odometry",
            "gz_topic_name": "/model/wamv/odometry",
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS"
        })
        
        # 添加关节状态桥接
        bridges.append({
            "ros_topic_name": "/model/wamv/joint_state",
            "gz_topic_name": "/model/wamv/joint_state",
            "ros_type_name": "sensor_msgs/msg/JointState",
            "gz_type_name": "gz.msgs.Model",
            "direction": "GZ_TO_ROS"
        })
        
        # 添加位姿桥接（用于TF）
        bridges.append({
            "ros_topic_name": "/model/wamv/pose",
            "gz_topic_name": "/model/wamv/pose",
            "ros_type_name": "tf2_msgs/msg/TFMessage",
            "gz_type_name": "gz.msgs.Pose_V",
            "direction": "GZ_TO_ROS"
        })

    # 遍历传感器配置，生成桥接配置
    sensors_config = config_data.get('sensors', {})
    
    # 处理激光雷达
    lidars = sensors_config.get('lidars', [])
    for lidar in lidars:
        if not lidar.get('enabled', True):
            continue
            
        topic = lidar.get('topic', f'/sensors/lidar/{lidar["name"]}/points')
        gz_topic = topic.replace('/sensors/', '/world/sydney_regatta/model/wamv/sensor/')
        bridges.append({
            "ros_topic_name": topic,
            "gz_topic_name": gz_topic,
            "ros_type_name": "sensor_msgs/msg/PointCloud2",
            "gz_type_name": "gz.msgs.PointCloudPacked",
            "direction": "GZ_TO_ROS"
        })
    
    # 处理摄像头
    cameras = sensors_config.get('cameras', [])
    for cam in cameras:
        if not cam.get('enabled', True):
            continue
            
        topic = cam.get('topic', f'/sensors/camera/{cam["name"]}/image_raw')
        info_topic = cam.get('info_topic', f'/sensors/camera/{cam["name"]}/camera_info')
        
        # 图像数据
        bridges.append({
            "ros_topic_name": topic,
            "gz_topic_name": topic.replace('/sensors/', '/world/sydney_regatta/model/wamv/sensor/'),
            "ros_type_name": "sensor_msgs/msg/Image",
            "gz_type_name": "gz.msgs.Image",
            "direction": "GZ_TO_ROS"
        })
        
        # 相机信息
        bridges.append({
            "ros_topic_name": info_topic,
            "gz_topic_name": info_topic.replace('/sensors/', '/world/sydney_regatta/model/wamv/sensor/'),
            "ros_type_name": "sensor_msgs/msg/CameraInfo",
            "gz_type_name": "gz.msgs.CameraInfo",
            "direction": "GZ_TO_ROS"
        })
    
    # 处理IMU
    imus = sensors_config.get('imus', [])
    for imu in imus:
        if not imu.get('enabled', True):
            continue
            
        topic = imu.get('topic', '/sensors/imu/data')
        bridges.append({
            "ros_topic_name": topic,
            "gz_topic_name": topic.replace('/sensors/', '/world/sydney_regatta/model/wamv/sensor/'),
            "ros_type_name": "sensor_msgs/msg/Imu",
            "gz_type_name": "gz.msgs.IMU",
            "direction": "GZ_TO_ROS"
        })
    
    # 处理GPS
    gps_sensors = sensors_config.get('gps_sensors', [])
    for gps in gps_sensors:
        if not gps.get('enabled', True):
            continue
            
        topic = gps.get('topic', '/sensors/gps/data')
        bridges.append({
            "ros_topic_name": topic,
            "gz_topic_name": topic.replace('/sensors/', '/world/sydney_regatta/model/wamv/sensor/'),
            "ros_type_name": "sensor_msgs/msg/NavSatFix",
            "gz_type_name": "gz.msgs.NavSat",
            "direction": "GZ_TO_ROS"
        })
    
    return bridges


def generate_rviz_config(config_data, session_dir):
    """
    根据传感器配置生成RViz配置
    
    Args:
        config_data (dict): 配置数据
        session_dir (str): 会话目录
        
    Returns:
        str: 生成的RViz配置文件路径
    """
    # 读取RViz模板
    template_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                 "templates", "default.rviz")
    
    with open(template_path, 'r') as f:
        rviz_config = yaml.safe_load(f)
    
    # 获取Display列表
    displays = rviz_config['Visualization Manager']['Displays']
    
    # 添加传感器显示项
    sensors_config = config_data.get('sensors', {})
    
    # 处理激光雷达
    lidars = sensors_config.get('lidars', [])
    for lidar in lidars:
        if not lidar.get('enabled', True):
            continue
            
        topic = lidar['topic']
        display_config = {
            'Class': 'rviz_default_plugins/PointCloud2',
            'Name': f"Lidar: {lidar['name']}",
            'Enabled': True,
            'Topic': {
                'Value': topic,
                'Depth': 5,
                'Durability Policy': 'Volatile',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Best Effort'  # Changed to Best Effort for point clouds
            },
            'Position Transformer': 'XYZ',
            'Shape': {'Value': 'Points'},
            'Size (m)': 0.009999999776482582,
            'Style': 'Points'
        }
        displays.append(display_config)
    
    # 处理摄像头
    cameras = sensors_config.get('cameras', [])
    for cam in cameras:
        if not cam.get('enabled', True):
            continue
            
        topic = cam['topic']
        display_config = {
            'Class': 'rviz_default_plugins/Image',
            'Name': f"Camera: {cam['name']}",
            'Enabled': True,
            'Topic': {
                'Value': topic,
                'Depth': 5,
                'Durability Policy': 'Volatile',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Best Effort'  # Changed to Best Effort for images
            }
        }
        displays.append(display_config)
    
    # 处理IMU
    imus = sensors_config.get('imus', [])
    for imu in imus:
        if not imu.get('enabled', True):
            continue
            
        topic = imu['topic']
        display_config = {
            'Class': 'rviz_default_plugins/Imu',
            'Name': f"IMU: {imu['name']}",
            'Enabled': True,
            'Topic': {
                'Value': topic,
                'Depth': 5,
                'Durability Policy': 'Volatile',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Best Effort'
            },
            'Box Color': '255; 0; 0',
            'Box Size': 0.30000001192092896,
            'Queue Size': 10,
            'Style': 'Axes'
        }
        displays.append(display_config)
    
    # 处理GPS
    gps_sensors = sensors_config.get('gps_sensors', [])
    for gps in gps_sensors:
        if not gps.get('enabled', True):
            continue
            
        # GPS数据显示配置
        display_config = {
            'Class': 'rviz_default_plugins/Axes',
            'Name': f"GPS: {gps['name']}",
            'Enabled': True,
            'Shaft Length': 1.0,
            'Head Radius': 0.5,
            'Head Length': 0.5,
            'Shaft Radius': 0.25
        }
        displays.append(display_config)
    
    # 保存RViz配置到会话目录
    rviz_config_path = os.path.join(session_dir, "session.rviz")
    with open(rviz_config_path, 'w') as f:
        yaml.dump(rviz_config, f, default_flow_style=False, indent=2)
    
    return rviz_config_path


def generate_obstacles(config_data, session_dir):
    """
    根据配置生成障碍物布局
    
    Args:
        config_data (dict): 配置数据
        session_dir (str): 会话目录
        
    Returns:
        str: 生成的障碍物布局文件路径
    """
    obstacles_config = config_data.get('obstacles', {})
    random_areas = obstacles_config.get('random_areas', [])
    fixed_list = obstacles_config.get('fixed_list', [])
    
    all_obstacles = []
    
    # 处理固定障碍物
    for obs in fixed_list:
        all_obstacles.append({
            'name': obs['name'],
            'type': obs['type'],
            'pose': obs['pose'],  # [x, y, z, roll, pitch, yaw]
            'size': obs['size'],
            'color': obs['color']
        })
    
    # 处理随机区域
    for area in random_areas:
        center_x, center_y = area['center'][0], area['center'][1]
        radius = area['radius']
        count = area['count']
        obs_type = area['type']
        size_range = area['size_range']
        color = area['color']
        
        for i in range(count):
            # 使用极坐标随机生成算法确保均匀分布
            r = radius * math.sqrt(random.random())
            theta = random.random() * 2 * math.pi
            x = center_x + r * math.cos(theta)
            y = center_y + r * math.sin(theta)
            
            # 避免生成在原点附近（保护区，防止生成在船身上）
            if abs(x) < 2.0 and abs(y) < 2.0:
                # 如果生成在保护区内，重新生成
                continue_attempts = 0
                while abs(x) < 2.0 and abs(y) < 2.0 and continue_attempts < 10:
                    r = radius * math.sqrt(random.random())
                    theta = random.random() * 2 * math.pi
                    x = center_x + r * math.cos(theta)
                    y = center_y + r * math.sin(theta)
                    continue_attempts += 1
                    
            # 如果还是在保护区内，跳过这次生成
            if abs(x) < 2.0 and abs(y) < 2.0:
                continue
            
            # 随机选择大小
            size_val = random.uniform(size_range[0], size_range[1])
            height = random.uniform(size_range[0]*2, size_range[1]*2)  # 高度稍大一点
            
            obstacle = {
                'name': f"{area['name']}_{obs_type}_{i}",
                'type': obs_type,
                'pose': [x, y, 0.0, 0.0, 0.0, 0.0],  # z设为0保证在水面上
                'size': [size_val, height],
                'color': color
            }
            all_obstacles.append(obstacle)
    
    # 保存障碍物布局到会话目录
    obstacle_layout_path = os.path.join(session_dir, "obstacle_layout.json")
    with open(obstacle_layout_path, 'w') as f:
        json.dump(all_obstacles, f, indent=2)
    
    return obstacle_layout_path


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Session Manager for USV Simulation')
    parser.add_argument('--config-path', type=str, required=True, help='Path to user config file')
    
    args = parser.parse_args()
    
    result = create_session(args.config_path)
    
    # 输出JSON格式的结果，供launch文件读取
    print(json.dumps(result))


if __name__ == "__main__":
    main()