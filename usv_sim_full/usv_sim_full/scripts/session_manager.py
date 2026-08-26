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
import copy
import yaml
import json
import shutil
from datetime import datetime
import subprocess
from ament_index_python.packages import get_package_share_directory
import math
import random
import re

# full_config 多船：顶层 robot_1, robot_2, ...（数字升序）；否则回退到单块 robot:
ROBOT_SLOT_KEY_RE = re.compile(r'^robot_(\d+)$')


def sanitize_robot_model_name(name):
    return re.sub(r'[^A-Za-z0-9_\-]', '_', str(name))


def iter_robot_slots(config_data):
    """返回 [(slot_key, robot_block), ...]，按 robot_N 数字排序。"""
    slots = []
    for key in config_data:
        m = ROBOT_SLOT_KEY_RE.match(key)
        if m and isinstance(config_data[key], dict):
            slots.append((int(m.group(1)), key, config_data[key]))
    slots.sort(key=lambda x: x[0])
    if slots:
        return [(key, block) for _, key, block in slots]
    rb = config_data.get('robot')
    if isinstance(rb, dict) and rb:
        return [('robot', rb)]
    return []


def build_effective_robot_config(base_config, robot_block):
    """构造与现有 generate_* / compile_* 兼容的单船 config（含顶层 sensors）。"""
    eff = {}
    for k, v in base_config.items():
        if ROBOT_SLOT_KEY_RE.match(k):
            continue
        if k == 'robot':
            continue
        eff[k] = copy.deepcopy(v)
    eff['robot'] = copy.deepcopy(robot_block)
    sensors = robot_block.get('sensors')
    if sensors is None:
        sensors = base_config.get('sensors', [])
    if not sensors and isinstance(base_config.get('robot'), dict):
        sensors = base_config['robot'].get('sensors')
    if sensors is None:
        sensors = []
    eff['sensors'] = copy.deepcopy(sensors)
    return eff


def list_effective_robot_configs(config_data):
    """[(slot_key, sanitized_name, effective_config), ...]"""
    out = []
    for slot_key, block in iter_robot_slots(config_data):
        eff = build_effective_robot_config(config_data, block)
        rn = eff['robot'].get('name', 'wamv')
        san = sanitize_robot_model_name(rn)
        out.append((slot_key, san, eff))
    return out


def resolve_ros_package_root():
    """ROS 包根目录（含 package.xml、config/、description/、logs/）。"""
    inner_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.dirname(inner_dir)


def resolve_package_asset(relative_path):
    """Resolve a package asset path in both source-tree and install layouts."""
    # 本脚本位于 <pkg>/usv_sim_full/scripts/；URDF 等在 <pkg>/description/（与 inner 包目录并列）
    inner_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pkg_root = resolve_ros_package_root()
    for base in (pkg_root, inner_dir):
        source_candidate = os.path.join(base, relative_path)
        if os.path.exists(source_candidate):
            return source_candidate

    try:
        pkg_share = get_package_share_directory('usv_sim_full')
        share_candidate = os.path.join(pkg_share, relative_path)
        if os.path.exists(share_candidate):
            return share_candidate
    except Exception:
        pass

    raise FileNotFoundError(f"Package asset not found: {relative_path}")


def resolve_xacro_executable():
    """
    解析 xacro 可执行文件路径。

    必须使用「直接调用 xacro」的方式，而不要用 ``ros2 run xacro xacro``：
    后者会按 xacro 包自身依赖重建环境，AMENT_PREFIX_PATH 中往往没有
    wamv_gazebo 等工作区包，导致模板里 ``$(find wamv_gazebo)`` 报错。
    """
    exe = shutil.which('xacro')
    if exe:
        return exe
    for entry in (os.environ.get('AMENT_PREFIX_PATH') or '').split(os.pathsep):
        entry = entry.strip()
        if not entry:
            continue
        cand = os.path.normpath(os.path.join(entry, '..', 'bin', 'xacro'))
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    for distro in ('humble', 'jazzy', 'iron'):
        fallback = f'/opt/ros/{distro}/bin/xacro'
        if os.path.isfile(fallback) and os.access(fallback, os.X_OK):
            return fallback
    return None


def resolve_sensor_config_path(config_data, user_config_path):
    """Resolve sensor internal-parameter YAML path.

    Priority:
    1) full_config top-level `sensor_config_path`
    2) default package config `config/sensor_config.yaml`
    """
    package_dir = resolve_ros_package_root()
    candidate = config_data.get('sensor_config_path', 'config/sensor_config.yaml')

    if os.path.isabs(candidate):
        resolved = candidate
    else:
        config_dir = os.path.dirname(os.path.abspath(user_config_path))
        local_candidate = os.path.join(config_dir, candidate)
        if os.path.exists(local_candidate):
            resolved = local_candidate
        else:
            # Source-tree fallback
            source_candidate = os.path.join(package_dir, candidate)
            if os.path.exists(source_candidate):
                resolved = source_candidate
            else:
                # Installed-package fallback (share/<pkg>/...)
                try:
                    pkg_share = get_package_share_directory('usv_sim_full')
                    resolved = os.path.join(pkg_share, candidate)
                except Exception:
                    resolved = source_candidate

    if not os.path.exists(resolved):
        raise FileNotFoundError(f"Sensor config file not found: {resolved}")

    return resolved


def generate_session_sensor_params_xacro(config_data, user_config_path, session_dir):
    """Create session-local sensor parameter files for deterministic runs.

    Returns:
        tuple(str, str): (session_sensor_params_xacro_path, session_sensor_config_yaml_path)
    """
    source_sensor_cfg = resolve_sensor_config_path(config_data, user_config_path)
    session_sensor_cfg = os.path.join(session_dir, 'sensor_config.yaml')
    shutil.copy2(source_sensor_cfg, session_sensor_cfg)

    template_path = resolve_package_asset(os.path.join('description', 'urdf', 'sensor_params.xacro'))
    with open(template_path, 'r') as f:
        sensor_params_tpl = f.read()

    session_sensor_params_xacro = os.path.join(session_dir, 'sensor_params.xacro')
    sensor_params_session = sensor_params_tpl.replace(
        "$(find usv_sim_full)/config/sensor_config.yaml",
        session_sensor_cfg
    )
    with open(session_sensor_params_xacro, 'w') as f:
        f.write(sensor_params_session)

    return session_sensor_params_xacro, session_sensor_cfg


def create_session(config_path, verbose=False):
    """
    创建一个新的会话，包括日志目录和配置快照
    
    Args:
        config_path (str): 用户配置文件路径
        verbose (bool): 为 True 时在 stdout 打印 URDF 生成等诊断信息（JSON 前）
        
    Returns:
        dict: 包含session_path、urdf_path、bridge_yaml_path和rviz_config_path的字典
    """
    # 获取当前时间戳
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 会话日志写入 ROS 包根 logs/（与 package.xml 同级，非内层 Python 包目录）
    package_dir = resolve_ros_package_root()

    session_dir = os.path.join(package_dir, "logs", f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)
    
    # 复制用户配置到会话目录
    source_config_path = os.path.join(session_dir, "source_config.yaml")
    shutil.copy2(config_path, source_config_path)
    
    # 读取配置文件
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)

    robot_cfgs = list_effective_robot_configs(config_data)
    if not robot_cfgs:
        raise ValueError(
            "full_config 中未找到任何机器人：请配置 robot_1 / robot_2 / ... 或单块 robot:"
        )

    # 生成会话级 sensor 参数文件（内部参数来自 sensor_config，全船共用一份）
    session_sensor_params_xacro, session_sensor_cfg = generate_session_sensor_params_xacro(
        config_data,
        config_path,
        session_dir
    )

    robots_out = []
    multi = len(robot_cfgs) > 1
    for _slot_key, sanitized, eff in robot_cfgs:
        if multi:
            overlay_fn = f'generated_sensors_{sanitized}.xacro'
            urdf_fn = f'final_robot_{sanitized}.urdf'
            bridge_fn = f'bridge_config_{sanitized}.yaml'
        else:
            overlay_fn = 'generated_sensors.xacro'
            urdf_fn = 'final_robot.urdf'
            bridge_fn = 'bridge_config.yaml'
        generate_sensors_overlay(
            eff,
            session_dir,
            session_sensor_params_xacro,
            overlay_filename=overlay_fn,
        )
        urdf_path_one = compile_xacro_to_urdf(
            None,
            eff,
            session_dir,
            urdf_filename=urdf_fn,
            sensors_overlay_filename=overlay_fn,
            multi_vehicle_session=multi,
            verbose=verbose,
        )
        bridge_config_one = generate_bridge_config(eff)
        bridge_yaml_one = os.path.join(session_dir, bridge_fn)
        with open(bridge_yaml_one, 'w') as f:
            yaml.dump(bridge_config_one, f)
        spawn_pose = eff['robot'].get(
            'spawn_pose', [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
        )
        robots_out.append({
            'name': eff['robot'].get('name', 'wamv'),
            'sanitized_name': sanitized,
            'urdf_path': urdf_path_one,
            'bridge_yaml_path': bridge_yaml_one,
            'spawn_pose': spawn_pose,
        })

    # RViz：仅针对第一艘船生成（避免重复 Display）；多船可在 RViz 中手动加 topic
    _sk0, _san0, first_eff = robot_cfgs[0]
    rviz_config_path = generate_rviz_config(first_eff, session_dir)

    # 障碍物与场景共用根配置
    obstacle_layout_path = generate_obstacles(config_data, session_dir)

    # 向后兼容：urdf_path / bridge_yaml_path 指向第一艘船
    result = {
        'session_path': session_dir,
        'urdf_path': robots_out[0]['urdf_path'],
        'bridge_yaml_path': robots_out[0]['bridge_yaml_path'],
        'rviz_config_path': rviz_config_path,
        'obstacle_layout_path': obstacle_layout_path,
        'sensor_config_path': session_sensor_cfg,
        'sensor_params_xacro_path': session_sensor_params_xacro,
        'robots': robots_out,
    }

    return result


def generate_sensors_overlay(
    config_data,
    session_dir,
    session_sensor_params_xacro,
    overlay_filename='generated_sensors.xacro',
):
    """
    根据传感器配置生成传感器叠加层Xacro文件

    多船 / 命名空间：各宏的 name 参数统一为「$(arg namespace)/{sensor['name']}」
    （namespace 来自 compile 时的 robot.name）。由此：
    - 典型 body link：{namespace}/base_link
    - 典型传感器 link / Gazebo gz_frame_id / 毫米波 PointCloud2.frame_id：{namespace}/{sensor_name}_link
    海事雷达额外使用 {namespace}/{sensor_name}_base_link 与 {namespace}/{sensor_name}_antenna_link。
    同一进程多实例时须使用不同 robot.name，否则 link 名与话题都会冲突。

    Args:
        config_data (dict): 配置数据
        session_dir (str): 会话目录

    Returns:
        str: 生成的传感器叠加层Xacro文件路径
    """
    sensors_overlay_path = os.path.join(session_dir, overlay_filename)
    
    # 读取模板文件
    template_path = resolve_package_asset(os.path.join('description', 'urdf', 'sensor_macros.xacro'))
    
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
    # 添加对参数文件的引用
    sensors_content = '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
    sensors_content += f'  <xacro:include filename="{session_sensor_params_xacro}"/>\n\n'
    
    # 添加提取的宏定义
    for line in macros_content:
        sensors_content += line + '\n'
    
    sensors_content += "\n"
    
    # 添加传感器链接
    sensors_input = config_data.get('sensors', [])
    sensors_list = []
    
    # 兼容旧字典格式
    if isinstance(sensors_input, dict):
        for k, v in sensors_input.items():
            if isinstance(v, list):
                for item in v:
                    if 'type' not in item:
                        if k == 'lidars': item['type'] = 'lidar'
                        elif k == 'cameras': item['type'] = 'camera'
                        elif k == 'imus': item['type'] = 'imu'
                        elif k == 'gps_sensors': item['type'] = 'gps'
                        elif k == 'magnetometers': item['type'] = 'magnetometer'
                        elif k == 'barometers': item['type'] = 'barometer'
                        elif k == 'radars': item['type'] = 'maritime_radar'
                    sensors_list.append(item)
    elif isinstance(sensors_input, list):
        sensors_list = sensors_input

    for sensor in sensors_list:
        if not sensor.get('enabled', True):
            continue
            
        sensor_type = str(sensor.get('type', '')).lower()
        sensor_name = sensor.get('name', 'sensor')
        parent_link = sensor.get('parent_link', 'base_link')
        xyz = ' '.join(map(str, sensor.get('xyz', [0,0,0])))
        rpy = ' '.join(map(str, sensor.get('rpy', [0,0,0])))
        
        if sensor_type == 'lidar' or sensor_type == 'vlp16':
            topic = sensor.get('override_topic') or f'/sensors/lidar/{sensor_name}/points'
            topic_ns = topic.lstrip('/')
            sensors_content += f'''
        <xacro:vlp16_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}"/>
    '''
        elif sensor_type == 'camera':
            topic = sensor.get('override_topic') or f'/sensors/camera/{sensor_name}/image_raw'
            info_topic = topic.replace('image_raw', 'camera_info')
            topic_ns = topic.lstrip('/')
            info_topic_ns = info_topic.lstrip('/')
            sensors_content += f'''
    <xacro:camera_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}" info_topic="$(arg namespace)/{info_topic_ns}"/>
    '''
        elif sensor_type == 'gimbal':
            image_topic = sensor.get('image_topic') or f'/sensors/camera/{sensor_name}/image_raw'
            imu_topic = sensor.get('imu_topic') or f'/sensors/imu/{sensor_name}/data'
            image_topic_ns = image_topic.lstrip('/')
            imu_topic_ns = imu_topic.lstrip('/')
            sensors_content += f'''
    <xacro:gimbal_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" command_prefix="command/{sensor_name}" image_topic="/$(arg namespace)/{image_topic_ns}" imu_topic="/$(arg namespace)/{imu_topic_ns}"/>
    '''
        elif sensor_type == 'imu':
            topic = sensor.get('override_topic') or '/sensors/imu/data'
            update_rate = sensor.get('update_rate', 100)
            topic_ns = topic.lstrip('/')
            sensors_content += f'''
    <xacro:imu_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}" update_rate="{update_rate}"/>
'''
        elif sensor_type == 'gps':
            topic = sensor.get('override_topic') or '/sensors/gps/data'
            update_rate = sensor.get('update_rate', 20)
            topic_ns = topic.lstrip('/')
            sensors_content += f'''
    <xacro:gps_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}" update_rate="{update_rate}"/>
'''
        elif sensor_type in ['magnetometer', 'magnetic_field', 'mag']:
            topic = sensor.get('override_topic') or f'/sensors/magnetometer/{sensor_name}/data'
            update_rate = sensor.get('update_rate', 50)
            topic_ns = topic.lstrip('/')
            sensors_content += f'''
    <xacro:magnetometer_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}" update_rate="{update_rate}"/>
'''
        elif sensor_type in ['barometer', 'air_pressure', 'pressure']:
            topic = sensor.get('override_topic') or f'/sensors/barometer/{sensor_name}/fluid_pressure'
            update_rate = sensor.get('update_rate', 20)
            topic_ns = topic.lstrip('/')
            sensors_content += f'''
    <xacro:barometer_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}" update_rate="{update_rate}"/>
'''
        elif sensor_type in ['maritime_radar', 'radar']:
            update_rate = sensor.get('update_rate', 48)
            sensors_content += f'''
    <xacro:maritime_radar_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" rpm="{update_rate}" radar_topic="/$(arg namespace)/{sensor_name}/spokes"/>'''
        elif sensor_type in ['mmwave_radar', 'mmwave']:
            topic = sensor.get('override_topic') or f'/sensors/mmwave/{sensor_name}/points'
            topic_ns = topic.lstrip('/')
            sensors_content += f'''
    <xacro:mmwave_radar_macro name="$(arg namespace)/{sensor_name}" parent_link="$(arg namespace)/{parent_link}" xyz="{xyz}" rpy="{rpy}" topic="$(arg namespace)/{topic_ns}"/>'''

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


def compile_xacro_to_urdf(
    root_xacro_path,
    config_data,
    session_dir,
    urdf_filename='final_robot.urdf',
    sensors_overlay_filename='generated_sensors.xacro',
    multi_vehicle_session=False,
    verbose=False,
):
    """
    使用 xacro 可执行文件编译最终 URDF（继承当前环境，以便解析 ``$(find ...)``）。
    支持选择不同模板（如无电池版）和推进器配置，以及传感器配置。

    Args:
        root_xacro_path (str): 已废弃，不使用
        config_data (dict): 配置数据
        session_dir (str): 会话目录
        
    Returns:
        str: 生成的URDF文件路径
    """
    urdf_path = os.path.join(session_dir, urdf_filename)
    robot_config = config_data.get('robot', {})
    
    # 获取要使用的xacro模板名称
    xacro_template = robot_config.get('xacro_template', 'wamv_gazebo.urdf.xacro')
    
    # 判断是否使用本地模板（例如无电池版本）
    if xacro_template == 'wamv_no_battery.urdf.xacro':
        # 使用本地模板文件
        local_template_path = resolve_package_asset(
            os.path.join('description', 'urdf', 'wamv_no_battery.urdf.xacro')
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
                raise FileNotFoundError(f"Could not find package {package_name} or xacro file {xacro_relative_path}")

    # 构建xacro命令
    # 使用配置中的 robot name 传入 xacro（默认 'wamv'）
    robot_name_for_xacro = config_data.get('robot', {}).get('name', 'wamv')
    # 直接传入不带外部引号的值（xacro会作为字符串处理）
    if isinstance(robot_name_for_xacro, str):
        robot_name_arg = f'name:={robot_name_for_xacro}'
    else:
        robot_name_arg = f'name:={robot_name_for_xacro}'

    # 同时传入 namespace 参数给 xacro，使用一个安全的 name 作为 namespace（替换不安全字符）
    sanitized_namespace = re.sub(r"[^A-Za-z0-9_\-]", '_', str(robot_name_for_xacro))
    # Avoid embedding extra quote characters that would become part of the value
    namespace_arg = f'namespace:={sanitized_namespace}'

    xacro_exe = resolve_xacro_executable()
    if not xacro_exe:
        raise RuntimeError(
            '找不到 xacro 可执行文件。请先 source ROS（如 source /opt/ros/humble/setup.bash）'
            '与工作区 install/setup.bash，并确保 PATH 含 $AMENT_PREFIX_PATH/../bin。'
        )

    cmd = [
        xacro_exe,
        xacro_input,
        robot_name_arg,
        namespace_arg
    ]

    # 多船时关闭 VRX DetachableJoint（否则多实例争抢 platform，常见现象为 Gazebo 仅一条船）
    if xacro_template == 'wamv_no_battery.urdf.xacro' and multi_vehicle_session:
        cmd.append('enable_detachable_joint:=false')

    # 添加物理参数覆盖
    overrides = robot_config.get('overrides', {})
    for key, value in overrides.items():
        if key == 'inertia' and isinstance(value, list):
            cmd.extend([
                f'ixx:={value[0]}',
                f'iyy:={value[1]}',
                f'izz:={value[2]}'
            ])
        elif isinstance(value, bool):
            cmd.append(f'{key}:={str(value).lower()}')
        elif isinstance(value, (int, float)):
            cmd.append(f'{key}:={value}')
        elif isinstance(value, str):
            cmd.append(f'{key}:={value}')

    # 添加浮力和水动力参数覆盖
    buoyancy_params = robot_config.get('buoyancy_params', {})
    for key, value in buoyancy_params.items():
        if isinstance(value, (int, float)):
            cmd.append(f'{key}:={value}')
        elif isinstance(value, str):
            cmd.append(f'{key}:={value}')

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
    sensors_overlay_path = os.path.join(session_dir, sensors_overlay_filename)
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
    post_process_urdf_for_gazebo(urdf_path, verbose=verbose)

    if verbose:
        print(f"Generated URDF at: {urdf_path}")
    return urdf_path


def post_process_urdf_for_gazebo(urdf_path, verbose=False):
    """
    后处理URDF文件，将package://协议的网格路径替换为正确的model://协议路径
    这样Gazebo就能正确找到模型文件
    
    Args:
        urdf_path (str): URDF文件路径
    """
    # 读取URDF文件
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()
    
    # 替换package://wamv_description/models/ 以及 package://usv_sim_full/description/models/ 为model://
    # 这样Gazebo就能在模型路径中找到文件
    # 例如: package://usv_sim_full/description/models/WAM-V-Base/mesh/M5_body.dae
    # 变为: model://WAM-V-Base/mesh/M5_body.dae
    modified_content = urdf_content.replace(
        'package://wamv_description/models/', 
        'model://'
    )
    modified_content = modified_content.replace(
        'package://usv_sim_full/description/models/', 
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
    
    if verbose:
        print(f"Post-processed URDF for Gazebo compatibility")


def generate_bridge_config(config_data):
    """
    根据传感器配置生成桥接配置
    
    Args:
        config_data (dict): 配置数据
        
    Returns:
        list: 桥接配置（YAML序列格式）
    """
    # 桥接配置必须是列表格式。
    # 注意：/clock 由 infra_sim.launch.py 中的 global_bridge 统一桥接，
    # 这里不要重复添加，避免双源时钟造成时间回跳和 TF buffer 清空。
    bridges = []
    # Determine namespace/robot name for topics
    robot_name_for_bridge = config_data.get('robot', {}).get('name', 'wamv')
    sanitized_bridge_ns = re.sub(r"[^A-Za-z0-9_\-]", '_', str(robot_name_for_bridge))
    world_name = config_data.get('environment', {}).get(
        'world_name', 'sydney_regatta'
    )
    
    # 尝试导入 topics 模块获取模板
    try:
        from usv_interfaces import topics as topics_mod
    except ImportError:
        topics_mod = None

    # 添加遥测配置
    if config_data.get('visualization', {}).get('enable_telemetry', True):
        # 添加里程计桥接（命名空间化）
        bridges.append({
            "ros_topic_name": f"/{sanitized_bridge_ns}/odom",
            "gz_topic_name": f"/model/{sanitized_bridge_ns}/odometry",
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS"
        })
        
        # 添加关节状态桥接（命名空间化）
        bridges.append({
            "ros_topic_name": f"/{sanitized_bridge_ns}/joint_states",
            "gz_topic_name": (
                f"/world/{world_name}/model/{sanitized_bridge_ns}/joint_state"
            ),
            "ros_type_name": "sensor_msgs/msg/JointState",
            "gz_type_name": "gz.msgs.Model",
            "direction": "GZ_TO_ROS"
        })
        
        # 添加位姿桥接（用于TF，命名空间化）
        bridges.append({
            "ros_topic_name": f"/model/{sanitized_bridge_ns}/pose",
            "gz_topic_name": f"/model/{sanitized_bridge_ns}/pose",
            "ros_type_name": "tf2_msgs/msg/TFMessage",
            "gz_type_name": "gz.msgs.Pose_V",
            "direction": "GZ_TO_ROS"
        })

    # 添加推进器控制桥接配置（关键修复！）
    thrusters_config = config_data.get('robot', {}).get('thrusters', {})
    if thrusters_config.get('enabled', True):
        # 默认仅桥接当前机器人命名空间，避免额外暴露历史 /wamv 话题。
        namespaces = [sanitized_bridge_ns]
        if thrusters_config.get('enable_legacy_wamv_alias', False):
            namespaces.append('wamv')
        
        for ns in namespaces:
            # 左推进器推力控制
            bridges.append({
                "ros_topic_name": f"/{ns}/thrusters/left/thrust",
                "gz_topic_name": f"/{sanitized_bridge_ns}/thrusters/left/thrust",
                "ros_type_name": "std_msgs/msg/Float64",
                "gz_type_name": "gz.msgs.Double",
                "direction": "ROS_TO_GZ"
            })
            
            # 左推进器角度控制
            bridges.append({
                "ros_topic_name": f"/{ns}/thrusters/left/pos",
                "gz_topic_name": f"/{sanitized_bridge_ns}/thrusters/left/pos",
                "ros_type_name": "std_msgs/msg/Float64",
                "gz_type_name": "gz.msgs.Double",
                "direction": "ROS_TO_GZ"
            })
            
            # 右推进器推力控制
            bridges.append({
                "ros_topic_name": f"/{ns}/thrusters/right/thrust",
                "gz_topic_name": f"/{sanitized_bridge_ns}/thrusters/right/thrust",
                "ros_type_name": "std_msgs/msg/Float64",
                "gz_type_name": "gz.msgs.Double",
                "direction": "ROS_TO_GZ"
            })
            
            # 右推进器角度控制
            bridges.append({
                "ros_topic_name": f"/{ns}/thrusters/right/pos",
                "gz_topic_name": f"/{sanitized_bridge_ns}/thrusters/right/pos",
                "ros_type_name": "std_msgs/msg/Float64",
                "gz_type_name": "gz.msgs.Double",
                "direction": "ROS_TO_GZ"
            })

    # 处理传感器桥接
    sensors_input = config_data.get('sensors', [])
    sensors_list = []
    
    # 兼容旧字典格式
    if isinstance(sensors_input, dict):
        for k, v in sensors_input.items():
            if isinstance(v, list):
                for item in v:
                    if 'type' not in item:
                        if k == 'lidars': item['type'] = 'lidar'
                        elif k == 'cameras': item['type'] = 'camera'
                        elif k == 'imus': item['type'] = 'imu'
                        elif k == 'gps_sensors': item['type'] = 'gps'
                        elif k == 'magnetometers': item['type'] = 'magnetometer'
                        elif k == 'barometers': item['type'] = 'barometer'
                        elif k == 'radars': item['type'] = 'maritime_radar'
                    sensors_list.append(item)
    elif isinstance(sensors_input, list):
        sensors_list = sensors_input
        
    # 类型映射定义 (ros_type_name, gz_type_name)
    type_mappings = {
        'LIDAR': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', '/points'),
        'VLP16': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', '/points'),
        'MMWAVE_RADAR': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', '/points'),
        'MMWAVE': ('sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', '/points'),
        'CAMERA': ('sensor_msgs/msg/Image', 'gz.msgs.Image', ''),
        'GPS': ('sensor_msgs/msg/NavSatFix', 'gz.msgs.NavSat', ''),
        'IMU': ('sensor_msgs/msg/Imu', 'gz.msgs.IMU', ''),
        'MAGNETOMETER': ('sensor_msgs/msg/MagneticField', 'gz.msgs.Magnetometer', ''),
        'MAGNETIC_FIELD': ('sensor_msgs/msg/MagneticField', 'gz.msgs.Magnetometer', ''),
        'MAG': ('sensor_msgs/msg/MagneticField', 'gz.msgs.Magnetometer', ''),
        'BAROMETER': ('sensor_msgs/msg/FluidPressure', 'gz.msgs.FluidPressure', ''),
        'AIR_PRESSURE': ('sensor_msgs/msg/FluidPressure', 'gz.msgs.FluidPressure', ''),
        'PRESSURE': ('sensor_msgs/msg/FluidPressure', 'gz.msgs.FluidPressure', ''),
        'MARITIME_RADAR': ('marine_sensor_msgs/msg/RadarSector', 'gz.msgs.RadarSector', '')
    }

    # 毫米波：gpu_ray → bridge 到 .../points_gz，再由 usv_mmwave_sim::mmwave_4d_cloud_node 发布最终 .../points
    no_bridge_sensor_types = {
        'MARITIME_RADAR',
        'RADAR',
    }

    # Camera Info 特殊处理
    camera_info_mapping = ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', '/camera_info')

    for sensor in sensors_list:
        if not sensor.get('enabled', True):
            continue
            
        sensor_name = sensor.get('name', 'sensor')
        sensor_type = str(sensor.get('type', '')).upper()

        if sensor_type == 'GIMBAL':
            image_topic = sensor.get('image_topic') or f'/sensors/camera/{sensor_name}/image_raw'
            imu_topic = sensor.get('imu_topic') or f'/sensors/imu/{sensor_name}/data'
            for axis in ('roll', 'pitch', 'yaw'):
                bridges.append({
                    "ros_topic_name": f"/{sanitized_bridge_ns}/gimbal/{sensor_name}/{axis}/cmd_pos",
                    "gz_topic_name": f"/model/{sanitized_bridge_ns}/command/{sensor_name}/{axis}",
                    "ros_type_name": "std_msgs/msg/Float64",
                    "gz_type_name": "gz.msgs.Double",
                    "direction": "ROS_TO_GZ"
                })
            for ros_topic, ros_type, gz_type in (
                (image_topic, "sensor_msgs/msg/Image", "gz.msgs.Image"),
                (image_topic.replace('image_raw', 'camera_info'), "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
                (imu_topic, "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
            ):
                bridges.append({
                    "ros_topic_name": f"/{sanitized_bridge_ns}{ros_topic}",
                    "gz_topic_name": f"/{sanitized_bridge_ns}{ros_topic}",
                    "ros_type_name": ros_type,
                    "gz_type_name": gz_type,
                    "direction": "GZ_TO_ROS"
                })
            continue

        if sensor_type in no_bridge_sensor_types:
            continue
        
        # 1. 尝试从 usv_interfaces 的常量获取
        template_var = f"TEMPLATE_{sensor_type}"
        exact_topic_var = f"TOPIC_SENSOR_{sensor_type}"
        
        ros_topic = None
        
        try:
            # 优先级1：YAML 配置文件中的 override_topic (最高优先级)
            if 'override_topic' in sensor and sensor['override_topic']:
                ros_topic = sensor['override_topic']
            # 优先级2：usv_interfaces 中的模板常量（推荐方式）
            elif topics_mod and hasattr(topics_mod, template_var):
                topic_template = getattr(topics_mod, template_var)
                ros_topic = topic_template.format(sensor_name=sensor_name)
            # 优先级3：usv_interfaces 中的硬编码常量（向后兼容）
            elif topics_mod and hasattr(topics_mod, exact_topic_var):
                ros_topic = getattr(topics_mod, exact_topic_var)
        except Exception as e:
            print(f"Error accessing topic constants from usv_interfaces: {e}")
            
        # 优先级4：全部找不到时的默认 fallback
        if not ros_topic:
            ros_topic = f"/sensors/{sensor_type.lower()}/{sensor_name}/data"
            
        # 自动补全斜杠前缀
        if not ros_topic.startswith('/'):
            ros_topic = '/' + ros_topic
            
        # 3. 推导 Gazebo 话题
        # 按照约定，Gazebo的话题为 /{sanitized_bridge_ns} + yaml中的路径
        mapped_info = type_mappings.get(sensor_type)
        if mapped_info:
            ros_type, gz_type, gz_suffix = mapped_info

            ros_topic_bridge = ros_topic
            if sensor_type in ('MMWAVE_RADAR', 'MMWAVE'):
                if ros_topic.endswith('/points'):
                    ros_topic_bridge = ros_topic[: -len('/points')] + '/points_gz'
                else:
                    ros_topic_bridge = ros_topic + '_gz'

            # Gazebo 话题路径通常与 ros_topic 对应，但也可能不同。
            # 通常对于基于ROS的话题配置，直接映射 gz_topic 为 /{namespace}{ros_topic} 
            # 加上特有的后缀（lidar / mmwave gpu_ray 均为 /points）
            if sensor_type in ('LIDAR', 'VLP16', 'MMWAVE_RADAR', 'MMWAVE'):
                gz_topic = f"/{sanitized_bridge_ns}{ros_topic}/points"
            else:
                gz_topic = f"/{sanitized_bridge_ns}{ros_topic}"

            ros_full = (
                f"/{sanitized_bridge_ns}{ros_topic_bridge}"
                if not ros_topic_bridge.startswith(f"/{sanitized_bridge_ns}")
                else ros_topic_bridge
            )
            bridges.append({
                "ros_topic_name": ros_full,
                "gz_topic_name": gz_topic,
                "ros_type_name": ros_type,
                "gz_type_name": gz_type,
                "direction": "GZ_TO_ROS"
            })
            
            # 对于 Camera 额外添加 CameraInfo
            if sensor_type == 'CAMERA':
                # 推导 info ros_topic
                if 'override_topic' in sensor and sensor['override_topic']:
                    ros_info_topic = sensor['override_topic'].replace('image_raw', 'camera_info')
                else:
                    ros_info_topic = ros_topic.rsplit('/', 1)[0] + '/camera_info'
                    
                gz_info_topic = f"/{sanitized_bridge_ns}{ros_info_topic}"
                
                bridges.append({
                    "ros_topic_name": f"/{sanitized_bridge_ns}{ros_info_topic}" if not ros_info_topic.startswith(f"/{sanitized_bridge_ns}") else ros_info_topic,
                    "gz_topic_name": gz_info_topic,
                    "ros_type_name": camera_info_mapping[0],
                    "gz_type_name": camera_info_mapping[1],
                    "direction": "GZ_TO_ROS"
                })

    import json
    # 保存生成的规则到临时的 bridge_generated.yaml
    bridge_generated_path = "/tmp/bridge_generated.yaml"
    try:
        with open(bridge_generated_path, 'w') as f:
            import yaml as yaml_lib
            yaml_lib.dump(bridges, f)
    except Exception as e:
        print(f"Failed to write bridge_generated.yaml: {e}")
        
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
    template_path = resolve_package_asset(os.path.join('description', 'rviz', 'default.rviz'))
    
    with open(template_path, 'r') as f:
        rviz_config = yaml.safe_load(f)

    # 规范化机器人命名空间（用于Fixed Frame和topic前缀）
    robot_config = config_data.get('robot', {})
    robot_name = robot_config.get('name', 'wamv')
    sanitized_namespace = re.sub(r"[^A-Za-z0-9_\-]", '_', str(robot_name))

    def prefix_namespace(topic_value, namespace):
        """确保传感器topic带有命名空间前缀。"""
        if not topic_value:
            return topic_value
        normalized = topic_value if topic_value.startswith('/') else f'/{topic_value}'
        ns_prefix = f'/{namespace}/'
        if normalized.startswith(ns_prefix):
            return normalized
        return f'/{namespace}{normalized}'
    
    # 与 Gazebo 世界/里程计一致：全局用 map，避免毫米波等「世界系点云 + 错误 frame_id」时在 base_link 下不可见
    global_options = rviz_config.get('Visualization Manager', {}).get('Global Options', {})
    global_options['Fixed Frame'] = "map"

    # 获取Display列表
    displays = rviz_config['Visualization Manager']['Displays']

    # 更新模板中已有的Odometry topic
    for display in displays:
        if display.get('Class') == 'rviz_default_plugins/Odometry':
            display_topic = display.get('Topic', {})
            display_topic['Value'] = f"/{sanitized_namespace}/odom"
            display['Topic'] = display_topic
    
    # 添加传感器显示项
    sensors_input = config_data.get('sensors', [])
    sensors_list = []
    
    if isinstance(sensors_input, dict):
        for k, v in sensors_input.items():
            if isinstance(v, list):
                for item in v:
                    if 'type' not in item:
                        if k == 'lidars': item['type'] = 'lidar'
                        elif k == 'cameras': item['type'] = 'camera'
                        elif k == 'imus': item['type'] = 'imu'
                        elif k == 'gps_sensors': item['type'] = 'gps'
                        elif k == 'magnetometers': item['type'] = 'magnetometer'
                        elif k == 'barometers': item['type'] = 'barometer'
                        elif k == 'radars': item['type'] = 'maritime_radar'
                    sensors_list.append(item)
    elif isinstance(sensors_input, list):
        sensors_list = sensors_input

    for sensor in sensors_list:
        if not sensor.get('enabled', True):
            continue
            
        sensor_type = str(sensor.get('type', '')).lower()
        sensor_name = sensor.get('name', 'sensor')
        ot = sensor.get('override_topic')
        if ot:
            topic_val = ot
        elif sensor_type in ('mmwave_radar', 'mmwave'):
            # 与 generate_sensors_overlay 中 mmwave 默认 topic 一致（无 override 时）
            topic_val = f'/sensors/mmwave/{sensor_name}/points'
        else:
            topic_val = f'/{sensor_name}/data'
        if not topic_val.startswith('/'):
            topic_val = '/' + topic_val
            
        topic = prefix_namespace(topic_val, sanitized_namespace)
        
        if sensor_type == 'lidar' or sensor_type == 'vlp16':
            display_config = {
                'Class': 'rviz_default_plugins/PointCloud2',
                'Name': f"Lidar: {sensor_name}",
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort'
                },
                'Position Transformer': 'XYZ',
                'Shape': {'Value': 'Points'},
                'Size (m)': 0.009999999776482582,
                'Style': 'Points'
            }
            displays.append(display_config)

        elif sensor_type == 'mmwave_radar' or sensor_type == 'mmwave':
            # 与 usv_mmwave_sim/rviz/4d_radar_minimal.rviz 一致：用 rcs 字段做 Intensity 彩虹着色
            display_config = {
                'Class': 'rviz_default_plugins/PointCloud2',
                'Name': f"mmWave: {sensor_name}",
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort'
                },
                'Position Transformer': 'XYZ',
                'Color Transformer': 'Intensity',
                'Channel Name': 'rcs',
                'Use rainbow': True,
                'Autocompute Intensity Bounds': True,
                'Invert Rainbow': False,
                'Shape': {'Value': 'Points'},
                'Size (m)': 0.02,
                'Style': 'Points'
            }
            displays.append(display_config)

        elif sensor_type == 'camera':
            display_config = {
                'Class': 'rviz_default_plugins/Image',
                'Name': f"Camera: {sensor_name}",
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'Durability Policy': 'Volatile',
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort'
                }
            }
            displays.append(display_config)
            
        elif sensor_type == 'imu':
            display_config = {
                'Class': 'rviz_imu_plugin/Imu',
                'Name': f"IMU: {sensor_name}",
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
            
        elif sensor_type == 'gps':
            display_config = {
                'Class': 'rviz_default_plugins/Axes',
                'Name': f"GPS: {sensor_name}",
                'Enabled': True,
                'Shaft Length': 1.0,
                'Head Radius': 0.5,
                'Head Length': 0.5,
                'Shaft Radius': 0.25
            }
            displays.append(display_config)

    # scenario.ground_truth_sim：CTRV 真值 Marker（Gazebo 另由 ground_truth_gazebo_models_node）
    displays.append({
        'Class': 'rviz_default_plugins/MarkerArray',
        'Enabled': True,
        'Name': 'Scenario CTRV targets (ground_truth_sim)',
        'Namespaces': {
            'target_pose': True,
            'target_path': True,
            'target_history': True,
        },
        'Topic': {
            'Value': '/sim/ground_truth_markers',
            'Depth': 10,
            'Durability Policy': 'Volatile',
            'Filter size': 10,
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Reliable',
        },
        'Queue Size': 10,
        'Value': True,
    })

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
        entry = {
            'name': obs['name'],
            'type': obs['type'],
            'pose': obs['pose'],  # [x, y, z, roll, pitch, yaw]
            'color': obs.get('color', 'Red'),
        }
        if 'size' in obs:
            entry['size'] = obs['size']
        all_obstacles.append(entry)
    
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
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Print URDF generation diagnostics to stdout before JSON (default: quiet)',
    )
    
    args = parser.parse_args()
    
    result = create_session(args.config_path, verbose=args.verbose)
    
    # 输出JSON格式的结果，供launch文件读取
    print(json.dumps(result))


if __name__ == "__main__":
    main()
