"""Sydney VRX with an opt-in model-2 planar vessel, direct thrust, PID or Nav2."""

import copy
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
from usv_sim_full.navigation_clock import set_navigation_clock


def _merge(base, override):
    """Merge a task-specific parameter profile.

    :param base: Base dictionary.
    :param override: Recursive override.
    :return: Merged dictionary.
    """
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def _setup(context):
    """Materialize a private world and parameters without modifying installed resources.

    :param context: Launch context.
    :return: Launch actions.
    """
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode')
    if mode not in ('direct', 'pid', 'navigation'):
        raise ValueError('mode must be direct, pid or navigation (one thrust writer)')
    share = Path(get_package_share_directory('usv_sim_full'))
    world = ET.parse(share / 'worlds/sydney_regatta.sdf')
    # Keep Sydney terrain, landmarks and water visuals. No environmental
    # force plugins are attached to the small boat.
    physics = world.find('world/physics')
    physics.find('max_step_size').text = '0.002'
    model = ET.parse(share / 'description/models/fossen_model2/model.sdf').find('model')
    for uri in model.findall('.//uri'):
        uri.text = uri.text.replace('package://usv_sim_full', str(share))
    if value('sensors').lower() != 'true':
        link = model.find('link')
        for sensor in list(link.findall('sensor')):
            link.remove(sensor)
    world.find('world').append(model)
    camera = world.find("world/gui/plugin[@name='3D View']/camera_pose")
    if camera is not None:
        camera.text = '5 -7 5 0 0.5 2.2'
    session = Path(tempfile.mkdtemp(prefix='fossen_model2_'))
    world_path = session / 'world.sdf'
    world.write(world_path, encoding='unicode', xml_declaration=True)
    actions = [ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', str(world_path)], output='screen')]
    if value('gui').lower() == 'true':
        actions.append(ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'))

    bridge = [
        {'ros_topic_name': '/clock', 'gz_topic_name': '/clock', 'ros_type_name': 'rosgraph_msgs/msg/Clock',
         'gz_type_name': 'gz.msgs.Clock', 'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': '/usv_1/odom', 'gz_topic_name': '/usv_1/odom', 'ros_type_name': 'nav_msgs/msg/Odometry',
         'gz_type_name': 'gz.msgs.Odometry', 'direction': 'GZ_TO_ROS'},
    ]
    for side in ('left', 'right'):
        topic = '/usv_1/thrusters/' + side + '/thrust'
        bridge.append({'ros_topic_name': topic, 'gz_topic_name': topic,
                       'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
                       'direction': 'ROS_TO_GZ'})
    if value('sensors').lower() == 'true':
        for topic, ros_type, gz_type in (
                ('/usv_1/sensors/lidar/front_lidar/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
                ('/usv_1/sensors/camera/front_cam/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image')):
            bridge.append({'ros_topic_name': topic, 'gz_topic_name': topic,
                           'ros_type_name': ros_type, 'gz_type_name': gz_type, 'direction': 'GZ_TO_ROS'})
    bridge_path = session / 'bridge.yaml'
    bridge_path.write_text(yaml.safe_dump(bridge))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        parameters=[{'config_file': str(bridge_path), 'use_sim_time': True}], output='screen'))
    actions.append(Node(package='usv_sim_full', executable='odom_tf_broadcaster',
                        parameters=[{'use_sim_time': True}], output='screen'))
    static = [('map', 'usv_1/odom', '0', '0', '0', '0', '0', '0'),
              ('usv_1/base_link', 'usv_1/front_lidar_link', '0', '0', '.65', '0', '0', '0'),
              ('usv_1/base_link', 'usv_1/front_cam_optical', '.35', '0', '.55', '-1.57079632679', '0', '-1.57079632679')]
    for index, (parent, child, x, y, z, roll, pitch, yaw) in enumerate(static):
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='fossen_static_' + str(index), parameters=[{'use_sim_time': True}],
                            arguments=['--x', x, '--y', y, '--z', z, '--roll', roll, '--pitch', pitch,
                                       '--yaw', yaw, '--frame-id', parent, '--child-frame-id', child]))

    control = value('control_params_file')
    if mode in ('pid', 'navigation'):
        actions.append(Node(package='usv_sim_full', executable='cmd_vel_to_thruster', namespace='usv_1',
                            parameters=[control, {'namespace': 'usv_1', 'use_sim_time': True}], output='screen'))
    if mode == 'navigation':
        params = yaml.safe_load((share / 'config/radar_nav2_param.yaml').read_text())
        for name in ('global_costmap', 'local_costmap'):
            p = params[name][name]['ros__parameters']
            p['plugins'] = (['static_layer'] if name == 'global_costmap' else ['obstacle_layer']) + ['inflation_layer']
            # An untyped empty YAML sequence is rejected by Humble's parser;
            # omit this override and use Costmap2DROS's empty default instead.
            p.pop('filters', None)
            p['footprint'] = '[[0.75,0.375],[0.75,-0.375],[-0.75,-0.375],[-0.75,0.375]]'
            p['footprint_padding'] = 0.05
            p['inflation_layer'].update(inflation_radius=1.0, cost_scaling_factor=3.0)
            p['obstacle_layer'] = {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'enabled': True,
                'observation_sources': 'scan', 'scan': {'topic': '/usv_1/sensors/lidar/front_lidar/scan',
                'data_type': 'LaserScan', 'marking': True, 'clearing': True, 'inf_is_valid': True,
                'obstacle_min_range': 0.2, 'obstacle_max_range': 50.0,
                'raytrace_min_range': 0.2, 'raytrace_max_range': 50.0,
                'min_obstacle_height': 0.0, 'max_obstacle_height': 3.0}}
        params['planner_server']['ros__parameters'] = {'planner_plugins': ['GridBased'],
            'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner', 'tolerance': 0.3,
                          'use_astar': True, 'allow_unknown': False}}
        bt = params['bt_navigator']['ros__parameters']
        bt.pop('default_nav_to_pose_bt_xml', None)
        bt['default_nav_through_poses_bt_xml'] = str(share / 'config/fossen_model2_through_poses.xml')
        control_data = yaml.safe_load(Path(control).read_text())
        _merge(params, {key: val for key, val in control_data.items() if key != '/**'})
        params = yaml.safe_load(yaml.safe_dump(params).replace('__ROBOT_NS__', 'usv_1'))
        set_navigation_clock(params, True)
        params_path = session / 'nav2.yaml'
        params_path.write_text(yaml.safe_dump(params))
        actions.extend([
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[{'yaml_filename': str(share / 'maps/sydney_map2.yaml'), 'use_sim_time': True}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}]),
            Node(package='usv_sim_full', executable='tf_namespace_relay',
                 parameters=[{'namespace': 'usv_1', 'use_sim_time': True}]),
            GroupAction([
                PushRosNamespace('usv_1'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                    Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py')),
                    launch_arguments={'namespace': 'usv_1', 'params_file': str(params_path),
                                      'use_sim_time': 'true'}.items()),
            ]),
        ])
    if value('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', namespace='usv_1',
                            arguments=['-d', str(share / 'rviz/fossen_model2.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    """Declare the isolated small-vessel entrypoint.

    :return: Launch description.
    """
    share = Path(get_package_share_directory('usv_sim_full'))
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='navigation', description='direct, pid or navigation'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('control_params_file', default_value=str(share / 'config/fossen_model2_control.yaml')),
        OpaqueFunction(function=_setup),
    ])
