from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tracked_ship_topic = LaunchConfiguration('tracked_ship_topic')
    names_topic = LaunchConfiguration('names_topic')
    storm_field_topic = LaunchConfiguration('storm_field_topic')
    clicked_point_topic = LaunchConfiguration('clicked_point_topic')
    frame_id = LaunchConfiguration('frame_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tracked_ship_topic',
            default_value='/dynamic_ship/tracked_ships',
            description='TrackedShipList output topic for the storm hazard.'),
        DeclareLaunchArgument(
            'names_topic',
            default_value='/storm_field/names',
            description='Active storm names topic for the RViz panel.'),
        DeclareLaunchArgument(
            'storm_field_topic',
            default_value='/storm_field/storms',
            description='Dedicated StormFieldArray output topic.'),
        DeclareLaunchArgument(
            'clicked_point_topic',
            default_value='/storm_field/clicked_point',
            description='PointStamped input topic used to add storm centers.'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID used by published TrackedShipList messages.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time.'),
        Node(
            package='usv_sim_full',
            executable='storm_field_manager_node',
            name='storm_field_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tracked_ship_topic': tracked_ship_topic,
                'names_topic': names_topic,
                'storm_field_topic': storm_field_topic,
                'clicked_point_topic': clicked_point_topic,
                'frame_id': frame_id,
            }],
        ),
    ])
