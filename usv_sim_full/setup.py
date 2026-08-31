from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'usv_sim_full'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/usv_sim_full']),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            [f for f in glob('config/*') if os.path.isfile(f)],
        ),
        (
            os.path.join('share', package_name, 'config', 'certificate_case'),
            glob('config/certificate_case/*'),
        ),
        (
            os.path.join('share', package_name, 'config', 'generated'),
            glob('config/generated/*') or ['config/generated/.gitkeep'],
        ),
        (
            os.path.join('share', package_name, 'config', 'three_vision_one_mmwave'),
            glob('config/three_vision_one_mmwave/*'),
        ),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.py') + glob('tools/*.sh')),
        (
            os.path.join('share', package_name, 'rviz'),
            [f for f in glob('rviz/*') if os.path.isfile(f)],
        ),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + ['launch/notes.md']),
        (os.path.join('share', package_name, 'launch/components'), glob('launch/components/*.py')),
        (os.path.join('share', package_name, 'sh'), glob('sh/*.sh')),
        (
            os.path.join('share', package_name, 'maps'),
            [f for f in glob('maps/*') if os.path.isfile(f)],
        ),
    ] + [
        (os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
        for root, dirs, files in os.walk('description')
    ] + [
        (os.path.join('share', package_name, root), [os.path.join(root, f) for f in files])
        for root, dirs, files in os.walk('worlds')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MurphyChen',
    maintainer_email='murphy.chen@xxx.com',
    description='A YAML-driven USV simulation package with dynamic sensor configuration',
    license='MIT',
    
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = usv_sim_full.scripts.odom_tf_broadcaster:main',
            'obstacle_spawner = usv_sim_full.scripts.obstacle_spawner:main',
            'dual_thruster_teleop_incre = usv_sim_full.scripts.dual_thruster_teleop_incre:main',
            'cmd_vel_to_thruster = usv_sim_full.scripts.cmd_vel_to_thruster:main',
            'session_manager = usv_sim_full.scripts.session_manager:main',
            'usv_env_dynamics = usv_sim_full.scripts.usv_env_dynamics:main',
            'usv_sim_wrapper = usv_sim_full.scripts.usv_sim_wrapper:main',
            'scenario_manager_node = usv_sim_full.scripts.scenario_manager_node:main',
            'tf_namespace_relay = usv_sim_full.scripts.tf_namespace_relay:main',
            'certi_own_ship_cmd_vel = usv_sim_full.scripts.certi_own_ship_cmd_vel:main',
            'gz_spawn_robot_when_ready = usv_sim_full.scripts.gz_spawn_robot_when_ready:main',
            'nav2_tf_readiness_gate = usv_sim_full.scripts.nav2_tf_readiness_gate:main',
            'clearing_scan_publisher = usv_sim_full.scripts.clearing_scan_publisher:main',
            'dynamic_ship_manager_node = usv_sim_full.scripts.dynamic_ship_manager_node:main',
            'storm_field_manager_node = usv_sim_full.scripts.storm_field_manager_node:main',
            'dynamic_buoy_manager_node = usv_sim_full.scripts.dynamic_buoy_manager_node:main',
            'tracked_ship_list_merger = usv_sim_full.scripts.tracked_ship_list_merger:main',
        ],
    },
)
