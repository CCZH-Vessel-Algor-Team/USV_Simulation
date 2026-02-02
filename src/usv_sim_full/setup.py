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
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
        (os.path.join('share', package_name, 'test_env'), glob('test_env/*')),
        (os.path.join('share', package_name, 'launch/components'), glob('launch/components/*.py')),
        # 移除不存在的description/models引用
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MurphyChen',
    maintainer_email='murphy.chen@xxx.com',
    description='A YAML-driven USV simulation package with dynamic sensor configuration',
    license='MIT',
    
    entry_points={
        'console_scripts': [
            'obstacle_spawner = usv_sim_full.scripts.obstacle_spawner:main',
        ],
    },
)