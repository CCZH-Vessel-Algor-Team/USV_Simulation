import sys
from setuptools import setup, find_packages
from glob import glob
import os


def _strip_unsupported_flags(argv):
    """Remove installer flags that setuptools/distutils don't understand."""

    unsupported_no_value = {'--editable', '-e'}
    unsupported_with_value = {
        '--build-directory',
        '--build-base',
    }

    cleaned = []
    skip_next = False
    for token in argv:
        if skip_next:
            skip_next = False
            continue
        if token in unsupported_no_value:
            continue
        if token in unsupported_with_value:
            skip_next = True
            continue
        cleaned.append(token)
    return cleaned


# Apply argv cleaning early so setup() sees only supported options
sys.argv = _strip_unsupported_flags(sys.argv)

package_name = 'usv_sim_full'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch/components'), glob('launch/components/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
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