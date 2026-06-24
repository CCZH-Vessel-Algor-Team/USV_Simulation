from setuptools import find_packages, setup

package_name = 'sim_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MurphyChen',
    maintainer_email='1471072588@qq.com',
    description='仿真调试与实验用 Python 工具（非稳定 API）',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_monitor = sim_test.main:main',
        ],
    },
)
