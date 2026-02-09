import sys
from setuptools import setup
from glob import glob
import os

# Some build tools forward long-form flags to setup.py which can cause
# 'option not recognized' errors when setuptools/distutils don't expect them
# (example: '--editable' passed through). Strip a small, conservative list
# of such flags when present so the package can be built in diverse CI/dev
# environments. Keep help/version flags intact so introspection works.
_strip_flags = {'--editable', '--uninstall', '--build-directory'}
if not any(x in sys.argv for x in ('--help', '--help-commands', '--version')):
    new_argv = []
    i = 0
    while i < len(sys.argv):
        a = sys.argv[i]
        if a in _strip_flags:
            i += 1
            # remove a possible value after flags like --build-directory
            if i < len(sys.argv) and not sys.argv[i].startswith('-'):
                i += 1
            continue
        new_argv.append(a)
        i += 1
    sys.argv = new_argv

package_name = 'usv_sim_full'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
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
from setuptools import setup, find_packages
import sys

# Colcon / pip may pass flags like --editable or --uninstall to setup.py in
# some environments which the legacy setuptools-based setup() does not accept
# directly. Remove known problematic flags so setup() can run in those CI/dev
# environments. This is a minimal, local workaround to allow building the
# package without changing the global environment.
def _strip_long_options(argv):
    # Remove long options and their possible values (used by some installers)
    out = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a.startswith('--'):
            # skip this option
            i += 1
            # if next token looks like a value (doesn't start with '-') skip it too
            if i < len(argv) and not argv[i].startswith('-'):
                i += 1
            continue
        out.append(a)
        i += 1
    return out

_orig_argv = sys.argv[:]
# If setup.py is being queried for available commands (e.g. --help-commands),
# don't strip arguments — colcon/ament expects the help query to work.
if any(x in _orig_argv for x in ('--help', '--help-commands', '--version')):
    # keep argv as-is so setup.py responds normally to help/version queries
    pass
else:
    sys.argv = _strip_long_options(sys.argv)
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