from glob import glob
from os.path import relpath
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'patrolbot_bridge'
source_root = next((p for p in Path(__file__).resolve().parents
                    if (p / 'contracts' / 'robot-tuning.registry.json').is_file()), None)
tuning_assets = [] if source_root is None else [
    relpath(source_root / 'contracts' / name, Path(__file__).resolve().parent)
    for name in ('robot-tuning.registry.json', 'robot-tuning-auth.json',
                 'robot-tuning-runtime.json')]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/contracts', tuning_assets),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yousef Hussein',
    maintainer_email='yousefh@andrew.cmu.edu',
    description='ROS 2 bridge for the PatrolBot ARIA hardware server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = patrolbot_bridge.bridge_node:main',
            'tuning_startup = patrolbot_bridge.tuning_startup:main',
            'tuning_bootstrap_preview = patrolbot_bridge.config_bootstrap:main',
            'tuning_initialize = patrolbot_bridge.tuning_initialize:main',
            'tuning_run = patrolbot_bridge.tuning_run:main',
            'tuning_recover = patrolbot_bridge.tuning_recover:main',
        ],
    },
)
