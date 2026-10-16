from glob import glob

from setuptools import find_packages, setup

package_name = 'patrolbot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
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
        ],
    },
)
