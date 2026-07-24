from glob import glob

from setuptools import find_packages, setup

package_name = 'patrolbot_gimbal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yousef Hussein',
    maintainer_email='ymh1874@gmail.com',
    description='SimpleBGC 32-bit camera gimbal control, isolated from the '
                'robot velocity chain',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gimbal_driver = patrolbot_gimbal.driver_node:main',
            'gimbal_joy = patrolbot_gimbal.joy_node:main',
            'gimbal_mux = patrolbot_gimbal.mux_node:main',
        ],
    },
)
