from setuptools import find_packages, setup

package_name = 'patrolbot_dock_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yousef Hussein',
    maintainer_email='yousefh@andrew.cmu.edu',
    description='Fail-closed PatrolBot guarded undock action server',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dock_manager = patrolbot_dock_manager.dock_manager:main',
        ],
    },
)
