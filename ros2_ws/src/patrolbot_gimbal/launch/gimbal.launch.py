"""Standalone gimbal launch.

Deliberately not included by patrolbot_navigation's bringup. The gimbal is a
camera peripheral: if it fails to start, or its board is unplugged, navigation
must be entirely unaffected. Keeping it in its own launch and its own process
group is what makes that true rather than merely intended.

Starts three nodes and nothing else. No Nav2, no mobile base, no bridge.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('patrolbot_gimbal')
    default_params = os.path.join(package_share, 'config', 'gimbal_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Gimbal parameter file')
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/patrolbot_gimbal',
        description='Serial device for the SimpleBGC controller')
    joy_arg = DeclareLaunchArgument(
        'start_joy_node', default_value='false',
        description=(
            'Start a joy_node here. Leave false when the navigation stack '
            'already publishes /joy — two joy_nodes on one device conflict.'))

    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        params_arg,
        port_arg,
        joy_arg,

        # Reads the serial port. The only hardware-owning node here.
        Node(
            package='patrolbot_gimbal',
            executable='gimbal_driver',
            name='gimbal_driver',
            output='screen',
            parameters=[params, {'port': LaunchConfiguration('port')}],
            respawn=True,
            respawn_delay=2.0,
        ),

        # Arbitrates joystick over dashboard.
        Node(
            package='patrolbot_gimbal',
            executable='gimbal_mux',
            name='gimbal_mux',
            output='screen',
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        ),

        # Reads /joy, published by the navigation stack's joy_node.
        Node(
            package='patrolbot_gimbal',
            executable='gimbal_joy',
            name='gimbal_joy',
            output='screen',
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
