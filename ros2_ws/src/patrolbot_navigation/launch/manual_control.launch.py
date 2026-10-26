"""Controller and hardware safety processes, independent of the Nav2 container."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    from patrolbot_bridge.tuning_startup import resolve
    params = resolve(os.path.join(get_package_share_directory('patrolbot_navigation'),
                                  'config', 'nav2_params.yaml'), 'navigation')
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'use_sim_time': False, 'device_id': 0, 'deadzone': .05,
                          'autorepeat_rate': 20., 'sticky_buttons': False,
                          'coalesce_interval_ms': 1}], respawn=True, respawn_delay=2.),
        Node(package='patrolbot_navigation', executable='patrolbot_joy_teleop.py',
             name='patrolbot_joy_teleop', parameters=[params], output='screen',
             remappings=[('/cmd_vel_joy', '/input/joy')], respawn=True, respawn_delay=2.),
        Node(package='patrolbot_navigation', executable='patrolbot_controller_feedback.py',
             name='patrolbot_controller_feedback', parameters=[params], output='screen',
             respawn=True, respawn_delay=2.),
        Node(package='patrolbot_navigation', executable='patrolbot_safety_watchdog.py',
             name='patrolbot_safety_watchdog', parameters=[params], output='screen',
             respawn=True, respawn_delay=2.),
    ])
