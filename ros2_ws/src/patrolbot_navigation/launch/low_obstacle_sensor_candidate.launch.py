"""Opt-in URG candidate launch; impossible to enable with unset calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_candidate(context):
    enabled = LaunchConfiguration('commissioned').perform(context).lower()
    if enabled not in ('1', 'true', 'yes'):
        return []

    names = (
        'serial_port', 'frame_id', 'x', 'y', 'z',
        'yaw', 'pitch', 'roll')
    values = {
        name: LaunchConfiguration(name).perform(context).strip()
        for name in names
    }
    missing = [name for name, value in values.items() if not value]
    if missing:
        raise RuntimeError(
            'low-obstacle sensor commissioning requires measured values: ' +
            ', '.join(missing))

    return [
        Node(
            package='urg_node',
            executable='urg_node_driver',
            name='patrolbot_low_obstacle_lidar',
            output='screen',
            parameters=[{
                'serial_port': values['serial_port'],
                'laser_frame_id': values['frame_id'],
            }],
            remappings=[('scan', '/scan_low')],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='low_obstacle_lidar_static_tf',
            arguments=[
                values['x'], values['y'], values['z'],
                values['yaw'], values['pitch'], values['roll'],
                'base_link', values['frame_id'],
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('commissioned', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value=''),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('z', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        DeclareLaunchArgument('pitch', default_value=''),
        DeclareLaunchArgument('roll', default_value=''),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(
        function=_launch_candidate)])
