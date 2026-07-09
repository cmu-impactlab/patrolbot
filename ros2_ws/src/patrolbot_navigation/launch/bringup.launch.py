import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction, RegisterEventHandler, EmitEvent)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('patrolbot_navigation')
    launch_dir = os.path.join(pkg_dir, 'launch')

    map_yaml_file = os.path.join(pkg_dir, 'maps', 'second_map.yaml')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    ekf_config_file = '/home/ubuntu/ros2_ws/ekf_test.yaml'

    # We do NOT use nav2_bringup's bringup_launch.py because its lifecycle
    # managers hard-code bond_timeout (4.0s) and never read our params file.
    # Inflating the large 3690x3132 map starves map_server's bond heartbeat
    # past 4s -> lifecycle_manager aborts -> AMCL never activates -> no
    # map->odom transform -> map is blank in RViz.
    #
    # Instead we use local copies of localization/navigation launch
    # (patrolbot_*_launch.py) patched with bond_timeout: 0.0.
    #
    # use_composition is True: all Nav2 nodes share ONE process (nav2_container).
    # This is required on this Pi because:
    #   - The huge map travels map_server -> costmaps via intra-process (zero
    #     copy), never over DDS, so it can't saturate the transport.
    #   - One DDS participant means no /dev/shm port / file-descriptor
    #     exhaustion (ulimit -n is only 1024; ~13 separate participants run out).
    # bond_timeout: 0.0 (in the patched launch files) prevents the slow large-map
    # costmap inflation from tripping the lifecycle bond watchdog.
    #
    # Startup ordering: localization (map_server + amcl) loads into the container
    # immediately so the map + map->odom transform are ready within seconds. The
    # heavy navigation stack (costmaps inflating the huge map) is delayed by
    # TimerAction so it does not compete with localization for the container's
    # sequential composable-node loading. Result: RViz map + 2D Pose Estimate are
    # usable almost immediately; Nav2 Goals become available once navigation
    # finishes activating a bit later.
    container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[nav2_params_file, {'autostart': True}],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'patrolbot_localization_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': nav2_params_file,
            'use_composition': 'True',
            'use_respawn': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'patrolbot_navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': nav2_params_file,
            'use_composition': 'True',
            'use_respawn': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        # force to use only one thread to not crash when loading large maps
        SetEnvironmentVariable('MAGICK_THREAD_LIMIT', '1'),
        SetEnvironmentVariable('OMP_NUM_THREADS', '1'),

        # Single composable container hosting all Nav2 nodes.
        # We do NOT respawn the container: a respawned container comes back empty
        # (LoadComposableNodes does not re-run). Instead, if it dies we shut the
        # whole launch down (handler below) so systemd Restart=always brings up a
        # fresh, fully-populated stack.
        container,

        # If the container process exits for any reason, tear down the launch so
        # systemd restarts the complete Nav2 stack cleanly.
        RegisterEventHandler(
            OnProcessExit(
                target_action=container,
                on_exit=[EmitEvent(event=Shutdown(reason='nav2_container exited'))],
            )
        ),

        # Localization first — map + map->odom ready in seconds.
        localization,

        # Navigation after a short delay so it doesn't slow localization startup.
        TimerAction(period=20.0, actions=[navigation]),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[{'use_sim_time': False}], respawn=True, respawn_delay=2.0),
        # Joystick teleop feeds twist_mux's 'input/joy' (priority 8 > nav's 5),
        # so moving the sticks overrides navigation; releasing them lets nav resume.
        # respawn so a teleop crash self-heals; its own joy-loss watchdog drops the
        # deadman if joy_node dies, so a crash can't leave the robot driving.
        Node(package='patrolbot_navigation', executable='patrolbot_joy_teleop.py',
             name='patrolbot_joy_teleop', output='screen',
             remappings=[('/cmd_vel_joy', '/input/joy')],
             respawn=True, respawn_delay=2.0),
        # Safety watchdog: holds the robot (zero Twist on twist_mux priority-10
        # input/safety_controller) whenever /scan or /odom goes stale, and releases
        # automatically when they return. Sensor-loss safe-hold for the whole stack.
        Node(package='patrolbot_navigation', executable='patrolbot_safety_watchdog.py',
             name='patrolbot_safety_watchdog', output='screen',
             respawn=True, respawn_delay=2.0),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf',
            # x y z yaw pitch roll. roll=pi: the SICK is mounted upside-down
            # (LaserFlipped=true in ARIA) and getRawReadings() returns the scan in
            # flipped order, so it arrives mirrored left<->right. Rolling the
            # laser_frame 180 deg about the forward axis un-mirrors it (front stays
            # front, left<->right corrected). x=0.037 is LaserX=37mm from the ARIA params.
            arguments=['0.037', '0', '0.2', '0', '0', '3.14159', 'base_link', 'laser_frame'],
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])
