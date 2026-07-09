# rosaria2

Legacy ROS 2 driver for the Pioneer PatrolBot base via the ARIA/AriaCoda library.

## Status: built but not in active launch

This package is compiled and installed (`rosaria2_debug` binary exists in
`~/ros2_ws/install/`), but is **not started by the current bringup.xml**.
It represents the earlier direct-hardware path that was superseded by the TCP
bridge (`patrolbot_bridge`).

## What it does

Connects to the Pioneer base either via:
- Serial: `/dev/ttyUSB0`
- TCP: `host:port` string (historically port 7000)

Publishes:
- `pose` (`nav_msgs/Odometry`)
- `bumper_state` (`rosaria2/BumperState`) — front/rear bump sensors
- `sonar` (`sensor_msgs/PointCloud`) — 16-sonar ring
- `battery_voltage`, `battery_recharge_state`, `battery_state_of_charge`
- `motors_state`

Subscribes: `cmd_vel` (`geometry_msgs/Twist`) with a 600 ms watchdog (stops
robot if no command received).

Services: `enable_motors`, `disable_motors`.

## Known conflicts if run alongside patrolbot_bridge

Running both simultaneously causes:
1. **TF conflict** — both broadcast `odom → base_link`
2. **TF conflict** — rosaria2's LaserPublisher broadcasts `base_link → laser_frame`,
   conflicting with the static_transform_publisher in bringup.launch.py
3. **Double cmd_vel** — both nodes subscribe to `cmd_vel`; the robot receives
   velocity commands on two channels simultaneously

If you need to activate rosaria2, disable `patrolbot_bridge` first and remap
TF frames accordingly.

## Dependencies

`rclcpp`, `geometry_msgs`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `std_srvs`,
`tf2_ros`, AriaCoda (`~/ARIA/lib/libAria.so`)
