# patrolbot_bridge

TCP-to-ROS 2 bridge between the Pi and the PatrolBot main PC.

## What it does

Connects to the SBC at `10.0.0.1:7272` over the dedicated Ethernet link. A background thread
receives a custom text protocol line at ~20 Hz and translates it into ROS 2
messages. It also forwards ROS velocity commands back to the main PC.

**Incoming from main PC:**
```
ODOM:x,y,th,vx,vth|LASER:r1,r2,...,rN\n
```
Publishes:
- `/odom` (`nav_msgs/Odometry`) — robot pose and twist
- `/scan` (`sensor_msgs/LaserScan`) — SICK LMS-200 scan (−90° to +90°, range 0.05–8 m)
- TF `odom → base_link`

**Outgoing to main PC:**
```
DRIVE:linear:angular\n
```
Subscribed topic: `/cmd_vel` (`geometry_msgs/Twist`) — final safe velocity from Nav2
collision monitor.

## How to run

```bash
ros2 run patrolbot_bridge bridge_node
```

## Dependencies

`rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`
