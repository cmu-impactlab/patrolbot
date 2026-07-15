# patrolbot_bridge

TCP-to-ROS 2 bridge between the Pi and the PatrolBot main PC.

## What it does

Connects to the SBC at `10.0.0.1:7272` over the dedicated Ethernet link. A
background thread receives navigation telemetry plus a separate lower-rate AUX
line and translates them into ROS 2 messages. It also forwards ROS velocity
commands back to the main PC.

The SBC source loop is nominally 20 Hz; the 2026-07-15 live ROS observation was
about 25 Hz, so operational checks accept the 20–25 Hz range.

**Incoming from main PC:**
```
ODOM:x,y,th,vx,vth|LASER:r1,r2,...,rN\n
AUX:SONAR=x,y;...|BATT=volt,soc,chargeState,temp|FLAGS=flags,fault,stall,motors,estop\n
```
Publishes:
- `/odom` (`nav_msgs/Odometry`) — robot pose and twist
- `/scan` (`sensor_msgs/LaserScan`) — SICK LMS-200 scan (−90° to +90°, range 0.25–8 m)
- `/sonar`, `/battery`, `/diagnostics` — lower-rate auxiliary state
- TF `odom → base_link` at 50 Hz from the latest pose

**Outgoing to main PC:**
```
DRIVE:linear:angular\n
```
Subscribed topic: `/cmd_vel` (`geometry_msgs/Twist`) — the command after Nav2,
collision monitoring, `twist_mux`, and final velocity smoothing.

## How to run

```bash
ros2 run patrolbot_bridge bridge_node
```

## Dependencies

`rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `diagnostic_msgs`,
`std_msgs`, `tf2_ros`
