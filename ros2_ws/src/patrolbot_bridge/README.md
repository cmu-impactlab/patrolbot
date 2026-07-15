# patrolbot_bridge

TCP-to-ROS 2 bridge between the Pi and the PatrolBot main PC.

## What it does

Connects to the SBC at `10.0.0.1:7272` over the dedicated Ethernet link. A
background thread continuously drains the socket into separate, bounded
latest-value navigation and AUX queues. Independent publisher workers translate
those lines into ROS 2 messages. This prevents a slow DDS visualization reader
from filling the SBC's TCP send buffer and interrupting both odometry and laser
data. The bridge also forwards ROS velocity commands back to the main PC.

The Pi 5 bridge container additionally sets
`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`. With `rmw_fastrtps_cpp`, this
keeps DDS network transmission in the middleware's background writer instead of
blocking a bridge publisher worker.

The SBC source loop is nominally 20 Hz; the 2026-07-15 live ROS observation was
about 25 Hz, so operational checks accept the 20–25 Hz range.

**Incoming from main PC:**
```
ODOM:x,y,th,vx,vth|LASER:r1,r2,...,rN\n
AUX:SONAR=x,y;...|BATT=volt,soc,chargeState,temp|FLAGS=flags,fault,stall,motors,estop\n
```
Publishes:
- `/odom` (`nav_msgs/Odometry`) — robot pose and twist
- `/scan` (`sensor_msgs/LaserScan`, sensor-data BEST_EFFORT QoS) — SICK LMS-200
  scan (−90° to +90°, range 0.25–8 m)
- `/sonar` (`sensor_msgs/PointCloud2`, sensor-data BEST_EFFORT QoS), `/battery`,
  `/diagnostics` — lower-rate auxiliary state
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
