# patrolbot_bridge

TCP-to-ROS 2 bridge between the Pi and the PatrolBot main PC.

## What it does

Connects to the SBC at `10.0.0.1:7272` over the dedicated Ethernet link. A
background thread continuously drains the socket into separate, bounded
latest-value navigation and AUX queues. Independent publisher workers translate
those lines into ROS 2 messages. This prevents a stalled DDS reader from filling
the SBC's TCP send buffer and interrupting both odometry and laser data. The
bridge also forwards ROS velocity commands and two explicit, idempotent
hardware-control requests back to the main PC.

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
CHARGE_RELEASE:request_uuid\n
MOTOR_ENABLE:request_uuid\n
```
Subscribed topics: `/cmd_vel` (`geometry_msgs/Twist`) — the command after Nav2,
collision monitoring, `twist_mux`, and final velocity smoothing — and the two
deadman-evidence topics, `/patrolbot/manual_active` (F710 joystick teleop) and
`/patrolbot/web_teleop_active` (dashboard on-screen teleop, relayed by the Pi
web bridge), `std_msgs/Bool` each. Fresh evidence from either admits nonzero
`DRIVE` while localization (recovery stage, Nav2 readiness, AMCL pose
freshness, map state) would hold autonomy.

Services:

- `/patrolbot/charge_release` (`patrolbot_interfaces/srv/ChargeRelease`)
- `/patrolbot/motor_enable` (`patrolbot_interfaces/srv/MotorEnable`)

Both require a caller-generated lowercase canonical UUID and return acceptance,
terminal success/code/message, the UUID, charge/motor/E-stop state, elapsed time,
and the process-local hardware `session_generation`. Charge release never sends
velocity or enables motors. `DRIVE` never enables motors; motor enable is a
separate deliberate operation with stationary, released, E-stop-clear,
fault-free, fresh-telemetry preconditions.

## How to run

```bash
ros2 run patrolbot_bridge bridge_node
```

## Dependencies

`rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `diagnostic_msgs`,
`std_msgs`, `tf2_ros`

## Odometry reconnects

A TCP client reconnect advances the hardware control session and invalidates
pending control requests, but preserves the wheel-odometry covariance and
continuity reference. ARIA can keep running with the same origin while the
socket reconnects; reconnecting alone must not reset uncertainty to its floor.

The current SBC protocol carries no odometry epoch identifier. A real SBC/ARIA
restart therefore cannot be reliably distinguished from a link interruption.
Automatic origin-reset detection and recovery remain unimplemented. An origin
jump is subject to the normal elapsed-time continuity check; sufficiently small
jumps or long outages can pass that check, so it is not proof of epoch identity.
Do not use a TCP session number as evidence to reset odometry or localization.
A bridge process restart still creates a fresh accumulator; persistence across
process restarts is outside this fix.

### Scan age on the SBC link

New servers append `LASER_AGE_MS` to navigation frames. `/scan` and optional
unfiltered scans subtract this conservative ARIA scan age plus local queue
wait; `/odom` remains receipt-stamped. Invalid age metadata drops only the scan.
Legacy servers remain readable, with a throttled warning about missing age.
Transport latency is not measured by this extension. See the canonical
[wire protocol](../../../docs/reference/wire-protocol.md).


## LiDAR power service

`/patrolbot/set_lidar_power` (`SetLidarPower`) sends an explicit target and UUID
through the existing SBC socket. `BaseState` reports `lidar_state`,
`lidar_power_supported`, and `lidar_scan_age` in seconds (`-1` unavailable).
Fresh READY scans can be usable even when power control is uncommissioned.
OFF means commanded off; an accepted ON means STARTING until measured fresh
scans arrive. No electrical power readback is claimed.

Power changes require commissioned, fresh scanner state, fresh stationary base
telemetry, no active map/undock/hardware control, and a fresh navigation-idle
readback. The bridge establishes a motion hold and cancels old navigation
before sending power control. Timeout/disconnect retains the hold until an
explicit successful power recovery; cached or late replies cannot release it.
A power transition discards old localization evidence without resetting
odometry or stopping base telemetry. Fresh scans permit a new operator pose;
movement still requires fresh accepted localization and existing re-arm gates.
All nonzero DRIVE, motor-enable, undock and map admission share the scanner
gate. Stop and undock cancellation remain available.

Firmware boot-on, power wiring/readback, scanner serial reconnect and controller
mapping require commissioning and supervised live acceptance. See
`hardware/patrolbot_hw_server/README.md`; never enable the commissioning flag to
work around an unknown physical mapping.
