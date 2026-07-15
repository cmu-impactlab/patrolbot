# patrolbot_navigation

Nav2 navigation stack configuration and joystick teleop for the PatrolBot.

## What it does

Contains the Nav2 bringup launch file, occupancy maps, Nav2 parameter config,
and a custom joystick teleop node.

## Active components

| File | Role |
|------|------|
| `launch/bringup.launch.py` | Launches the composed Nav2 stack (AMCL + RPP + collision monitor), joystick teleop, safety watchdog, and laser static TF |
| `scripts/patrolbot_joy_teleop.py` | Logitech F710 Xinput teleop: RB deadman, left-stick drive/turn, publishes `/cmd_vel_joy` |
| `scripts/patrolbot_safety_watchdog.py` | Publishes a priority-10 zero command while `/scan` or `/odom` is stale |
| `config/nav2_params.yaml` | AMCL differential model, RPP (`0.22 m/s` cruise), `0.26 m/s` Nav2 smoother cap, and a 0.6×0.6 m collision-monitor stop box |
| `maps/second_map.yaml` + `.pgm` | Active map: 3192×2205 at 0.075 m/px, origin `[-1,-1,0]` |

## Laser TF

`bringup.launch.py` publishes `base_link → laser_frame` at `x=0.037 m`,
`z=0.2 m`, with **roll=π**. The scanner faces forward and is mounted upside
down; the roll matches `LaserFlipped=true` in `patrolbot-sh.p`.

## Env vars set at launch

- `ROS_DOMAIN_ID=0`
- `MAGICK_THREAD_LIMIT=1`, `OMP_NUM_THREADS=1` — prevent OOM crash when loading
  the large map on the Pi

## How to run

```bash
ros2 launch patrolbot_navigation bringup.launch.py
```

## Present but not in the active launch

- `scripts/lms200_sanitizer.py` — fixes SICK LMS-200 header fields
  (`/bad_scan` → `/good_scan`); no `/bad_scan` publisher in current stack
- `scripts/twist_mux.yaml` — reference velocity-priority config; the active
  `twist_mux` is launched by `patrolbot-launch` with `param/defaults/mux.yaml`
- `maps/cmuq_1st_floor.yaml` + `.pgm` — older CMU-Q map, not loaded at launch

## Dependencies

`nav2_bringup`, `nav2_msgs`, `slam_toolbox`, `joy`, `twist_mux`, `rclpy`,
`geometry_msgs`, `sensor_msgs`, `launch`, `launch_ros`
