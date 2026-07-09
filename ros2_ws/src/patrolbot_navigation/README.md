# patrolbot_navigation

Nav2 navigation stack configuration and joystick teleop for the PatrolBot.

## What it does

Contains the Nav2 bringup launch file, occupancy maps, Nav2 parameter config,
and a custom joystick teleop node.

## Active components

| File | Role |
|------|------|
| `launch/bringup.launch.py` | Launches full Nav2 stack (AMCL + DWB + collision monitor), joy_node, `patrolbot_joy_teleop`, and a static TF publisher for the laser frame |
| `scripts/patrolbot_joy_teleop.py` | Converts PS controller buttons to incremental velocity commands on `/cmd_vel_joy` |
| `config/nav2_params.yaml` | Full Nav2 parameter set: AMCL differential model, DWB controller (max 0.26 m/s), collision_monitor stop box 0.6×0.6 m |
| `maps/second_map.yaml` + `.pgm` | Active production map (0.1 m/px) |

## Laser TF

`bringup.launch.py` publishes `base_link → laser_frame` at z=0.2 m, yaw=π
(sensor is mounted facing rear/flipped; matches `LaserFlipped=true` in
patrolbot-sh.p).

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
- `scripts/twist_mux.yaml` — velocity priority config; twist_mux is not
  launched (collision_monitor handles cmd_vel arbitration directly)
- `maps/cmuq_1st_floor.yaml` + `.pgm` — older CMU-Q map, not loaded at launch

## Dependencies

`nav2_bringup`, `slam_toolbox`, `joy`, `teleop_twist_joy`, `rclpy`
