---
title: rosaria2 (legacy)
description: The superseded direct-ARIA driver for the Pioneer base. Why it was replaced by the SBC server + bridge split, and the conflicts that make it incompatible with the live stack.
---

# rosaria2 (legacy)

The **original** robot driver: a C++ ROS 2 node that drove the Pioneer base directly through ARIA
on the Pi. It is **built but not launched** in the production stack — superseded by the
[SBC server](patrolbot_hw_server.md) + [bridge](patrolbot_bridge.md) split. It is documented here
so its status is unambiguous and so anyone tempted to re-enable it understands the conflicts first.

!!! warning "Not part of the live system"
    Nothing in the active stack runs `rosaria2`. It is classified **legacy / fallback**. Do not run
    it alongside `patrolbot_bridge` — see [Conflicts](#conflicts-if-run-alongside-the-bridge).

| | |
|---|---|
| **Deploys to** | **Raspberry Pi** (historically) |
| **Build type** | `ament_cmake` (with `rosidl` message generation) |
| **Executable** | `rosaria2_debug` (built from `src/test.cpp`) |
| **Status** | built/installed, **not in any active launch** |
| **Versioning** | source-only legacy package in the PatrolBot monorepo |

## Why it was replaced

`rosaria2` required ARIA, the base, and the laser to all be reachable from the **Pi**. The current
architecture instead puts ARIA and the hardware on the **SBC** and reduces the Pi to a TCP client.
That split is what lets the Pi run modern ROS 2 Jazzy without dragging the legacy ARIA toolchain
and serial drivers onto it — the central [design decision](../architecture/overview.md#design-philosophy).

## What it did

| Aspect | Detail |
|---|---|
| **Connection** | Pioneer base via serial `/dev/ttyUSB0` **or** TCP `host:port` (historically :7000) |
| **Publishes** | `pose` (Odometry), `bumper_state` (`rosaria2/BumperState`), `sonar` (PointCloud), `battery_voltage`, `battery_recharge_state`, `battery_state_of_charge`, `motors_state` |
| **Subscribes** | `cmd_vel` with a 600 ms watchdog (stops the base if commands stop) |
| **Services** | `enable_motors`, `disable_motors` |
| **Laser** | `laser_publisher.cpp` published ARIA laser as `LaserScan` and broadcast `base_link→laser_frame` |

### Custom message

`rosaria2/BumperState`:

```
std_msgs/Header header
bool[] front_bumpers
bool[] rear_bumpers
```

## Package layout

| Path | Role |
|---|---|
| `src/test.cpp` | entry point → `RosAria2Node` → spin (builds `rosaria2_debug`) |
| `src/rosaria2_node.cpp` | ARIA wrapper: connect, drive, 600 ms cmd_vel watchdog |
| `src/laser_publisher.cpp` | ARIA laser → `LaserScan` + laser TF |
| `include/rosaria2/*.hpp` | headers (aria_utils, artime_to_ros_time, dynamic_parameter, laser_publisher, rosaria2_node) |
| `msg/BumperState.msg` | custom bumper message |

## Conflicts if run alongside the bridge

Running `rosaria2` and `patrolbot_bridge` together breaks the robot in three ways:

1. **TF conflict on `odom→base_link`** — both broadcast it.
2. **TF conflict on `base_link→laser_frame`** — `rosaria2`'s `laser_publisher` vs. the
   `laser_static_tf` in `bringup.launch.py`.
3. **Double `cmd_vel` consumption** — both subscribe `cmd_vel`; the base receives velocity on two
   channels.

To use `rosaria2` at all you must disable `patrolbot_bridge` first and remap frames — i.e., revert
to the old single-machine architecture. There is no supported mixed mode.

## Status classification

In the project's Used / Maybe / Not-Used taxonomy, the whole `rosaria2` package is **Maybe**
(kept as a fallback). It is listed once in [Legacy Components](../internals/legacy-components.md);
it is **not** documented elsewhere as part of the live system.
