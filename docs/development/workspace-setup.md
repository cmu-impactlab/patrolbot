---
title: Workspace Setup
description: Setting up a development environment for PatrolBot — the colcon workspace on the Pi, the SBC build, and the build_backup gotcha that catches everyone once.
---

# Workspace Setup

PatrolBot development spans the two machines. The Pi holds a colcon workspace; the SBC holds a
single Makefile project. This page gets you to a buildable, runnable state on each.

## Prerequisites

| Machine | Stack |
|---|---|
| **Pi** | Ubuntu, ROS 2 **Jazzy**, `colcon`, Nav2 (`nav2_bringup`, `nav2_msgs`), `twist_mux`, `joy`, `slam_toolbox` |
| **SBC** | C++ toolchain (`g++`, `make`), ARIA/AriaCoda installed at `/usr/local/Aria` |
| **Operator laptop** | ROS 2 + RViz2 for visualization (optional) |

The robot machines are reached over SSH (aliases `robot-pi`, `robot-sbc`). This documentation was
generated against a mounted snapshot; for active development you work directly over SSH or a mount.

## Pi: the colcon workspace

```
~/ros2_ws/
  src/
    patrolbot_bridge/        # ament_python — the TCP bridge
    patrolbot_navigation/    # ament_cmake  — Nav2 + teleop + TF   (own .git)
    patrolbot-launch/        # ament_python — mobile base
    rosaria2/                # ament_cmake  — legacy ARIA driver    (own .git)
  build/  install/  log/
```

Build and source:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash          # also done by ~/.bashrc on login
export ROS_DOMAIN_ID=0
```

!!! danger "The `build_backup/` gotcha — read this before you edit anything"
    The **mobile-base** package that actually runs at boot is launched from
    `~/build_backup/patrolbot-launch/`, **not** from `~/ros2_ws/src/patrolbot-launch/` or
    `install/`. Editing the `src` copy and rebuilding does **not** change what
    `patrolbot-bringup.service` runs. To make a mobile-base change take effect you must update the
    `build_backup` copy. The `src` copy is the source of truth; treat `build_backup` as the
    deployment target. See [Repository Structure](../internals/repository-structure.md).

!!! warning "Two nested git repos"
    `patrolbot_navigation/` and `rosaria2/` each contain their own `.git/`. They are versioned
    independently of the rest of the workspace — commit in the right place.

## SBC: the ARIA server

```bash
cd ~/patrolbot_hw_server
make                                # g++ -I/usr/local/Aria/include -lAria -lArNetworking -lpthread
```

There is no colcon here — it's a plain Makefile project producing the `patrolbot_server` binary.
See [`patrolbot_hw_server`](../packages/patrolbot_hw_server.md).

## A development loop without the robot

You can develop the bridge against a fake SBC. The Pi carries a dev tool for exactly this:

```bash
# On the Pi (or any machine), emulate the SBC's TCP server on :7272
python3 ~/yousef/mock_sbc_server.py
# then run the bridge pointed at it
ros2 run patrolbot_bridge bridge_node
```

`mock_sbc_server.py` and `test_laser.py` under `~/yousef/` are **dev tools, not production** — they
let you exercise the bridge and probe the laser without a live robot.

## Verifying a working setup

```bash
ros2 node list                 # bridge + nav2 nodes + twist_mux + teleop + joy
ros2 topic hz /odom /scan      # ~20 Hz each when the SBC is connected
ros2 run tf2_tools view_frames # map → odom → base_link → laser_frame
ssh robot-pi ./patrolbot-logs.sh status
```

## Editing this documentation

```bash
pip install -r requirements.txt
mkdocs serve         # http://127.0.0.1:8000, live reload
mkdocs build --strict
```

Next: [Coding Standards](coding-standards.md), [Testing](testing.md),
[Debugging](debugging.md).
