---
title: Workspace Setup
description: Setting up a development environment for PatrolBot — the canonical monorepo, Pi 5 Docker runtime, Pi 4 rollback workspace, and SBC build.
---

# Workspace Setup

Develop the Pi stack in this monorepo and deploy revisions to the Pi. The SBC holds a
separate Makefile project pending a verified source import.

## Prerequisites

| Machine | Stack |
|---|---|
| **Pi** | Ubuntu, ROS 2 **Jazzy**, `colcon`, Nav2 (`nav2_bringup`, `nav2_msgs`), `twist_mux`, `joy`, `slam_toolbox` |
| **SBC** | C++ toolchain (`g++`, `make`), ARIA/AriaCoda installed at `/usr/local/Aria` |
| **Operator laptop** | ROS 2 + RViz2 for visualization (optional) |

The robot machines are reached over SSH:
- **Pi 5:** `ssh ubuntu@patrolbot-ros2`
- **Pi 4 rollback:** `ssh ubuntu@patrolbot-ros.qatar.cmu.edu`
- **SBC:** `ssh ros@172.20.87.231`

For active development, work directly over SSH or a remote mount.

## Pi: the colcon workspace

```
~/ros2_ws/
  src/
    patrolbot_bridge/        # ament_python — the TCP bridge
    patrolbot_navigation/    # ament_cmake  — Nav2 + teleop + TF
    patrolbot-launch/        # ament_python — mobile base
    rosaria2/                # ament_cmake  — legacy ARIA driver
  build/  install/  log/
```

Build and source:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash          # also done by ~/.bashrc on login
export ROS_DOMAIN_ID=0
```

!!! success "No `build_backup/` runtime copy"
    The Pi 5 bringup container and Pi 4 rollback service run
    `ros2 launch patrolbot-launch bringup.xml`. Older notes that say to copy
    `patrolbot-launch` into `~/build_backup/` are stale.

!!! warning "No nested repositories"
    All Pi packages belong to this monorepo. Do not copy their historical `.git`
    directories into `ros2_ws/src`.

## SBC: the ARIA server

```bash
cd ~/patrolbot_hw_server
make                                # g++ -I/usr/local/Aria/include -lAria -lArNetworking -lpthread
```

There is no colcon here — it's a plain Makefile project producing the `patrolbot_server` binary.
See [`patrolbot_hw_server`](../packages/patrolbot_hw_server.md).

## Verifying a working setup

```bash
ssh robot-pi2 'cd /home/ubuntu/patrolbot-repo && ./docker/status.sh'
ssh robot-pi2 "docker exec patrolbot-bridge bash -lc \
  'source /opt/ros/\$ROS_DISTRO/setup.bash; ros2 topic hz /odom /scan'"
```

## Editing this documentation

```bash
pip install -r requirements.txt
mkdocs serve         # http://127.0.0.1:8000, live reload
mkdocs build --strict
```

Next: [Coding Standards](coding-standards.md), [Testing](testing.md),
[Debugging](debugging.md).
