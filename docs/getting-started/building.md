---
title: Building
description: Build PatrolBot from source — the Pi colcon workspace, the SBC ARIA server, and the build_backup deployment step the mobile base requires.
---

# Building

PatrolBot builds in two places: a **colcon** workspace on the Pi and a **Makefile** project on the
SBC. This page is the build reference; environment setup is on [Installation](installation.md).

## Pi: colcon workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=0
```

Build a single package while iterating:

```bash
colcon build --packages-select patrolbot_bridge
# or patrolbot_navigation / patrolbot-launch / rosaria2
```

The workspace has four packages:

| Package | Build type | Notes |
|---|---|---|
| `patrolbot_bridge` | ament_python | the TCP bridge |
| `patrolbot_navigation` | ament_cmake | Nav2 + teleop + TF (**own `.git/`**) |
| `patrolbot-launch` | ament_python | mobile base (source; deployed copy is in `build_backup/`) |
| `rosaria2` | ament_cmake | legacy; needs ARIA (`~/ARIA/lib/libAria.so`) to build (**own `.git/`**) |

!!! danger "Mobile-base builds need the `build_backup` step"
    `colcon build` updates `install/`, but `patrolbot-bringup.service` runs the mobile base from
    **`~/build_backup/patrolbot-launch/`**. After changing `patrolbot-launch`, copy the launch/param
    files into `build_backup` (see [Updates](../deployment/updates.md#the-mobile-base-deployment-step)),
    or the change won't run. This is the #1 build gotcha in the workspace.

!!! warning "Two nested git repos"
    Commit `patrolbot_navigation` and `rosaria2` changes in *their* repos, not the workspace's.

## SBC: ARIA server

```bash
cd ~/patrolbot_hw_server
make            # g++ -I/usr/local/Aria/include -lAria -lArNetworking -lpthread
```

Produces the `patrolbot_server` binary. No colcon, no ROS. See
[`patrolbot_hw_server`](../packages/patrolbot_hw_server.md). (The SBC may be unreachable — see
[Known Gaps](../known-gaps.md).)

## Verifying the build

```bash
# Pi
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep patrolbot          # bridge, navigation, launch present
ros2 pkg executables patrolbot_bridge   # bridge_node

# SBC
ls -l ~/patrolbot_hw_server/patrolbot_server
```

## Running lint/tests

```bash
cd ~/ros2_ws
colcon test --packages-select patrolbot_bridge patrolbot-launch
colcon test-result --verbose
```

These are flake8 / pep257 / copyright (ament lint) checks — style gates, not behavior tests. See
[Testing](../development/testing.md).

## Clean rebuild

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

Remember: `build/`, `install/`, and `log/` are **generated** — never edit them as if they were
source ([Repository Structure](../internals/repository-structure.md)).

## Next

- [Quickstart](quickstart.md) — run what you just built.
- [Workspace Setup](../development/workspace-setup.md) — the full development environment.
