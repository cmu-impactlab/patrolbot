---
title: Building
description: Build PatrolBot from source — the Pi colcon workspace, the SBC ARIA server, and the Docker image used for the Pi 5 migration.
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
| `patrolbot_navigation` | ament_cmake | Nav2 + teleop + TF |
| `patrolbot-launch` | ament_python | mobile base (`twist_mux` + final velocity smoother) |
| `rosaria2` | ament_cmake | legacy; needs external ARIA and is excluded from Docker |

`patrolbot-bringup.service` now launches the mobile-base package by name:
`ros2 launch patrolbot-launch bringup.xml`. The old `~/build_backup/patrolbot-launch/` deployment
target was removed on 2026-06-28 and should not be recreated.

All four packages are versioned by the monorepo. Deployed Pi working trees must not
contain nested `.git` directories.

## SBC: ARIA server

```bash
cd ~/patrolbot_hw_server
make            # g++ -I/usr/local/Aria/include -lAria -lArNetworking -lpthread
```

Produces the `patrolbot_server` binary. No colcon, no ROS. See
[`patrolbot_hw_server`](../packages/patrolbot_hw_server.md). When the SBC is unavailable, use
`SKILLS/sbc-architecture.md` in the source workspace as the current truth source for documented
behavior.

## Docker image for the Pi 5

The image is built from the monorepo root:

```bash
cd ~/patrolbot-repo/docker
cp .env.example .env
docker compose build
```

See [Docker Deployment](../deployment/docker.md) for prerequisites, cutover, verification, and
rollback and readiness reporting.

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
