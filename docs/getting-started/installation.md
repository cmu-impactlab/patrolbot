---
title: Installation
description: What you need to run, develop, or document PatrolBot — the Pi and SBC software prerequisites, and how to get access to each machine.
---

# Installation

PatrolBot is a deployed robot, not a package you `pip install`. "Installation" means getting the
right software onto the SBC and selected Pi runtime (and, optionally, onto your laptop for visualization).
This page covers prerequisites and access; the build steps are on [Building](building.md) and the
day-one run-through is on [Quickstart](quickstart.md).

## What runs where

| Machine | Needs |
|---|---|
| **Raspberry Pi 5 main driver** | Ubuntu 24.04.4 LTS (`aarch64`), Docker + Compose |
| **SBC** | C++ toolchain (`g++`, `make`), **ARIA/AriaCoda** at `/usr/local/Aria`, `socat` |
| **Operator laptop** (optional) | ROS 2 + RViz2 for visualization/commanding |

The SBC and main Pi 5 are already provisioned. You generally
do **not** reinstall the OS or ROS; you build/deploy the PatrolBot packages on top.

## Pi prerequisites

```bash
# ROS 2 Jazzy is assumed already installed. Then the Nav2 + helper packages:
sudo apt update
sudo apt install -y \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
  ros-jazzy-twist-mux ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-nav2-velocity-smoother ros-jazzy-slam-toolbox \
  python3-colcon-common-extensions
```

ARIA (`libAria.so`) is present under `~/ARIA/` on the Pi and is only needed to **build** the legacy
[`rosaria2`](../packages/rosaria2.md) package — the active stack does not use it at runtime.

## SBC prerequisites

The SBC needs the ARIA SDK installed at `/usr/local/Aria` and a C++ toolchain. The server is built
with a plain `Makefile` (no ROS, no colcon). See
[`patrolbot_hw_server`](../packages/patrolbot_hw_server.md).

## Access to the machines

| Machine | SSH address | Notes |
|---|---|---|
| Pi 5 | `robot-pi2` / `ubuntu@patrolbot-ros2` | Main Docker driver |
| SBC | `robot-sbc` / `ros@172.20.87.231` | Hardware server; Pi runtime endpoint is `10.0.0.1:7272` |
| Pi 5 and operator | same subnet | Required for ROS 2 DDS discovery without extra config |

!!! note "Off-site access"
    ROS 2 discovery (for RViz) uses plain LAN multicast. On the LAN it just works; from home over a
    VPN/NAT it generally does not. The FastDDS discovery-server experiment was reverted on
    2026-06-29 — see [Remote Operation](../deployment/remote-operation.md).

## Documentation toolchain (this site)

To build or preview this documentation:

```bash
pip install -r requirements.txt   # mkdocs-material
mkdocs serve                      # http://127.0.0.1:8000
mkdocs build --strict             # static site → ./site
```

## Next

- [Building](building.md) — compile the Pi workspace and the SBC server.
- [Quickstart](quickstart.md) — bring the robot up and drive it.
