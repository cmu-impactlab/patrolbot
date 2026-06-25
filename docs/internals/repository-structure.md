---
title: Repository Structure
description: The on-robot filesystem layout — the Pi colcon workspace, the build_backup deployment quirk, the nested git repos, and the SBC project directory.
---

# Repository Structure

PatrolBot's code is **not** in one repository — it is spread across two machines and several
independently-versioned directories. This page maps what lives where, so you know which file to
edit and which copy actually runs.

## Pi filesystem (`/home/ubuntu`)

```
/home/ubuntu/
├── ros2_ws/                          # the colcon workspace
│   ├── src/
│   │   ├── patrolbot_bridge/         # ament_python — TCP bridge
│   │   ├── patrolbot_navigation/     # ament_cmake  — Nav2  (OWN .git/)
│   │   ├── patrolbot-launch/         # ament_python — mobile base (SOURCE)
│   │   └── rosaria2/                 # ament_cmake  — legacy  (OWN .git/)
│   ├── build/  install/  log/        # colcon artifacts — NOT source of truth
├── build_backup/
│   └── patrolbot-launch/             # the mobile base that ACTUALLY RUNS
├── ARIA/                             # AriaCoda library (libAria.so, headers, params)
├── .config/systemd/user/             # the three Pi services
├── patrolbot_fastdds_pi.xml          # prepared FastDDS discovery-server profile
├── patrolbot-logs.sh                 # live log/diagnostics helper
├── patrolbot-sh.p                    # ARIA hardware profile (PatrolBot-SH)
└── yousef/                           # dev tools (mock_sbc_server.py, test_laser.py)
```

### Three filesystem traps

!!! danger "1. The mobile base runs from `build_backup/`"
    `patrolbot-bringup.service` launches `~/build_backup/patrolbot-launch/launch/bringup.xml`. The
    `ros2_ws/src/patrolbot-launch/` copy is the **source of truth**, but editing it changes nothing
    at runtime until the `build_backup` copy is updated. This catches everyone exactly once. See
    [Updates](../deployment/updates.md#the-mobile-base-deployment-step).

!!! warning "2. `build/`, `install/`, `log/` are not source"
    Standard colcon rule: never treat generated directories as canonical. `src/` (and, for the
    mobile base, `build_backup/`) is what you edit.

!!! warning "3. Nested git repositories"
    `patrolbot_navigation/` and `rosaria2/` each contain their own `.git/`. They are versioned
    independently of the rest of the workspace — commit in the correct repo.

## SBC filesystem (`/home/ros`)

Snapshot from the last sync; not live-verifiable here (see [Known Gaps](../known-gaps.md)).

```
/home/ros/
├── patrolbot_hw_server/
│   ├── patrolbot_server.cpp          # the ARIA TCP server source
│   ├── patrolbot_server              # compiled binary
│   └── Makefile
└── .config/systemd/user/
    └── patrolbot-server.service       # + system-level socat-boot.service
```

The SBC has **no colcon workspace and no ROS 2** — just the Makefile project. See
[`patrolbot_hw_server`](../packages/patrolbot_hw_server.md).

## This documentation repository

Separate from both robots:

```
patrolbot/  (github.com/cmu-impactlab/patrolbot)
├── docs/                 # this site's Markdown
├── mkdocs.yml
├── requirements.txt
└── .github/workflows/deploy.yml
```

The documentation repo contains **no robot code** — the source lives on the machines.

## Where to edit for a given change

| Change | Edit | Then |
|---|---|---|
| Bridge behavior | `ros2_ws/src/patrolbot_bridge/...` | `colcon build`, restart bridge |
| Nav2 params / launch / map | `ros2_ws/src/patrolbot_navigation/...` (own git) | `colcon build`, restart nav |
| Mobile base (twist_mux/smoother) | `ros2_ws/src/patrolbot-launch/...` **and** copy to `build_backup/` | restart bringup |
| SBC server | `patrolbot_hw_server/patrolbot_server.cpp` (on SBC) | `make`, restart server |
| These docs | `docs/...` in this repo | `mkdocs serve` to preview |

See [Component Breakdown](component-breakdown.md) for what each of these components does.
