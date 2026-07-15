---
title: Repository Structure
description: The canonical monorepo layout, deployed Pi workspace, Docker files, and SBC project directory.
---

# Repository Structure

The Pi source and deployment definitions live in this monorepo. Robot machines contain
deployed copies; they are not the canonical editing location.

## Pi filesystem (`/home/ubuntu`)

```
/home/ubuntu/
├── ros2_ws/                          # the colcon workspace
│   ├── src/
│   │   ├── patrolbot_bridge/         # ament_python — TCP bridge
│   │   ├── patrolbot_navigation/     # ament_cmake  — Nav2
│   │   ├── patrolbot-launch/         # ament_python — mobile base (SOURCE)
│   │   └── rosaria2/                 # ament_cmake  — legacy
│   ├── build/  install/  log/        # colcon artifacts — NOT source of truth
├── ARIA/                             # AriaCoda library (libAria.so, headers, params)
├── .config/systemd/user/             # Pi 4 rollback units (not Pi 5 runtime)
├── patrolbot-repo/                   # Pi 5 deployed source + Compose runtime
├── patrolbot-logs.sh                 # Pi 4 rollback diagnostics helper
└── patrolbot-sh.p                    # ARIA hardware profile (PatrolBot-SH)
```

### Filesystem traps

!!! success "1. `build_backup/` is no longer a runtime target"
    The Pi 5 container and Pi 4 rollback service launch `ros2 launch
    patrolbot-launch bringup.xml`. Older notes
    that say `~/build_backup/patrolbot-launch/` is the deployed mobile-base copy are stale.

!!! warning "2. `build/`, `install/`, `log/` are not source"
    Standard colcon rule: never treat generated directories as canonical. `src/` is what you edit.

!!! warning "3. Deployed trees are not canonical"
    Edit and commit in this monorepo, then deploy the revision. Do not create nested Git
    repositories inside `ros2_ws/src`.

## SBC filesystem (`/home/ros`)

The SBC filesystem and services were re-verified over live SSH on 2026-07-15;
`SKILLS/sbc-architecture.md` remains the detailed architecture record.

```
/home/ros/
├── patrolbot_hw_server/
│   ├── patrolbot_server.cpp          # the ARIA TCP server source
│   ├── patrolbot_server              # compiled binary
│   └── Makefile
└── .config/systemd/user/
    └── patrolbot-server.service       # system units: socat-boot + patrolbot-wired-ip
```

The SBC has **no colcon workspace and no ROS 2** — just the Makefile project. See
[`patrolbot_hw_server`](../packages/patrolbot_hw_server.md).

## This monorepo

```
patrolbot/  (github.com/cmu-impactlab/patrolbot)
├── docs/                 # this site's Markdown
├── ros2_ws/src/          # Pi ROS 2 package source
├── docker/               # main Pi 5 image, Compose, health and status tools
├── mkdocs.yml
├── requirements.txt
└── .github/workflows/deploy.yml
```

## Where to edit for a given change

| Change | Edit | Then |
|---|---|---|
| Bridge behavior | `ros2_ws/src/patrolbot_bridge/...` | rebuild/restart bridge |
| Nav2 params / launch / map | `ros2_ws/src/patrolbot_navigation/...` | rebuild/restart nav |
| Mobile base (twist_mux/smoother) | `ros2_ws/src/patrolbot-launch/...` | restart bringup; rebuild if adding files |
| SBC server | `patrolbot_hw_server/patrolbot_server.cpp` (on SBC) | `make`, restart server |
| Docker deployment | `docker/...` and `ros2_ws/src/...` | build/deploy a revision |
| These docs | `docs/...` | `mkdocs serve` to preview |

See [Component Breakdown](component-breakdown.md) for what each of these components does.
