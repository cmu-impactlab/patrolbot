---
title: Component Breakdown
description: Every component of PatrolBot classified as Used, Maybe (legacy), or Not Used (dead code), so the live system is never confused with the cruft around it.
---

# Component Breakdown

This page classifies **every** component using the project's Used / Maybe / Not-Used taxonomy. The
point is honesty: documentation that treats dead code as load-bearing is worse than none. Only the
**Used** components are documented elsewhere as part of the live system; **Maybe** components get a
single line in [Legacy Components](legacy-components.md); **Not Used** components are listed here
once as cleanup candidates and nowhere else.

## Used — the live system

| Component | Machine | Role | Documented in |
|---|---|---|---|
| `patrolbot_server` | SBC | ARIA TCP server | [package](../packages/patrolbot_hw_server.md) |
| `patrolbot-wired-ip.service` | SBC | keep `10.0.0.1/24` on `enp2s5` | [deployment](../deployment/robot-deployment.md) |
| `socat-boot.service` | SBC | serial→TCP shim | [deployment](../deployment/robot-deployment.md) |
| `patrolbot-server.service` | SBC | run the server at boot | [deployment](../deployment/robot-deployment.md) |
| `patrolbot_bridge` / `bridge_node` | Pi | TCP↔ROS bridge | [package](../packages/patrolbot_bridge.md) |
| `patrolbot_navigation` (launch, params, maps) | Pi | Nav2 stack | [package](../packages/patrolbot_navigation.md) |
| `patrolbot_localization_launch.py` / `patrolbot_navigation_launch.py` | Pi | patched Nav2 launches (`bond_timeout: 0.0`) | [launch](../ros2/launch-system.md) |
| `second_map.{pgm,yaml}` | Pi | active 3192×2205 map at 0.075 m/px | [package](../packages/patrolbot_navigation.md) |
| `patrolbot_joy_teleop.py` | Pi | Xinput teleop | [node](../ros2/nodes.md#patrolbot_joy_teleop) |
| `laser_static_tf` | Pi | `base_link→laser_frame` | [node](../ros2/nodes.md#laser_static_tf) |
| `patrolbot-launch` | Pi | mobile base | [package](../packages/patrolbot-launch.md) |
| `mux.yaml`, `smoother.yaml`, `lifecycle_mgr.py` | Pi | mux + smoother config/mgr | [package](../packages/patrolbot-launch.md) |
| `docker/` | Pi 5 | main Docker Compose runtime | [deployment](../deployment/docker.md) |

## Maybe — kept, not active

These are real but not part of the running stack. Each is one line in
[Legacy Components](legacy-components.md); none get their own page.

| Component | Why "Maybe" |
|---|---|
| `rosaria2` package | superseded direct-ARIA driver, kept as source-only fallback |
| `lms200_sanitizer.py` | `/bad_scan → /good_scan` fixer; no `/bad_scan` publisher today |
| `patrolbot_navigation/scripts/twist_mux.yaml` | reference config; twist_mux runs from `patrolbot-launch` |
| `patrolbot_nav.rviz` | operator RViz layout (manual use) |
| `maps/cmuq_1st_floor.{yaml,pgm}` | older map, not in active launch |

## Not Used — dead code / cleanup candidates

Listed once, here, as cleanup candidates — **not** documented anywhere as part of the live system.

| Component | Note |
|---|---|
| `patrolbot-launch/launch/rosaria2.xml`, `joy.xml` | commented out of `bringup.xml` |
| `patrolbot-launch/launch/teleop-key*.xml`, `readlidar.py`, `rosaria2.py`, `rosaria2-lidar.xml`, `smoother_lifecycle.xml`, `launch_teleop_keyboard.bash` | dev experiments not included by active launch |
| `ekf_test.yaml` reference in `bringup.launch.py` | variable defined, file already removed, never used in the launch body |

## Summary counts

| Label | Approx. count |
|---|---|
| **Used** | main Pi 5 Docker stack plus SBC services |
| **Maybe** | versioned fallbacks and references listed above |

The numbers are approximate and exclude ARIA library internals and colcon `build/`. The taxonomy
itself — not just the counts — is the deliverable: keep it current when components are added or
retired (see [Coding Standards](../development/coding-standards.md#keep-written-docs-in-sync-with-the-live-system)).
