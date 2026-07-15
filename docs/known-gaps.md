---
title: Known Gaps
description: Remaining open questions and risk items after the 2026-07-15 live documentation audit.
---

# Known Gaps

This page tracks what remains after the 2026-07-15 source and live-system audit. The
SBC, Pi 4, and Pi 5 were all reachable during the audit.

## Current Open Items

### Pi 4 ROS Graph Isolation

The Pi 4 is powered and its three bare-metal ROS services are enabled and active.
Because both Pis use `ROS_DOMAIN_ID=0` on the same Wi-Fi LAN, a live graph query
showed duplicate node names from the Pi 4 and Pi 5. Keep the Pi 4 services stopped
while the Pi 5 is the main driver; enable them only for an intentional rollback.

## Verified Live State

### Dedicated Ethernet and Pi 5 Runtime

The final audit confirmed:

- `/etc/netplan/60-sbc-link.yaml` persistently configures Pi 5 `eth0` as
  `10.0.0.2/24` with DHCP disabled;
- `eth0` is 100 Mbps/full duplex, the route to `10.0.0.1` uses `eth0`, and
  the SBC socket is reachable;
- all three Pi 5 containers are healthy with zero restarts at revision
  `fa14b9b5cedfd56beaffca746e1c37256d67d1f0`;
- `/odom` and `/scan` are live (observed near 25 Hz), TF publishes at 50 Hz,
  both required transforms resolve, lifecycle nodes are active, and the restarted
  navigation container publishes its local costmap;
- the on-demand status command reports `OVERALL=ready`; it uses fresh `/odom` as
  the SBC data-path signal and retries lifecycle discovery without disrupting the
  bridge connection.

### SBC Services

The live audit confirmed:

- system units `socat-boot.service` and `patrolbot-wired-ip.service` are enabled
  and active; the latter re-applies `10.0.0.1/24` to `enp2s5` after carrier loss;
- the user unit `patrolbot-server.service` is enabled and active with
  `Linger=yes` for user `ros`;
- the server runs `patrolbot_server -rh 127.0.0.1 -rrtp 7000`, with TCP
  `:7272` listening and a live `ODOM|LASER` stream;
- the deployed source contains the 750 ms command watchdog, stale-laser handling,
  motor re-enable logic, and e-stop diagnostics.

### Motion Acceptance

No joystick or Nav2 motion was commanded during this documentation audit. Perform
those checks only in a clear area with a local operator at the deadman and physical
e-stop, after isolating the Pi 4 ROS services.

### Code Hygiene

Minor cleanup items from source review:

- **Package metadata/dependencies.** `patrolbot_bridge` and `patrolbot-launch`
  still carry scaffold-default TODO fields in `setup.py`; `rosaria2/package.xml`
  has placeholder metadata. The bridge manifest omits `nav_msgs`, and the launch
  manifest does not declare all of its runtime dependencies.
- **Dead `ekf_config_file` reference.** `bringup.launch.py` defines an unused path to
  `ekf_test.yaml`.
- **Dead launch and experimental files** remain in `patrolbot-launch/launch/`; see
  [Legacy Components](internals/legacy-components.md#known-dead-code--cleanup-candidates).

## Resolved Facts To Keep Stable

| Fact | Current truth |
|---|---|
| Main driver | Raspberry Pi 5 (`robot-pi2`, `patrolbot-rpi5`) Docker deployment |
| Rollback board/path | Powered Raspberry Pi 4 bare-metal deployment; services must stay isolated during Pi 5 operation |
| Active map | `second_map.{yaml,pgm}` = `3192×2205 @ 0.075 m`, origin `[-1,-1,0]` |
| Controller | RPP (`nav2_regulated_pure_pursuit_controller`), not DWB |
| Planner | NavFn |
| Laser TF | `base_link → laser_frame`, `x=0.037`, `z=0.2`, `roll=π` |
| Sonar | 4 physical rear sensors; `/sonar` publishes valid detections only |
| Mobile-base launch | `ros2 launch patrolbot-launch bringup.xml`; no `build_backup` runtime copy |

When any of these changes, update the source workspace `SKILLS/` file and this documentation in the
same change.
