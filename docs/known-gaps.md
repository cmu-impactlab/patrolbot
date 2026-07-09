---
title: Known Gaps
description: Remaining open questions and risk items in this documentation after the 2026-07-07 refresh.
---

# Known Gaps

This page tracks what is still uncertain after the 2026-07-07 documentation refresh. The SBC is
currently down, so SBC details are documented from the source workspace truth file
`SKILLS/sbc-architecture.md` rather than a fresh live SSH audit.

## Current Open Items

### SBC Live Re-Verification

The SBC architecture is treated as known from `SKILLS/sbc-architecture.md`:

- `socat-boot.service` bridges `/dev/ttyS0` to TCP `:7000`.
- `patrolbot-server.service` runs `patrolbot_server -rh 127.0.0.1 -rrtp 7000` and serves TCP `:7272`.
- The wire protocol is `ODOM|LASER`, separate `AUX`, and inbound `DRIVE`.
- The SBC command watchdog stops the base after 750 ms without `DRIVE`.
- Empty `LASER:` means stale/missing laser data and lets the Pi-side safety path detect loss.

When the SBC is reachable again, re-check service status, binary timestamp, listening ports, and a
short sample of the TCP stream.

### Pi 5 Hardware Acceptance Pending

`robot-pi2` runs the monorepo Docker stack. Container liveness and SBC-off degraded
behavior are validated, but the SBC and Pi 4 are currently off and Pi 5 `eth0` has no
link. Before calling the migration complete, perform the hardware checks in
[Docker Deployment](deployment/docker.md), with the robot in a clear space and a
local operator at the joystick deadman/emergency stop.

### Code Hygiene

Minor cleanup items from source review:

- **Scaffold-default manifest metadata.** `patrolbot_bridge` and `patrolbot-launch` still carry
  scaffold-default maintainer/description/license fields; `rosaria2` also has old placeholder
  metadata.
- **Dead `ekf_config_file` reference.** `bringup.launch.py` defines an unused path to
  `ekf_test.yaml`.
- **Dead launch / editor temp files** remain in `patrolbot-launch/launch/`; see
  [Legacy Components](internals/legacy-components.md#known-dead-code--cleanup-candidates).
- **Committed colcon log** under `patrolbot_navigation/log/`.
- **Stale comment in `nav2_params.yaml`.** The trailing comment still mentions
  `use_composition:=False`; the actual launch uses composition and patched lifecycle managers with
  `bond_timeout: 0.0`.

## Resolved Facts To Keep Stable

| Fact | Current truth |
|---|---|
| Active migration board | Raspberry Pi 5 (`robot-pi2`, `patrolbot-rpi5`) |
| Rollback board/path | Raspberry Pi 4 bare-metal deployment, currently off |
| Active map | `second_map.{yaml,pgm}` = `3192×2205 @ 0.075 m`, origin `[-1,-1,0]` |
| Controller | RPP (`nav2_regulated_pure_pursuit_controller`), not DWB |
| Planner | NavFn |
| Laser TF | `base_link → laser_frame`, `x=0.037`, `z=0.2`, `roll=π` |
| Sonar | 4 physical rear sensors; `/sonar` publishes valid detections only |
| Mobile-base launch | `ros2 launch patrolbot-launch bringup.xml`; no `build_backup` runtime copy |

When any of these changes, update the source workspace `SKILLS/` file and this documentation in the
same change.
