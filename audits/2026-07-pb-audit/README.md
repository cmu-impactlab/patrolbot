# PatrolBot PB-AUDIT July 2026

This directory is the authoritative record set for the July 2026 PatrolBot
PB-AUDIT remediation and stationary commissioning campaign.

## Current records

- `PB-AUDIT-STATUS-SUMMARY.md` — short finding-by-finding completion index.
- `PATROLBOT_RPI5_AUDIT.md` — canonical audit, evidence, and disposition.
- `PB-AUDIT-001-002-REMEDIATION-PLAN.md` — remediation design and gates.
- `PB-AUDIT-001-002-IMPLEMENTATION-REPORT.md` — implementation and live
  acceptance evidence.
- `PB-AUDIT-002-STATIONARY-CUTOVER-REPORT.md` — SBC cutover and stationary
  protocol commissioning.
- `PI5-SEVEN-ITEM-HARDENING-IMPLEMENTATION.md` — Pi hardening deployment.
- `PI5-SEVEN-ITEM-HARDENING-CODE-CHANGES.md` — focused code-change review.

The superseded `AUDIT-2026-07-16.md` first-pass report was deleted after its
still-relevant evidence was incorporated into the canonical audit.

## Session boundary

The 2026-07-23 pre-dashboard session ended with the robot charging, motors
disabled, the physical E-stop latched, zero fault flags, zero velocity and
odometry, one final `/cmd_vel` publisher, one hardware TCP client, and fresh
scan/sonar/battery/base telemetry.

Completed during this session:

- real power-cycle acceptance closed PB-AUDIT-013;
- Chrony automatically recovered the bad RTC time, while RTC replacement
  remains open under PB-AUDIT-010;
- the single physical E-stop, its PWR ON reset, typed telemetry transition,
  and priority-10 software hold were stationary-validated; and
- live host inspection reconfirmed that no joystick receiver or connected
  low-obstacle URG interface is present.

Bumper mapping, joystick/deadman testing, physical URG/envelope/LiDAR
measurement, gyro query, enabled-motor E-stop cutoff, localization/dock pose,
guarded reverse, undock/re-dock, and all motion tests remain deferred.

No dashboard source was edited and no velocity, goal, pose, dock/undock, motor,
charge-release, or other hardware-control command was sent.

## 2026-07-23 continuation

The operator subsequently exercised all six front and six rear panels while
the robot remained charging and motor-disabled. The measured front
`0x7e00`/rear `0x007e` masks are deployed through the existing bridge, and the
existing guarded-undock manager now handles the dock-held center-front contact
directionally. Bridge and navigation remained healthy with zero odometry and
one final `/cmd_vel` publisher. Dock localization, motor enable, guarded
reverse, and re-docking remain pending; no motion or hardware-control request
was sent during this mapping phase.

## Retained rollback sets

All retained off-host checksum manifests were reverified immediately before
the 2026-07-23 cleanup.

Workstation `backups/`:

- `20260721T181448Z-pb002-stationary-cutover`
- `20260721T195507Z-pb019-container-cleanup`
- `20260721T204423Z-pb002-manual-release-deploy`
- `20260721T212034Z-pb020-commit-provenance`
- `20260721T212751Z-pb010-pb013-pb022-root-maintenance`
- `20260721T215358Z-pb010-pb013-pb022-postchange`
- `20260721T221729Z-pb020-commit-provenance-postchange`

Pi `/home/ubuntu/patrolbot-backups/`:

- `20260721T181448Z-pb002-stationary-cutover`
- `20260721T204423Z-pb002-manual-release-deploy`
- `20260721T212034Z-pb020-commit-provenance`
- `20260721T221729Z-pb020-commit-provenance-postchange`

SBC `/home/ros/patrolbot-backups/`:

- `20260721T181448Z-pb002-stationary-cutover`
- `20260721T195507Z-pb019-container-cleanup`
- `20260721T204423Z-pb002-manual-release-deploy`
- `20260721T212751Z-pb010-pb013-pb022-root-maintenance`
- `20260721T215358Z-pb010-pb013-pb022-postchange`

The superseded July 16 backup generations were permanently deleted from all
three locations after the retained sets passed checksum verification. The
deleted workstation set included the abandoned 1.3 GB `.incomplete` archive.
The remaining backup footprint is approximately 225 MiB off-host, 38 MiB on
the Pi, and 94 MiB on the SBC.
