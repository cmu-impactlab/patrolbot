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

## 2026-07-26 guarded-undock continuation

The main charger pose is commissioned and the existing guarded initializer
repeatedly produced healthy AMCL convergence. A stationary-pose freshness bug
was found when the first goal failed before release; the dock manager now uses
AMCL's existing no-motion update service rather than adding a new node or
command path. The deployed image is
`patrolbot:amcl-refresh-20260726t-lab` with exact source digest
`f2722d806d8cf3a0fde89ad356379496e4640fef04e9d77cb861936cefde045b`;
82 focused tests passed and older images remain available for rollback.

A later release/enable race correction kept the existing typed services inside
one operator-authorized action. The final image passed 95 focused tests, the
forced C++ protocol test, and both isolated ROS integrations. Live explicit
motor enable succeeded and the shared Nav2 BackUp path moved
`0.10012492537498474 m` straight backward at `0.05 m/s`, with fresh rear
sonar, charge `0`, E-stop clear, and zero rotation throughout motion.

The action still failed closed during electrical-clear confirmation because
charging briefly re-latched after the robot stopped. Hardware later stabilized
at charge `0` for a six-second sample, both bumpers clear, motors disabled,
faults zero, and zero odometry twist. Thus the bounded reverse is physically
commissioned, while sufficient clearance distance, a complete successful
undock action, joystick acceptance, and supervised re-docking remain pending.

## 2026-07-26 production-undock completion

The final immutable bridge/navigation image,
`patrolbot:undock-complete-20260726t1242z`, and SBC executable
`97655f948a57c1b71d147272639ae5460feace7aa5b9a31b25c0c2f3f4472c7e`
completed goal `bb6701246df5443882920929dfd22b6b`. The guarded action released
charging, explicitly enabled the motors, reversed
`0.5031948089599609 m` at `0.08 m/s`, completed the requested 180-degree
turn at a 0.70 rad/s cap, stopped, disabled motors, and finalized charge
release. It returned code `0`, state `COMPLETE`, and action status `SUCCEEDED`
without manual recovery.

The exact image passed 96 focused tests, the forced C++ protocol test, and two
isolated ROS integrations. Post-run checks showed zero odometry, motors
disabled, E-stop clear, no faults or bumper contacts, no active action, one
hardware client, and one final `/cmd_vel` publisher. Raw charge state later
re-latched to float state `3` away from dock contact; that firmware/controller
behavior remains reported but no longer interrupts the scoped departure.

The implementation and audit were committed as
`67bbb361163ec6a0db8a4070089c6a5a594b5771`. The source-identical production
image `patrolbot:git-67bbb361`, manifest
`sha256:bad7002e3097d3291d9d51d9557756eba2ba48f14a07436c613b84ff5585afcf`,
passed the same tests and is deployed to bridge/navigation. The final
no-motion readiness gate returned `OVERALL=ready`.

PB-AUDIT-002 is closed for its original inability-to-leave-the-dock finding.
Automatic docking stays disabled; joystick/deadman motion, enabled-motor E-stop
cutoff, and the remaining physical sensor/calibration work remain open.

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
