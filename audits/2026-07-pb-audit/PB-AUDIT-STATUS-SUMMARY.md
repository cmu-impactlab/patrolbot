# PB-AUDIT Status Summary

Status date: 2026-07-23.

This file is the short completion index for the 24 findings in
`PATROLBOT_RPI5_AUDIT.md`. Detailed evidence remains in the canonical audit and
the separate implementation/cutover reports.

Classification rules:

- **Fully completed:** the original finding is resolved for its stated scope.
- **In progress:** remediation, staging, or partial verification exists, but at
  least one required acceptance item or dependency remains.
- **Not touched:** no remediation has been performed beyond investigation and
  documentation.

## Fully completed — 9

| ID | Short status |
|---|---|
| PB-AUDIT-001 | Unsafe active docking bypass contained and live-verified. |
| PB-AUDIT-011 | Original charging-state interface defect corrected and stationary release/recharge transition verified. |
| PB-AUDIT-013 | Compatible user unit and bounded persistent journal survived a real power cycle and retained both boot records. |
| PB-AUDIT-014 | Final velocity-smoother configuration corrected, deployed, and live-verified. |
| PB-AUDIT-016 | Unexplained-outage cause resolved from operator and log evidence; prevention is tracked by PB-AUDIT-012. |
| PB-AUDIT-017 | External DDS command-graph exposure contained with loopback-only discovery. |
| PB-AUDIT-019 | Proven-dead launch/container artifacts removed; retained source/images have explicit offline rollback roles. |
| PB-AUDIT-020 | Commit-SHA-labelled image/source provenance is deployed and live-verified with checksum-backed rollback. |
| PB-AUDIT-024 | Informational protocol-design review accepted; no corrective action required. |

## In progress — 14

| ID | Short status |
|---|---|
| PB-AUDIT-002 | Stationary cutover/release and all 12 bumper mappings complete; dock localization, guarded departure, and re-docking remain. |
| PB-AUDIT-003 | Recovery-tree implementation complete; live motion regression remains. |
| PB-AUDIT-004 | Rear-sonar integration complete; physical geometry, dropout, and stop validation remain. |
| PB-AUDIT-005 | Low-obstacle sensor scaffold exists; hardware presence, wiring, geometry, and commissioning remain. |
| PB-AUDIT-006 | All 12 panels are physically mapped and the typed undock gate is commissioned; firmware BumpStall and enabled-motion cutoff validation remain. |
| PB-AUDIT-007 | Safe dock-pose initialization mechanism exists; surveyed pose and localization commissioning remain. |
| PB-AUDIT-008 | Measurement/calibration scaffold exists; real envelope measurement and footprint update remain. |
| PB-AUDIT-009 | DRIVE-triggered auto-enable defect is fixed; successful explicit-enable/PWR ON validation remains. |
| PB-AUDIT-010 | System clock and persistent Chrony policy are corrected and survived reboot; RTC-battery replacement plus a post-replacement cold boot remain. |
| PB-AUDIT-012 | Pi fault/battery protections are deployed; real SBC power-fail signal and graceful shutdown path remain. |
| PB-AUDIT-015 | F710 deadman/takeover software is deployed; the absent receiver still blocks physical control validation. |
| PB-AUDIT-018 | Direction-aware mitigations and opt-in sensor scaffold exist; physical near-field validation remains. |
| PB-AUDIT-021 | Measurement gate exists; main-LiDAR height measurement and TF correction remain. |
| PB-AUDIT-022 | Key-only/no-root-login SSH hardening is live-verified; eventual SBC platform replacement remains. |

## Not touched — 1

| ID | Short status |
|---|---|
| PB-AUDIT-023 | Gyro firmware mode and physical performance remain unqueried and unvalidated. |

## Latest safe non-motion completion — 2026-07-23

A real operator power cycle supplied the deferred PB-AUDIT-013 acceptance
evidence. The enabled `patrolbot-server.service` started automatically, opened
the expected binary SHA-256 `b9606502...94393`, initialized the laser, and
accepted exactly one Pi client. The 16 MiB persistent journal retained the
complete previous clean shutdown as boot `-1` and recorded the current server
startup as boot `0`. This closes PB-AUDIT-013.

The same boot confirms why PB-AUDIT-010 remains open: the SBC began from its
bad 2008 RTC time, then the deployed Chrony policy automatically stepped it to
current 2026 time and synchronized normally. RTC-battery replacement and a
post-replacement cold-boot check are still required.

The live stationary baseline after reconnection reported charge state `3`,
motors disabled, E-stop engaged by the operator, zero faults, zero `/cmd_vel`
and odometry twist, fresh scan/sonar/battery/base telemetry, and one final
`/cmd_vel` publisher. No velocity, goal, pose, docking, motor, release, or
hardware command was sent.

The stationary E-stop signal and recovery path was then commissioned. Physical
inspection confirmed this robot has one top/rear twist-release E-stop, despite
stray plural wording in two manufacturer-guide procedures. Releasing the
button alone correctly left the ESRB latch asserted; pressing physical PWR ON
changed typed E-stop state from true to false while charging stayed at `3`,
motors stayed disabled, and odometry stayed zero. Pressing the E-stop again
changed it false to true and caused the priority-10 watchdog to publish zero;
final command and odometry remained zero. Actual motor-power removal while
motors are enabled remains coupled to the later supervised motor test.

The final pre-dashboard inventory still found no non-hub USB device on either
computer, no Pi serial device, no SBC USB/ACM serial device, and only the four
legacy 16550A UARTs. The configured URG Laser 2 remains disabled on unavailable
`COM5`, so PB-AUDIT-005 cannot advance without physical sensor/cable
inspection. The joystick receiver also remains absent, leaving PB-AUDIT-015
open. Bumper mapping was prepared but no raw transition was observed; no mask
was guessed or enabled.

All current audit records were consolidated under
`audits/2026-07-pb-audit/`; the superseded first-pass audit was deleted. After
re-verifying every retained checksum manifest, the superseded July 16 backup
generations were deleted from the workstation, Pi, and SBC. The verified July
21 cutover, old-binary, container-cleanup, root-maintenance, provenance, and
post-change rollback sets remain.

The tracked operational documentation was reconciled with the commissioned
runtime after a source-to-doc contradiction sweep. It now consistently records:

- motors disabled at SBC startup and no DRIVE-triggered enable;
- separate, UUID-correlated `CHARGE_RELEASE` and `MOTOR_ENABLE` operations;
- the upstream autonomous mux, direction-aware scan/sonar collision path, and
  single final `/cmd_vel` publisher;
- the fail-closed dock manager, disabled generic docking server, and invalid
  localization until an authorized pose converges;
- loopback-only DDS with SSH/reviewed-bridge operator access; and
- commit/digest-backed deployment and checksum-verified rollback practice.

The root `AGENTS.md` was also removed and its existing repository-wide ignore
rule was allowed to apply, as requested. This housekeeping change does not alter
robot runtime behavior.

Final read-only live checks at approximately 01:40 +03 showed three healthy
core containers on `patrolbot:git-b98090a7796d`, matching revision/source
provenance, fresh base state, charge state `3`, motors disabled, E-stop clear,
zero faults, zero odometry twist, one `/cmd_vel` publisher/subscriber, one TCP
7272 client, SBC PID `21102`, and unchanged server hash `b9606502...94393`.
Chrony, SSH, journald, and the hardware-server unit were active. Dashboard
command mode remained enabled but added no `/cmd_vel` publisher. No velocity,
goal, pose, hardware-control, docking, joystick, or E-stop command was sent.

## Latest bumper and undock-path commissioning — 2026-07-23

With the robot charging in float state `3`, motors disabled, E-stop clear, and
odometry zero, the operator pressed all six front bumper panels left-to-right
and then all six rear panels left-to-right. Existing typed telemetry measured
front SIP stall bits 9–14 (`0x7e00`, decimal `32256`) and rear bits 1–6
(`0x007e`, decimal `126`). The dock normally holds the two center-front inputs
`0x1800` active, matching the manufacturer procedure that approaches the
charger until a front panel triggers.

The measured masks are now loaded by the existing Pi bridge and read back live
as `bumpers_valid=true`, `front_bumper_pressed=true`, and
`rear_bumper_pressed=false`. The existing `/patrolbot/undock` manager was
corrected to permit that commissioned front-dock contact only while backing
straight away, continue to reject any rear contact, and require charging off,
stationary odometry, and every bumper clear before reporting dock departure
complete. No new motion node or direct velocity path was added.

The Pi image `patrolbot:undock-bumper-20260723t1513z` carries source digest
`8ff5d99928a7c013f4c4f6182264a823118ca7911798c4709db3ac6b67132a7d`.
Bridge and navigation containers are healthy; one `/cmd_vel` publisher remains;
fresh telemetry reports zero odometry and disabled motors. The remaining
undock blockers are the unconfigured dock pose and invalid localization. No
velocity, goal, pose, charge-release, motor-enable, or undock action was sent.

The finding counts are now **9 fully completed, 14 in progress, and 1 not
touched**. All remaining audit closure work is physical or requires a real
hardware signal/platform change:

- enabled-motor E-stop cutoff, joystick/deadman, firmware bumper cutoff, sonar, low-obstacle,
  envelope, LiDAR-height, and gyro commissioning;
- surveyed dock pose, successful explicit motor enable/PWR ON, exact-bounds
  guarded reverse, and full undock/re-dock/charging regression; and
- RTC-battery/cold-boot persistence, a demonstrated SBC power-fail shutdown
  input, and eventual EOL SBC replacement.

## Detailed evidence

- `PATROLBOT_RPI5_AUDIT.md` — canonical finding definitions and current
  dispositions.
- `PB-AUDIT-001-002-IMPLEMENTATION-REPORT.md` — implementation and deployment
  evidence.
- `PB-AUDIT-002-STATIONARY-CUTOVER-REPORT.md` — SBC cutover and zero-motion
  protocol evidence.
- `PI5-SEVEN-ITEM-HARDENING-IMPLEMENTATION.md` — Pi hardening evidence.
