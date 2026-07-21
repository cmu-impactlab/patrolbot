# PB-AUDIT Status Summary

Status date: 2026-07-22.

This file is the short completion index for the 24 findings in
`PATROLBOT_RPI5_AUDIT.md`. Detailed evidence remains in the canonical audit and
the separate implementation/cutover reports.

Classification rules:

- **Fully completed:** the original finding is resolved for its stated scope.
- **In progress:** remediation, staging, or partial verification exists, but at
  least one required acceptance item or dependency remains.
- **Not touched:** no remediation has been performed beyond investigation and
  documentation.

## Fully completed — 8

| ID | Short status |
|---|---|
| PB-AUDIT-001 | Unsafe active docking bypass contained and live-verified. |
| PB-AUDIT-011 | Original charging-state interface defect corrected and stationary release/recharge transition verified. |
| PB-AUDIT-014 | Final velocity-smoother configuration corrected, deployed, and live-verified. |
| PB-AUDIT-016 | Unexplained-outage cause resolved from operator and log evidence; prevention is tracked by PB-AUDIT-012. |
| PB-AUDIT-017 | External DDS command-graph exposure contained with loopback-only discovery. |
| PB-AUDIT-019 | Proven-dead launch/container artifacts removed; retained source/images have explicit offline rollback roles. |
| PB-AUDIT-020 | Commit-SHA-labelled image/source provenance is deployed and live-verified with checksum-backed rollback. |
| PB-AUDIT-024 | Informational protocol-design review accepted; no corrective action required. |

## In progress — 15

| ID | Short status |
|---|---|
| PB-AUDIT-002 | Stationary cutover and release/replay complete; physical departure and re-docking campaign remains. |
| PB-AUDIT-003 | Recovery-tree implementation complete; live motion regression remains. |
| PB-AUDIT-004 | Rear-sonar integration complete; physical geometry, dropout, and stop validation remain. |
| PB-AUDIT-005 | Low-obstacle sensor scaffold exists; hardware presence, wiring, geometry, and commissioning remain. |
| PB-AUDIT-006 | Bumper safety path exists but is intentionally disabled until physical mapping and firmware validation. |
| PB-AUDIT-007 | Safe dock-pose initialization mechanism exists; surveyed pose and localization commissioning remain. |
| PB-AUDIT-008 | Measurement/calibration scaffold exists; real envelope measurement and footprint update remain. |
| PB-AUDIT-009 | DRIVE-triggered auto-enable defect is fixed; successful explicit-enable/PWR ON validation remains. |
| PB-AUDIT-010 | System clock and persistent Chrony policy are corrected; RTC battery replacement and reboot validation remain. |
| PB-AUDIT-012 | Pi fault/battery protections are deployed; real SBC power-fail signal and graceful shutdown path remain. |
| PB-AUDIT-013 | Compatible user unit and bounded persistent journal are live; reboot-persistence validation remains. |
| PB-AUDIT-015 | Device/readiness support exists; joystick receiver and stationary control validation remain. |
| PB-AUDIT-018 | Direction-aware mitigations and opt-in sensor scaffold exist; physical near-field validation remains. |
| PB-AUDIT-021 | Measurement gate exists; main-LiDAR height measurement and TF correction remain. |
| PB-AUDIT-022 | Key-only/no-root-login SSH hardening is live-verified; eventual SBC platform replacement remains. |

## Not touched — 1

| ID | Short status |
|---|---|
| PB-AUDIT-023 | Gyro firmware mode and physical performance remain unqueried and unvalidated. |

## Detailed evidence

- `PATROLBOT_RPI5_AUDIT.md` — canonical finding definitions and current
  dispositions.
- `PB-AUDIT-001-002-IMPLEMENTATION-REPORT.md` — implementation and deployment
  evidence.
- `PB-AUDIT-002-STATIONARY-CUTOVER-REPORT.md` — SBC cutover and zero-motion
  protocol evidence.
- `PI5-SEVEN-ITEM-HARDENING-IMPLEMENTATION.md` — Pi hardening evidence.
