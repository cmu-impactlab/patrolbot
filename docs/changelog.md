---
title: Changelog
description: CalVer release history for the PatrolBot system and its documentation.
---

# Changelog

PatrolBot uses [CalVer](https://calver.org/) in `YYYY.MM` form. A second release cut in the same
month takes an incrementing suffix — `2026.08.1`, `2026.08.2`.

Entries describe what changed **in the system**. They are not a work log: an entry appears when
behavior, configuration, or documented fact changes, not when work starts.

ROS package versions in `package.xml` follow their own upstream expectations and are not
governed by this scheme.

## 2026.08

**Documentation**

- Documentation is now written against a defined standard — Diátaxis page structure,
  REP-conformant units and frame names, and RFC 2119 keywords for load-bearing values. See
  [Documentation Standards](contributing/documentation-standards.md).
- Added [Safety Chain](safety/safety-chain.md), documenting every layer between a velocity
  command and the motors.
- Added [Running the Robot](operations/running-the-robot.md),
  [Troubleshooting](troubleshooting/symptoms.md), and [Maintenance](maintenance/upkeep.md).
- Added reference pages for [Configuration](reference/configuration.md),
  the [Wire Protocol](reference/wire-protocol.md), and [Limitations](reference/limitations.md).
- Removed the status-tracking page that mirrored internal audit records. The site now describes
  the deployed system only.

**Corrections to previously documented facts**

- Sonar was documented as feeding visualization only. It is the observation source for the
  collision monitor's `rear_velocity_stop` polygon at `min_points: 1`, so a single echo halts
  reverse motion. It remains absent from the costmaps, so Nav2 never plans around it.
- The two velocity smoothers were documented as one. Nav2 is capped at 0.26 m/s upstream;
  everything, including the joystick, is capped at 0.30 m/s and 0.7 rad/s downstream of the
  mux. Nav2's configured 1.0 rad/s is never reached.
- The Pi backup location is `/home/ubuntu/backups/`. The previously documented
  `/home/ubuntu/patrolbot-backups/` does not exist, and there are no `SHA256SUMS` files to
  verify against.
- `bumper_enforcement_enabled` is `False` on the deployed robot, so bumper contact does not stop
  it. This was previously implied to be active.

<!-- SPDX-License-Identifier: Apache-2.0 -->
