---
title: Known Gaps
description: Current open PatrolBot commissioning and platform work, updated 2026-07-22.
---

# Known Gaps

The stationary SBC/Pi cutover and safe software remediation are deployed. The
items below remain open because they require physical inspection, supervised
actuation, a real hardware signal, or platform replacement. The authoritative
finding-by-finding index is `PB-AUDIT-STATUS-SUMMARY.md` in the repository root.

## Physical safety commissioning

- Validate both hardware E-stops and any required physical PWR ON recovery.
- Attach and stationary-test the joystick receiver, centered sticks, deadman,
  timeouts, priority, and zero output before allowing manual motion.
- Map every bumper segment and verify firmware BumpStall before enabling the
  installed fail-closed bumper policy.
- Verify rear-sonar geometry/dropout behavior and the low-obstacle sensor
  inventory; measure the true physical envelope and main-LiDAR height before
  changing footprint or TF values.
- Query the µARCS `HasGyro` setting and perform a later controlled heading test.

## Docking and motion

- Survey and configure the real dock pose, then commission the guarded
  `/patrolbot/initialize_dock_pose` convergence path.
- Validate successful explicit motor enable and any physical PWR ON sequence.
- After exact bounds and explicit operator approval, run the very short guarded
  reverse test, then the complete undock/re-dock/charging regression.
- Keep generic Nav2 docking disabled and `/input/docking` publisher-free until
  those tests pass.

## SBC platform

- Replace the RTC battery and verify clock/journal/user-service persistence
  across a cold boot.
- Implement graceful shutdown only after a real power-fail signal is wired and
  demonstrated; the current serial-to-TCP topology does not expose the original
  RI signal to Linux.
- Replace or retire the EOL Ubuntu 16.04/i686 SBC. Key-only/no-root-login SSH
  reduces exposure but cannot make the platform supportable.

## Minor repository hygiene { #code-hygiene }

- `patrolbot_bridge` still has scaffold-default package metadata. This is not a
  runtime or safety defect and can be corrected with a later image revision.
