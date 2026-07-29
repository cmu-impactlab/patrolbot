---
title: Known Gaps
description: Current open PatrolBot commissioning and platform work, updated 2026-07-23.
---

# Known Gaps

The stationary SBC/Pi cutover and safe software remediation are deployed. The
items below remain open because they require physical inspection, supervised
actuation, a real hardware signal, or platform replacement. The authoritative
finding-by-finding index is
`audits/2026-07-pb-audit/PB-AUDIT-STATUS-SUMMARY.md`.

## Physical safety commissioning

- Validate the single hardware E-stop's physical motor-power cutoff with motors
  enabled. Its button signal, priority-10 hold, and required physical PWR ON
  recovery are already stationary-validated.
- Attach and stationary-test the joystick receiver, centered sticks, deadman,
  timeouts, priority, and zero output before allowing manual motion.
- Map every bumper segment and verify firmware BumpStall before enabling the
  installed fail-closed bumper policy.
- Verify rear-sonar geometry/dropout behavior and the low-obstacle sensor
  inventory; measure the true physical envelope and main-LiDAR height before
  changing footprint or TF values.
- Query the µARCS `HasGyro` setting and perform a later controlled heading test.

## Docking and motion

Contact separation is now an exclusive SBC operation (see
[State machines](internals/state-machines.md)). What remains is measurement, not design.

- Commission the undock hardware profile
  (`hardware/patrolbot_hw_server/undock_profile.conf`). Until then a production
  undock is refused with `HARDWARE_PROFILE_UNCOMMISSIONED` by design. Unmeasured:
  `CHARGE 0` release/re-assert timing, E-stop signal semantics, rear-sonar
  indices and near-field floor, and the idle fault/stall flags behind
  `critical_fault_mask`. `patrolbot_commission_probe` collects all four.
- Encoder fidelity on the charging plates: compare SBC encoder progress, Pi
  `/odom`, a tape measure and LiDAR range to the dock during the first
  supervised run. Completion rests on that displacement.
- Establish which signal changes first when motion stops during a charge
  re-latch — SBC gate, µARCS directly, mechanical latch → stall, or watchdog.
- Validate the redock inhibition policy (60 s / 0.50 m) against the dock's real
  capture area and the battery policy.
- Bumper mapping is **established** (front `0x7e00`, rear `0x007e`, pressed
  segment by segment on 2026-07-23); what remains is a functional regression
  check that each segment sets only its own bit and that none stays asserted.
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
