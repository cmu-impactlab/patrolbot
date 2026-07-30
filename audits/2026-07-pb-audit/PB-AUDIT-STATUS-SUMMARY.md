# PB-AUDIT Status Summary

Status date: 2026-07-30.

This file is the short completion index for the 26 findings in
`PATROLBOT_RPI5_AUDIT.md`. Detailed evidence remains in the canonical audit and
the separate implementation/cutover reports.

Classification rules:

- **Fully completed:** the original finding is resolved for its stated scope.
- **In progress:** remediation, staging, or partial verification exists, but at
  least one required acceptance item or dependency remains.
- **Not touched:** no remediation has been performed beyond investigation and
  documentation.

## Fully completed — 11

| ID | Short status |
|---|---|
| PB-AUDIT-001 | Unsafe active docking bypass contained and live-verified. |
| PB-AUDIT-002 | The production undock action now releases charging, explicitly enables motors, reverses 0.50 m through the guarded Nav2 path, turns away 180 degrees, and ends stopped with motors disabled. |
| PB-AUDIT-011 | Original charging-state interface defect corrected and stationary release/recharge transition verified. |
| PB-AUDIT-013 | Compatible user unit and bounded persistent journal survived a real power cycle and retained both boot records. |
| PB-AUDIT-014 | Final velocity-smoother configuration is deployed and live-verified at 0.30 m/s, matching the SBC drive gate. |
| PB-AUDIT-016 | Unexplained-outage cause resolved from operator and log evidence; prevention is tracked by PB-AUDIT-012. |
| PB-AUDIT-017 | External DDS command-graph exposure contained with loopback-only discovery. |
| PB-AUDIT-019 | Proven-dead launch/container artifacts removed; retained source/images have explicit offline rollback roles. |
| PB-AUDIT-020 | Commit-SHA-labelled image/source provenance is deployed and live-verified with checksum-backed rollback. |
| PB-AUDIT-024 | Informational protocol-design review accepted; no corrective action required. |
| PB-AUDIT-025 | Web-bridge image rebuilt from the running core image; all four `BaseState` endpoints share one type hash and dock telemetry reaches the dashboard again. Readiness now compares interface hashes. |

## In progress — 14

| ID | Short status |
|---|---|
| PB-AUDIT-003 | Recovery-tree implementation is complete and the shared Nav2 BackUp path moved 0.100125 m under live guarding; the full recovery-tree regression remains. |
| PB-AUDIT-004 | Rear-sonar integration stayed fresh and clear during the live reverse; obstacle/dropout stop tests and geometry validation remain. |
| PB-AUDIT-005 | Low-obstacle sensor scaffold exists; hardware presence, wiring, geometry, and commissioning remain. |
| PB-AUDIT-006 | All 12 panels are physically mapped and the typed undock gate is commissioned; firmware BumpStall and enabled-motion cutoff validation remain. |
| PB-AUDIT-007 | The measured main-charger pose and guarded initializer are commissioned; restart-away-from-dock recovery and general navigation gating remain. |
| PB-AUDIT-008 | Measurement/calibration scaffold exists; real envelope measurement and footprint update remain. |
| PB-AUDIT-009 | DRIVE-triggered auto-enable is fixed; explicit post-release motor enable, enabled feedback, and automatic disable after charge reassertion are live-verified. Enabled-motor E-stop cutoff remains. |
| PB-AUDIT-010 | System clock and persistent Chrony policy are corrected and survived reboot; RTC-battery replacement plus a post-replacement cold boot remain. |
| PB-AUDIT-012 | Pi fault/battery protections are deployed; real SBC power-fail signal and graceful shutdown path remain. |
| PB-AUDIT-015 | The F710 receiver and centered `/joy` path are live-verified; the 0.35/0.30 m/s rejection mismatch is fixed, while supervised full-stick, release, and timeout motion acceptance remains. |
| PB-AUDIT-018 | Direction-aware mitigations and opt-in sensor scaffold exist; physical near-field validation remains. |
| PB-AUDIT-021 | Measurement gate exists; main-LiDAR height measurement and TF correction remain. |
| PB-AUDIT-022 | Key-only/no-root-login SSH hardening is live-verified; eventual SBC platform replacement remains. |
| PB-AUDIT-026 | Deployed and live-verified: readiness now reports `ready_for_undock: true`, `blocking_reason: ready` and `localization_age` 0.20 s where it read `localization invalid` at 7.8-27.3 s. Closure still needs the supervised undock regression with the re-seed loop deliberately stopped. |

## Not touched — 1

| ID | Short status |
|---|---|
| PB-AUDIT-023 | Gyro firmware mode and physical performance remain unqueried and unvalidated. ARIA's `ArRobotConfigPacketReader` does expose `getHasGyro()`, `getGyroType()` and `getGyroFWVersion()`, and `patrolbot-sh.p` carries only `GyroScaler` (no `HasGyro`), so the answer lives in firmware and the deployed server never asks for it. Reading it needs a server change plus a restart window — it is not obtainable read-only while the server owns `ttyS2`. |

## Added after the original 24 — 2026-07-30

Both new findings were confirmed live while diagnosing an operator report that
the dashboard said the dock observer was not returning valid data. The dock
observer itself was healthy throughout (`dock_state: DOCKED_CONFIRMED`,
`dock_state_valid: true`, `telemetry_age: 0.21`, commissioned undock profile
loaded on the SBC). PB-AUDIT-025 is the reported symptom; PB-AUDIT-026 is the
refusal that surfaces next once the dashboard can see the robot again, because
the dock-pose re-seed loop that normally masks it lives in the very node
PB-AUDIT-025 detached. No robot state was changed during the investigation.

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

## Latest guarded-undock commissioning — 2026-07-26

The operator supplied a map-frame pose while the robot was seated on the main
charger. The measured pose `(99.28925458355519, 126.33778290362004,
-1.578938606346 rad)` is now the configured `main_charger` pose. The existing
`/patrolbot/initialize_dock_pose` service published it only after confirming
charging, motors disabled, stationary odometry, valid hardware state, and no
competing action; AMCL returned a converged pose with covariance below the
manager limits.

The first guarded goal exposed a real stationary-localization defect: AMCL
normally publishes once after initialization and then remains silent until
motion, while the manager required pose age below 0.5 s throughout precheck.
That goal aborted before release with `distance_traveled=0.0`. The manager now
uses AMCL's existing `/request_nomotion_update` service during precheck, the
operator motor window, the final localization gate, and guarded reverse. No
new node or velocity path was added. Image
`patrolbot:amcl-refresh-20260726t-lab`, source digest
`f2722d806d8cf3a0fde89ad356379496e4640fef04e9d77cb861936cefde045b`,
is deployed to the navigation container; 82 focused dock/navigation contract
tests passed. Previous images remain present for rollback.

The post-fix guarded goal proved stable precheck and no-motion AMCL refresh,
then successfully changed charge state from `3` to `0`. The operator motor
window expired with motors still disabled, so the manager aborted before
BackUp with `distance_traveled=0.0` and `manual_recovery_required=true`.
Following the manufacturer procedure, the operator held the dock's yellow
manual-charge button and restored charging. Live state then showed charge
state `2`, motors disabled, E-stop clear, faults zero, front dock contact only,
zero odometry twist, one final `/cmd_vel` publisher, and another successful
guarded dock-pose initialization.

That operator-window-only result was superseded later the same day by the
successful motor-enable and physical reverse evidence below.

## Live motor-enable and guarded reverse — 2026-07-26

The SBC and Pi release/enable race was corrected without adding another motion
node or command publisher. The deployed SBC accepts motor enable after a
same-session, recently confirmed `CHARGE_RELEASE` when the commissioned front
dock contact proves the robot is still seated. It reissues `CHARGE 0`
immediately before enable and permits only a short straight reverse while
contact charging is transiently visible. The Pi bridge preserves the same
UUID/session boundary, and the dock manager invokes both existing typed
services inside the operator-authorized action before starting Nav2 BackUp.
No deployed source path sends `CHARGE 1`; the observed recharge was the
charger/controller physically re-latching while the plates remained close.

The deployed navigation image is
`patrolbot:instant-undock-20260726t-lab3`, revision
`working-tree-20260726-instant-undock3`, with embedded source digest
`1ecf1b115bef2dea77d2701baec7bed1aa8a9ae3fa053b6ae5b2e7dbddd76e32`
and image manifest digest
`sha256:2d71e8b2ce23759d030efc0c6c0f8cd777ecdd2b91768d07256f4481872e95ab`.
The exact image passed 95 focused Python tests, the forced C++ protocol test,
and both isolated ROS undock integrations. Only the navigation container was
recreated; bridge and bringup remained healthy and the hardware link retained
one TCP client.

At `2026-07-26T11:09:34Z`, goal
`db6b5d9077a049d7a0e3a3cbde0a563f` was accepted with the explicitly announced
validation bounds: 0.10 m straight backward, 0.05 m/s, and zero rotation. The
action:

- refreshed and validated AMCL, rear sonar, bumpers, odometry, the command
  path, E-stop, faults, and the single hardware session;
- changed charge state from `3` to `0`;
- explicitly enabled the motors and observed `motors_enabled=true`;
- entered `GUARDED_REVERSE` approximately 0.42 s after the enable phase began;
- held charge state `0`, E-stop clear, rear sonar clear, and localization valid
  throughout the reverse; and
- completed the Nav2 BackUp with `distance_traveled=0.10012492537498474`.

The action intentionally returned `SAFETY_FAULT` during `CLEAR_OF_DOCK`
because charge state briefly returned to `3` inside its two-second confirmation
window, even though both bumper contacts were clear. The hardware subsequently
reported charge state `0` continuously for a six-second sample, both bumpers
clear, motors disabled, faults zero, navigation idle, and zero odometry twist.
The operator confirmed the physical sequence: charging stopped, the robot
moved slightly away, contact charging briefly re-latched, and then released.

This commissions the explicit motor-enable path and the bounded guarded
reverse without overstating the action result. PB-AUDIT-002 remains in
progress because the production undock distance/clear-confirmation behavior
must be tuned from this measured result, followed by one successful complete
action and a supervised re-docking/charging regression. A proposed 1.5 m/s
departure was rejected: the live undock contract is capped at 0.10 m/s, the
commissioning run used 0.05 m/s, and the correct next variable is sufficient
clearance distance rather than an unsafe speed increase.

## Successful production undock — 2026-07-26

PB-AUDIT-002 is now complete for its original finding: the robot can leave the
charger through a bounded, guarded, operator-authorized action. The final
immutable image is `patrolbot:undock-complete-20260726t1242z`, revision
`ca66321-undock-complete`, source-tree digest
`9d32c4ed6a444c7fe9c9a4caa331bd1b895070a5d6ebe8b0e3f6fe309817a8ad`,
and image manifest
`sha256:6f01f7a385760069a046c6968c52966a4a047365924c90cd241c5e66c7262622`.
The deployed SBC executable is
`97655f948a57c1b71d147272639ae5460feace7aa5b9a31b25c0c2f3f4472c7e`.
After physical acceptance, the implementation was committed as
`67bbb361163ec6a0db8a4070089c6a5a594b5771` and the source-identical image
`patrolbot:git-67bbb361`, manifest
`sha256:bad7002e3097d3291d9d51d9557756eba2ba48f14a07436c613b84ff5585afcf`,
was tested and deployed to bridge/navigation. The final no-motion readiness
gate returned `OVERALL=ready`.

Goal `bb6701246df5443882920929dfd22b6b` passed the live preconditions with one
TCP hardware client, one final `/cmd_vel` publisher, fresh and clear rear
sonar, valid bumper telemetry, charging state `3`, motors disabled, E-stop
clear, zero faults, stationary odometry, and a freshly converged dock pose.
It then:

- released charging and observed raw state `0`;
- explicitly enabled the motors;
- reversed `0.5031948089599609 m` at the requested `0.08 m/s`;
- completed the existing Nav2 Spin goal for a 180-degree turn, capped at
  `0.70 rad/s`;
- stopped and disabled the motors;
- retried the final release exactly once after the first request correctly
  detected residual ARIA rotation; and
- returned `success=true`, code `0`, state `COMPLETE`,
  `manual_recovery_required=false`, and action status `SUCCEEDED`.

The action completed with raw charge state `0`; later telemetry returned to
raw float state `3` even though both bumpers were clear and the robot had
physically moved away. That known controller relatch is no longer allowed to
interrupt the scoped reverse or turn. It remains visible rather than being
masked: the post-run sample was 25.4 V, motors disabled, no faults, zero
odometry twist, no active action, and one `/cmd_vel` publisher. Automatic
docking, joystick/deadman motion, enabled-motor E-stop cutoff, and the other
physical sensor campaigns remain separate open findings.

Current finding counts are **10 fully completed, 13 in progress, and 1 not
touched**.

## PB-AUDIT-025 closed and PB-AUDIT-026 fixed in repo — 2026-07-30

PB-AUDIT-025 is closed on the robot. The dashboard bridge image is now built
from the exact core image the containers run (`docker/Dockerfile.web-bridge`),
so its `patrolbot_interfaces` cannot lag behind the publisher again. Live
`/patrolbot/base_state` carries one publisher and three subscriptions with a
single type hash `RIHS01_c42605b3...`, the bridge reports
`dock capabilities: undock, charge_release, motor_enable`, and its 30 s
dock-pose re-seed loop resumed. `docker/status.sh` now compares the RIHS01
hashes of `BaseState` and `DockReadiness` across every container that carries
the interfaces, read from each image's installed type descriptions rather than
from the DDS graph — an unmatched subscription is absent from
`ros2 topic info -v` entirely, so the graph could not have detected this.
Only the web-bridge container was recreated; bridge, navigation and bringup
stayed healthy and no robot state changed.

PB-AUDIT-026 turned out to be two defects behind one blocking reason. The
variance half was real but transient: with the re-seed loop dead for ~24 h the
docked x-variance had grown to 0.57-0.63, while live covariance after the fix
is `xx=0.041, yy=0.033, yaw=0.053`, inside the existing 0.25 gate. The half
that actually blocks an idle robot is freshness — a stationary AMCL publishes
nothing, so the docked pose is always older than the 0.5 s `sensor_timeout`
(measured 7.8 s and 12.3 s), and readiness reported a blocker the precheck
clears itself. Both are fixed in the repo and unlaunched: a docked-and-idle
no-motion refresh, and an evidence-gated `docked_max_position_variance` that
requires mated plates *and* agreement with the surveyed dock pose, leaving
`max_position_variance` untouched for general navigation.

Counts are now **11 fully completed, 14 in progress, and 1 not touched**.

## PB-AUDIT-026 deployed — 2026-07-30

The fix is on the robot. Commit `dabeb87980670df9ee172f6cfb5f7e27b4978164`
(branch `sbc-owned-undock`, pushed to origin) built image
`patrolbot:git-dabeb879` with source digest
`62300d8e436bbb54f7cce014303b0df31edc41f6d538a29647776a49a24d55a5`. The exact
image passed 69 dock-manager checks, 50 bridge checks with 11 skips, and 59
navigation checks with 10 skips before anything was recreated. Only
`patrolbot-navigation` was recreated; the web-bridge image was then re-derived
from the same core image so the PB-AUDIT-025 invariant still holds.

Live evidence after the change: `ready_for_undock: true`,
`blocking_reason: ready`, `localization_fresh`/`localization_valid`/
`dock_pose_valid` all true, `localization_age` 0.203 s against the earlier
7.8-27.3 s, `dock_state: DOCKED_CONFIRMED`, charge state 2, motors disabled,
E-stop clear, zero faults, stationary, rear clear, one `/cmd_vel` publisher and
one subscriber, zero odometry twist on both axes, four healthy containers with
zero restarts, and `status.sh` reporting verified provenance, one `BaseState`
hash, one `DockReadiness` hash and `OVERALL=ready`. No velocity, goal, pose,
charge-release, motor-enable or undock command was sent.

Two consequences to carry forward. First, `ready_for_undock` is now true while
the web bridge runs with commands enabled, so the dashboard's Undock button is
accepted rather than refused — deliberate, and the precondition for the
supervised regression, but it changes what a dashboard press does. Second,
`patrolbot-bridge` and `patrolbot-bringup` still carry older image labels; their
behaviour already comes from the committed tree through the read-only bind
mounts and no message definition changed, so every interface hash still agrees,
but the full single-identity cutover of those two containers remains.

Rollback lives in
`/home/ubuntu/patrolbot-backups/20260730T133000Z-pre-audit026-deploy/` with the
pre-change `.env`, a checksummed copy of the previous source tree, the four
container inspects and the previous deployed digest `4ec08f46...`.

## Detailed evidence

- `PATROLBOT_RPI5_AUDIT.md` — canonical finding definitions and current
  dispositions.
- `PB-AUDIT-001-002-IMPLEMENTATION-REPORT.md` — implementation and deployment
  evidence.
- `PB-AUDIT-002-STATIONARY-CUTOVER-REPORT.md` — SBC cutover and zero-motion
  protocol evidence.
- `PI5-SEVEN-ITEM-HARDENING-IMPLEMENTATION.md` — Pi hardening evidence.
