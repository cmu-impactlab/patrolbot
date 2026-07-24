# PB-AUDIT-002 Stationary SBC Cutover Report

## Status

**Stationary cutover and zero-motion charge-release commissioning complete.**

On 2026-07-21 the verified Pi image and 32-bit SBC server were deployed, the
charging motor-enable interlock was exercised, and one charge-release request
was verified together with its idempotent replay. The robot never moved. It
ended physically seated on the charger, at stable raw charge state `3`, with
motors disabled, zero fault flags, zero odometry twist, and no active motion
command.

PB-AUDIT-002 remains open for the deliberately deferred physical campaign:
E-stop and joystick validation, bumper mapping, surveyed dock pose and
localization, successful explicit motor enable, guarded reverse, full undock,
and full re-docking regression. No velocity, pose, navigation goal,
`/patrolbot/undock` goal, or successful motor-enable request was sent in this
session.

## Final deployed state

| Item | Verified result |
|---|---|
| Pi image | `patrolbot:pb002-manual-release-20260721t210500z`; image ID `sha256:2b683343b4afdb4d1b24e7368469f94051456f9ada3e30c07d91b90d3e766d59`; revision `pb002-manual-release-20260721T210500Z` |
| Pi source provenance | Image label and deployed source both `fc0b8eabc028748873ed7af3ba2e8ef12bbad434866ed13b5ad33dbe48603eb6` |
| SBC executable | Running and deployed SHA-256 `b9606502dc815c602397f93f48a0af7eb783ab3e819d87e77fcdf01d40394393` |
| SBC source | Deployed `patrolbot_server.cpp` SHA-256 `a122210a6bc8887924cf06ec85a2e3b31d696717699397122ba8694dd33182e9` |
| Hardware link | Exactly one established TCP 7272 connection; no probe or second client opened |
| Reconnection | One planned post-SBC-cutover bridge reconnect; `session_generation` changed from `1` to `2` |
| Final hardware state | Raw charge `3`, motors disabled, E-stop reported clear, zero fault flags, bumpers invalid/unmapped |
| Final telemetry | Scan, odometry, sonar, battery, and base state fresh; three separated charge samples stable at `3`; battery 29.7 V with charging status; odometry twist exactly zero |
| Command path | One final `/cmd_vel` publisher (`teleop_velocity_smoother`), bridge as sole subscriber, command path valid, navigation manager `IDLE` |
| SBC service logging | Compatible user unit/drop-in installed and daemon-reloaded without restart; live resolution is journal/journal with identifier `patrolbot_server`; PID remained 21102 |
| Dashboard | Clean dashboard commit `d4b3cf9093927183d4c7adb1e7cb9a07529bf2c0`; standalone bridge remained running and was not recreated |

The standalone dashboard bridge still has `WEB_BRIDGE_ENABLE_COMMANDS=1`.
Inspection of that exact checkout and the live ROS graph confirmed its command
surface is limited to NavigateToPose, `/initialpose`, and goal cancellation. It
has no `/cmd_vel`, charge-release, motor-enable, undock, or direct SBC path. No
dashboard command was issued by this audit.

## Backups and rollback

The full pre-cutover backup remains:

- backup ID `20260721T181448Z-pb002-stationary-cutover`;
- SBC host-local: `/home/ros/patrolbot-backups/20260721T181448Z-pb002-stationary-cutover/`;
- Pi host-local: `/home/ubuntu/patrolbot-backups/20260721T181448Z-pb002-stationary-cutover/`;
- off-host: `backups/20260721T181448Z-pb002-stationary-cutover/`.

All 34 SBC and 167 Pi manifest entries were recomputed from the backup copies
and passed. The original live SBC executable is preserved as:

`13f2d881752552b2bff1033a42222f452ea5492bdd3bb8e5e0db926dfb9fd317`

A second narrow backup was captured immediately before deployment:

- backup ID `20260721T204423Z-pb002-manual-release-deploy`;
- SBC host-local: `/home/ros/patrolbot-backups/20260721T204423Z-pb002-manual-release-deploy/`;
- Pi host-local: `/home/ubuntu/patrolbot-backups/20260721T204423Z-pb002-manual-release-deploy/`;
- off-host: `backups/20260721T204423Z-pb002-manual-release-deploy/`.

The narrow set preserves the active Pi environment, all replaced Pi files,
container/image/source evidence, the active SBC executable and source, protocol
files, build inputs, unit/drop-in, socket state, and service journal. Every
host-local and off-host manifest entry passed `sha256sum -c` from the copied
files. Access-controlled environment data was backed up without printing its
secret values.

SBC rollback is limited to stopping `patrolbot-server.service`, atomically
restoring the verified old executable and matching source with owner `ros:ros`
and modes `0775`/`0664`, then starting that same service. Pi rollback restores
the narrow backup files and protected environment, selects the prior image
`patrolbot:pi5-hardening-20260716t221749z`, recreates only the three PatrolBot
containers, and reruns the complete safety gate. Socat, the dashboard
container, host networking, and unrelated services are not part of rollback.
Rollback was not needed.

## Manual-derived protocol correction

The third-generation user guide establishes two separate µARCS operations:

- `CHARGE` command 68, argument `0`, gracefully releases charging;
- `ENABLE` command 4 controls motor power independently.

The guide also says the charging electromagnet maintains dock contact and that
the robot may be operated while charging other than driving. It does not say
that charging mechanically locks the wheels. Live manual docking confirmed the
important implementation detail: charge state `3` can coexist with
`motors_enabled=true`.

The original staged precondition required motors already disabled before
`CHARGE_RELEASE`. That was a real deadlock: the software needed release to
leave charging but rejected release in the state produced by manual docking.
The corrected zero-motion operation now:

1. rejects motion, invalid state, and hardware faults;
2. disables motors if they are reported enabled;
3. requires three stable motor-disabled observations;
4. sends µARCS `CHARGE 0` exactly once;
5. requires three stable raw charge-state-zero observations;
6. leaves motors disabled and never calls a velocity or enable API.

Nonzero `DRIVE` remains rejected while charging. The later movement sequence is
therefore release, explicit motor enable, then a separately approved guarded
reverse—not DRIVE as a way to stop charging.

## Candidate build and Pi image

The final SBC candidate was built only in:

`/home/ros/patrolbot-builds/20260721T181448Z-pb002-stationary-cutover/`

- `make -B test`: passed;
- `make -B patrolbot_server`: passed with only legacy ARIA-header warnings;
- output: dynamically linked 32-bit i386 ELF;
- every `ldd` dependency resolved;
- candidate SHA-256:
  `b9606502dc815c602397f93f48a0af7eb783ab3e819d87e77fcdf01d40394393`;
- matching source SHA-256:
  `a122210a6bc8887924cf06ec85a2e3b31d696717699397122ba8694dd33182e9`;
- static contract proved release contains `disableMotors()` before command 68
  and contains no `setVel`, `setRotVel`, or `enableMotors` call.

The Pi candidate was built from an isolated staging tree. Its build rejected
any digest other than `fc0b8eabc028748873ed7af3ba2e8ef12bbad434866ed13b5ad33dbe48603eb6`.
All six selected ROS packages compiled. Disposable, network-isolated image
tests produced 92 focused contract passes and five fake-hardware ROS
integration passes. A broader exploratory invocation also produced 92 passes
and four expected skips but triggered two pre-existing package-wide linter
failures by scanning generated `build/` and `install/` trees; those linter
results were not used as the focused acceptance signal.

## Controlled cutover evidence

The Pi source delta was applied first and its protected environment selected
the new immutable image. Only `bringup`, `bridge`, and `navigation` were
recreated. All three returned healthy with zero restarts, their image ID and
deployed source digest matched, charging stayed at state `3`, and odometry
remained zero. The dashboard container was not touched.

Only `patrolbot-server.service` was then stopped. The staged executable and
matching source were installed by same-filesystem atomic rename, preserving
the recorded owner and modes, and only that service was started. The first
immediate ARIA connection attempt encountered the expected three-second
close/reopen race; the unit's existing restart policy retried once and PID
21102 connected successfully. The laser completed its documented power cycle
in approximately 50 seconds. At 2026-07-21T20:49:00Z the recovery gate showed:

- one established TCP client;
- `session_generation=2` after the one planned bridge reconnect;
- fresh scan, odometry, sonar, battery, and base state;
- raw charge state `3`, motors disabled, E-stop clear, faults zero;
- one final `/cmd_vel` publisher and no unexpected velocity replay;
- deployed/running executable hash equal to the candidate.

## Zero-motion protocol results

### Motor enable while charging

At approximately 2026-07-21T20:50:03Z, request
`47934916-09c4-445c-abf8-f14836667678` returned:

```text
accepted=false
success=false
code=3 (PRECONDITION_FAILED)
message="motor enable refused while charging"
request_id=47934916-09c4-445c-abf8-f14836667678
charge_state=3
motors_enabled=false
estop_pressed=false
session_generation=2
```

Motors remained disabled and odometry remained all zero.

### Charge release and replay

At approximately 2026-07-21T20:50:32Z, request
`d1e71d90-ce9f-4937-b749-0de3b9372e06` returned after 3.255 s:

```text
accepted=true
success=true
code=0 (OK)
message="charge released"
request_id=d1e71d90-ce9f-4937-b749-0de3b9372e06
charge_state=0
motors_enabled=false
estop_pressed=false
session_generation=2
```

The server had confirmed three stable raw state-zero observations. Motors and
odometry remained zero. The robot stayed physically on the charging plates and
subsequently returned to raw charge state `3` without moving.

The identical UUID was replayed at approximately 2026-07-21T20:51:03Z. It
returned the same cached success in 0.067 s with session `2`. The SBC journal
contains exactly one accepted line and one completed line for this UUID, which
proves no second hardware release was sent. The next live base-state sample
already showed the independently resumed charge state `3`; the replay response
correctly preserved the original terminal result snapshot at state `0`.

Three later samples through 2026-07-21T20:51:47Z remained stable at charge
state `3`, motors disabled, E-stop clear, zero faults, and session `2`.
`/battery` remained 29.7 V with charging status and odometry remained exactly
zero. Manual re-seating was unnecessary because contact charging resumed while
the robot stayed in place.

## Remaining work

The following are explicitly deferred and remain prerequisites for autonomous
operation:

1. stationary E-stop and joystick/deadman validation;
2. measured bumper mappings and fail-closed bumper enforcement;
3. physical rear-sonar geometry, obstacle-stop, and dropout validation;
4. surveyed dock pose and operator-authorized localization initialization;
5. successful explicit `MOTOR_ENABLE` and any required PWR ON sequence;
6. a separately approved, very short guarded reverse;
7. full undock, re-dock, charging, cancellation, link-loss, and stale-command
   regression;
8. SBC clock/RTC, root-owned persistent journaling, graceful shutdown input,
   and platform hardening findings PB-AUDIT-010/012/013/022. The PB-AUDIT-013
   user-unit/drop-in defect is already fixed live.

Before any nonzero velocity publication, the operator must receive and approve
the exact speed, duration or distance, path, sensor gates, and stop conditions.

## Verification

- isolated SBC `make -B test`: passed;
- isolated SBC rebuild, ELF/`ldd`, source hash, and candidate hash: passed;
- repository static safety contracts: 29 passed;
- focused pre-image ROS contracts: 63 passed, 5 skipped as designed;
- corrected dock-manager integration: 2 passed;
- immutable image focused contracts: 92 passed;
- immutable image fake-hardware integrations: 5 passed;
- live cutover gate: passed;
- charging motor-enable rejection: passed;
- release stable-zero confirmation: passed;
- duplicate UUID/no-second-action check: passed;
- stable automatic recharge with motors disabled and zero odometry: passed.

- final `git diff --check`: passed;
- final deterministic repository digest: `fc0b8eab...03eb6`, matching the Pi
  image and deployed source;
- final `.venv/bin/mkdocs build --strict`: passed;
- final `docker/status.sh`: container, provenance, DDS scope, command path,
  single-publisher, sensor freshness/QoS, lifecycle, odometry, and TF checks
  passed; it ended `OVERALL=degraded reason=ros-readiness` because the
  deliberately deferred localization/dock physical gates remain unavailable,
  not because of an SBC or cutover failure.

## Overnight persistence recheck — 2026-07-22

Both management aliases remained reachable. The three Pi containers were
healthy with zero restarts on the deployed PB-AUDIT-002 image, and the live Pi
source digest remained `fc0b8eab...03eb6`. The SBC service retained PID 21102,
the deployed/running binary remained `b9606502...94393`, and exactly one TCP
7272 connection remained established.

Fresh ROS evidence still reported session generation `2`, charge state `3`,
motors disabled, E-stop clear, zero faults, fresh/clear rear sonar, navigation
manager `IDLE`, and every odometry twist component equal to zero. `/cmd_vel`
still had exactly one publisher and the dashboard container remained separate
and running.

The PB-AUDIT-013 user-unit change also persisted: systemd resolved journal
output for both streams and identifier `patrolbot_server`. The narrow SBC
rollback manifest reverified successfully. Root-owned maintenance remains
unavailable because `sudo -n` is still denied; the system clock remains in
2021, Chrony reports a current 2026 source with an approximately 167-million-
second unstepped offset, and `/var/log/journal` is absent.

A finding-by-finding review found no further immediately executable no-motion
remediation. Remaining items require one of: operator/root authority, a real
power-fail signal design, attached/physical hardware and measurements,
successful motor enable and controlled motion, or review/commit of the shared
dirty worktree for PB-AUDIT-020 release identity. None was started during this
recheck.

## Root-maintenance and provenance follow-up — 2026-07-22

The later supervised non-motion session resolved the immediately addressable
parts of the final paragraph above. Runtime commit
`b98090a7796d9418cb9df92903ff56b439323114` now identifies the reviewed
remediation. Commit-labelled image `patrolbot:git-b98090a7796d` passed isolated
build, digest, 104 focused-contract passes, and five fake-hardware integration
passes, then deployed successfully. The dashboard bridge was stopped during
cutover to remove its independent goal/initial-pose endpoint and restored only
after stationary acceptance; it still has no direct velocity path.

With motors disabled and odometry zero, the SBC clock was advanced to current
time using authenticated Chrony 2.1 command mode, persistent `makestep` policy
was installed, bounded persistent journaling was enabled, and SSH was hardened
to key-only/no-root-login. Neither the hardware server nor its TCP session was
restarted. Pre/post host-local and off-host rollback sets verified, and final
telemetry remained fresh with charge state `3`, motors disabled, zero faults,
session generation `2`, one `/cmd_vel` publisher, and zero odometry.

The three core containers are healthy with zero restarts. Image revision,
embedded digest, labelled digest, and deployed source match; one planned SBC
disconnect/reconnect occurred, charge stayed `3`, motors stayed disabled,
faults stayed zero, and odometry stayed exactly zero. The recreated bridge's
session generation is correctly `1` because the counter is process-local.

Remaining SBC items are now narrower: RTC-battery replacement plus cold-boot
time/journal validation, a real graceful-shutdown input for PB-AUDIT-012, and
eventual replacement of the EOL appliance. All physical commissioning and
motion gates listed above remain unchanged.

## Power-cycle persistence acceptance — 2026-07-23

The operator subsequently performed a real shutdown and power-up. With an
E-stop latched and no motion command, the enabled hardware-server unit
auto-started, initialized the laser, and accepted exactly one Pi client. The
running binary remained SHA-256 `b9606502...94393`.

Persistent journal boot `-1` retained the complete prior clean shutdown and
boot `0` recorded the new server/laser/bridge startup. Journal usage remained
16 MiB. This closes the remaining PB-AUDIT-013 reboot-persistence acceptance.

The power cycle also reconfirmed the narrower PB-AUDIT-010 hardware defect:
the SBC initially booted with its 2008 RTC time, then the deployed Chrony
policy stepped it automatically to current 2026 time and synchronized.
RTC-battery replacement and a post-replacement cold-boot test remain open.

After reconnection, charge state was `3`, motors were disabled, the E-stop was
engaged, fault flags were zero, all final command samples and odometry twist
were zero, and scan/sonar/battery/base telemetry was fresh. No velocity, goal,
pose, docking, motor, charge-release, or other hardware command was sent.

## Stationary E-stop follow-up — 2026-07-23

The operator confirmed the robot has one physical top/rear twist-release
E-stop; the manual's two stray plural references conflict with the datasheet,
equipment table, operating description, and actual robot.

Twist-release alone correctly left the ESRB latch asserted. Physical PWR ON
then changed typed E-stop state from true to false (`flags 1060 -> 1028`) while
charge stayed `3`, motors stayed disabled, and odometry stayed zero. A repeated
button press changed the state false to true, restored raw flags `1060`, and
caused the priority-10 watchdog to publish zero. Final command and odometry
remained zero throughout.

This verifies button detection, typed reporting, fail-closed software response,
and the required PWR ON recovery. It does not yet prove physical power removal
from enabled motors; that observation remains with the supervised motor-enable
phase. No ROS or hardware control command was sent.
