# V1 deployment acceptance — 2026-11-22

This is an acceptance plan, not a commissioning record or permission to operate.
Local source fix only: core base `ea1a238efe950335bb6ef16ec5b7e60d48579e75`
(PR #86), branch `fix/localization-independent-undock-20261122`. Parent owns
commit, PR, publication, full review and hosted native CI. Nothing here closes an issue.

## Evidence boundaries and release binding

The supplied `patrolbot-github-inventory.json` and
`patrolbot-dashboard-github-inventory.json` contain actual PR/issue bodies.
They were read as the supplied November 22 snapshot, not refreshed from GitHub.
The dashboard inventory has **zero open issues as of that snapshot**; that does
not establish a fully commissioned deployment. Reported historical PR test counts
below are retained evidence, not fresh results at this fix's head. Current local
commands, failed attempts and results belong in sibling `artifacts/IMPLEMENTATION_REPORT.md`.
These sibling inventories/logs are handoff artifacts, not tracked PR dependencies;
the concrete acceptance criteria are retained in this document.

Bind every run to exact core commit, dashboard server/frontend/web-bridge commit
and immutable image digests, SBC source/binary/profile SHA256, Nav2 source version
and patch manifest, interface build hashes, map catalog/raster revisions, ROS distro,
launch/Compose/env configuration (redact secrets), hardware/session/origin generation,
clock synchronization state and tuning desired/committed/verified revision IDs.
A historical deployed hash is not proof of today's installation. Required pair:
core at this fix after review plus the paired dashboard gate correction below;
dashboard source inspected is main
`4fb4b36535588158b8ae572770fddd48f3287fac` (#43) or a separately reviewed successor.
Use ROS Jazzy and the recipe-pinned Nav2 **1.3.12** plus the complete current patch
manifest; no inherited build directory or silently rejected patch. SBC must match
current `HardwareUndock` protocol and, for the separate F710 tests, advertise
`MANUAL_UNDOCK_V1`. Rebuild shared interfaces on both ROS consumers where required.

Core `contracts/web-teleop.json` and the eight robot-tuning contract artifacts must
match their dashboard copies exactly. Map catalogs must match both canonical
floors, transforms, identities and raster hashes; no stale legacy path fallback.
Use the tracked standard Compose with reset GID/socket directory requirements,
current bringup-owned joystick/watchdog roles and exactly one instance of each
velocity/TF publisher. Keep adopted SUBNET exposure visible pending #20 decision.
No runtime activation, new speed settings, guessed profile values or seeds are
part of this plan. RuntimeControls **unmerged #87**, its paired work and general
deployment-tooling work are **DEFERRED / HELD**, excluded from these acceptance claims.
Already merged tuning PRs below are inventoried, not authorization to apply them.

## Matrix for recent merged work

O = portable/source evidence (historical unless implementation report says rerun).
N = isolated native ROS/build/loopback, no connected robot. S = deployed stationary
observation. M = separately authorized supervised motion. A dash means no new
motion case for that PR, not exemption from release regression. Procedure IDs below
supply the PASS/FAIL and abort rules for each row. None of the S/M cells is passed
by this local task; parent must attach matching evidence and operator acceptance.

### Core #69–86

| PR / immutable merge | Actual relevant change | O: retained evidence | N | S required | M required |
|---|---|---|---|---|---|
| [#69](https://github.com/cmu-impactlab/patrolbot/pull/69) `fc2bca2ff36d` | Motor-disable/LiDAR/reset and measured-pose integration; seed ACK, TF/newer-scan barriers, map/origin/innovation checks, clean smoother shutdown. | Contracts, five hardware suites; nine native lifecycle/seed cases reported. | N1 | S1,S3 | M3,M5 |
| [#70](https://github.com/cmu-impactlab/patrolbot/pull/70) `1c345bf418b7` | Forward F710 DRIVE during localization hold using fresh intent; reject stale raw odometry and pending/unknown owners; remove unintended return. | Exact recorded DRIVE bytes and intent expiry; loopback ROS case reported. | N1 | S2 | M2 |
| [#71](https://github.com/cmu-impactlab/patrolbot/pull/71) `37a4c1a99e91` | Separate web intent channel, freshest-of-two evidence; F710 priority 8 above web 7; byte-pinned web-teleop contract. | Forwarding and contract tests; ROS evidence-expiry case reported. | N1,N2 | S2 | M2 |
| [#72](https://github.com/cmu-impactlab/patrolbot/pull/72) `29d5066e5944` | Persistent immutable tuning revisions; signed mutations; fixed lifecycle activation; durable navigation hold until fresh seed/costmap ACK. | Unit-stage evidence only in body; contract/store/auth tests. | N1 | S4 | HELD |
| [#73](https://github.com/cmu-impactlab/patrolbot/pull/73) `0aca02960470` | Clean inherited native source/build outputs; extract/rebuild every patched Nav2 package, including behavior_tree; canonical docking overlay. | Recipe coverage and provenance tests. | N1 | S1 | — |
| [#74](https://github.com/cmu-impactlab/patrolbot/pull/74) `9a5271db4fc5` | Stop actual recipe loop on first rejected patch instead of accepting last-command success. | Deliberate first-patch failure regression; pristine Nav2 1.3.12 patch application reported. | N1 | S1 | — |
| [#75](https://github.com/cmu-impactlab/patrolbot/pull/75) `b514eea6b5a8` | Read-only GetParameters lane without saved tuning store; generation/age-bound observations; precise signed-save/link refusals. | Readback freshness, missing services, invalid values and writer recheck tests. | N1 | S4 | HELD |
| [#76](https://github.com/cmu-impactlab/patrolbot/pull/76) `964d76cf507b` | Serialize readback/policy work in joined worker; remove pending futures on shutdown to avoid executor starvation. | Worker/backend/service regressions; no age or bound widened. | N1 | S4,S5 | — |
| [#77](https://github.com/cmu-impactlab/patrolbot/pull/77) `6e0f6f3d8a3e` | Standard Compose retains reset directory/socket/GID; absent GID/directory refuses safely; legacy override idempotent. | Actual Compose rendering and reset contract tests. | N1 | S1,S6 | — |
| [#78](https://github.com/cmu-impactlab/patrolbot/pull/78) `f710cc814fe1` | Validate each distinct immutable tuning revision once per snapshot under writer lock; no cache across requests. | 39 service/store tests and corruption-after-read regression reported. | N1 | S4 | HELD |
| [#79](https://github.com/cmu-impactlab/patrolbot/pull/79) `bafc4484f050` | F710 Start gesture dispatches separate fixed manual undock; raw encoder turn; manual processes move to bringup; capability required. | Portable/controller suites and DDS loopback without AMCL/Nav2 reported; historical six C++ failures retained in PR body. | N1 | S1,S2 | M1,M2 |
| [#80](https://github.com/cmu-impactlab/patrolbot/pull/80) `c926d5d12a6b` | Create restricted reset mount directory at sysinit before Docker, avoiding exited web bridge after boot. | Seven deployment feature regressions; physical reboot not accepted by PR. | N1 | S6 | — |
| [#81](https://github.com/cmu-impactlab/patrolbot/pull/81) `c661ed4ce086` | Recoverable stop preserves already-enabled motors only after proven clearance; correlated persistent reason/unknown state; explicit stationary recovery. | 1,511 controller/116 protocol checks and isolated loopback reported; older six C++ failures retained. | N1,N2 | S2,S7 | M1,M2 |
| [#82](https://github.com/cmu-impactlab/patrolbot/pull/82) `f2329cdf3644` | Retain manual-undock deviation guards through clearance/handoff; remove unsupported gimbal; select all portable hardware suites. | Reviewed fixes-only backport; portable/native and paired browser evidence reported. | N1 | S1,S7 | M1 |
| [#83](https://github.com/cmu-impactlab/patrolbot/pull/83) `53c6d660148e` | Ship patch/rosdep fixtures, pytest-timeout and factories compatible with Ubuntu pytest; fresh strict test-image smoke. | Recipe/discovery checks and 42-test smoke reported; not full commissioning. | N1 | S1 | — |
| [#84](https://github.com/cmu-impactlab/patrolbot/pull/84) `1308054792ff` | Correct dock-manager setuptools dependency; fail on rosdep failures; check installed dependencies and automatic main safeguards. | Actual shell failure-propagation/policy regressions. | N1 | S1 | — |
| [#85](https://github.com/cmu-impactlab/patrolbot/pull/85) `771c5294961e` | Require full native functional suites, no skips/deselections/warnings/leaks; own partial fixture teardown and exact collision outputs. | 1,163 native cases and strict evidence reported; stale-sonar hold 0.533625 s in isolated case. | N1 | S1,S3 | M1,M3 |
| [#86](https://github.com/cmu-impactlab/patrolbot/pull/86) `ea1a238efe95` | Remove verified unused dependencies/private helpers/unloaded mux; retain public APIs, maps, joystick and guarded undock. | 1,478 portable tests plus six subtests, five hardware suites reported. | N1 | S1 | M1,M2 |

### Dashboard #30–43

| PR / immutable merge | Actual relevant change | O: retained evidence | N | S required | M required |
|---|---|---|---|---|---|
| [#30](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/30) `fdc5b54b56a5` | Measured localization action/covariance and age display; retain disable action; truthful expired AMCL status. | Frontend/server/adapter and six browser shards reported; historical optional PostgreSQL skips. | N1,N2 | S1,S3 | M3 |
| [#31](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/31) `211fc8a0a25c` | Expose independent Disable even when fresh hardware is invalid or cached motors say off; retain authority gates. | 304 frontend tests and desktop/phone/tablet disable regressions reported. | N2 | S2 | M2 |
| [#32](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/32) `d5e04f398ea6` | Add idempotent timestamp-leading SQLite index for history/retention without deleting rows or hiding queue loss. | Query-plan tests and copied 1.59M-row/600-write benchmark reported. | N2 | S5 | — |
| [#33](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/33) `5dc06d76d640` | Replace divergent Floor 1 raster/revision and map-catalog parser; pin both canonical identities in mocks/tests. | Map-parity, server/mock/adapter tests reported. | N1,N2 | S1,S3 | M3 |
| [#34](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/34) `62f732606827` | Session-bound web deadman channel, bounded forwarding/zero watchdog, bridge zero-drain ACK and no session restore. | Server/bridge/frontend tests; initial lab readiness race explicitly retained. | N2 | S2 | M2 |
| [#35](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/35) `b3858f55ac88` | Admin signed durable tuning mutations, dedupe/socket-bound replies and desired/committed/verified UI; separate runtime permissions. | Unit-stage server/frontend/mock/adapter and eight-artifact parity only in body. | N1,N2 | S4 | HELD |
| [#36](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/36) `cc1d006efbda` | Accept uninitialized read-only observations independently of saved session; expire by generation/age; fix speed-gauge clock flicker. | 478 frontend, server and parity evidence reported; browser deferred in body. | N2 | S4 | HELD |
| [#37](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/37) `c168dce244af` | Compact Advanced catalog; fixed-scale live-cap markers, last-known observations, coalesced polling and honest pending badges. | 487 frontend, build and four targeted browser tests reported. | N2 | S4 | HELD |
| [#38](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/38) `18a4ea195b06` | Persistent operation fault and explicit recovery controls paired with core #81; retain truthful failure instead of transient toast. | 493 frontend, 880 server/226 adapter reported; optional browser workflow separated from mandatory build. | N1,N2 | S7 | M1,M2 |
| [#39](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/39) `f557f1dc6e38` | Retain Stop owner across delayed/unknown acceptance; recheck authority after awaits; protect replies under pressure; correct UTC history. | 1,050 clean server cases and paired browser/native evidence reported; optional DB/ROS skips historical. | N2 | S2,S5,S7 | M2 |
| [#40](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/40) `dc5d76e4efb8` | Compile literal protocol fixtures against real TypeScript types; exact session/robot/audited-stop receipt identity; either watchdog ordering. | 573 frontend; contract/script tests; six native receipt checks reported, with stale-load failure retained. | N2 | S1,S2 | M2 |
| [#41](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/41) `79890d8892ed` | Track/test all 19 automatic matrix requirements, current base and administrator enforcement; exclude manual browser check. | Workflow-derived protection payload regression; no runtime change. | N2 | S1 | — |
| [#42](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/42) `5946892bdea2` | Supervise native teleop lab readiness/load/cleanup; require exact stale-odometry failure and zero/inactive receipts; later author-review policy. | Six serial/load cases with seven exact checks; no retries/leaks reported. | N2 | S2,S5 | M2 |
| [#43](https://github.com/cmu-impactlab/patrolbot-dashboard/pull/43) `4fb4b3653558` | Reject obsolete PostgreSQL configuration safely; remove only duplicate legacy maps/unused UI; verify historical manifests at source ref. | 19 hosted checks and 14 source-validation stages reported; six native serial/load cases; 1,527 server/575 frontend. | N2 | S1,S5 | — |

## Reproducible offline and isolated-native gates

These commands read source or run isolated software doubles/ARIA-free binaries.
Run only in the authorized disposable checkout/evidence directory. The runner
rebuilds local test binaries; it never attaches to a ROS graph or robot.

```bash
git rev-parse HEAD
git status --short
git diff --check
PY=/home/yousef/Projects/repos/patrolbot-runtime-controls-20261122/artifacts/test-venv/bin/python
PYTHONPATH=ros2_ws/src/patrolbot_dock_manager "$PY" -B -m pytest -q -p no:cacheprovider \
  ros2_ws/src/patrolbot_dock_manager/test/test_localization_independent_undock.py \
  ros2_ws/src/patrolbot_dock_manager/test/test_contracts.py \
  ros2_ws/src/patrolbot_dock_manager/test/test_undock_rejection.py
PYTHONPATH=ros2_ws/src/patrolbot_bridge "$PY" -B -m pytest -q -p no:cacheprovider \
  ros2_ws/src/patrolbot_bridge/test/test_local_only_undock_turn.py
"$PY" -B docker/run-portable-review-tests.py
```

Allow 900 s for the broad runner. Preserve each exit status, full stdout/stderr,
source identity and distinct log name; preserve failed candidates. Do not count
an import/collection failure as a useful safety refusal. AST fixtures execute the
actual production methods with synthetic clocks, futures, telemetry and lifecycle
clients; they cannot establish ROS serialization, installed plugin behavior or
physical clearance. C++ tests exercise the actual controller without ARIA hardware.

**N1 — core native gate (parent/CI, not run here):** use the unchanged
`.github/workflows/native-test-image.yml`, `docker/Dockerfile.ros-tests`,
`docker/run-ros-tests.sh` and `docker/native_ci_gate.py` in the existing isolated
network-none/private-IPC, non-root/read-only CI setup. Build current interfaces and
all patched packages. Run bridge, dock-manager and navigation functional suites;
include `test_manager_integration.py`'s missing/stale/invalid AMCL action/readiness
cases, fake-SBC, manual-undock and installed Collision Monitor cases. Require actual
collection/setup/call/teardown/JUnit/cleanup agreement, all sentinels, zero unexpected
skips/deselections/warnings/thread/task errors, timeouts, crashes or leaked processes.
Do not turn integration opt-in on in an arbitrary connected shell. New tests must
be collected in the native inventory; no no-skip policy weakening or selective
smoke-only replacement. FAIL on any omitted test, rejected patch or missing evidence.

**N2 — paired dashboard immutable-source gates (parent/CI):** verify the exact
#43 source and current contract parity in its supported locked test environments;
run server, frontend unit/type/build/bundle, adapter, mock, scripts and protocol
contract stages. Run existing isolated native teleop lab's fixed serial/bounded-load
matrix with real mux/smoother, all seven exact receipt checks and clean teardown.
Require session/robot/command/audit IDs, exact expected terminals, actual zero and
inactive subscriber receipts, and refusal of stale-odometry restart. Include
optional browser acceptance separately for desktop/phone/tablet Chromium/WebKit;
record any declared optional exclusions explicitly. Historical optional skips do
not authorize skips in the core required native functional gate. FAIL on stale
synthetic hardware in a healthy case, unrelated receipts, hidden drop/failure,
source mismatch, session restoration or weakened freshness/deadlines.

## Stationary procedures — separate deployed observation authority

No procedure here has been executed by this task. Even read-only deployed access
needs the parent to identify the target and authorize access. No SSH/container/
connected-graph commands were run. The following are **read-only examples for that
later authorized session**, not permission to start services or publish commands:

```bash
# In an already approved target shell; do not dump environment/secrets.
date -u
timedatectl show -p NTPSynchronized -p TimeUSec -p RTCTimeUSec
sha256sum <approved-SBC-binary> <approved-undock-profile> <approved-patch-manifest>
cat /sys/class/thermal/thermal_zone0/temp
vcgencmd get_throttled
# In an already sourced, explicitly approved observation-only ROS shell:
ros2 topic echo /patrolbot/base_state --once
ros2 topic echo /patrolbot/dock_readiness --once
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_vel_out --verbose
```

Do not substitute guessed paths, run control services, emit goals/pose/Twist, or
restart anything as part of these examples. Capture bounded subscription logs
rather than repeatedly launching high-load graph tools. Correlate ROS stamps,
monotonic receipt ages and UTC; stamp age alone cannot diagnose receipt loss.

| ID | Procedure and required evidence | PASS / FAIL / abort condition |
|---|---|---|
| S1 Release/config parity | Compare every bound artifact above to approved source; inspect map IDs, capabilities, publisher graph and actual configured profile readback. Verify dashboard UI asset/server/adapter pair. | PASS only exact match and one intended owner per chain. Abort motion planning on mixed interfaces, duplicate joystick/TF/velocity owner, unknown profile, missing capability or unexpected setting. Config flags alone do not prove physical commissioning. |
| S2 Input/ownership observation | Motors disabled under the existing procedure; observe receiver connection, centered input, intent ages, server lease, zero-drain receipts, Disable availability with stale/invalid hardware, and no session restore after reconnect. Use synthetic faults in N1/N2 first. | PASS when UI and wire ownership agree, zero/inactive receipts are correlated and no stale input replays. Real Stop, Disable, reconnect/fault injections are separately authorized actions; unknown results retain owner and persistent fault, never offer automatic retry. |
| S3 Localization/scanner diagnosis | Log scan acquisition age vs receipt age, raw odometry, AMCL stamp/covariance, TF/map identity, planner/costmap result and guard timestamps. Check wall/corner geometry independently in #34. | Distinguish delayed AMCL guard cancellation from real stale scan; distinguish planner no-path/costmap timeout from scanner loss. FAIL on generic retry trap, mislabeled `localization scan stale`, TF filter drops, stale seed reuse or displayed false localized success. No timeout/variance widening to pass. |
| S4 Readback only | Observe live GetParameters lane vs desired/committed/verified revision, source/generation/age, missing service, expired sample and disconnected link; bound polling latency/worker shutdown evidence from isolated tests. | PASS only actual runtime values shown as live; expired values labeled last-known; precise unavailable/write-disabled reason. Runtime/Advanced changes, saves/apply/redeploy/restore and speed commissioning remain HELD. No production mutation/fault injection in this observation. |
| S5 Load/power/history | Identify current replacement Pi, PSU/cable/connectors; qualified person measures voltage under representative stationary load. Record CPU/temp/throttle/current-vs-historical flags, queue/drop counters, acquisition/receipt delays, bounded SQLite history latency/index plans on a copy and no data loss. | PASS against the agreed hardware specification/unchanged safety budgets with no new undervoltage/throttle events and truthful drop reporting. At observed 84–85°C investigate headroom; active throttling is NOT proven from temperature alone. Abort operation on thermal/power fault or stale safety data; do not hide bridge latest-value drops or relax timers. |
| S6 Boot/reset path | In a separately authorized reboot/reset window, verify sysinit directory/socket/GID ordering, web bridge recovery and correct cold clock synchronization before signed/age-sensitive commands. Record first-boot skew and correction timeline. | PASS only target comes back with truthful capabilities/health, no stranded exited bridge and no replay/automatic motor/goal recovery. Known initially +60 s boot skew corrected after container startup needs acceptance. Reboots/resets are actions, not part of read-only observation. |
| S7 Unknown/fault truth | Exercise delayed/unknown action/reset results and persistent fault display first in isolated fixtures; later approved target scenario must preserve correlated command/operation IDs through browser reconnect. | PASS only unknown remains unknown and ownership held until authoritative terminal or explicit approved recovery. Stop ACK alone is not proof of a physical stop. FAIL on disappearing fault, blind retry, false reset success, fresh input auto-arm or old-goal replay. |

## Motion procedures — explicit separate authority required

Each M procedure requires a newly authorized session naming the operation and
fault injection, a physically present supervisor at a verified working E-stop,
a cleared surveyed test area including the full reverse/turn swept volume, and
current profile/sensor/power acceptance. Use the existing operator workflow;
this checklist provides **no executable motion, seed, motor-enable or reset command**.
Record supervisor, authorization, installed versions, preconditions, timestamps,
raw sensor/command/result evidence and independent displacement/heading/stop
measurements. Agree quantitative physical tolerances before the run; never invent
commissioning measurements or alter safety settings to obtain a pass.

Common abort: E-stop/contact/fault/stall, missing/stale/invalid required telemetry,
origin/session change, owner conflict, unknown response, unexpected motion, drift,
heading/distance/time bound violation, sensor coverage uncertainty or supervisor
concern. Use the established physical stop procedure; keep safe holds and record
failure. No automatic retry, arming, motor-enable, navigation or turn resumption.

| ID | Reproducible supervised sequence | Required PASS evidence / specific refusal |
|---|---|---|
| M1 Standard unlocalized undock, plus distinct manual path | After #39/#7 prerequisites and N1 pass, establish confirmed charging/docked, fresh measured local origin/odometry/scans, commissioned profile, native navigation-idle and exclusive owner. Test missing, stale and invalid AMCL on approved runs without creating a map pose. Request the existing standard dashboard Undock once. Observe SBC reverse, stopped CLEAR_CONFIRMED handoff, local-only Nav2 Spin and stable zero completion. Separately test the F710 Start path without substituting it for dashboard Undock. | Standard request must retain configured full reverse (default 0.53 m at 0.10 m/s) and bounded turn (default pi, <=0.70 rad/s, 20 s bound), measured drift/heading/distance/time hard limits from exact profile, physical clearance and zero terminal. No `/request_nomotion_update`, lifecycle seed/AMCL reset caused by undock, initialpose, map mutation or global resume. Preparation may pause global navigation and activate/deactivate only the existing local collision chain. End WAIT_SEED/local-only with global navigation still refused until separately valid localization. Pending/unknown lifecycle owner legitimately refuses; do not relabel that as AMCL missing. A failed-latch case is blocked, not a successful bypass. |
| M2 Manual input/Stop/loss/reconnect | With valid hardware, test F710 center, shallow/full stick, turns and deadman release, then authorized receiver/input loss. Test web pointer/key release, focus loss, transport loss and reconnect; F710 takeover must outrank web/autonomy. Test Stop and independent motor Disable through current paths. Release controls and require fresh RB/deadman/start after recovery. | Correlated actual zero/stopped measurements, no stale replay, no automatic motor-enable or old-goal resume, precise fault/refusal and retained unknown owner. For the separate F710 automatic Start maneuver, receiver disconnect alone is NOT a cancellation contract; test documented fresh Start cancellation and physical E-stop/Disable separately. Do not claim its disconnect behavior equals deadman driving. |
| M3 Navigation/recovery | Only after separate valid localization, exercise both actual BackUp-first recovery trees and independently measured ordinary navigation. Record contiguous Clear/ClearingActions → BackUp → Spin → Wait twice plus final paired clear; test natural frozen-plant failure separately. | Both actual trees, bounded physical reverse/turn/stop with no old command revival. Natural frozen plant must meet original 120 s acceptance bound; injected success does not pass it. Preserve RPP settings/BackUp-first order; separate planner/costmap failures from scan/AMCL freshness. |
| M4 Rear guarding and current profile | Under separately approved obstacle/dropout procedure, verify straight-reverse obstacle stop, near-field refusal, sonar stale/missing refusal/stop, bumper/E-stop/fault/stall behavior and independent encoder vs real displacement. Test ordinary reverse recovery and SBC undock as distinct guarding paths. | A single relevant rear point can stop; units 11/12/13 evidence matches installed profile, no dead unit 10 added. Measure causal command→stop→clearance/re-latch ordering and bounds. Rear-right arcing coverage remains unproven; no claimed all-around collision protection. Feed results into M1/M3 only after accepted. |
| M5 Localization geometry/endurance | Independently measure walls/corners, moving pose/heading and covariance stability; long-duration parked validity clear of dock, moving-without-AMCL then parked, encoder-fault and stale-odometry cases. Test approved override once against installed adapter with correlated result/refusal and no crash. | Preserve map/origin/seed identity and positive usable covariance, correct moving invalidation and truthful cross-consumer uncertainty/age. Docked 90-minute retained validity is not undocked endurance, moving accuracy, continued scan convergence or a new survey. No automatic pose seed or navigation as a side effect of undock. |

For M1/M4 the source profile contains historical measured masks and flags; it is
not a fresh measurement on the current robot. Preserve all existing reverse
lateral/yaw/displacement guards and stricter ordinary drivetrain backstops.
The bridge's existing spin-phase hold policy is E-stop/fault contact evidence
plus command-source/publisher/collision and lifecycle liveness; full hardware
checks apply in preparation/stopping. This fix does not broaden or narrow that
adopted policy. Do not advertise a new per-bumper spin guarantee from the fakes.

## Remaining open issue closure evidence

All nine remain OPEN. The latest reconciled issue sections supersede historical
contradictory text. A close needs attached exact-target evidence and operator
acceptance of every remaining criterion, or an explicitly approved transfer to
a named tracker. Merged software and this checklist alone satisfy none of these
physical closure rules.

| Core issue | Credit already retained | Evidence still needed to CLOSE / dependencies |
|---|---|---|
| #6 Reverse recovery | Both installed trees completed isolated native sequences, ~66.103/66.474 s; delayed-flush test-reader defect corrected with original failure retained. | M3 both physical BackUp-first trees, contiguous correlated progression, actual reverse/turn stopping, no resumption; #7/M4 rear guard prerequisite. Natural frozen 120 s failure remains #65. |
| #7 Rear sonar | 207/207 detections attributed to live 11/12/13; max residual 2e-6 m and timing characterized. | M4 actual obstacle stop and authorized sonar dropout refusal/stop for ordinary reverse AND SBC undock, exact installed profile and sensors. Preserve known dead 9/10 and rear-right coverage gap; no `min_points` increase above one. |
| #15 Real power-fail shutdown | Pi voltage/fault telemetry and fail-closed holds exist; HOST RI not demonstrated through socat. | Qualified electrical proof of a real power-fail signal reaching SBC, scoped graceful-shutdown implementation with timing/durable-state evidence, then separately authorized bench power-failure acceptance. Reset button/software health is not this protection; never install a daemon with no demonstrated input. |
| #18 F710 | Core #70/#71/#79/#81 plus paired dashboard teleop/fault recovery merged; receiver/hotplug history retained. | S1/S2 then M2 physical sticks/turn/release/loss/reconnect, unlocalized control, priority/takeover/rearm, no stale replay or autonomous resumption, current bumper truth and matched SBC fault-recovery binary. Separate Start-path contract from deadman; coordinate #7/#39. |
| #20 DDS exposure | Tracked centralized SUBNET discovery is the adopted remote-visualization policy. | Approved containment/authentication posture and residual exposure scope/duration, deployed configuration evidence plus authorized unauthorized-sender refusal AND intended remote visualization/discovery compatibility. No silent SUBNET/LOCALHOST switch; speed limits and endpoint node names do not authenticate senders. |
| #34 Laser timing/TF/alignment | Acquisition-age wire/readback and sole calibrated TF authority retained; native seed/TF/newer-scan tests already passed historically. | S3 measured acquisition vs receipt timing, authorized reconnect showing zero TF message-filter drops and proper barriers, independent wall/corner/height/datum geometry (M5/#65). No second TF owner or future-dated transform. Historical x=-0.422/yaw=-20.22° is not a calibration to restore. |
| #39 Measured undock profile | July 29 bumper masks, sonar 11/12/13, E-stop semantics, 3,552 ms release and 5,000 ms observation recorded; validity flags already set in source. | S1 target/profile reconciliation; M4 encoder-vs-real distance/clearance, idle fault/stall masks, causal stop/re-latch ordering; current Floor 1 capture geometry and unvalidated 60 s/0.50 m redock inhibit policy. M1 standard full reverse/turn/handoff with no AMCL and separate manual/recoverable-stop acceptance; #7/#18/#65. No inferred uncommissioned refusal without readback, no guessed profile values. |
| #59 Pi power integrity | Normal-user power reporting and older docked throttle=0 evidence; two historic undervoltage events retained, not proven localization cause. | S5 identify actual replacement Pi/PSU/cable/connectors; physical voltage under representative stationary load, bounded exact-image performance run with no new undervoltage/throttle flags. 84–85°C/CPU pressure/latest-value drops need causal investigation; active throttling not yet proven. No navigation-timer relaxation. |
| #65 Localization umbrella | Docked 5,400.087676 s endurance/5,003 valid checks, same seed/origin/map, raw SHA256 `5eaa9981f8332646f09a8a8870a4868e9420b292d8002ebbc0ae9bdfc8a83dd8`; source/installed repairs and native recovery/clean shutdown history retained. | S3+M5 independent geometry and moving accuracy/covariance; clear-of-dock endurance; moving-then-parked/encoder/stale-odom invalidation; matched cross-consumer map/epoch/valid-invalid-degenerate-age cases; override result with no adapter crash; actual gyro contribution and away-from-dock restart; natural frozen recovery <=120 s; #6/#7/#18/#34/#39/#59 evidence or approved named transfer. Undock can succeed unlocalized while navigation still refuses. No wider variance/freshness limits or background no-motion resampling. |

## Paired dashboard change is required for missing AMCL

Read-only inspection of dashboard main #43 found an indirect map dependency:
`server/app/commands/gates.py::undock_reason` calls `_stationary_reason`, which
requires a fresh `telemetry.pose`; `facts_from_state` derives stationary only
from that pose. `frontend/src/lib/dockGates.ts` mirrors it. The adapter's
`bridge_node.py::_maybe_send_pose` sends only if `_on_active_map(_latest_amcl)`
passes; a missing AMCL sample or absent active map therefore suppresses pose
frames despite the normalizer's odom fallback. Existing measured
`telemetry.motion` is independent of AMCL/map, but those Undock gates do not use it.
The actual server gate was executed offline: fresh commissioned dock hardware
with absent pose refuses with “recent position update”; the same hardware and
fresh pose-shaped velocity passes even with `localized=False`.

Thus core-only fixes the coordinator action path, but **does not complete the
missing-AMCL dashboard path** at this inspected revision. Parent needs a focused
paired change to frontend/server stationary admission using existing fresh finite,
hardware-session/origin-generation-bound measured motion (and rechecking at
admission), without publishing raw odometry as map pose or setting localized=true.
Preserve current hardware/ownership/capability gates, strict moving refusal and
missing/stale/invalid/wrong-generation motion refusal. Navigation retains its
map/localization gates. Add actual server-gate, UI and adapter forwarding tests
with missing/stale/invalid AMCL and good/bad measured motion, followed by N2/M1.
No dashboard files were edited in this source-only worktree. This paired work is
a specific remaining blocker, not permission to expand RuntimeControls or tooling.

## Current blockers requiring honest triage

- Failed localization latch → generic “preconditions” retry trap: capture actual
  map/lifecycle stage, pending future kind, failed latch and owner. Do not clear
  `maps.hold`/`_failed` merely to allow undock; unresolved mutations may be in flight.
- Delayed AMCL pose guard cancelling goals and mislabeling “localization scan stale”:
  preserve timestamps and distinguish it from actual scanner failure (S3/#65).
- Planner no-path and costmap timeout need separate diagnoses from scan staleness;
  source-only localization-independent undock does not fix either.
- Raw odometry `position-discontinuity` is separate from AMCL age. The unchanged
  validator rejects a position step above `0.30 * (receipt_dt + 0.05)` metres
  with checked-in defaults. Bounded live logs show position and reported-speed
  rejections inside stable SBC process lifetimes, but lack rejected coordinates
  and acquisition timing. Offline probes on both previous/current revisions
  reproduce receipt-compression false rejection and eventual admission of a
  persistent same-origin reset as elapsed allowance grows. Neither proves the
  live cause. Capture bounded numeric rejection context and source timing before
  choosing a fix; do not relax bounds, reset anchors on rejection or infer an
  origin change from a numeric jump. Keep this safety-design work separate.
  No recovered diagnostic is currently emitted, so a displayed last rejection
  alone does not establish ongoing failure. Preserve counters when clarifying
  current health.
- Pi 84–85°C, CPU pressure and bridge latest-value drops: S5 performance/power
  evidence still needed; active throttling has not been established.
- Boot clock initially +60 s, corrected after containers start: S6 signed-request
  and age-barrier behavior still needs exact-target acceptance.
- Standard unlocalized reverse + turn still needs the parent N1 native run and
  supervised M1 run. Passing AST/C++ doubles is not physical commissioning.
- F710/web loss/reconnect, Stop/Disable and no replay need M2; unknown/reset
  outcomes, persistent operation fault and truthful UI need S7.
- Rear sonar and measured undock profile commissioning remain mandatory (#7/#39).

LiDAR boot/power commissioning #68 was cancelled as not planned on November 19;
it is not silently reopened here. Actual scanner freshness remains mandatory.
Keep Runtime/Advanced speed controls and general deployment-tooling work deferred.
