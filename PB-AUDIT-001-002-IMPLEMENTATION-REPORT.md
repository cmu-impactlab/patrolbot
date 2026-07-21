# PB-AUDIT-001 and PB-AUDIT-002 Implementation Report

## 1. Outcome and deployment status

Implementation date: 2026-07-16. Stationary SBC cutover: 2026-07-21.

PB-AUDIT-001 containment is deployed and verified on `patrolbot-rpi5`:

- `docking_server` is absent from the composed and non-composed launch paths and
  from the Nav2 lifecycle list.
- Generic Nav2 docking actions are not advertised. The separate custom
  `/patrolbot/undock` action is advertised but remains fail-closed and was not
  called.
- `/cmd_vel` has exactly one publisher, `teleop_velocity_smoother`, and exactly
  one subscriber, `patrolbot_bridge`.
- The original single hardware session was preserved until the operator shut
  down the SBC. No replacement or second connection was opened.

The Pi portions of Stages 1–3 for PB-AUDIT-002 are implemented, tested, and
deployed under immutable image
`patrolbot:pb002-manual-release-20260721t210500z`. The corrected 32-bit SBC
server is also deployed. The stationary campaign verified reconnect/session
handling, charging rejection of motor enable, one successful zero-motion
charge release, UUID replay without a second hardware action, and automatic
resumption of charging with motors disabled. Physical bumper mapping, dock
pose, live rear-sonar obstacle validation, E-stop/joystick/PWR ON behavior,
successful motor enable, and the controlled motion campaign remain unresolved
gates. PB-AUDIT-002 is therefore not closed.

One deliberately rejected motor-enable request and one successful
charge-release request plus its identical UUID replay were sent. No velocity,
pose, navigation goal, dock/undock action, or successful motor-enable command
was sent. The robot did not move. No second connection to port 7272 was opened.

During implementation, no files under `docs/` or `SKILLS/` were changed. After
the implementation was complete, the operator separately authorized updates to
the canonical audit and this implementation report. `docs/` and `SKILLS/`
remain unchanged.

## 2. Live production change

### Stage 0: docking-server containment

Changed and deployed:

- `ros2_ws/src/patrolbot_navigation/launch/patrolbot_navigation_launch.py`

The following were removed from both launch variants:

- the `docking_server` executable/component;
- `docking_server` in the lifecycle-managed node list.

The navigation container alone was restarted to load the containment change.
The bridge, bringup container, Docker daemon, SBC server, socat, network, serial
interfaces, and robot hardware state were not restarted or changed.

Deployed checksum:

```text
010afb9d8583f269daf302f18faae6b3a03d177e5a643069decfe09705c22b8f
```

Adjacent live rollback copy:

```text
/home/ubuntu/patrolbot-repo/ros2_ws/src/patrolbot_navigation/launch/
  patrolbot_navigation_launch.py.bak
SHA-256 ae745b2d98f4c119c0d70a0a6acfa702f7bfcc89c4d3f3423c30fc8f8d2e84e2
```

## 3. Repository changes and reasons

### 3.1 Build and package integration

| File | Change | Why |
|---|---|---|
| `docker/Dockerfile` | Build `patrolbot_interfaces` and `patrolbot_dock_manager`; include the SBC source as a static-test fixture | Make the typed interfaces and guarded-undock node available in a future image and allow CI to prove the release branch has no motion/enable side effect |
| `ros2_ws/src/patrolbot_bridge/package.xml` | Add `nav_msgs` and `patrolbot_interfaces` dependencies | Match actual imports and generated service/message use |
| `ros2_ws/src/patrolbot_bridge/setup.py` | Install bridge YAML; correct package description/license and entry-point indentation | Package the fail-closed bumper defaults and keep lint/build clean |
| `ros2_ws/src/patrolbot_navigation/CMakeLists.txt` | Register the safety-contract pytest suite | Make containment and configuration invariants build-time gates |
| `ros2_ws/src/patrolbot_navigation/package.xml` | Add interface/manager runtime dependencies and pytest/YAML test dependencies | Ensure deterministic build and test resolution |

### 3.2 Typed ROS interfaces

New package: `ros2_ws/src/patrolbot_interfaces/`

| File | Purpose |
|---|---|
| `msg/BaseState.msg` | Session-aware link, raw charge, motor, E-stop, fault, and bumper validity state |
| `srv/ChargeRelease.srv` | UUID-correlated, structured zero-motion release request/result |
| `srv/MotorEnable.srv` | Separate explicit motor-enable request/result; never implied by `DRIVE` |
| `action/Undock.action` | Cancelable undock goal, feedback, result codes, final state, and manual-recovery flag |
| `CMakeLists.txt`, `package.xml` | Jazzy `rosidl` generation and runtime dependencies |

### 3.3 Pi bridge and hardware protocol

| File | Change | Why |
|---|---|---|
| `patrolbot_bridge/bridge_node.py` | Publish typed `/patrolbot/base_state`; add charge-release and motor-enable services; track hardware session generation; route ACK/RESULT separately; serialize all socket writes; fail pending calls on disconnect | Prevent stale state/replies across reconnects and provide bounded hardware-only controls without creating another port-7272 client |
| `patrolbot_bridge/control_protocol.py` | Strict lowercase canonical-UUID formatter/parser for `CHARGE_RELEASE`, `MOTOR_ENABLE`, `ACK`, and `RESULT` | Keep the Pi and legacy C++ parser identical and reject ambiguous/malformed tokens |
| `patrolbot_bridge/config/bridge_params.yaml` | Default bumper mapping to invalid with zero masks | Make automatic reverse unreachable until physical bumper bits are proven |
| `patrolbot_bridge/test/test_control_protocol.py` | Test exact framing, zero-motion payloads, UUID rejection, malformed replies, and telemetry separation | Prevent protocol regressions and accidental velocity content |

Bridge-side controls require all of the following before writing to the existing
socket:

- current hardware session and fresh charge/flags;
- fresh odometry and zero linear/angular motion;
- zero hardware fault flags;
- release: valid charge state and motors disabled;
- motor enable: charge state zero and E-stop clear.

### 3.4 Canonical SBC server

New canonical project: `hardware/patrolbot_hw_server/`

| File | Change | Why |
|---|---|---|
| `patrolbot_server.cpp` | Preserve telemetry/watchdogs; remove startup and `DRIVE`-triggered motor enable; add bounded control operations and state confirmation | Eliminate implicit enable and keep all motion under the Pi safety path |
| `protocol.hpp`, `protocol.cpp` | Strict parser, fragmented/coalesced TCP line buffer, response formatting, and bounded UUID result cache | Make commands testable, idempotent, and safe under TCP framing/retry behavior |
| `tests/protocol_test.cpp` | Test valid/malformed commands, zero-motion controls, line fragmentation, duplicate pending, busy, and terminal replay | Prove the hardware command is not repeated for a duplicate UUID |
| `Makefile` | Add `make test` and legacy ARIA server build targets | Reproduce tests/build on the actual 32-bit SBC toolchain |

The release operation:

1. Accepts `CHARGE_RELEASE:<uuid>` only while stationary, fault-free, and with
   motors disabled.
2. If already at charge state zero, returns idempotent success without issuing a
   new hardware command.
3. Otherwise sends µARCS command number `68` with argument `0` exactly once.
4. Never calls `setVel`, `setRotVel`, `enableMotors`, or an E-stop clear.
5. Reports success only after charge state zero is stable for three samples.
6. Times out after five seconds and returns a structured terminal result.

`MOTOR_ENABLE:<uuid>` is separate. It is refused while moving, charging,
E-stopped, or faulted and succeeds only after three stable enabled samples.
Nonzero `DRIVE` is refused unless bounded, charge state is zero, E-stop is clear,
motors are already enabled, fault flags are zero, and no hardware operation is
pending. A refused drive commands zero and never enables motors.

### 3.5 Navigation safety prerequisites

| File | Change | Why |
|---|---|---|
| `config/nav2_params.yaml` | Disable automatic AMCL origin; tune BackUp minimum/acceleration; add front-scan/rear-sonar velocity polygons; add fail-closed dock-manager parameters | Prevent invalid restart localization and protect reverse motion with fresh rear sensing |
| `config/behavior_trees/navigate_recovery_always_try.xml` | Use a 0.10 m, 0.05 m/s BackUp before Spin | Make the intended conservative reverse reachable before rotation near a front obstruction |
| `config/behavior_trees/navigate_through_poses_recovery_always_try.xml` | Apply the same BackUp-first ordering and limits | Keep recovery behavior consistent for multi-pose navigation |
| `launch/bringup.launch.py` | Stage the guarded-undock manager as a fail-closed node | Provide the state-machine entry point in a future coordinated image deployment |
| `launch/patrolbot_navigation_launch.py` | Remove docking server in both variants/lifecycle list | Contain PB-AUDIT-001 in every launch path |
| `test/test_safety_contract.py` | Add static and exhaustive architecture/configuration tests | Pin incident-verified values and prove no command-path/polygon gaps |

Pinned values retained from the actual deployment:

- `collision_monitor.base_shift_correction: False`;
- RPP `use_collision_detection: false`;
- local/global inflation `cost_scaling_factor: 4.0` as a float;
- bridge `SCAN_RANGE_MIN = 0.25`.

The review addendum named `1.0` for inflation scaling, but the deployed source,
runtime-era backup, and canonical pre-edit file all contain `4.0`. The
implementation follows the revalidated live value and pins its float type rather
than replacing it with an unverified value.

The collision-monitor design uses:

- front scan polygon: `min_points: 8`;
- rear sonar polygon: `min_points: 1` for four sparse transducers;
- per-source and common timeout: `0.5 s`;
- fresh empty sonar cloud: clear;
- missing/stale sonar: automatic reverse prohibited;
- no sonar costmap clearing or marking change;
- front dock wall excluded only from the reverse travel envelope;
- front protection restored for forward motion and rotation.

Each front and rear velocity polygon covers the complete configured envelope
`v_x = [-0.26, 0.26]` and `omega = [-1.0, 1.0]`, including combined arcs and
the `+/-0.01` boundary bands. The exhaustive test evaluates 209,442
velocity/polygon combinations and requires exactly the intended first match.

### 3.6 Guarded-undock state machine

New package: `ros2_ws/src/patrolbot_dock_manager/`

| File | Purpose |
|---|---|
| `contracts.py` | Pure fail-closed safety snapshot, hard goal caps, precheck/reverse gates, and explicit allowed-transition table |
| `dock_manager.py` | `/patrolbot/undock` action server and Nav2 `/backup` client |
| `test/test_contracts.py` | Test every safety field, reconnect, caps, valid transitions, abort/fault paths, and invalid skips/re-entry |
| `test/test_flake8.py` | Keep the safety node lint-clean |
| package/setup/resource files | Install the node as `dock_manager` |

Implemented happy path:

```text
IDLE
  -> PRECHECK
  -> REQUEST_DOCK_RELEASE
  -> WAIT_FOR_CHARGE_RELEASE
  -> VERIFY_ESTOP_AND_MOTOR
  -> VERIFY_REAR_SONAR
  -> VERIFY_LOCALIZATION
  -> VERIFY_COMMAND_PATH
  -> GUARDED_REVERSE
  -> CLEAR_OF_DOCK
  -> STOPPED
  -> COMPLETE
```

Every active state may enter `ABORTED` or `FAULT`; skips, re-entry, and unknown
transitions are rejected. Amendment A-1 is implemented as one shared ten-second
post-release E-stop/motor window. The manager never invokes motor enable itself.

The manager continuously checks:

- hardware session continuity and telemetry freshness;
- charge zero, E-stop clear, motors enabled, and zero faults;
- valid mapped bumpers with neither front nor rear asserted;
- fresh rear sonar with no point in the guarded rear envelope;
- fresh odometry;
- fresh AMCL pose/covariance within the configured dock pose tolerance;
- no other navigation, behavior, assisted-teleop, or foreign BackUp goal;
- every endpoint of the exact safe velocity chain;
- exactly one final `/cmd_vel` publisher and the bridge subscriber.

On cancel, timeout, or safety failure it cancels Nav2 BackUp and requires fresh
odometry to confirm zero velocity. A TCP session-generation change aborts the
operation; no prior velocity is stored or replayed.

The manager uses a **positive** Nav2 BackUp target distance and speed. Nav2's
BackUp behavior converts that goal into reverse velocity; passing a negative
goal would be incorrect for Jazzy.

## 4. Resulting safe paths

### Velocity

```text
controller_server or behavior_server BackUp
  -> /cmd_vel_nav
  -> navigation velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor (/scan + /sonar)
  -> /input/navi
  -> cmd_vel_mux
  -> /cmd_vel_out
  -> teleop_velocity_smoother
  -> /cmd_vel                 [sole final publisher]
  -> patrolbot_bridge
  -> existing TCP session
  -> patrolbot_server
  -> ARIA / µARCS / motors
```

The dock manager publishes no `Twist`. It invokes Nav2 BackUp, whose behavior
server output is remapped to the same pre-smoother topic as the controller.

### Hardware release

```text
/patrolbot/undock action
  -> dock-manager safety precheck
  -> /patrolbot/charge_release service
  -> existing bridge TCP session
  -> CHARGE_RELEASE:<uuid>
  -> SBC sends µARCS command 68, argument 0
  -> stable raw charge-state confirmation
  -> UUID-correlated RESULT
```

Release and reverse remain separate. The SBC never releases and then drives.

## 5. Corrections discovered during implementation

1. The installed legacy ARIA headers do not define `ArCommands::CHARGE`.
   The PatrolBot manual's µARCS command is numeric command `68`; the staged
   server uses that hardware-protocol value explicitly.
2. Nav2 Jazzy BackUp takes positive distance and speed goals and produces the
   negative velocity internally. The implementation follows that API.
3. The Pi UUID validator originally accepted uppercase UUIDs while the C++
   validator rejected them. Both now require the same lowercase canonical form.
4. Runtime command-path validation was strengthened from a final-topic check to
   exact publisher/subscriber validation at every hop.
5. A foreign direct `/backup` action is now treated as another active behavior
   and blocks/aborts undock; the manager recognizes only its own BackUp goal ID.
6. Hardware fault flags now block release, motor enable, and nonzero drive.

## 6. Verification performed

### Offline Pi build and tests

- Built all five active packages in a disposable ARM64 Jazzy image on the Pi:
  `patrolbot_interfaces`, `patrolbot_bridge`, `patrolbot_dock_manager`,
  `patrolbot-launch`, and `patrolbot_navigation`.
- Final disposable image digest:
  `sha256:90e02665e3eafcffdc1845c6f11bc67a5346ad34e580d6377c9ed77cfac57eea`.
- Ran `ros2 launch patrolbot_navigation bringup.launch.py --show-args` in a
  network-isolated disposable container.
- Ran `colcon test` for interfaces, bridge, dock manager, and navigation:
  **69 tests, 0 errors, 0 failures, 1 unchanged copyright-template skip**.
- Ran local Python bytecode compilation and the navigation safety suite.
- Ran `git diff --check` successfully.
- Confirmed `docs/` and `SKILLS/` have no diff.

The tests cover protocol parsing/framing, malformed/duplicate commands,
idempotent results, zero-motion release, explicit motor enable, fail-closed
state inputs, transition ordering, hard motion limits, cancel/zero confirmation,
no old-velocity replay, launch containment, incident-setting pins, full polygon
coverage, recovery ordering, and the exact runtime command path.

### SBC-compatible build

- `make -B test`: passed.
- `make -B patrolbot_server`: passed against `/usr/local/Aria` on `patrolbot1`.
- Result: 32-bit i386 ELF with the expected ARIA and system-library linkage.
- Final staged binary SHA-256:
  `cac8ec006e8fc6ef1042382e68eaa0d98d565f7dedc19225fdc138e8aac3543a`.
- Build and binary remain only in `/tmp/pb-audit-impl-20260716/`.
- The SBC clock is still intentionally unchanged and reports old time; `make -B`
  forced a complete compile despite the resulting clock-skew warnings.

### Pre-deployment live read-only checks

- Pi hostname/address: `patrolbot-rpi5`, `10.0.0.2/24`.
- SBC hostname/address: `patrolbot1`, `10.0.0.1/24`.
- All three production containers healthy, zero restarts.
- SBC `patrolbot-server.service` active; PID 1289 still owns listener/client
  sockets on port 7272.
- One established hardware connection:
  `10.0.0.2:49444 <-> 10.0.0.1:7272`.
- `/scan` and `/odom` fresh; both have one publisher.
- Odometry twist: linear `0.0`, angular `0.0`; robot stationary.
- No docking server node and no docking/undocking action.
- `/cmd_vel`: one publisher (`teleop_velocity_smoother`) and one subscriber
  (`patrolbot_bridge`); no active nonzero command observed.
- No unattended ROS topic-monitor processes.
- `docker/status.sh`: `OVERALL=ready`.

## 7. Backups and rollback

Backup ID: `20260716T161104Z-pb001-pb002`.

| Copy | Location | Verification |
|---|---|---|
| Pi host-local | `/home/ubuntu/patrolbot-backups/20260716T161104Z-pb001-pb002/` | `sha256sum -c SHA256SUMS` passed again |
| SBC host-local | `/home/ros/patrolbot-backups/20260716T161104Z-pb001-pb002/` | `sha256sum -c SHA256SUMS` passed again |
| Off-host Pi | `backups/20260716T161104Z-pb001-pb002/pi5/` | 186 checksum entries passed |
| Off-host SBC | `backups/20260716T161104Z-pb001-pb002/sbc/` | 39 checksum entries passed |

A second stage-specific rollback snapshot was taken immediately before Pi/SBC
deployment work:

| Copy | Location | Verification |
|---|---|---|
| Pi host-local | `/home/ubuntu/patrolbot-backups/20260716T192954Z-pb001-pb002-deploy/` | `sha256sum -c SHA256SUMS` passed before deployment |
| SBC host-local | `/home/ros/patrolbot-backups/20260716T192954Z-pb001-pb002-deploy/` | `sha256sum -c SHA256SUMS` passed before the SBC was frozen/offlined |
| Off-host Pi | `backups/20260716T192954Z-pb001-pb002-deploy/pi5/` | 60 checksum entries passed; manifest SHA-256 `092633046556808a2acf9d82453f3893b0da3cd065a0240db772818363b73ffc` |

The Pi portion of the second snapshot is copied off-host and verified. The
original complete off-host backup remains the external rollback baseline for
both hosts; no additional SBC access was made after the operator froze SBC
work.

Every edited pre-existing repository source/configuration file also has an
ignored adjacent `.bak` copy. New files did not require a prior-file backup.

### Stage 0 rollback

In an approved stationary maintenance window only:

```bash
cp -a \
  /home/ubuntu/patrolbot-repo/ros2_ws/src/patrolbot_navigation/launch/patrolbot_navigation_launch.py.bak \
  /home/ubuntu/patrolbot-repo/ros2_ws/src/patrolbot_navigation/launch/patrolbot_navigation_launch.py
cd /home/ubuntu/patrolbot-repo/docker
docker compose --env-file .env -f docker-compose.yml up -d --no-deps navigation
```

Then re-check the checksum, node/action graph, socket, fresh sensors, stationary
odometry, and publisher count. This rollback reintroduces the unsafe docking
server and therefore must not be used except to diagnose a containment-caused
regression; autonomous operation would remain prohibited.

### Pi deployment rollback

The Pi portions of Stages 1–3 are deployed. A Pi rollback must restore the
stage-specific source/configuration and `docker/.env` from backup ID
`20260716T192954Z-pb001-pb002-deploy`, then replace only the affected Pi
containers with the prior image `patrolbot:jazzy-fa14b9b5cedf`. That backup was
captured after Stage 0 containment, so docking-server containment must remain in
the restored navigation launch. Do not roll back to an image/launch combination
that advertises the docking server.

If a future Stage 2 deployment must be rolled back, restore the exact SBC binary
from the SBC backup (expected original SHA-256
`13f2d881752552b2bff1033a42222f452ea5492bdd3bb8e5e0db926dfb9fd317`)
and the matching Pi bridge/image in one approved coordinated window. Never
restart socat or open a second hardware client.

## 8. Remaining deployment and physical gates

Before the remaining SBC deployment and commissioning:

1. Identify and accept a supervised joystick/manual-control path.
2. Map every physical front/rear bumper segment and configure nonzero masks.
3. Validate rear sonar coverage, blind regions, minimum range, and stop distance.
4. Measure and configure the exact map-frame dock pose and localization bounds.
5. Confirm the front-bumper dock baseline without weakening bumper protection.
6. Reconcile the already-synchronized SBC source with the still-original
   executable, deploy the verified server binary in a supervised window, and
   permit exactly one planned Pi bridge reconnect.
7. Restore fresh hardware telemetry and establish an operator-confirmed initial
   pose so `map -> odom` exists and every Nav2 lifecycle node becomes active.
8. Validate live `BaseState`, charge, motor, E-stop, bumper, sonar, and session
   generation without calling either hardware-control service.

Before any autonomous reverse:

1. Determine whether docking/charging asserts the reported E-stop flag.
2. Confirm both red E-stop buttons' physical state.
3. Test µARCS command 68/argument 0 with motors disabled and zero motion.
4. Determine whether physical PWR ON is required after release.
5. Confirm the dock cannot release into gravity-driven rolling.
6. Validate bumper and sonar stops before motion.
7. Run a maximum 0.10 m reverse at 0.05 m/s with a skilled operator and a
   second person on the physical E-stop.
8. Only after that passes, validate full departure and re-docking/charging.

Stop immediately on any stale sensor/link, unclear E-stop, unexpected motor
enable, invalid localization, foreign velocity/action source, publisher-count
change, reconnect, old velocity, host instability, or safety-path bypass.

## 9. Final disposition

- PB-AUDIT-001: contained in production; permanent docking re-plumb remains
  future work after commissioning.
- PB-AUDIT-002: the Pi implementation is deployed and offline/Pi-only verified.
  The finding remains open until the SBC server is deployed, connected
  stationary validation passes, and release, short reverse, full departure, and
  re-docking/charging regression all pass.
- Audit documentation: the canonical audit and this separate implementation
  record were updated only after later operator authorization; `docs/` and
  `SKILLS/` remain unchanged.
- Commit/push: not performed.

## 10. Intermediate Pi-only deployment update — 2026-07-16

This section records the first immutable Pi deployment. It is retained for
traceability and was superseded later the same day by the final hardening in
§11 and image `patrolbot:pb001-pb002-20260716t203659z`.

### 10.1 Changes deployed on `patrolbot-rpi5`

- Added missing `.dockerignore` allow rules for `patrolbot_interfaces`,
  `patrolbot_dock_manager`, and the hardware protocol static-test fixture.
  Compiled server/test artifacts remain excluded from the Docker context.
- Built immutable image `patrolbot:pb001-pb002-20260716t192954z` with image ID
  `sha256:311872aae8e45c6168eb7e12ac86f388e73648278dc46d16485467def6b17d8d`
  and revision label `pb001-pb002-20260716T192954Z`.
- Selected that image in the protected Pi `docker/.env` without displaying any
  unrelated environment values.
- Replaced `patrolbot-navigation`, `patrolbot-bridge`, and
  `patrolbot-bringup`. All three containers finished healthy with zero restart
  count on the new image.
- Deployed the typed interfaces, bridge services, direction-aware collision
  configuration, guarded-undock manager, localization gate, and revised BackUp
  trees. Generic Nav2 docking remains disabled.

The deployment sent no ROS action goal, service request, velocity, motor-enable,
charge-release, µARCS, or docking command.

### 10.2 Build and test evidence

- All five packages built successfully in the immutable ARM64 Jazzy image.
- `colcon test` result: **69 tests, 0 errors, 0 failures, 1 unchanged skip**.
- `ros2 launch patrolbot_navigation bringup.launch.py --show-args` passed in a
  network-isolated disposable container.
- The C++ protocol parser test passed in a network-isolated disposable
  container.
- The exact live topic chain is present:
  `cmd_vel_nav -> cmd_vel_smoothed -> input/navi -> cmd_vel_out -> cmd_vel`.
  Every hop after the upstream Nav2 producers has one publisher and one
  subscriber; `/cmd_vel` has one publisher and one bridge subscriber.

### 10.3 Stationary/degraded validation

- `/patrolbot/charge_release` and `/patrolbot/motor_enable` are advertised by
  the bridge; neither was called.
- `/patrolbot/undock` is advertised by the manager; no goal was sent.
- `/patrolbot/base_state` is wired from the bridge to the manager.
- `/sonar` is BEST_EFFORT and has both collision-monitor and dock-manager
  subscribers.
- Fail-closed deployed values were read back live:
  `bumpers_mapped=false`, both bumper masks `0`,
  `dock_pose_configured=false`, `allow_front_bumper_at_dock=false`, AMCL
  `set_initial_pose=false`, and Collision Monitor
  `base_shift_correction=false`.
- Bounded post-restart reads produced no stale `BaseState` or `/odom` sample.
- No unattended `ros2 topic hz`/`echo` monitor remained.
- `docker/status.sh` reported
  `OVERALL=degraded reason=sbc-unavailable`, with all three containers healthy,
  zero restarts, asynchronous bridge DDS publication, and telemetry/TF checks
  correctly skipped.

Navigation is intentionally not ready for autonomous use while the SBC is off.
The controller configured, but activation stops at the global-costmap
`map -> base_link` requirement because no fresh odometry/scan or `map -> odom`
exists. Consequently the navigation smoother, Collision Monitor, and BT
navigator remain inactive. This is a safe localization/telemetry gate, not an
accepted stationary-integration result.

### 10.4 Deployment incident and resolution

The Pi rebooted at local time `2026-07-16 22:44:38` while deployment was in
progress. On reboot, the old bridge image loaded the newly synchronized
bind-mounted Python source and restart-looped with
`ModuleNotFoundError: patrolbot_interfaces`. Selecting the tested immutable
image resolved the mismatch; the replacement bridge became healthy with zero
restarts and safely retried the unavailable SBC.

### 10.5 Explicitly deferred work

- No further SBC access or change was made after the operator froze SBC work.
- At the last read-only check, the running SBC executable was still the original
  SHA-256
  `13f2d881752552b2bff1033a42222f452ea5492bdd3bb8e5e0db926dfb9fd317`.
  Reviewed source had been synchronized earlier, but the executable was not
  replaced and `patrolbot-server.service` was not restarted.
- Live BaseState contents, rear-sonar points, bumper mapping, charge release,
  E-stop interpretation, motor policy, bridge session generation, and complete
  lifecycle activation require the SBC to return under supervision.
- No release or physical-motion test is authorized by this Pi-only deployment.

## 11. Final Pi-only hardening update — 2026-07-16

### 11.1 What changed and why

| Area | Files | Change | Reason |
|---|---|---|---|
| Upstream command arbitration | `patrolbot_navigation_launch.py`, `nav2_params.yaml` | Added `nav_command_mux`; controller publishes to `/input/nav_controller`, behavior server to `/input/nav_behavior`, and the reserved docking input is `/input/docking`. The mux is the only publisher of `/cmd_vel_nav`. | Prevent simultaneous autonomous producers and permanently constrain any future docking producer to the pre-smoother, pre-collision path. |
| Runtime path validation | `dock_manager.py`, `test_safety_contract.py` | Extended the exact publisher/subscriber contract to include all three upstream mux inputs and the mux output. | Make an absent mux, extra publisher, missing subscriber, or reintroduced `docking_server` an undock blocker. |
| Typed readiness | `DockReadiness.msg`, `dock_manager.py` | Added `/patrolbot/dock_readiness` at 2 Hz with every link, sensor, localization, bumper, motor, charge, action-idle, and command-path gate plus a blocking reason. | Expose one machine-readable fail-closed commissioning status instead of requiring operators to infer readiness from unrelated topics. |
| Operator-controlled dock localization | `InitializeDockPose.srv`, `dock_manager.py`, `contracts.py`, `nav2_params.yaml` | Added `/patrolbot/initialize_dock_pose`. It requires explicit authorization, a configured dock ID/pose, fresh connected hardware state, confirmed charging, disabled motors, zero faults, fresh stationary odometry, and no active navigation. It publishes `/initialpose` and reports success only after a newer AMCL estimate converges within configured pose/covariance limits. | Establish a bounded, auditable dock-pose initialization mechanism without assuming that every restart occurred at the dock. Production remains blocked with `dock_pose_configured=false`. |
| PatrolBot charging adapter | New `patrolbot_docking_plugins` package and docking parameters | Added a Jazzy `opennav_docking_core::ChargingDock` plugin that requires fresh typed `BaseState` plus fresh `/battery`, uses `BaseState.charge_state` rather than `BatteryState.current`, and requests zero-motion release through `/patrolbot/charge_release`. | Correct the battery-interface mismatch and provide a future-safe dock plugin compatible with PatrolBot's real charging telemetry and release protocol. The generic docking server remains disabled. |
| Bridge testability | `bridge_node.py` | Parameterized server address/port and connection, receive, reconnect, and control timeouts while preserving the production defaults `10.0.0.1:7272`, 5/3/3/6 seconds. | Allow a real bridge instance to be tested against loopback without ever probing the single-client hardware server. |
| Pi readiness command | `docker/status.sh` | Added exact endpoint-identity checks for the complete safe path, a zero-publisher assertion on `/input/docking`, docking-server absence, required service/action checks, and readiness-topic publisher count before the SBC availability gate. | Verify PB-AUDIT-001 containment even when the SBC is offline and `/odom` cannot become fresh. |
| Isolated integration coverage | New fake-SBC, manager, Collision Monitor, and plugin tests | Added loopback protocol/reconnect tests, a synthetic full manager action/service harness, real Jazzy Collision Monitor tests, exhaustive boundary coverage, and typed charge-state tests. | Prove release produces no velocity, reconnect does not replay velocity, stale rear sonar cancels reverse, and direction-aware polygons cover forward/reverse/rotation/arcs. |
| Image integration | `.dockerignore`, `docker/Dockerfile`, package manifests/CMake | Added the plugin package and new interfaces/tests to the deterministic image build. | Ensure the runtime and test artifacts are built against the same installed Jazzy ABI. |

The ROS `float32` integration test also exposed that nominal `0.10 m` and
`0.05 m/s` goals arrive a few nanounits above their decimal literals. Goal
validation now admits only a `1e-6` representation tolerance; real over-limit
goals are still rejected rather than clamped.

### 11.2 Resulting deployed velocity path

```text
controller_server -> /input/nav_controller --+
                                                |
behavior_server   -> /input/nav_behavior -----+--> nav_command_mux
                                                |        |
future docking    -> /input/docking -----------+        v
                                                    /cmd_vel_nav
                                                          |
                                             velocity_smoother
                                                          |
                                                /cmd_vel_smoothed
                                                          |
                                               collision_monitor
                                                          |
                                                    /input/navi
                                                          |
                                                     cmd_vel_mux
                                safety priority 10 --------+
                                manual priority 8 ---------+
                                                          |
                                                   /cmd_vel_out
                                                          |
                                           teleop_velocity_smoother
                                                          |
                                                     /cmd_vel
                                                          |
                                                patrolbot_bridge
```

`/input/docking` has one subscriber and zero publishers. `docking_server` is
absent. `nav_command_mux` has short 0.25-second input timeouts and is the only
publisher of `/cmd_vel_nav`. `teleop_velocity_smoother` remains the only final
publisher of `/cmd_vel`.

### 11.3 Jazzy behavior revalidated

- Installed Pi package: `ros-jazzy-twist-mux 4.5.0`; its shipped configuration
  confirms `topics.<name>.topic`, `timeout`, and `priority`, with higher numeric
  priority winning.
- The installed Jazzy `ChargingDock` header was used as the plugin ABI. Its
  `disableCharging()` and `hasStoppedCharging()` calls are non-blocking; the
  adapter therefore starts an asynchronous release request and polls its future.
- Official Nav2 Collision Monitor documentation and Jazzy source confirm that
  an enabled source exceeding `source_timeout` produces a stop, and that a
  velocity polygon selects the first matching sub-polygon. This matches the
  configured nonzero 0.5-second scan/sonar timeouts and the ordered boundary
  bands.
- Official Nav2 docking documentation confirms `SimpleChargingDock` determines
  charging using `BatteryState.current` when battery status is enabled. The
  PatrolBot plugin intentionally does not use that inference.

References used for this update:

- <https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html>
- <https://docs.nav2.org/configuration/packages/configuring-docking-server.html>
- <https://docs.nav2.org/tutorials/docs/using_docking.html>
- <https://api.nav2.org/nav2-jazzy/html/md_nav2_docking_README.html>
- Installed Jazzy headers and package examples under `/opt/ros/jazzy` in the
  immutable test image.

### 11.4 Verification results

- Final immutable image:
  `patrolbot:pb001-pb002-20260716t203659z`.
- Image ID:
  `sha256:cfdc6b61abc31746df2590ebe93d3360b8799116975f26bc3d9bedc8eb2a27ef`.
- Revision label: `pb001-pb002-20260716T203659Z`.
- Final image build: all six selected ROS packages compiled against Jazzy.
- Static/unit pass: **101 discovered, 0 errors, 0 failures, 16 intentional
  skips**. The skipped set was 15 explicitly isolated integration cases and one
  unchanged copyright-template skip.
- Explicit isolated integration pass on the exact final image:
  - fake SBC/real bridge: **3 passed**;
  - fake telemetry/release/BackUp/real dock manager: **2 passed**;
  - real Jazzy Collision Monitor with synthetic scan, sonar, and velocity:
    **10 passed**.
- Therefore **100 unique tests passed** and only the unchanged copyright test
  remained skipped.
- All integration containers used `--network none`, loopback-only fake hardware,
  and isolated ROS domains 92, 93, and 94. They had no route to the SBC.
- `git diff --check`, Bash syntax validation, Python byte-code compilation, YAML
  parsing through the tests, flake8, and the C++ plugin build passed.

### 11.5 Live Pi deployment and stationary evidence

- All three containers run the final image, report healthy, and have zero
  restarts.
- The live path checker reported the exact endpoints shown in section 11.2.
- `/docking_server` is absent.
- `/patrolbot/charge_release`, `/patrolbot/motor_enable`,
  `/patrolbot/initialize_dock_pose`, and `/patrolbot/undock` are advertised but
  were not called.
- `/patrolbot/dock_readiness` has one publisher and reported
  `ready_for_undock=false`, `command_path_valid=true`,
  `dock_pose_valid=false`, `bumpers_valid=false`, `sonar_fresh=false`, and
  hardware link/state invalid while the SBC is offline.
- Live parameters remain fail-closed: `dock_pose_configured=false`,
  `allow_front_bumper_at_dock=false`, AMCL `set_initial_pose=false`, Collision
  Monitor `base_shift_correction=false`, rear-sonar timeout `0.5`, and upstream
  mux `use_stamped=false`.
- One bounded read of the priority-10 safety input and final `/cmd_vel` showed
  all six Twist components equal to zero. No ROS monitor process remained.
- `docker/status.sh` ended with
  `OVERALL=degraded reason=sbc-unavailable`, which is the intended state until
  supervised SBC restoration.

No velocity, docking/undocking action goal, charge-release service request,
motor-enable service request, µARCS command, or hardware-server connection was
sent during this deployment.

### 11.6 Backups and rollback for this deployment

The pre-deployment Pi set remains at:

- Pi host:
  `/home/ubuntu/patrolbot-backups/20260716T192954Z-pb001-pb002-deploy/`;
- off-host:
  `backups/20260716T192954Z-pb001-pb002-deploy/pi5/`.

The off-host copy contains 60 checksummed files. Re-running `sha256sum -c
SHA256SUMS` produced 60 `OK` results in
`SHA256SUMS.verify-offhost.txt`. The checksum of the checksum manifest is
`092633046556808a2acf9d82453f3893b0da3cd065a0240db772818363b73ffc`.
The backup environment file remains protected and its values are not reproduced
here.

Rollback target:

- image `patrolbot:pb001-pb002-20260716t192954z`;
- image ID
  `sha256:311872aae8e45c6168eb7e12ac86f388e73648278dc46d16485467def6b17d8d`;
- revision `pb001-pb002-20260716T192954Z`.

Restore the checksummed `files/home/ubuntu/patrolbot-repo/...` paths with
`cp -a`/`rsync -a`, restore the protected prior Docker environment from the
same backup, validate `docker compose config --quiet`, select the rollback
image, recreate the three Pi containers, then rerun `docker/status.sh` and the
exact endpoint checks. Do not perform rollback while a hardware connection or
motion command is active.

No new SBC backup was created or accessed in this update because the operator
froze all SBC work. The previously verified SBC backup remains unchanged.

### 11.7 Still physically unverified

PB-AUDIT-002 remains open. These items require the SBC to return and a controlled
two-person hardware session:

- physical bumper-bit mapping and bumper actuation;
- actual rear-sonar geometry, dropout behavior, and obstacle stops on the robot;
- physical E-stop state versus dock/charge inhibition;
- whether `CHARGE 0` releases the dock inhibit and whether physical PWR ON is
  required afterward;
- deployed SBC acknowledgement, duplicate replay, timeout, motor-enable policy,
  and reconnect/session behavior;
- surveyed dock pose and operator-approved `dock_pose_configured=true`;
- zero-motion release, very short low-speed reverse, full departure, re-docking,
  and charging regression.

Autonomous docking and undocking must remain disabled until those gates pass.

## 12. Canonical-audit reconciliation

This file is the separate record of what changed and how it was implemented.
`PATROLBOT_RPI5_AUDIT.md` remains the full audit evidence record and now has a
dated §1A disposition that distinguishes original findings from the later Pi
deployment.

### Implemented on the Raspberry Pi 5

- PB-AUDIT-001 active bypass containment, upstream producer arbitration, exact
  endpoint enforcement, and the sole final `/cmd_vel` publisher contract.
- PB-AUDIT-002 Pi-side typed interfaces, fail-closed readiness, guarded undock
  state machine, cancellation/timeout handling, and constrained Nav2 BackUp.
- PB-AUDIT-003 BackUp-first custom recovery trees without globally enabling RPP
  reverse.
- PB-AUDIT-004 direction-aware rear-sonar Collision Monitor integration with
  stale/missing data blocking reverse.
- PB-AUDIT-007 removal of forced origin reset and an operator-authorized,
  convergence-checked dock-pose initialization service.
- PB-AUDIT-009 Pi-side explicit motor-enable service, separate from release and
  motion.
- PB-AUDIT-011 a PatrolBot-specific Jazzy charging plugin using typed raw charge
  state rather than `BatteryState.current` inference.
- Immutable-image provenance, checksum-verified Pi rollback data, fake-hardware
  integration coverage, and a machine-readable readiness endpoint.

### Deliberately not claimed as complete

- The original SBC executable still lacks the deployed release/explicit-enable
  protocol and retains its prior motor-enable behavior.
- No hardware acknowledgement, charge release, motor enable, E-stop, PWR ON,
  bumper, sonar-obstacle, reverse-motion, dock-departure, or re-docking test has
  passed on the physical robot.
- Bumper masks and dock pose remain unconfigured, so the deployed manager is
  intentionally unable to authorize autonomous reverse.
- The generic docking server remains disabled. The new docking input and plugin
  are future integration points, not an enabled autonomous feature.
- PB-AUDIT-002 remains open until all stationary and controlled physical gates
  in §11.7 and the canonical audit §1A pass.

### Follow-on Pi-only audit work

The seven-item follow-on was completed and deployed after §11. PB-AUDIT-014 is
closed on the Pi; the Pi portions of PB-AUDIT-012 and PB-AUDIT-006 are
implemented; DDS discovery is loopback-only; selected PB-AUDIT-019 cleanup and
PB-AUDIT-020 runtime source provenance are complete; and the sensor/geometry
work is present only as deliberately uncommissioned scaffolding. PB-AUDIT-015
remains blocked because the joystick receiver is absent.

The exact follow-on implementation, code delta, backup identity, tests, live
state, and rollback are recorded in:

- `PI5-SEVEN-ITEM-HARDENING-IMPLEMENTATION.md`;
- `PI5-SEVEN-ITEM-HARDENING-CODE-CHANGES.md`.

## 13. Current production handoff — 2026-07-17

The current production Pi supersedes the image identity in §11 while retaining
the same PB-AUDIT-001/002 safe command architecture:

| Item | Current value |
|---|---|
| Image | `patrolbot:pi5-hardening-20260716t221749z` |
| Image ID | `sha256:80cba4ee514bcf36756540baa446a8532108a27a4f0f2cd25618c00779e09927` |
| Revision | `pi5-hardening-20260716T221749Z` |
| Source digest | `8ab058d2f68342c41a7db510cf3d924b3133fb98e3afea8d78e6b55f9e5f885f` |
| Runtime | Three healthy Pi containers, zero restarts |
| Readiness | `OVERALL=degraded reason=sbc-unavailable` |

Live revalidation again confirmed `docking_server` absent, no publisher on
`/input/docking`, `nav_command_mux` as the only `/cmd_vel_nav` publisher, and
`teleop_velocity_smoother` as the only final `/cmd_vel` publisher. The source
digest embedded in the image exactly matches the deployed copied source.

The new pre-change and post-change backup sets are:

- `backups/20260716T210902Z-pi5-seven-item-hardening/`;
- `backups/20260716T223238Z-pi5-seven-item-hardening-postchange/`.

Both Pi archives have verified same-host and off-host copies. The SBC was not
accessed or changed. No hardware interface was called and the robot did not
move. PB-AUDIT-002 therefore remains open under the physical dependency order
in the canonical audit §1A.

## 14. SBC return and stationary cutover preparation — 2026-07-21

The SBC returned under supervision. The preparation phase completed without a
service restart, hardware-control call, velocity publication, pose publication,
navigation goal, or robot motion.

### 14.1 Live reconciliation

- Both hosts were reachable through the operator-approved explicit user SSH
  configuration. The workstation's unsafe system SSH include was bypassed but
  not modified.
- One and only one TCP 7272 session existed between the Pi bridge and SBC
  server. `/scan`, `/odom`, `/sonar`, `/battery`, and typed base state were
  fresh; odometry velocity was zero.
- The canonical server source, protocol unit, header, test, and Makefile on the
  SBC match this repository byte-for-byte.
- The running executable is still the original
  `13f2d881752552b2bff1033a42222f452ea5492bdd3bb8e5e0db926dfb9fd317`
  binary. This is a concrete deployment gap: the source is correct, but the
  active process cannot handle the staged release or motor-enable protocol.

### 14.2 New verified rollback set

Backup ID: `20260721T181448Z-pb002-stationary-cutover`.

| Copy | Location | Verification |
|---|---|---|
| SBC host-local | `/home/ros/patrolbot-backups/20260721T181448Z-pb002-stationary-cutover/` | 34 entries passed `sha256sum -c`; original binary hash pinned |
| Pi host-local | `/home/ubuntu/patrolbot-backups/20260721T181448Z-pb002-stationary-cutover/` | 167 entries passed `sha256sum -c`; deployed source digest `8ab058d2f68342c41a7db510cf3d924b3133fb98e3afea8d78e6b55f9e5f885f` recorded |
| SBC off-host | `backups/20260721T181448Z-pb002-stationary-cutover/sbc/` | 34 entries recomputed and passed from the copied files |
| Pi off-host | `backups/20260721T181448Z-pb002-stationary-cutover/pi5/` | 167 entries recomputed and passed from the copied files |

The source backup contains the active executable, source/tests/Makefile,
service unit/drop-in, socket/process state, ARIA parameters, logs, metadata,
and checksums. The Pi backup contains deployed configuration, image/container
inspection, source provenance, ROS graph/parameters/endpoints, live telemetry,
dock readiness, and dashboard bridge state. Access-controlled configuration was
backed up without printing its secret values.

### 14.3 Isolated candidate build

The checksum-matched source was copied into a new SBC build directory rather
than built in the deployed tree:

`/home/ros/patrolbot-builds/20260721T181448Z-pb002-stationary-cutover/`

- `make -B test`: passed;
- `make -B patrolbot_server`: passed with only legacy ARIA-header unused-
  parameter warnings;
- output: 32-bit i386 ELF with every dynamic dependency resolved;
- required protocol strings: `CHARGE_RELEASE` and `MOTOR_ENABLE` present;
- candidate SHA-256:
  `cac8ec006e8fc6ef1042382e68eaa0d98d565f7dedc19225fdc138e8aac3543a`;
- repository static safety-contract test proving that the release branch calls
  neither velocity nor motor-enable APIs: passed as part of 29 passing static
  tests.

### 14.4 Deferred live work

The robot currently reports charge state zero and motors enabled; the joystick
receiver is absent. The operator deferred E-stop, joystick, and charging work.
Therefore the active binary was not replaced and no protocol request was sent.

The next supervised non-motion window must first establish stationary odometry
and disabled motors, then atomically install the tested candidate and restart
only `patrolbot-server.service`. Charging-state transition, actual dock release,
dock departure, and re-docking remain explicitly deferred. Any future velocity
publication is approval-gated: state the exact command and expected bound to the
operator before sending it.

### 14.5 Logging-query correction and staged fix

The first systemd-229 query form (`journalctl --user -u`) incorrectly appeared
to show no user-service records. Querying the journal by
`_SYSTEMD_USER_UNIT=patrolbot-server.service` recovered the full ARIA startup,
laser, listener, and Pi-connection output. The server already logs to stdout;
no ARIA source change is required.

PB-AUDIT-013 remains valid but narrower: the two `append:` directives are
rejected on systemd 229, the fallback journal is volatile, timestamps are wrong
because of PB-AUDIT-010, and retention is not explicitly bounded. Canonical
systemd-229-compatible unit/drop-in files are now staged under
`hardware/patrolbot_hw_server/systemd/`, using journal output only. A static
contract rejects any future `append:` regression. The live unit/drop-in were
not changed because the operator deferred the service maintenance window.

The staged root-maintenance package also contains:

- a Chrony-2.1-compatible `makestep 1.0 -1` directive and supervised one-time
  clock-step procedure for PB-AUDIT-010;
- a persistent journald policy capped at 128 MiB, retaining at most 30 days and
  keeping 2 GiB free for PB-AUDIT-013;
- an OpenSSH-7.2-compatible key-only/no-root-login policy for the addressable
  part of PB-AUDIT-022; and
- an explicit PB-AUDIT-012 stop: no shutdown daemon is provided until a real
  RI/modem-line or independent power-controller signal is selected and
  bench-tested.

The journald keys were confirmed present in the installed systemd-229 manual.
The SSH policy parsed on the actual SBC and resolved to
`PasswordAuthentication no`, `ChallengeResponseAuthentication no`, and
`PermitRootLogin no`. Chrony 2.1 has no usable unprivileged parse-only command,
so its live validation remains part of the root maintenance procedure. None of
these assets was installed on the live host.

### 14.6 Residual SBC container cleanup

PB-AUDIT-019's 14 stopped experimental Docker containers were individually
confirmed exited. Thirteen had zero-byte writable layers; `myros2` had a
33.6 MiB writable layer and a host bind mount. Before removal, backup ID
`20260721T195507Z-pb019-container-cleanup` preserved:

- `docker ps -as`, container inspection, image inspection, and every container
  log;
- a full export of the `myros2` filesystem; and
- independently recomputed passing SHA-256 manifests in both
  `/home/ros/patrolbot-backups/` and the repository's ignored `backups/`
  directory.

Only the 14 exact stopped container IDs were removed. No image, volume,
bind-mounted host file, active process, service, or network was deleted. After
cleanup, `patrolbot-server.service` remained active, the same single TCP 7272
session remained established, all three Pi containers were healthy with zero
restarts, the final `/cmd_vel` publisher remained
`teleop_velocity_smoother`, and odometry twist remained exactly zero.

### 14.7 Historical docked-state recheck

The operator physically re-seated the robot on the charger. Three fresh typed
samples over approximately eight seconds consistently reported raw charge
state `3`; `/battery` reported charging at 29.7 V; fault flags were zero; and
odometry twist was exactly zero. This confirmed charging but did **not** clear
the cutover gate: every sample still reported `motors_enabled=true`. Contact
charging therefore cannot be treated as proof of motor disable on this robot.

No binary was replaced, service restarted, charge-release request sent, motor
request sent, velocity published, pose set, or navigation goal issued during
that recheck. At that point, the staged Pi/SBC preconditions required a
separate motor-disable window and rejected release while motors were enabled.
Section 15 supersedes this historical disposition: manual review and live
manual-dock behavior proved that requirement created a deadlock, and the
corrected release now owns the zero-motion disable-and-confirm transition.

### 14.8 Verification refresh

- `make -B test` on the isolated SBC candidate: passed;
- `make -B patrolbot_server`, `file`, `ldd`, and `sha256sum` on the isolated
  SBC candidate: passed; checksum remained
  `cac8ec006e8fc6ef1042382e68eaa0d98d565f7dedc19225fdc138e8aac3543a`;
- repository static safety contracts: 29 passed;
- focused bridge/control/telemetry/dock contracts in the deployed ROS image:
  63 passed, 5 skipped as designed;
- fake dock-manager ROS integration in isolated domain 221: 2 passed;
- `.venv/bin/mkdocs build --strict`: passed;
- `git diff --check` and focused trailing-whitespace scan: passed.

An initial fake-integration invocation used ROS domain 251, which Fast DDS
rejects because its calculated port exceeds the valid range. That was a test
harness invocation error, not a product failure; the unchanged tests passed in
valid isolated domain 221.

### 14.9 PB-AUDIT-016 cause confirmation

The operator confirmed that the recorded SBC outages were forced shutdowns
performed before the main PC/SBC finished its shutdown sequence. This matches
the bridge's no-FIN disconnect, long host absence, and SBC log-corruption
evidence. PB-AUDIT-016's root cause is therefore resolved as operator-induced
unclean power removal, not an unexplained network or charger failure.

The prevention gap is not relabeled as solved: operators must wait for the main
PC/SBC shutdown to complete before physical PWR OFF, and PB-AUDIT-012 continues
to track the missing automatic graceful-shutdown signal path.

The canonical SBC maintenance guide now records the interim manual prevention
procedure: request main-PC shutdown, wait for the LCD `PC Power OFF` finished
state, and only then remove physical power. An SSH disconnect alone is not
accepted as evidence that shutdown completed.

### 14.10 Dashboard concurrency refresh

After the separate dashboard agent finished, the dashboard repository was
clean at `d4b3cf9093927183d4c7adb1e7cb9a07529bf2c0`. The Pi still runs the
hand-made standalone `patrolbot-web-bridge`, not the prepared compose service,
and its command flag is enabled. Static inspection of that exact checkout
confirmed its command surface is limited to NavigateToPose, `/initialpose`,
and goal cancellation; it contains no `/cmd_vel`, charge-release,
motor-enable, undock, or direct SBC path.

No dashboard command was sent in this audit. Before any server cutover or
physical phase, the live graph must still prove no active navigation goal and
the exact single final `/cmd_vel` publisher. Migrating the standalone bridge to
Compose remains a separately coordinated operational change because the
current compose service intentionally omits command mode.

## 15. Stationary Pi/SBC cutover and protocol commissioning — 2026-07-21

Section 15 supersedes §14's deferred-cutover disposition. The exact command
responses, timestamps, hashes, backup paths, and rollback procedure are in
`PB-AUDIT-002-STATIONARY-CUTOVER-REPORT.md`.

### 15.1 Manual-charge deadlock found and corrected

The third-generation guide documents separate µARCS `CHARGE 68` and `ENABLE
4` operations and does not describe charging as a mechanical wheel lock. Live
manual docking then proved that raw charge state `3` can coexist with
`motors_enabled=true`. The initial Pi/SBC release precondition required motors
already disabled and therefore could not release the exact state produced by
manual docking.

The release operation now owns a strictly zero-motion transition: it disables
motors when necessary, confirms three stable disabled observations, sends
`CHARGE 0` once, confirms three stable raw state-zero observations, and leaves
motors disabled. It never sets velocity or enables motors. Nonzero `DRIVE`
remains rejected until charge state is zero, E-stop is clear, motors are
explicitly enabled, faults are zero, and no control operation is pending.

The Pi bridge now permits this server-owned disable step while charging. The
dock-manager precheck no longer creates the old motor-state deadlock, while
dock-pose initialization still independently requires motors disabled.

### 15.2 Final builds and rollback set

The final isolated SBC build passed `make -B test`, `make -B
patrolbot_server`, 32-bit ELF inspection, and dependency resolution:

- executable SHA-256:
  `b9606502dc815c602397f93f48a0af7eb783ab3e819d87e77fcdf01d40394393`;
- source SHA-256:
  `a122210a6bc8887924cf06ec85a2e3b31d696717699397122ba8694dd33182e9`.

The final Pi build enforced source digest
`fc0b8eabc028748873ed7af3ba2e8ef12bbad434866ed13b5ad33dbe48603eb6`
and produced image
`patrolbot:pb002-manual-release-20260721t210500z`, image ID
`sha256:2b683343b4afdb4d1b24e7368469f94051456f9ada3e30c07d91b90d3e766d59`.
Disposable image acceptance produced 92 focused contract passes and five
fake-hardware ROS integration passes.

In addition to full backup `20260721T181448Z-pb002-stationary-cutover`, narrow
pre-deployment backup `20260721T204423Z-pb002-manual-release-deploy` preserved
every replaced Pi/SBC file, protected Pi environment, old image/container
identity, running SBC binary, service/socket evidence, and journal. Host-local
and off-host checksum verification passed from the copied files. The original
SBC executable remains preserved as
`13f2d881752552b2bff1033a42222f452ea5492bdd3bb8e5e0db926dfb9fd317`.

### 15.3 Controlled cutover

The seven-file Pi source delta and inert provenance assets were installed, the
protected environment selected the new immutable image, and only the three
PatrolBot containers were recreated. All returned healthy with zero restarts,
and live source matched the image digest. The standalone dashboard container
was not touched.

Only `patrolbot-server.service` was stopped. The candidate and matching source
were installed atomically and only that service was started. One immediate
ARIA connection attempt encountered a three-second close/reopen race; the
existing unit retry then succeeded. The laser recovered in approximately 50
seconds. The running/deployed binary hash matched the candidate, exactly one
TCP 7272 client returned, and the bridge recorded the one planned reconnect as
`session_generation=2`.

Post-cutover telemetry was fresh, charge state remained `3`, motors were
disabled, E-stop was reported clear, faults were zero, the command path had one
final `/cmd_vel` publisher, navigation was idle, and odometry was exactly zero.

### 15.4 Zero-motion results

Motor-enable request `47934916-09c4-445c-abf8-f14836667678` was rejected while
charging with `accepted=false`, `success=false`, code `3`, message `motor
enable refused while charging`, charge state `3`, motors disabled, and session
`2`. No movement followed.

Charge-release request `d1e71d90-ce9f-4937-b749-0de3b9372e06` succeeded after
3.255 seconds with code `0`, raw charge state `0`, motors disabled, and session
`2`. Odometry remained zero. Replaying the identical UUID returned the cached
terminal result in 0.067 seconds. The SBC journal contains exactly one accepted
and one completed entry for that UUID, proving that replay did not send a
second hardware action or change the session.

Because the robot remained physically seated on the contacts, raw charge state
subsequently returned to `3` without motion. Three later samples remained
stable at state `3`, 29.7 V charging status, motors disabled, E-stop clear,
faults zero, session `2`, and zero odometry. The required stationary end state
was therefore achieved without a manual re-seat.

### 15.5 Current boundary

PB-AUDIT-009's DRIVE-triggered auto-enable defect is corrected and the live
charging rejection is commissioned. A successful explicit enable is still
deferred because it belongs immediately before the physical reverse phase.
PB-AUDIT-002 remains open for E-stop/joystick validation, measured bumpers and
rear-sonar obstacle behavior, dock-pose localization, successful motor enable,
guarded reverse, full undock, and full re-docking/link-loss/cancellation
regression.

No nonzero velocity is authorized by this stationary result. Before any future
velocity publication, the operator must approve the exact speed, distance or
duration, path, sensor gates, and stop conditions.

### 15.6 Final verification

- `git diff --check`: passed;
- repository static safety contracts: 29 passed;
- immutable-image focused contracts: 92 passed;
- immutable-image fake-hardware ROS integrations: 5 passed;
- `.venv/bin/mkdocs build --strict`: passed;
- running/deployed SBC binary hash: `b9606502...94393`;
- SBC journal: exactly one accepted and one completed line for the release
  UUID;
- Pi image and deployed source digest: `fc0b8eab...03eb6`;
- three Pi containers: healthy, zero restarts, same immutable image;
- final TCP 7272 client count: one;
- final `/cmd_vel` publisher count: one;
- final charge/motor/motion state: stable charge `3`, motors disabled, zero
  faults, odometry zero.

`docker/status.sh` ended `OVERALL=degraded reason=ros-readiness`. Its hardware,
provenance, command-path, sensor, QoS, lifecycle, odometry, and TF checks
passed; the degraded result is the expected fail-closed effect of the still
deferred localization/dock physical gates, not an SBC-link or cutover failure.

## 16. Remaining SBC non-motion maintenance audit — 2026-07-21

After the stationary protocol phase, the robot still reported stable charging,
motors disabled, zero faults, session `2`, and zero odometry. The remaining
SBC-only findings were then rechecked against the root-maintenance runbook.

### 16.1 PB-AUDIT-013 user-unit fix deployed

The verified systemd-229-compatible unit and drop-in were staged on the SBC,
matched their recorded SHA-256 values, installed as `ros:ros` mode `0644`, and
loaded with `systemctl --user daemon-reload`. The server was not restarted.

Live systemd resolution now reports:

```text
StandardOutput=journal
StandardError=journal
SyslogIdentifier=patrolbot_server
```

The PID remained 21102, the one TCP 7272 session remained established, the
running binary hash remained `b9606502...94393`, charge remained `3`, motors
remained disabled, and odometry remained zero. This closes the unsupported
`append:` user-unit portion of PB-AUDIT-013. Persistent journal storage remains
open because it is root-owned and must follow clock correction.

### 16.2 Root and hardware boundary

Non-interactive `sudo -n` is not available to the `ros` account. The live host
still reports:

- system time in 2021, approximately 167,322,912 seconds slow;
- RTC time in 2008;
- Chrony 2.1 active with a selected current 2026 source, but the system clock
  not synchronized because the large offset has never been stepped;
- no active `makestep` directive in `/etc/chrony/chrony.conf`;
- no `/var/log/journal` persistent store;
- SSH root/password policy not fully hardened in the root-owned main config.

Therefore PB-AUDIT-010, the root-owned remainder of PB-AUDIT-013, and the
addressable portion of PB-AUDIT-022 require an operator-provided root
maintenance session. The agent did not request a password, bypass sudo policy,
or partially edit root configuration. RTC correction also needs physical
battery replacement and reboot validation.

PB-AUDIT-012 remains intentionally without an executable daemon: the live
serial-to-socat topology exposes no demonstrated RI/modem-line or independent
power-fail signal to Linux. A daemon without a real bench-tested input would
not prevent the operator-confirmed forced-shutdown failure mode.
