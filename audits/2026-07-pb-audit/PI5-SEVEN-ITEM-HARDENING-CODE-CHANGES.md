# Raspberry Pi 5 Seven-Item Hardening Code Changes

## 1. Review boundary

This is the file-level review companion to
`PI5-SEVEN-ITEM-HARDENING-IMPLEMENTATION.md`. It describes the exact delta from
the verified pre-change backup
`20260716T210902Z-pi5-seven-item-hardening`, not the larger Git diff against the
older repository HEAD.

The earlier PB-AUDIT-001/002 packages, upstream command mux, guarded undock
manager, PatrolBot docking plugin, and typed interfaces are documented in
`PB-AUDIT-001-002-IMPLEMENTATION-REPORT.md`. They were included and retested in
the deployed image but are not misrepresented here as new seven-item work.

Generated `__pycache__`, `.pyc`, build, install, log, backup, and compiled test
files are not production source. They are excluded from the Docker context and
the source digest.

## 2. Safety invariants reviewers must check first

The following values and topology are intentional and protected by tests:

| Invariant | Required value |
|---|---|
| Generic docking server | absent |
| Final `/cmd_vel` publisher | `teleop_velocity_smoother` only |
| Future docking input | `/input/docking` into `nav_command_mux`, never the outer mux |
| Navigation path | upstream mux → nav smoother → Collision Monitor → outer mux → final smoother |
| `SCAN_RANGE_MIN` | `0.25` |
| RPP internal collision setting | `use_collision_detection: false` |
| RPP global reverse | disabled |
| Collision Monitor reconnect setting | `base_shift_correction: false` |
| Inflation cost scaling | `4.0` as a float in both costmaps |
| Active footprint | unchanged until physical measurement |
| Bumper enforcement | false until physical segment mapping |
| Raw scan candidate | false and absent from active safety configuration |
| Physical calibration | `commissioned: false` |
| DDS discovery | `LOCALHOST` in all three containers |

Any proposed review correction that changes one of these must first reproduce
the incident or physical evidence that justified the existing value.

## 3. Exact file inventory

### 3.1 Container, deployment, and provenance

| File | Change | Why |
|---|---|---|
| `.dockerignore` | Includes the digest/status scripts and hardware protocol fixture while excluding generated caches, builds, binaries, backups, and editor files. | Makes the build context deterministic and prevents local artifacts from altering the image. |
| `docker/Dockerfile` | Installs official Jazzy `urg_node`; copies the digest script and hardware protocol fixture; verifies expected digest at build time; stores the digest; applies OCI revision/source labels. | Supports uncommissioned URG testing and makes the exact built source machine-verifiable. |
| `docker/docker-compose.yml` | Sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in the common block and each service; passes revision/source digest build arguments. | Contains unauthenticated DDS discovery to the Pi while preserving loopback communication and build identity. |
| `docker/source-tree-digest.sh` | New deterministic hash over the six built ROS packages and hardware protocol source. | Detects copied-tree drift without requiring a Git checkout on the Pi. |
| `docker/status.sh` | Adds source/image digest comparison, DDS-scope checks, exact endpoint owners, docking/candidate absence, smoother/battery/bumper readback, joystick readiness, guarded interface presence, and no-leaked-monitor checks. Empty endpoint results require three samples to avoid a DDS discovery race. | Turns audit assumptions into a read-only deployment gate and still reports the expected SBC-off degraded state. |

### 3.2 `patrolbot-launch`

| File | Change | Why |
|---|---|---|
| `launch/mobile_base.xml` | Removes the empty component container and keeps only the actual mux/final smoother launch path. | Eliminates an unused process/DDS participant without changing command behavior. |
| `launch/readlidar.py` | Deleted. | Obsolete, unreferenced, and contradicted the current bridge-owned LiDAR path. Its lingering deployed copy was the only file preventing source/image digest equality and was removed after backup. |
| `lifecycle_mgr.py` | Removes lifecycle logic tied to the deleted empty container and uses the active node set. | Aligns lifecycle observation with the processes that actually exist. |
| `package.xml` | Cleans dependencies/metadata associated with the removed path and declares active launch dependencies. | Keeps the installed package internally consistent. |
| `param/defaults/smoother.yaml` | Replaces all ignored Kobuki-era keys with official Jazzy velocity-smoother parameters and explicit limits/timeouts. | Closes PB-AUDIT-014 and prevents Jazzy defaults from silently allowing larger commands. |
| `setup.py` | Installs only live launch/config resources and removes obsolete script packaging. | Prevents the deleted path from reappearing in the installed package. |

### 3.3 `patrolbot_bridge`

| File | Change | Why |
|---|---|---|
| `config/bridge_params.yaml` | Adds explicit uncommissioned bumper masks and disabled raw-scan candidate settings. | Makes unsafe physical assumptions visible and fail-closed. |
| `patrolbot_bridge/bridge_node.py` | Adds protected raw-scan candidate publishing; named battery/fault diagnostics; typed bumper validity/masks; explicit ROS diagnostic byte constants; stop event and worker-thread joins. | Supplies PB-AUDIT-012/006 data safely, allows near-field testing without changing `/scan`, and prevents shutdown-time publishing through a destroyed ROS context. |
| `patrolbot_bridge/safety_telemetry.py` | New pure fault decoder and 24/23 V battery policy helper. | Keeps manual-derived interpretation deterministic and unit-testable. |
| `test/test_safety_telemetry.py` | New boundary and bit-decoding tests. | Proves documented bits are named, unknown bits are retained, and exact voltage boundaries are correct. |

### 3.4 `patrolbot_navigation`

| File | Change | Why |
|---|---|---|
| `CMakeLists.txt` | Installs the safety-policy/calibration scripts, physical calibration, candidate launch, and new tests. | Ensures source and installed image have the same hardening artifacts. |
| `config/nav2_params.yaml` | Removes ignored nav-smoother `cmd_vel_topic`; sets explicit timeout/unstamped mode; configures battery/base-state/fault/E-stop watchdog inputs; keeps bumper enforcement false. | Uses declared Jazzy behavior and extends the priority-10 fail-closed policy without commissioning unknown bumper masks. |
| `config/physical_calibration.yaml` | New intentionally incomplete schema for sensor, transform, footprint, LiDAR-height, and near-field gates. | Records unknowns rather than encoding guessed geometry. |
| `launch/bringup.launch.py` | Loads the bridge parameter file while leaving the candidate sensor launch absent. | Applies fail-closed bridge defaults without activating uncommissioned hardware. |
| `launch/low_obstacle_sensor_candidate.launch.py` | New opt-in official `urg_node_driver` plus static-transform launch that rejects missing arguments. | Provides a Jazzy-compatible commissioning path without changing production bringup. |
| `package.xml` | Adds the Jazzy `urg_node` runtime dependency. | Makes the candidate launch dependency explicit. |
| `scripts/patrolbot_safety_watchdog.py` | Subscribes to battery, typed base state, and selected command; enforces faults/E-stop/23 V; implements direction-aware bumper hold; preserves the last requested direction while holding. | Extends the priority-10 safety layer instead of placing checks in an individual behavior. |
| `scripts/safety_policy.py` | New pure fail-closed evaluator. | Makes missing/stale telemetry, power, hardware, and bumper decisions exhaustively testable without ROS or hardware. |
| `scripts/validate_physical_calibration.py` | New schema/commissioning validator. | Makes half-filled or guessed physical configuration fail review and deployment checks. |
| `test/test_physical_calibration.py` | Tests safe-uncommissioned defaults, partial-activation rejection, production-launch absence, and unchanged active geometry. | Prevents scaffolding from becoming active by accident. |
| `test/test_safety_contract.py` | Pins the full command path, historical landmines, smoother params, source provenance, DDS scope, candidate absence, manual-control readiness, and diagnostic byte constants. | Converts audit conclusions into CI-enforced contracts. |
| `test/test_safety_policy.py` | Exhaustive stream-staleness, voltage, fault, E-stop, mapping, direction, rotation, and policy-switch tests. | Proves the watchdog is fail-closed and bumper behavior is direction-aware. |

## 4. Key logic for independent review

### 4.1 Priority-10 hold

The pure policy evaluates all required safety inputs on every watchdog tick:

```text
hold = scan stale
    or odometry stale
    or battery stale/invalid/below 23 V
    or typed base state stale/invalid
    or FAULTFLAGS asserted
    or E-stop asserted
    or commissioned directional bumper conflict
```

While `hold` is nonempty, the watchdog publishes a zero `Twist` on
`/input/safety_controller`, priority 10. When all enforced conditions recover,
it goes silent so the mux timeout releases the hold. It resets the remembered
desired direction to zero on release.

Bumper enforcement is not active in production. Once physically commissioned,
front contact blocks forward/rotation and rear contact blocks reverse/rotation.
A contact does not block deliberate motion away from it.

### 4.2 Battery and fault diagnostics

The bridge does not infer undocumented hardware state. It:

1. masks FAULTFLAGS to 16 bits;
2. emits every asserted bit in ascending order;
3. uses manual-defined names where available;
4. exposes every unknown bit as `reserved_bit_N`;
5. maps the pure helper's numeric level to the generated ROS byte constants.

The motion gate uses `< 23.0 V`, so exactly 23.0 V is still allowed by this
single threshold. The diagnostic layer warns from 23.0 V up to below 24.0 V.
Mission behavior must use the warning to return/stop scheduling work before the
hard motion gate is reached.

### 4.3 Raw near-field candidate

The bridge's production `/scan` behavior and 0.25 m filter are unchanged. When
explicitly enabled for a supervised commissioning session, the candidate
publisher can expose pre-filter data on `/scan_unfiltered_candidate`. No active
costmap, Collision Monitor source, or launch file subscribes to that topic.

This separation matters: the candidate cannot silently change robot safety,
and self-reflections can be measured before deciding whether any raw near-field
data is usable.

### 4.4 Provenance digest

The digest hashes file contents plus canonical repository-relative names. It
supports both repository layout (`ros2_ws/src`) and image layout (`src`) so the
same source produces the same identity in both places. The digest covers:

- `patrolbot_interfaces`;
- `patrolbot_bridge`;
- `patrolbot_dock_manager`;
- `patrolbot_docking_plugins`;
- `patrolbot-launch`;
- `patrolbot_navigation`;
- `hardware/patrolbot_hw_server` source.

The live checker compares the image label with the copied deployed source and
marks readiness false on any mismatch. The current equality is:

```text
8ab058d2f68342c41a7db510cf3d924b3133fb98e3afea8d78e6b55f9e5f885f
```

### 4.5 DDS scope

`LOCALHOST` is a Jazzy `rcl` discovery policy, not a firewall rule. It prevents
automatic discovery beyond loopback while preserving the three host-network
containers' shared loopback graph. Reviewers must not claim it provides DDS
authentication or encryption. Its current purpose is to remove the externally
discoverable unauthenticated command surface while the system has no secure
remote DDS design.

## 5. Test-to-risk mapping

| Risk | Protecting test/evidence |
|---|---|
| Smoother silently uses defaults | `test_teleop_smoother_uses_only_declared_jazzy_parameters`; live readback |
| Wrong manual voltage boundary | `test_battery_thresholds_match_the_patrolbot_manual`; `test_battery_boundary_matches_manual_operating_minimum` |
| Fault bits silently lost | `test_every_documented_fault_flag_has_a_stable_name`; reserved-bit test |
| Invalid ROS diagnostic type | `test_battery_diagnostic_uses_ros_byte_constants`; exact-image probe |
| Bumper guessed active | `test_bumper_enforcement_is_explicitly_uncommissioned_by_default`; live parameter readback |
| Wrong-direction bumper block | `test_bumpers_block_only_motion_toward_contact_or_rotation` |
| Stale safety input treated clear | `test_every_missing_or_stale_required_stream_holds` |
| Candidate sensor changes active safety | near-field candidate contract and physical-calibration tests |
| Historical parameter regression | static tests for scan minimum, RPP setting, base-shift setting, and float cost scaling |
| Remote DDS discovery restored | Compose contract plus two-container loopback test and live inspect |
| Source/image drift | build-time digest check, static contract, live equality |
| Endpoint bypass reintroduced | command-path static tests and live `status.sh` endpoint identities |
| DDS discovery transient creates false status | three-sample empty endpoint logic in `topic_endpoint_nodes` |
| Diagnostic monitor leaks and back-pressures bridge | internal timeout placement and no-monitor process check |

## 6. Deployed artifact and backup identity

| Artifact | Identity |
|---|---|
| Candidate archive | `/tmp/patrolbot-pi5-seven-item-candidate-20260716T221749Z.tar.gz` |
| Candidate archive SHA-256 | `f902a4caf9183ce7787c36766055c44b4a96b3b68f62afb8659a3c422eebfdfd` |
| Production image | `patrolbot:pi5-hardening-20260716t221749z` |
| Image ID | `sha256:80cba4ee514bcf36756540baa446a8532108a27a4f0f2cd25618c00779e09927` |
| Source digest | `8ab058d2f68342c41a7db510cf3d924b3133fb98e3afea8d78e6b55f9e5f885f` |
| Pre-change Pi archive | `0c37aff1c0b7561a859827c3e889f76bb072127ef1b3de9bfdd28297eded6ff5` |
| Post-change Pi archive | `d8f5ae916abda8d0891d9b5b5f2db05c3fe2ac6b8fca17428495c741e4443312` |
| Workstation post-change review archive | `ef85fa0cdba5ce7c6c25ba3f09be2ac37467b88fdc2c2db7e98659d38b9dd15c` |

The pre-hardening rollback image remains present on the Pi:
`patrolbot:pb001-pb002-20260716t203659z`, image ID
`sha256:cfdc6b61abc31746df2590ebe93d3360b8799116975f26bc3d9bedc8eb2a27ef`.

## 7. Reviewer checklist

An independent reviewer should verify, in this order:

1. Run `sha256sum -c` in both backup directories.
2. Compare the files listed in §3 against the pre-change archive, excluding
   generated files.
3. Run `docker/source-tree-digest.sh` and compare it with the image label.
4. Parse every changed YAML file and run the static/unit suite.
5. Build all six packages in the same Jazzy image.
6. Run normal tests plus the isolated fake-SBC, dock-manager, and real Collision
   Monitor tests.
7. Confirm the pinned invariants in §2.
8. Confirm candidate launch/calibration remain uncommissioned and absent from
   normal bringup.
9. Inspect the live graph without publishing: one final `/cmd_vel` publisher,
   no docking producer, and correct upstream mux endpoints.
10. Confirm all three containers use `LOCALHOST`, source/image digests match,
    joystick is reported absent, and SBC-off readiness remains degraded.
11. Confirm no physical or SBC-dependent finding is marked closed by software
    evidence alone.

## 8. What must not be reviewed as a defect

- `base_shift_correction: false` prevents a known reconnect crash.
- RPP `use_collision_detection: false` prevents known doorway freezes; the
  external Collision Monitor remains the active navigation safety layer.
- `cost_scaling_factor: 4.0` must remain a floating-point YAML value.
- Empty-but-fresh rear-sonar clouds are valid clear observations because the SBC
  protocol emits `AUX:SONAR=` every cycle even with zero echoes.
- Bumper enforcement being false is a safety decision until physical masks are
  measured, not an omitted implementation.
- The raw scan candidate being absent is the correct production state.
- The overall SBC-off degraded status is the correct readiness result.

## 9. Remaining implementation boundary

No remaining Pi parameter should be guessed. The next changes require physical
evidence or the SBC:

- joystick hardware and stationary manual-control validation;
- bumper bit masks and BumpStall commissioning;
- URG presence/device/mount/coverage;
- footprint and main-LiDAR height measurements;
- near-field self-occlusion and stopping tests;
- SBC release/enable binary deployment and acknowledgement tests;
- physical E-stop, PWR ON, dock release, reverse, re-docking, and charging tests.

Until those are complete, autonomous operation and the generic docking server
remain disabled.
