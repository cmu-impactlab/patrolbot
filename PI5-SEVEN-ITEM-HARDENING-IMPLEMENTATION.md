# Raspberry Pi 5 Seven-Item Hardening Implementation Record

## 1. Outcome

The seven safe Raspberry Pi 5 tasks identified in
`PATROLBOT_RPI5_AUDIT.md` §1A were implemented, built against ROS 2 Jazzy,
deployed to `patrolbot-rpi5`, and revalidated while the legacy SBC was offline.

The production Pi now runs:

| Item | Deployed value |
|---|---|
| Image | `patrolbot:pi5-hardening-20260716t221749z` |
| Image ID | `sha256:80cba4ee514bcf36756540baa446a8532108a27a4f0f2cd25618c00779e09927` |
| Revision | `pi5-hardening-20260716T221749Z` |
| Source digest | `8ab058d2f68342c41a7db510cf3d924b3133fb98e3afea8d78e6b55f9e5f885f` |
| Containers | `patrolbot-bringup`, `patrolbot-bridge`, and `patrolbot-navigation`: healthy, zero restarts |
| Live readiness | `OVERALL=degraded reason=sbc-unavailable` |

The degraded result is intentional and safe. The SBC was not contacted or
changed, so `/odom`, `/scan`, rear sonar, battery, and typed hardware state are
unavailable. The priority-10 watchdog therefore holds zero velocity. The
joystick receiver is also absent. Autonomous operation remains prohibited.

No robot motion, docking/undocking action, charge-release request, motor-enable
request, µARCS command, direct hardware-server connection, or port 7272 probe
was issued in this work.

## 2. Scope and source hierarchy

The implementation followed this order of authority:

1. `PATROLBOT_RPI5_AUDIT.md`, including its post-audit §1A disposition.
2. `PB-AUDIT-001-002-REMEDIATION-PLAN.md` and its binding §26 amendments.
3. `PB-AUDIT-001-002-IMPLEMENTATION-REPORT.md` for the already deployed
   PB-AUDIT-001/002 Pi architecture.
4. The PatrolBot manuals for 24 V recharge, 23 V minimum operating voltage,
   fault-bit meaning, charging, motor, and E-stop behavior.
5. Current deployed Pi files, image labels, ROS endpoints, parameters, and
   container state.
6. Official ROS 2 Jazzy, Nav2 Jazzy, Collision Monitor, velocity smoother, and
   `urg_node` documentation for version-sensitive behavior.

The work deliberately preserved these incident-verified constraints:

- `SCAN_RANGE_MIN = 0.25`;
- RPP `use_collision_detection: false`;
- Collision Monitor `base_shift_correction: false`;
- both inflation-layer `cost_scaling_factor` values as floating-point `4.0`;
- the active costmap footprint;
- RPP `allow_reversing: false`;
- `docking_server` absent from the active stack;
- bumper enforcement disabled until physical mapping;
- the raw near-field scan candidate disabled and disconnected from safety.

## 3. Backup record

### 3.1 Pre-change backups

Backup ID: `20260716T210902Z-pi5-seven-item-hardening`

| Copy | Path | SHA-256 | Verification |
|---|---|---|---|
| Workstation critical source | `backups/20260716T210902Z-pi5-seven-item-hardening/workstation-prechange-critical.tar.gz` | `3dcd55bb05e671dc888dabb481a5212586a491fa6cdf081ea45dc1799b8ee297` | Archive listing and `sha256sum -c` passed |
| Pi pre-change deployment | `/home/ubuntu/patrolbot-backups/20260716T210902Z-pi5-seven-item-hardening/pi-deployment-prechange.tar.gz` | `0c37aff1c0b7561a859827c3e889f76bb072127ef1b3de9bfdd28297eded6ff5` | Verified on Pi |
| Off-host Pi copy | `backups/20260716T210902Z-pi5-seven-item-hardening/pi5/pi-deployment-prechange.tar.gz` | `0c37aff1c0b7561a859827c3e889f76bb072127ef1b3de9bfdd28297eded6ff5` | Byte-identical checksum |

The file named `workstation-prechange.tar.gz.incomplete` is retained only as a
record of a failed full-tree compression attempt. It is excluded from
`SHA256SUMS` and must never be used for restoration.

### 3.2 Post-change backups

Backup ID: `20260716T223238Z-pi5-seven-item-hardening-postchange`

| Copy | Path | SHA-256 | Verification |
|---|---|---|---|
| Pi post-change deployment | `/home/ubuntu/patrolbot-backups/20260716T223238Z-pi5-seven-item-hardening-postchange/pi-deployment-postchange.tar.gz` | `d8f5ae916abda8d0891d9b5b5f2db05c3fe2ac6b8fca17428495c741e4443312` | All 13 manifest entries passed `sha256sum -c` on the Pi |
| Off-host Pi copy | `backups/20260716T223238Z-pi5-seven-item-hardening-postchange/pi5/pi-deployment-postchange.tar.gz` | `d8f5ae916abda8d0891d9b5b5f2db05c3fe2ac6b8fca17428495c741e4443312` | All 13 entries passed independently after copying |
| Workstation post-change source and reports | `backups/20260716T223238Z-pi5-seven-item-hardening-postchange/workstation-postchange-review.tar.gz` | `ef85fa0cdba5ce7c6c25ba3f09be2ac37467b88fdc2c2db7e98659d38b9dd15c` | Checksum and archive listing passed |

The post-change set also contains container/image inspect output, Compose
resolution, process state, deployed-tree metadata, ROS node/topic snapshots,
command endpoint evidence, and live bridge/watchdog/smoother parameter dumps.
The protected environment and resolved Compose files are mode `0600`; their
contents must not be pasted into review documents.

## 4. Item-by-item implementation

### 4.1 PB-AUDIT-014: valid Jazzy final smoother configuration

The legacy Kobuki-style keys in `smoother.yaml` were replaced with declared
`nav2_velocity_smoother` Jazzy parameters:

- 20 Hz open-loop smoothing;
- maximum velocity `[0.35, 0.0, 0.7]`;
- minimum velocity `[-0.35, 0.0, -0.7]`;
- maximum acceleration `[1.0, 0.0, 1.5]`;
- maximum deceleration `[-1.4, 0.0, -2.0]`;
- 0.5 s command timeout;
- unstamped `Twist`, matching the deployed command path.

The final smoother remains the only publisher of `/cmd_vel`. Live parameter
readback returned `Double values are: [0.35, 0.0, 0.7]`.

Status: **closed on the Pi**. Manual response testing is still blocked by the
absent joystick receiver and belongs to PB-AUDIT-015.

### 4.2 PB-AUDIT-012: named faults and battery motion policy

The bridge now decodes the documented 16-bit FAULTFLAGS into stable diagnostic
names, including laser/PDB, temperature, low-battery, gyro, encoder, critical
drive, and safe-mode conditions. Unknown asserted bits remain visible as
`reserved_bit_N` rather than being dropped.

Battery diagnostics now distinguish:

- below 23.0 V: error, below the operating minimum;
- 23.0 V to below 24.0 V: warning, recharge required;
- at least 24.0 V: normal or charging according to typed charge state.

The priority-10 watchdog now requires fresh `/battery` and typed
`/patrolbot/base_state`, and holds zero velocity for:

- stale scan, odometry, battery, or base state;
- voltage below 23.0 V or invalid voltage;
- invalid typed hardware state;
- any FAULTFLAGS value when fault stopping is enabled;
- asserted E-stop.

The implementation does not claim to replace the missing legacy SBC
RI/genpowerd shutdown chain. That remains a hardware/SBC task.

During exact-image testing, ROS type support exposed that
`DiagnosticStatus.level` is a `byte`; assigning a plain integer triggered a
generated-message assertion. The bridge now maps policy results explicitly to
`DiagnosticStatus.OK`, `WARN`, and `ERROR`, and a static contract prevents the
unsafe regression.

Status: **Pi portion complete; SBC graceful shutdown open**.

### 4.3 PB-AUDIT-006: general bumper safety hold

The bridge publishes typed bumper state only when the base state is fresh and
both nonzero physical masks have been deliberately configured. The priority-10
policy is direction-aware:

- a front bump blocks positive linear motion and rotation;
- a rear bump blocks negative linear motion and rotation;
- invalid mapping blocks motion if enforcement has been explicitly enabled.

The selected `/cmd_vel_out` command is observed before the final smoother so a
bumper hold evaluates the direction that was requested. Once holding, the last
hazardous direction is latched until the contact releases rather than being
overwritten by the watchdog's own zero command.

Production defaults remain:

```yaml
bumpers_mapped: false
front_bumper_mask: 0
rear_bumper_mask: 0
bumper_enforcement_enabled: false
```

This is deliberate. The constant audit-time raw bumper bit is not treated as a
mapping, and no segment or BumpStall behavior was guessed.

Status: **software implemented and tested; physical commissioning open**.

### 4.4 PB-AUDIT-015: manual-control availability

The Compose device mount remains read-only and the readiness script now checks
both `/dev/input/js0` and `/joy` freshness. The receiver was not present on the
Pi, so no joystick test was possible.

Status: **open and hardware-blocked**. When the receiver is available, the
stationary test must prove `/joy` freshness, deadman release, command timeout,
mux priority, and zero final output before any movement test.

### 4.5 PB-AUDIT-017: DDS/network containment

All three host-network containers now set the Jazzy discovery policy:

```text
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

The containers share the host network namespace and therefore continue to
discover one another over loopback. A separate two-container test in isolated
ROS domain 117 proved that loopback discovery still reports one publisher.
Live container inspection confirmed `LOCALHOST` on bringup, bridge, and
navigation. SSH management remained available throughout deployment.

The operational tradeoff is explicit: remote LAN/Wi-Fi DDS and remote RViz
discovery no longer work. They must not be restored by changing discovery back
to `SUBNET`; a future remote workflow needs a separately reviewed secure
transport or coordinated SROS2 design.

Status: **external DDS command-surface exposure contained**.

### 4.6 PB-AUDIT-019/020: cleanup and provenance

High-confidence Pi cleanup removed:

- the empty `mobile_base_nodelet_manager` component container;
- the ignored `cmd_vel_topic` parameter;
- the obsolete, unreferenced `patrolbot-launch/launch/readlidar.py` path.

Package metadata, installed launch files, and lifecycle handling were adjusted
to match the active architecture. Legacy rollback packages and all SBC
artifacts were deliberately preserved.

`docker/source-tree-digest.sh` now deterministically hashes every file used from
the six built PatrolBot ROS packages plus the hardware-server protocol source.
Generated files, backups, build products, and binaries are excluded. The build:

1. receives the expected digest;
2. recomputes it inside the build context;
3. fails on mismatch;
4. stores it in the image and OCI label.

`docker/status.sh` recomputes the digest from the deployed copied source and
requires exact equality with the image label. Live verification returned the
same `8ab058...e5f885f` digest for both.

Status: **high-confidence Pi cleanup complete; runtime drift detection closed**.
The timestamp revision should be replaced by a reviewed commit/release identity
after the current working tree is committed.

### 4.7 PB-AUDIT-005/008/018/021: no-guess physical scaffolding

The image now contains official Jazzy `urg_node`. An opt-in candidate launch can
start `urg_node_driver` and a static transform only when all required device,
frame, and measured transform arguments are supplied. It is not included in
normal bringup.

`physical_calibration.yaml` records the still-unknown values without inventing
them:

- low-obstacle sensor presence, stable device, frame, and six-DOF transform;
- measured footprint polygon;
- measured main-LiDAR height;
- near-field self-occlusion, stopping-distance, and Collision Monitor approval.

The validator accepts the safely empty uncommissioned record and rejects
`--require-commissioned`. A separately named raw near-field candidate topic is
implemented in the bridge but defaults disabled. It is not advertised live and
is not wired to Collision Monitor or costmaps.

Status: **Pi scaffolding complete; every physical finding remains open until
measured and tested**.

## 5. Additional reliability correction

The bridge now owns an explicit stop event, makes reconnect waits interruptible,
and joins its receive/publish workers before ROS node destruction. This prevents
background threads from publishing through a destroyed rclpy context during
container shutdown or controlled redeployment. Production socket addresses and
timeouts remain unchanged.

## 6. Deployment procedure performed

1. Created and verified the pre-change workstation and Pi archives.
2. Built the immutable candidate from a timestamped source archive.
3. Ran normal and isolated tests against the exact candidate image.
4. Copied the validated source to `/home/ubuntu/patrolbot-repo`.
5. Updated the protected Pi environment to select the new image, revision, and
   source digest without displaying secret values.
6. Recreated only the three Pi PatrolBot containers. Docker itself was not
   restarted and the SBC was not accessed.
7. Removed one obsolete deployed `readlidar.py` file after a read-only tree
   comparison proved it was the only leftover preventing source-digest equality.
8. Revalidated container health, source provenance, DDS scope, command endpoint
   ownership, required interfaces, and fail-closed readiness.
9. Created the post-change Pi backup and copied it off-host.

## 7. Verification evidence

### 7.1 Exact-image results

- All six selected ROS packages built successfully against Jazzy.
- Normal `colcon test`: **130 tests, 0 errors, 0 failures, 17 intentional
  skips**.
- Static/unit subset: **27 passed**.
- Fake SBC with real bridge: **3 passed**.
- Fake telemetry/release/BackUp with real dock manager: **2 passed**.
- Real Jazzy Collision Monitor synthetic integration: **10 passed**.
- Candidate calibration validates as safely uncommissioned.
- `--require-commissioned` fails as required.
- Two-container Jazzy `LOCALHOST` discovery test: publisher count **1**.
- `docker compose config --quiet`, Python compilation, Bash syntax, YAML
  parsing, and `git diff --check` passed.

### 7.2 Live stationary results

- Three production containers healthy with zero restarts.
- Image and deployed source digest match exactly.
- `docking_server` absent.
- `/input/docking` has no publisher and one `nav_command_mux` subscriber.
- `nav_command_mux` is the only publisher of `/cmd_vel_nav`.
- `teleop_velocity_smoother` is the only publisher of `/cmd_vel`.
- `patrolbot_bridge` is the only `/cmd_vel` subscriber.
- Smoother cap read back as `[0.35, 0.0, 0.7]`.
- Raw scan candidate absent/uncommissioned.
- Bumper enforcement read back disabled/uncommissioned.
- Required release, enable, pose-init, and undock interfaces are advertised but
  were not called.
- No unattended `ros2 topic hz` monitor remained.
- Final readiness is degraded because the SBC and joystick receiver are absent.

## 8. Rollback

Rollback target already present on the Pi:

| Item | Value |
|---|---|
| Image | `patrolbot:pb001-pb002-20260716t203659z` |
| Image ID | `sha256:cfdc6b61abc31746df2590ebe93d3360b8799116975f26bc3d9bedc8eb2a27ef` |
| Revision | `pb001-pb002-20260716T203659Z` |

Rollback is authorized only in a stationary maintenance window with the robot
command path quiet. Do not touch the SBC or probe port 7272.

1. Verify the pre-change Pi archive with `sha256sum -c SHA256SUMS`.
2. Extract it into a temporary review directory, never directly over production.
3. Compare the archived `docker/` and `ros2_ws/src/` with production.
4. Restore only the seven-item changed files with `cp -a` or `rsync -a`.
5. Restore the protected pre-change `docker/.env` from the same archive.
6. Run `docker compose config --quiet`.
7. Select `patrolbot:pb001-pb002-20260716t203659z` and recreate only the three
   Pi PatrolBot containers.
8. Run `docker/status.sh` and exact endpoint checks.
9. Stop if the source/image state is inconsistent, a command endpoint has an
   unexpected owner, or any nonzero velocity is observed.

The post-change backup provides a forward-restore point if rollback itself must
be reversed.

## 9. Remaining work and prohibition on autonomous operation

The seven Pi tasks do not close PB-AUDIT-002. The remaining gates are:

1. Attach and stationary-validate the joystick receiver.
2. Restore the SBC under operator supervision without opening a second hardware
   connection.
3. Validate live `/odom`, `/scan`, `/sonar`, `/battery`, and typed base state.
4. Map all physical bumper bits and verify firmware BumpStall before enabling
   the general bumper hold.
5. Identify and measure the URG, footprint, main-LiDAR height, and near-field
   self-occlusion/stopping behavior before commissioning any scaffold.
6. Deploy and validate the SBC's idempotent `CHARGE_RELEASE` and explicit
   `MOTOR_ENABLE` implementation with zero motion.
7. Resolve dock-related E-stop/PWR ON behavior physically.
8. Survey and configure the dock pose, then validate AMCL convergence.
9. Perform the two-person short reverse, full departure, re-docking, charging,
   cancellation, sensor-loss, link-loss, and no-old-velocity tests.

Until every relevant gate passes, keep `docking_server` disabled,
`dock_pose_configured=false`, bumper enforcement false, physical calibration
uncommissioned, and autonomous operation suspended.

## 10. External software references

- Nav2 Jazzy velocity smoother configuration:
  <https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html>
- ROS 2 Jazzy automatic discovery range:
  <https://docs.ros.org/en/jazzy/p/rcl/generated/function_discovery__options_8h_1ab32988da7cf0f37b4103121d2f9f74a2.html>
- Nav2 Collision Monitor configuration:
  <https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html>
- ROS 2 Jazzy `urg_node` 1.1.2 documentation:
  <https://docs.ros.org/en/jazzy/p/urg_node/__README.html>
