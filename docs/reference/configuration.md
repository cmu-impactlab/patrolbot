---
title: Configuration
description: Where PatrolBot's tunables live, their deployed values, and for the load-bearing ones the specific failure each prevents.
---

# Configuration

This page lists PatrolBot's configuration files and the values deployed on the robot. Some of
those values look like removable defaults and are not; those are marked and carry the failure
they prevent.

The key words **MUST**, **MUST NOT**, and **SHOULD NOT** are to be interpreted as described in
[RFC 2119](https://www.rfc-editor.org/rfc/rfc2119).

## Where configuration lives

All paths are inside the container, under `/opt/patrolbot_ws/`.

| File | Package | Governs |
|---|---|---|
| `config/nav2_params.yaml` | `patrolbot_navigation` | AMCL, costmaps, controller, collision monitor, docking |
| `config/physical_calibration.yaml` | `patrolbot_navigation` | Measured robot geometry |
| `scripts/twist_mux.yaml` | `patrolbot_navigation` | Mux inputs and priorities |
| `param/defaults/mux.yaml` | `patrolbot-launch` | Launch-time mux defaults |
| `param/defaults/smoother.yaml` | `patrolbot-launch` | Final velocity smoother |
| `config/bridge_params.yaml` | `patrolbot_bridge` | Bumper masks, candidate scan topic |

The `install/` tree is what actually runs; `src/` is the source of the build. Editing `src/`
without rebuilding changes nothing on a running robot.

## Settings that MUST NOT change

Each of these prevents a specific, reproduced failure. Changing one to "clean up a default"
reintroduces the failure it was added for.

### `base_shift_correction: False` — collision monitor

With `True`, the collision monitor looks up `laser_frame` → `odom` at each scan's exact
timestamp. After a bridge or SBC reconnect there is a brief TF gap, so a scan can arrive
stamped earlier than the rebuilt TF cache, raising `tf2::BackwardExtrapolationException`. The
collision monitor does not wrap this in a message filter, so the uncaught throw **SIGABRTs the
entire composed `nav2_container`** — every Nav2 node dies together.

`False` uses the latest transform instead, costing about 1 cm of motion per scan at this
robot's top speed.

### `use_collision_detection: false` — controller (RPP)

With `true`, Regulated Pure Pursuit checks its lookahead arc against the local costmap using
the **inscribed** radius. In a doorway the inflation band covers the whole opening, so the
check never clears and the robot freezes mid-doorway with no error. The collision monitor
already provides the protective stop this would duplicate.

### `source_timeout: 0.5` — collision monitor

**MUST NOT** be zero for either source. Both automatic-motion sources are fail-closed. At
~20–25 Hz for `/scan` and ~5 Hz for `/sonar`, 0.5 s tolerates one missed sonar period but stops
after roughly two to three missed samples.

### `transform_tolerance: 1.0` — costmaps and collision monitor

When the SBC is down or reconnecting, the bridge is not publishing odometry, so the costmap has
no `base_link` → `odom` transform. With the stock ~3 s activation timeout, costmap activation
**failed**, the lifecycle manager aborted bringup, and the container stayed alive but *empty* —
so nothing restarted it and Nav2 never recovered even after the SBC returned.

### `always_send_full_costmap: True` — local costmap only

**MUST NOT** be set on the global costmap. The static layer forces the global costmap to full
updates regardless, and setting it there produces large redundant messages.

### `cost_scaling_factor` — inflation layer

Deployed at `4.0`. The value **MUST** be written as a YAML float (`4.0`, never `4`). An integer
literal is rejected as the wrong parameter type and the node fails to configure.

### `bond_timeout: 0.0` — lifecycle managers

Disables bond-based liveness supervision. Under load the bond heartbeat starves and the
lifecycle manager tears down healthy nodes.

## Nav2 parameters

### Localization (AMCL)

| Parameter | Value | Note |
|---|---|---|
| `transform_tolerance` | 1.0 s | |
| `max_position_variance` | 0.25 | Gates general navigation |
| `dock_pose_configured` | `true` | Dock pose has been surveyed |

AMCL **MUST NOT** claim the map origin after a restart. A valid pose comes from the surveyed
dock pose or from a deliberate operator action — never from a guess.

While docked, the dock structure fills the laser's near field, the match score drops, and AMCL's
variance does not converge. A narrow docked-only allowance applies **only** while the SBC's dock
observer reports mated plates *and* the pose already agrees with the surveyed dock pose.
`max_position_variance` **MUST NOT** be raised to solve a docked problem — it still gates
general navigation.

### Controller — Regulated Pure Pursuit

| Parameter | Value |
|---|---|
| `desired_linear_vel` | 0.22 m/s (base maximum is 0.26 m/s) |
| `lookahead_dist` | 0.45 m |
| `transform_tolerance` | 0.5 s |
| `use_rotate_to_heading` | `true` |
| `use_collision_detection` | `false` (see above) |

RPP replaced an untuned DWB controller. DWB scored the near-stationary trajectory best —
commanding about 0.05 rad/s, below the base deadband — so the robot did not move and reported
"Failed to make progress". RPP rotates in place to face the path first.

### Costmaps

| Parameter | Local | Global |
|---|---|---|
| `update_frequency` | 5.0 Hz | 1.0 Hz |
| `resolution` | 0.1 m | 0.2 m |
| `transform_tolerance` | 1.0 s | 1.0 s |
| `cost_scaling_factor` | 4.0 | 4.0 |
| `inflation_radius` | 0.40 m | 0.40 m |
| `expected_update_rate` (scan) | 0.5 s | 0.5 s |

The local costmap runs at 5 Hz to match the controller. At 1 Hz the controller outran costmap
updates and goals aborted with "Costmap timed out waiting for update".

Both costmaps observe `/scan` only. Sonar is **not** a costmap source — see
[Limitations](limitations.md).

### Footprint

Both costmaps use the same measured octagon, in metres:

```text
[[0.255, 0.138], [0.197, 0.213], [-0.197, 0.213], [-0.255, 0.138],
 [-0.255, -0.138], [-0.197, -0.213], [0.197, -0.213], [0.255, -0.138]]
```

This replaced an undersized circular radius. Footprint and TF values **MUST NOT** be changed
without a physical measurement of the envelope and the laser height.

## Mux priorities

There are two muxes. `nav_command_mux` arbitrates within Nav2 and publishes `/cmd_vel_nav`;
`cmd_vel_mux` arbitrates between Nav2, manual control, and safety, and publishes
`/cmd_vel_out`.

`nav_command_mux` — all timeouts 0.25 s:

| Input | Priority |
|---|---|
| `input/docking` | 9 |
| `input/nav_behavior` | 7 |
| `input/nav_controller` | 5 |

`cmd_vel_mux`:

| Input | Priority | Timeout |
|---|---|---|
| `input/safety_controller` | 10 | 0.2 s |
| `input/joy` | 8 | 0.5 s |
| `input/teleop` | 7 | 1.0 s |
| `input/switch` | 6 | 1.0 s |
| `input/navi` | 5 | 1.0 s |

See [Safety Chain](../safety/safety-chain.md) for what these priorities mean in practice.

!!! warning "`scripts/twist_mux.yaml` is not the running configuration"
    `patrolbot_navigation/scripts/twist_mux.yaml` declares a `twist_mux` node with priorities
    100 and 50 on `/cmd_vel_joy` and `/cmd_vel_nav`. No such node runs, and neither topic is a
    mux input. It matches neither deployed mux and **MUST NOT** be used as a reference for
    current priorities.

## Velocity smoothers

There are two, with different limits and different scopes.

| | `velocity_smoother` (Nav2 only) | `teleop_velocity_smoother` (everything) |
|---|---|---|
| `max_velocity` | `[0.26, 0.0, 1.0]` | `[0.30, 0.0, 0.7]` |
| `min_velocity` | `[-0.26, 0.0, -1.0]` | `[-0.30, 0.0, -0.7]` |
| `max_accel` | `[2.5, 0.0, 3.2]` | `[1.0, 0.0, 1.5]` |
| `max_decel` | `[-2.5, 0.0, -3.2]` | `[-1.4, 0.0, -2.0]` |
| `velocity_timeout` | — | 0.5 s |
| `smoothing_frequency` | — | 20.0 Hz |

The teleop smoother's cap matches the SBC's `classifyDrive` ceiling deliberately. A larger
upstream cap makes the SBC alternate between motion and a forced zero near full-stick input, so
it **MUST** stay aligned with the joystick's own limits. Its linear acceleration stays below the
base manual's 1.4 m/s² firmware ceiling.

## Bridge parameters

| Parameter | Value | Meaning |
|---|---|---|
| `server_ip` | `10.0.0.1` | SBC address |
| `server_port` | `7272` | SBC listening port |
| `connect_timeout` | 5.0 s | |
| `receive_timeout` | 3.0 s | |
| `reconnect_delay` | 3.0 s | |
| `control_timeout` | 6.0 s | Hardware control requests |
| `bumpers_mapped` | `true` | Bumper bit mapping is established |
| `front_bumper_mask` | `32256` (`0x7e00`) | SIP stall bits 9–14 |
| `rear_bumper_mask` | `126` (`0x007e`) | SIP stall bits 1–6 |
| `publish_unfiltered_scan_candidate` | `false` | Commissioning topic only |

The dock normally holds the two centre-front bumper panels (`0x1800`) active, so a docked robot
reports front bumper contact as a matter of course.

!!! warning "Mapped is not enforced"
    `bumpers_mapped: true` records that the bit mapping is known. It does **not** mean bumper
    contact stops the robot — that is `bumper_enforcement_enabled` on the safety watchdog,
    which is `False`. See [Safety Chain](../safety/safety-chain.md).

`publish_unfiltered_scan_candidate` **MUST** remain `false` unless physical self-occlusion
testing has explicitly approved the unfiltered scan as a safety source.

<!-- SPDX-License-Identifier: Apache-2.0 -->
