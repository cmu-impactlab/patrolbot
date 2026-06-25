---
title: ROS 2 Parameters
description: The parameters that actually matter on PatrolBot — Nav2 (AMCL, DWB, costmaps, collision monitor), the velocity smoothers, twist_mux priorities, joystick teleop, and the bridge constants.
---

# ROS 2 Parameters

This page covers the parameters you are most likely to tune, grouped by file. It is not an
exhaustive dump of every Nav2 default — it focuses on the values that are **specific to this
robot** and *why* they are set the way they are. The authoritative files are
`config/nav2_params.yaml`, `param/defaults/mux.yaml`, `param/defaults/smoother.yaml`, and the
constants in `bridge_node.py`.

## Bridge constants (`bridge_node.py`)

The bridge declares no ROS parameters; these are module constants you would edit in code.

| Constant | Value | Why |
|---|---|---|
| `server_ip` | `172.20.87.231` | SBC LAN address |
| `server_port` | `7272` | SBC telemetry server port |
| `RECV_TIMEOUT` | `3.0 s` | 20 Hz stream → 3 s silence ⇒ dead link ⇒ reconnect (vs. a blocking `recv()` hanging forever) |
| `SCAN_RANGE_MIN` | `0.2 m` | self-occlusion filter; just under the 0.22 m robot radius |

## Joystick teleop (`patrolbot_joy_teleop.py`)

Declared ROS parameters (override with `--ros-args -p`):

| Parameter | Default | Meaning |
|---|---|---|
| `max_linear` | `0.4` | m/s at full stick |
| `max_angular` | `0.8` | rad/s at full stick |
| `deadzone` | `0.12` | stick deadzone |
| `axis_linear` | `1` | left-stick Y |
| `axis_angular` | `3` | right-stick X |
| `deadman_button` | `5` | RB; `-1` disables the interlock |

## Nav2 — `config/nav2_params.yaml`

### AMCL (localization)

| Parameter | Value | Note |
|---|---|---|
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | matches the differential base |
| `min_particles` / `max_particles` | `500` / `1500` | particle budget on the Pi |
| `laser_model_type` | `likelihood_field` | |
| `transform_tolerance` | `1.0` | generous, for TF timing on a loaded Pi |
| `set_initial_pose` | `true` | starts at `initial_pose [0,0,0,0]` |
| `always_reset_initial_pose` | `true` | re-seeds on (re)activation |
| `scan_topic` | `scan` | from the bridge |

### Controller — DWB (`controller_server` / `FollowPath`)

| Parameter | Value | Note |
|---|---|---|
| `controller_frequency` | `5.0` Hz | the control loop rate (costmaps must keep up — see below) |
| `max_vel_x` / `max_vel_trans` | `0.26` m/s | indoor patrol speed cap |
| `max_vel_theta` | `1.0` rad/s | |
| `acc_lim_x` / `decel_lim_x` | `2.5` / `-2.5` | |
| `vx_samples` / `vtheta_samples` | `20` / `20` | trajectory sampling |
| `sim_time` | `1.7` s | trajectory rollout horizon |
| critics | RotateToGoal, Oscillation, ObstacleFootprint, GoalAlign, PathAlign, PathDist, GoalDist | path/goal alignment weighted heavily |

### Costmaps

| Parameter | local_costmap | global_costmap | Note |
|---|---|---|---|
| `update_frequency` | `5.0` | `1.0` | **local raised to 5.0** to match DWB; a 1 Hz local caused "Costmap timed out" goal aborts |
| `publish_frequency` | `2.0` | `1.0` | |
| `resolution` | `0.1` | `0.2` | global matches the downsampled map; local kept fine for close-range avoidance |
| `robot_radius` | `0.22` | `0.22` | |
| `rolling_window` | `true` (3×3 m) | — | |
| `inflation_radius` | `0.55` | `0.55` | `cost_scaling_factor: 3.0` |
| layers | obstacle + inflation | static + obstacle + inflation | both fed by `/scan` |

### Collision monitor

| Parameter | Value | Note |
|---|---|---|
| `cmd_vel_in_topic` | `cmd_vel_smoothed` | the smoother's real output (the `cmd_vel_topic` param of the smoother is ignored) |
| `cmd_vel_out_topic` | `input/navi` | feeds twist_mux at priority 5 |
| `base_shift_correction` | **`False`** | **must stay False** — `True` throws an uncaught extrapolation exception on reconnect that SIGABRTs the whole composed container |
| `stop_box` | `0.6×0.6 m` polygon | `action_type: stop`, `min_points: 4` |
| `source_timeout` | `2.0` | |

### Nav2 internal velocity smoother

| Parameter | Value |
|---|---|
| `smoothing_frequency` | `20.0` Hz |
| `max_velocity` | `[0.26, 0.0, 1.0]` |
| `feedback` | `OPEN_LOOP` |
| (publishes `cmd_vel_smoothed`) | |

### Docking (optional)

`docking_server` is configured with `opennav_docking::SimpleChargingDock` (`use_battery_status:
true`, `staging_x_offset: -0.7`). It is wired but niche — patrol operation does not require it.

!!! danger "Stale trailing comment in `nav2_params.yaml`"
    The comment at the bottom of the file says bond starvation is avoided by launching with
    `use_composition:=False`. The live launch uses `use_composition:=True` and sets
    `bond_timeout: 0.0` in the patched lifecycle managers instead. Trust the launch file. See
    [Known Gaps](../known-gaps.md#contradictions-live-pi-source-vs-written-notes).

## twist_mux (`mux.yaml`)

| Input | Topic | Priority | Timeout | Active? |
|---|---|---|---|---|
| safety_controller | `input/safety_controller` | 10 | 0.2 s | no publisher |
| teleop | `input/teleop` | 8 | 1.0 s | no publisher |
| **joy** | `input/joy` | 8 | 1.0 s | **yes** (`p3dxJoyTeleop`) |
| switch | `input/switch` | 6 | 1.0 s | no publisher |
| **navigation** | `input/navi` | 5 | 1.0 s | **yes** (`collision_monitor`) |

Higher priority wins; the joystick (8) outranks navigation (5). The 1.0 s joy timeout is what lets
navigation resume after the operator releases the sticks.

## teleop velocity smoother (`smoother.yaml`)

| Parameter | Value |
|---|---|
| `speed_lim_v` / `speed_lim_w` | `1.0` / `5.4` |
| `accel_lim_v` / `accel_lim_w` | `2.0` / `1.5` |
| `frequency` | `20.0` Hz |
| `decel_factor` | `2.5` |

## Launch-time environment variables (`bringup.launch.py`)

| Variable | Value | Why |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | fixed domain |
| `MAGICK_THREAD_LIMIT` | `1` | prevent OOM during large-map image decode |
| `OMP_NUM_THREADS` | `1` | same |
| `RCUTILS_LOGGING_USE_STDOUT` | `1` | logs to stdout for journald |
