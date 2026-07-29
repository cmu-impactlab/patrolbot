---
title: ROS 2 Parameters
description: The parameters that actually matter on PatrolBot — Nav2 (AMCL, RPP, costmaps, collision monitor), the velocity smoothers, twist_mux priorities, joystick teleop, and the bridge constants.
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
| `server_ip` | `10.0.0.1` | SBC address on the dedicated Ethernet link |
| `server_port` | `7272` | SBC telemetry server port |
| `RECV_TIMEOUT` | `3.0 s` | 20 Hz stream → 3 s silence ⇒ dead link ⇒ reconnect (vs. a blocking `recv()` hanging forever) |
| `SCAN_RANGE_MIN` | `0.25 m` | footprint-clearance filter; do not raise much higher because real close obstacles can be 0.25-0.40 m away |

## Joystick teleop (`patrolbot_joy_teleop.py`)

Declared ROS parameters (override with `--ros-args -p`):

| Parameter | Default | Meaning |
|---|---|---|
| `max_linear` | `0.35` | m/s at full stick |
| `max_angular` | `0.7` | rad/s at full stick |
| `min_linear` / `max_linear_cap` | `0.05` / `1.5` | limits for A/Y speed trim |
| `min_angular` / `max_angular_cap` | `0.1` / `3.0` | limits for X/B speed trim |
| `linear_step` / `angular_step` | `0.05` / `0.1` | per-button speed adjustment |
| `deadzone` | `0.12` | stick deadzone |
| `accel_linear` / `accel_angular` | `0.7` / `2.0` | output ramp in m/s² and rad/s² |
| `rate` | `30.0` | output timer frequency in Hz |

The axis and button map is implemented as constants, not ROS parameters: RB button
5 is the deadman; left-stick axes 1/0 drive and turn; D-pad axes 7/6 are fallbacks;
and `JOY_TIMEOUT=0.4` s drops the deadman on input loss.

## Nav2 — `config/nav2_params.yaml`

### AMCL (localization)

| Parameter | Value | Note |
|---|---|---|
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | matches the differential base |
| `min_particles` / `max_particles` | `500` / `1500` | particle budget on the Pi |
| `laser_model_type` | `likelihood_field` | |
| `transform_tolerance` | `1.0` | generous, for TF timing on a loaded Pi |
| `set_initial_pose` | `false` | never claim map origin after a restart |
| `always_reset_initial_pose` | `false` | requires an operator pose or commissioned dock-pose initializer |
| `scan_topic` | `scan` | from the bridge |

### Controller — RPP (`controller_server` / `FollowPath`)

| Parameter | Value | Note |
|---|---|---|
| `controller_frequency` | `5.0` Hz | the control loop rate (costmaps must keep up — see below) |
| `plugin` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | DWB was replaced 2026-06-29 |
| `desired_linear_vel` | `0.22` m/s | cruise speed, below the 0.26 m/s cap |
| `lookahead_dist` | `0.45` m | fixed lookahead; min/max `0.25` / `0.7` |
| `use_rotate_to_heading` | `true` | rotates in place before driving when path heading differs |
| `rotate_to_heading_angular_vel` | `0.9` rad/s | above the base's rotational deadband |
| `use_collision_detection` | `false` | keep false; collision guard is the `collision_monitor` stop box |

### Costmaps

| Parameter | local_costmap | global_costmap | Note |
|---|---|---|---|
| `update_frequency` | `5.0` | `1.0` | **local raised to 5.0** to match the controller; a 1 Hz local caused "Costmap timed out" goal aborts |
| `publish_frequency` | `2.0` | `1.0` | |
| `resolution` | `0.1` | `0.2` | active static map is 0.075 m/px; global planning costmap is intentionally coarser |
| `footprint` | octagon | octagon | derived from 510 mm length, 425 mm width, 0.29 m swing radius |
| `initial_transform_timeout` | `1000000.0` | `1000000.0` | parks activation while SBC/TF is absent instead of permanently aborting |
| `rolling_window` | `true` (3×3 m) | — | |
| `inflation_radius` | `0.40` | `0.40` | `cost_scaling_factor: 4.0` |
| layers | obstacle + inflation | static + obstacle + inflation | both fed by `/scan` |

### Collision monitor

| Parameter | Value | Note |
|---|---|---|
| `cmd_vel_in_topic` | `cmd_vel_smoothed` | the smoother's real output (the `cmd_vel_topic` param of the smoother is ignored) |
| `cmd_vel_out_topic` | `input/navi` | feeds the final twist_mux at priority 5 |
| `base_shift_correction` | **`False`** | **must stay False** — `True` throws an uncaught extrapolation exception on reconnect that SIGABRTs the whole composed container |
| `front_velocity_stop` | direction-selected scan polygon | full 0.6×0.6 m while stopped/forward; rear-only envelope while reversing; `min_points: 8` |
| `rear_velocity_stop` | direction-selected sonar polygon | rear envelope while stopped/reversing; `min_points: 1` for four sparse transducers |
| source timeouts | `0.5 s` for `/scan` and `/sonar` | either stale automatic-motion source fails closed |

### Nav2 internal velocity smoother

| Parameter | Value |
|---|---|
| `smoothing_frequency` | `20.0` Hz |
| `max_velocity` | `[0.26, 0.0, 1.0]` |
| `feedback` | `OPEN_LOOP` |
| (publishes `cmd_vel_smoothed`) | |

### Docking and hardware-readiness gates

The generic `docking_server` parameter block is retained only for a future reviewed integration;
the node is not launched or lifecycle-managed. Its configured PatrolBot plugin uses typed base
state rather than inferring charge from `BatteryState.current`.

`patrolbot_dock_manager` orchestrates the undock but performs no contact separation itself; the
values it sends are a *request* to the SBC, which rejects anything outside its own hard caps
rather than clamping it.

| Parameter | Value | Meaning |
|---|---|---|
| `dock_pose_configured` | `true` | surveyed main-charger pose is commissioned |
| `dock_id` | `main_charger` | expected identifier |
| `sensor_timeout` | `0.5 s` | base state / odom / localization must be fresh |
| `undock_clearance_distance` | `0.53 m` | reverse distance requested of the SBC |
| `undock_reverse_speed` | `0.10 m/s` | **hardware ceiling, not a tuning knob** — at 0.20 m/s the base re-latched charge and cut the motors 0.24 m in (2026-07-28) |
| `undock_max_duration` | `45.0 s` | backstop only; the SBC owns the real operation timeout |
| `undock_turn_yaw` | `π` | post-clearance turn, executed as a Nav2 `Spin` |
| `post_undock_localization_timeout` | `5.0 s` | require a valid AMCL sample received after the hardware result |
| `handoff_timeout` | `4.0 s` | bounded wait for independent Pi telemetry to confirm the SBC terminal handoff |
| `spin_timeout` | `20.0 s` | upper bound for that turn |

Retired with the timed-authorization design: `dock_departure_charge_grace`,
`operator_motor_window`, `backup_timeout`, `allow_front_bumper_at_dock`. Their jobs moved inside
the SBC's exclusive `UNDOCK` state machine, which is configured by
`hardware/patrolbot_hw_server/undock_profile.conf` rather than by ROS parameters — the values are
hardware facts, not deployment tuning, and must not be changeable from the Pi.

The `nav_command_mux` separately arbitrates autonomous producers before smoothing and collision
monitoring: controller priority 5, behavior priority 7, and reserved docking priority 9, each with
a 0.25 s timeout. `/input/docking` intentionally has no publisher.

## twist_mux (`mux.yaml`)

| Input | Topic | Priority | Timeout | Active? |
|---|---|---|---|---|
| safety_controller | `input/safety_controller` | 10 | 0.2 s | **yes** (`patrolbot_safety_watchdog.py`) |
| teleop | `input/teleop` | 8 | 1.0 s | no publisher |
| **joy** | `input/joy` | 8 | 0.5 s | **yes** (`p3dxJoyTeleop`) |
| switch | `input/switch` | 6 | 1.0 s | no publisher |
| **navigation** | `input/navi` | 5 | 1.0 s | **yes** (`collision_monitor`) |

Higher priority wins; the joystick (8) outranks navigation (5). RB cancels the
active Nav2 goal, and the 0.5 s timeout releases the manual mux input only after
the zero ramp and cancellation confirmation. Navigation never resumes that
canceled goal automatically.

## Final velocity smoother (`smoother.yaml`)

| Parameter | Value |
|---|---|
| `max_velocity` | `[0.35, 0.0, 0.7]` |
| `min_velocity` | `[-0.35, 0.0, -0.7]` |
| `max_accel` | `[1.0, 0.0, 1.5]` |
| `max_decel` | `[-1.4, 0.0, -2.0]` |
| `smoothing_frequency` | `20.0` Hz |
| `velocity_timeout` | `0.5 s` |

These are the declared Jazzy `nav2_velocity_smoother` keys. The retired Kobuki-style
`speed_lim_*`, `accel_lim_*`, `frequency`, and `decel_factor` keys are not valid here.

## Launch-time environment variables (`bringup.launch.py`)

| Variable | Value | Why |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | fixed domain |
| `MAGICK_THREAD_LIMIT` | `1` | prevent OOM during large-map image decode |
| `OMP_NUM_THREADS` | `1` | same |
| `RCUTILS_LOGGING_USE_STDOUT` | `1` | logs to stdout for journald |
