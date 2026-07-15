---
title: Glossary
description: ROS 2, Nav2, and robotics terms a new PatrolBot contributor might not know — plus the project-specific terms (SBC, the bridge, Pi 5 Docker runtime, the AUX line).
---

# Glossary

Terms used throughout this documentation. Project-specific terms are marked **(PatrolBot)**.

## Project-specific

**SBC** **(PatrolBot)**
: The robot's "main PC". Owns the hardware (base + laser) and runs the ARIA
  `patrolbot_server` on the dedicated Pi link at `10.0.0.1:7272`. **Runs no ROS 2.**
  It may also be reachable for SSH on the lab LAN as `172.20.87.231`.

**Pi** **(PatrolBot)**
: The Raspberry Pi running the entire ROS 2 Jazzy navigation stack. The main
  driver is the Pi 5 (`robot-pi2`, hostname `patrolbot-rpi5`); the powered Pi 4
  path is retained as an isolated rollback deployment. Home is `/home/ubuntu`.

**The bridge** **(PatrolBot)**
: `patrolbot_bridge` / `bridge_node` — the Pi node that translates the SBC's TCP text stream into
  ROS 2 topics + TF and forwards `/cmd_vel` back as `DRIVE` commands.

**SBC-Pi TCP link** **(PatrolBot)**
: The single TCP socket (SBC `:7272`) where the two machines meet. They share no ROS graph.

**`AUX` line** **(PatrolBot)**
: The lower-rate (~5 Hz) telemetry line carrying sonar, battery, and base flags — separate from and
  independent of the `ODOM|LASER` navigation line.

**Docker deployment** **(PatrolBot)**
: The main Raspberry Pi 5 runtime under `docker/`. It builds one image and runs
  the three active services with Docker Compose.

**Self-occlusion filter** **(PatrolBot)**
: The bridge rule that forces laser returns < 0.25 m to `+inf`, preventing the laser grazing the
  robot's own body from painting a phantom obstacle inside the footprint.

## ROS 2

**ROS 2 / Jazzy**
: Robot Operating System 2; "Jazzy Jalisco" is the distribution PatrolBot's Pi runs.

**Node**
: A process (or composed component) that does one job and communicates over the ROS graph.

**Topic**
: A named, typed, many-to-many message stream (e.g. `/scan`). Publishers write; subscribers read.

**Service**
: A synchronous request/response call (e.g. `change_state`).

**Action**
: A long-running, cancelable, goal-oriented call with feedback (e.g. `navigate_to_pose`).

**Parameter**
: A named, typed configuration value on a node (set from YAML or the command line).

**TF / TF2**
: The transform system tracking coordinate frames over time (`map → odom → base_link →
  laser_frame`).

**Frame**
: A coordinate system. `map` (fixed world), `odom` (smooth but drifting), `base_link` (robot
  body), `laser_frame` (the scanner).

**DDS / FastDDS**
: The middleware ROS 2 uses for discovery and transport. FastDDS is the default implementation here.

**`ROS_DOMAIN_ID`**
: An integer that isolates ROS 2 graphs on a network; PatrolBot uses `0`.

**Lifecycle (managed) node**
: A node with explicit states (`Unconfigured → Inactive → Active → Finalized`) driven by a lifecycle
  manager. Most Nav2 nodes are lifecycle nodes.

**Composition / composable node / component container**
: Running multiple nodes in one process to save resources and enable intra-process (zero-copy)
  message passing. PatrolBot composes all Nav2 nodes into `nav2_container`.

**Bond / `bond_timeout`**
: A heartbeat between a lifecycle manager and its nodes. PatrolBot sets `bond_timeout: 0.0` so slow
  large-map operations don't trip the watchdog.

**colcon**
: The ROS 2 build tool for a `src/ build/ install/ log/` workspace.

**ament**
: The ROS 2 build system / package format (`ament_python`, `ament_cmake`) and its linters.

## Nav2 and navigation

**Nav2**
: The ROS 2 Navigation stack — localization, planning, control, recovery.

**AMCL**
: Adaptive Monte-Carlo Localization — particle-filter localization against a known map using the
  laser scan; publishes `map → odom`.

**Costmap (global / local)**
: A grid of traversal costs. The global costmap covers the whole map; the local costmap is a small
  rolling window for immediate obstacle avoidance.

**DWB**
: Dynamic Window Approach (B-variant) — the Nav2 local controller PatrolBot previously used.
  Replaced by RPP on 2026-06-29.

**RPP (Regulated Pure Pursuit)**
: The Nav2 local controller PatrolBot currently uses
  (`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`). Follows a lookahead
  point on the global plan; `desired_linear_vel: 0.22 m/s` (capped to 0.26 m/s hardware max).

**NavFn**
: The global planner PatrolBot uses (`GridBased`) — computes a path over the global costmap.

**collision_monitor**
: A Nav2 safety node that gates velocity using polygon zones (PatrolBot's 0.6×0.6 m "stop box").

**Velocity smoother**
: A node that limits acceleration/jerk on velocity commands. PatrolBot has two (one in Nav2, one in
  the mobile base).

**twist_mux**
: A multiplexer that selects among multiple velocity sources by priority (joystick > navigation).

**Behavior tree (`bt_navigator`)**
: The XML-defined logic that orchestrates planning, control, and recovery for a navigation goal.

**Occupancy grid / map**
: A 2-D grid marking free / occupied / unknown cells, with a resolution in meters per pixel (m/px).

## Hardware

**Pioneer PatrolBot-SH**
: The differential-drive mobile base. Modeled in Nav2 as an octagonal footprint derived from the
  ARIA dimensions and 0.29 m swing radius.

**ARIA / AriaCoda**
: The C++ library for controlling Pioneer/MobileRobots bases. Runs only on the SBC.

**SICK LMS-200**
: The planar laser scanner; on PatrolBot it gives a 180° forward scan, mounted flipped.

**Sonar ring**
: The robot physically has 4 rear sonar sensors. The ARIA parameter file defines a generic
  16-position ring, but the 12 unpopulated positions are filtered out; `/sonar` publishes only real
  detections.

**Differential drive**
: A two-driven-wheel base that steers by varying wheel speeds (linear `x` + angular `z` only).

**socat**
: A relay utility; here it bridges the base's serial port `/dev/ttyS0` to `TCP:7000` on the SBC.

**Deadman / interlock**
: A button that must be held for motion (PatrolBot's joystick uses RB) — a safety device.

**Linger (`loginctl enable-linger`)**
: systemd setting that lets a user's services run at boot without an interactive login.
