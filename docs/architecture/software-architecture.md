---
title: Software Architecture
description: The Pi's ROS 2 graph — nodes, the cmd_vel arbitration chain, the composed Nav2 container, and the engineering decisions behind the large-map handling.
---

# Software Architecture

All of PatrolBot's software autonomy lives on the **Raspberry Pi**. The SBC runs a single
C++ program and no ROS 2 (see [Hardware Architecture](hardware-architecture.md) and
[`patrolbot_hw_server`](../packages/patrolbot_hw_server.md)). This page describes the Pi's
ROS 2 graph: which nodes exist, how data and commands move between them, and *why* the graph
is shaped the way it is.

## The ROS 2 graph

```mermaid
flowchart TB
    SBC(["SBC patrolbot_server\nTCP :7272"])

    subgraph PI["Raspberry Pi — ROS_DOMAIN_ID=0"]
        BRIDGE["patrolbot_bridge"]

        subgraph CONTAINER["nav2_container (single composed process)"]
            MAPSRV["map_server"]
            AMCL["amcl"]
            PLANNER["planner_server (NavFn)"]
            CONTROLLER["controller_server (RPP)"]
            VSMOOTH["velocity_smoother"]
            COLMON["collision_monitor"]
            BT["bt_navigator"]
            BEHAV["behavior_server"]
            WPF["waypoint_follower"]
            LCMGR["lifecycle_manager(s)"]
        end

        subgraph BASE["mobile base (separate container)"]
            TWISTMUX["twist_mux"]
            TELEOPSMOOTH["teleop_velocity_smoother"]
            LCSCRIPT["lifecycle_mgr.py"]
        end

        JOYNODE["joy_node"]
        JOYTELEOP["patrolbot_joy_teleop"]
        LASERTF["laser_static_tf"]
    end

    SBC -->|"ODOM|LASER"| BRIDGE
    BRIDGE -->|"/odom"| AMCL
    BRIDGE -->|"/scan"| AMCL
    BRIDGE -->|"/scan"| CONTROLLER
    BRIDGE -->|"/scan"| COLMON
    BRIDGE -->|"TF odom→base_link"| CONTAINER
    LASERTF -->|"TF base_link→laser_frame"| CONTAINER
    MAPSRV -->|"map (intra-process)"| AMCL
    AMCL -->|"TF map→odom"| CONTAINER

    BT --> CONTROLLER
    CONTROLLER -->|"cmd_vel"| VSMOOTH
    VSMOOTH -->|"cmd_vel_smoothed"| COLMON
    COLMON -->|"input/navi (prio 5)"| TWISTMUX

    JOYNODE -->|"/joy"| JOYTELEOP
    JOYTELEOP -->|"input/joy (prio 8)"| TWISTMUX

    TWISTMUX -->|"cmd_vel_out"| TELEOPSMOOTH
    TELEOPSMOOTH -->|"/cmd_vel"| BRIDGE
    BRIDGE -->|"DRIVE"| SBC
```

The full node-by-node reference is in [ROS 2 → Nodes](../ros2/nodes.md); topic and parameter
tables are in [Topics](../ros2/topics.md) and [Parameters](../ros2/parameters.md).

## Layers of responsibility

The software falls into four layers, each a clean boundary:

1. **Transport layer — [`patrolbot_bridge`](../packages/patrolbot_bridge.md).** The only code that
   knows the SBC exists. Converts the TCP text stream into `/odom`, `/scan`, `/sonar`, `/battery`,
   `/diagnostics`, and the `odom→base_link` transform; converts `/cmd_vel` into `DRIVE` commands.
   Everything above it is pure ROS 2 and hardware-agnostic.
2. **Autonomy layer — Nav2 (`nav2_container`).** Localization (AMCL), global planning (NavFn),
   local control (RPP), behaviors (spin/back-up/wait), and collision gating. All composed into one
   process.
3. **Arbitration layer — mobile base ([`patrolbot-launch`](../packages/patrolbot-launch.md)).** A
   `twist_mux` decides whose velocity command wins (joystick vs. navigation) and a velocity
   smoother shapes the result before it reaches the bridge.
4. **Teleop layer — joystick.** `joy_node` + [`patrolbot_joy_teleop`](../ros2/nodes.md#patrolbot_joy_teleop)
   inject a high-priority manual command that overrides autonomy on demand.

## The `cmd_vel` arbitration chain

This is the most intricate part of the graph and the easiest to get wrong, because two
different velocity smoothers and several `input/*` topics are involved. The chain has a single
arbiter — `twist_mux` — and a single final output topic, `/cmd_vel`, which only the bridge
consumes.

```mermaid
flowchart LR
    RPP["controller_server\n(RPP)"] -->|cmd_vel| VS["velocity_smoother\n(in nav2_container)"]
    VS -->|cmd_vel_smoothed| CM["collision_monitor\nstop-box 0.6×0.6 m"]
    CM -->|input/navi · prio 5| TM{{"twist_mux"}}
    JT["patrolbot_joy_teleop"] -->|input/joy · prio 8| TM
    TM -->|cmd_vel_out| TVS["teleop_velocity_smoother"]
    TVS -->|/cmd_vel| BR["patrolbot_bridge"]
    BR -->|DRIVE| SBC([SBC])
```

Reading it carefully:

- Inside `nav2_container`, RPP publishes `cmd_vel`; the Nav2 `velocity_smoother` republishes
  `cmd_vel_smoothed`; `collision_monitor` reads `cmd_vel_smoothed`, applies its stop-box, and
  emits the result on **`input/navi`** at twist_mux priority **5**.
- `patrolbot_joy_teleop` emits on **`input/joy`** at priority **8** — *higher* than navigation —
  so any joystick motion overrides autonomy. It publishes **only while the sticks are moving**, so
  twist_mux times the joy input out (1 s) on release and navigation resumes.
- `twist_mux` publishes the winner on `cmd_vel_out`. A *second*, separate
  `teleop_velocity_smoother` (the `nav2_velocity_smoother` executable started by the mobile-base
  launch) re-shapes it and republishes **`/cmd_vel`**, which the bridge forwards to the SBC.

!!! note "Two smoothers, similar names"
    There are two velocity smoothers with overlapping topic names. The one *inside* `nav2_container`
    smooths RPP output to `cmd_vel_smoothed`. The one in the mobile-base launch
    (`teleop_velocity_smoother`) remaps `/cmd_vel → /cmd_vel_out` (input) and `cmd_vel_smoothed →
    cmd_vel` (output), so its final output is the real `/cmd_vel`. The remaps are why the names look
    circular. See [Launch System](../ros2/launch-system.md#mobile-base-launch).

!!! warning "Configured but unused mux inputs"
    `mux.yaml` also defines `input/safety_controller` (prio 10), `input/teleop` (prio 8), and
    `input/switch` (prio 6). **No node currently publishes to these** — only `input/joy` and
    `input/navi` are wired. They are reserved priority slots, not active paths.

**Failure mode — `cmd_vel_in_topic` mismatch.** Earlier, `collision_monitor` read `cmd_vel_raw`,
which had no publisher, so navigation commands silently never reached the robot. It now reads
`cmd_vel_smoothed` (the smoother's real output topic). The lesson is baked into a comment in
`nav2_params.yaml`: the smoother's `cmd_vel_topic` parameter is **not** a real Nav2 parameter and
is ignored — the smoother always publishes `cmd_vel_smoothed`.

## The composed `nav2_container` — and why composition is mandatory

Every Nav2 lifecycle node runs in **one** process, `nav2_container`
(`component_container_isolated`), loaded via `LoadComposableNodes`. This is not a preference; it
is forced by the Pi's resource limits.

!!! abstract "Why one container instead of separate node processes?"
    **Intent:** survive on a Pi with `ulimit -n = 1024` and a large map.

    **The problem with separate processes:** running Nav2 with `use_composition:=False` spawns ~13
    DDS participants. FastDDS allocates shared-memory port locks under `/dev/shm` per participant;
    ~13 of them exhaust the file-descriptor budget and break lifecycle discovery, so nodes never
    finish activating.

    **Why composition fixes it:** one container = one DDS participant = one set of port locks. And
    the large map travels `map_server → costmaps` **intra-process** (zero-copy), so it never crosses
    DDS and can't saturate the transport.

    **Tradeoff:** a crash in any node takes down all of them (shared process). PatrolBot accepts this
    and turns it into a clean recovery story — see below.

### Crash handling: tear down, don't respawn

`nav2_container` is deliberately **not** respawned. A respawned container comes back *empty*,
because `LoadComposableNodes` does not re-run. Instead, `bringup.launch.py` registers an
`OnProcessExit` handler: if the container dies for any reason, it emits a launch `Shutdown`. The
`patrolbot-navigation.service` systemd unit has `Restart=always`, so the entire launch restarts
and brings up a fresh, fully-populated stack (localization + map back in ~4 s). This converts
"the container died and is now a useless empty shell" into "the whole stack restarted cleanly."

### Startup ordering inside the container

Localization (`map_server` + `amcl`) loads first so the map and `map→odom` are ready within
seconds. The heavy navigation half (costmaps inflating the large map) is delayed **20 s** with a
`TimerAction` so it does not starve localization during the container's sequential composable-node
loading. Practical result: the RViz map and *2D Pose Estimate* are usable almost immediately;
*Nav2 Goal* becomes available a bit later. The full sequence is on
[Startup Sequence](../internals/startup-sequence.md).

## The large-map problem

The map is the dominant scaling pressure on the Pi, and several decisions exist only to manage it:

| Decision | Reason | Tradeoff |
|---|---|---|
| **Downsample map 0.1 → 0.2 m/px** (4× fewer cells) | Nav2 startup fell from ~8 min to ~2.5 min | Coarser global costmap; local costmap kept at 0.1 m for fine avoidance |
| **`bond_timeout: 0.0`** in the patched lifecycle managers | Inflating the huge map starves `map_server`'s bond heartbeat past the default 4 s, aborting the lifecycle manager → no AMCL → blank map | Loses bond-based liveness detection of lifecycle nodes |
| **`MAGICK_THREAD_LIMIT=1`, `OMP_NUM_THREADS=1`** | Multi-threaded image decode of the large PGM OOM-kills the process on the Pi | Slightly slower one-time map load |
| **Local copies of the nav2_bringup launches** | Upstream `bringup_launch.py` hard-codes `bond_timeout: 4.0` and never reads our params | Must re-sync if upstream changes |

!!! danger "Stale comment in `nav2_params.yaml`"
    The trailing comment block in `nav2_params.yaml` claims the stack avoids bond starvation by
    launching with `use_composition:=False` (separate processes). **This is stale and contradicts the
    actual launch.** The live `bringup.launch.py` uses `use_composition:=True` and applies
    `bond_timeout: 0.0` in the patched launch files. Treat the launch file as authoritative. This is
    tracked in [Known Gaps](../known-gaps.md#contradictions-live-pi-source-vs-written-notes).

## Reliability properties

Because the SBC link is the system's single point of fragility, the Pi software is built to ride
out its loss without operator intervention:

- **SBC down/up:** the bridge reconnects every 3 s; Nav2 nodes stay active (`bond_timeout: 0.0`);
  `map→odom` and 25 Hz data resume automatically when the SBC returns.
- **Bridge crash:** `patrolbot-bridge.service` restarts it; it reconnects.
- **Nav container/node crash:** full launch shutdown → systemd restart; localization + map back in
  ~4 s.
- **Reconnect TF skew:** `collision_monitor` runs with `base_shift_correction: False`, so a scan
  stamped just before the rebuilt TF cache no longer throws an uncaught extrapolation exception that
  would SIGABRT the whole container.

The one case software cannot mask is a **physical SBC reboot**, which resets ARIA odometry to
`0,0,0`; AMCL's pose is then wrong and the operator must re-set it with *2D Pose Estimate*. See
[Known Gaps](../known-gaps.md) and [Debugging](../development/debugging.md).
