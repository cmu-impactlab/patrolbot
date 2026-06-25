---
title: Profiling
description: Performance characteristics of PatrolBot on a resource-constrained Pi — startup time, the large-map cost, DDS/file-descriptor limits, and how to measure them.
---

# Profiling

PatrolBot runs on a Raspberry Pi, and most of its non-obvious design decisions exist to fit inside
that machine's limits. This page collects the performance characteristics worth knowing and how to
measure them.

## The numbers that matter

| Metric | Value | Notes |
|---|---|---|
| Localization ready | a few seconds | `map_server` + `amcl` load first |
| Full navigation active | **~2.5 min** | was ~8 min before the map downsample |
| Navigation start delay | 20 s | `TimerAction` keeps costmaps from starving localization |
| Telemetry rate | ~20 Hz (`/odom`, `/scan`) | from the SBC |
| Bridge TF rate | 50 Hz | `odom→base_link` |
| DWB control loop | 5 Hz | local costmap must match |
| Velocity smoothers | 20 Hz | |
| File-descriptor limit | `ulimit -n = 1024` | forces single-container composition |

## The three Pi constraints that shaped the design

### 1. Memory — the large map OOMs

Loading the occupancy map's PGM with multi-threaded image decode OOM-kills the process. Mitigations:

- `MAGICK_THREAD_LIMIT=1`, `OMP_NUM_THREADS=1` (single-threaded decode).
- The **map downsample** from 0.1 → 0.2 m/px (4× fewer cells) is the biggest lever — it cut
  startup from ~8 min to ~2.5 min.

Measure:

```bash
ssh ubuntu@patrolbot-ros.qatar.cmu.edu 'free -h; cat /proc/$(pgrep -f nav2_container)/status | grep VmRSS'
```

### 2. File descriptors — DDS port exhaustion

With `ulimit -n = 1024`, running Nav2 as ~13 separate processes exhausts FastDDS shared-memory port
locks under `/dev/shm` and breaks lifecycle discovery. One composed `nav2_container` = one DDS
participant = one set of locks. This is **why composition is mandatory** (see
[Software Architecture](../architecture/software-architecture.md#the-composed-nav2_container-and-why-composition-is-mandatory)).

Measure:

```bash
ssh ubuntu@patrolbot-ros.qatar.cmu.edu 'ulimit -n; ls /dev/shm | wc -l; ls -l /proc/$(pgrep -f nav2_container)/fd | wc -l'
```

### 3. CPU — startup contention

The 20 s navigation delay exists because costmap inflation of the large map competes with
localization during the container's sequential composable-node loading. Staging lets `map→odom`
lock in fast, then loads the heavy half.

Measure:

```bash
ssh ubuntu@patrolbot-ros.qatar.cmu.edu 'top -b -n1 | head -20'      # during startup, watch the container process
```

## Measuring runtime performance

```bash
# Topic rates (quick health)
ssh ubuntu@patrolbot-ros.qatar.cmu.edu ./patrolbot-logs.sh topics
ros2 topic hz /scan /odom /cmd_vel

# End-to-end latency sanity: command issued vs. /cmd_vel published
ros2 topic delay /cmd_vel       # if your distro's tooling supports it

# Per-node CPU
ros2 run rqt_top rqt_top         # or plain top, filtered to ROS processes
```

## Tuning levers (and their costs)

| Lever | Effect | Cost |
|---|---|---|
| Map resolution (0.2 → 0.1) | finer global costmap | ~8 min startup, OOM risk — reverting needs the `.bak` files |
| `controller_frequency` | smoother control | more CPU; local costmap must keep pace |
| `max_particles` (AMCL) | better localization | more CPU per scan |
| `max_vel_x` (DWB) | faster robot | re-tune accel limits + re-check `base_shift_correction` assumption |
| Composition off | easier per-node debugging | **breaks** on this Pi (FD exhaustion) — don't |

## What is **not** profiled

- The SBC side. ARIA loop timing and the server's send path are not measured in this snapshot, and
  the SBC isn't remotely reachable here — see [Known Gaps](../known-gaps.md).
- Network throughput of the TCP stream. At ~20 Hz of short text lines it is negligible on a LAN;
  it would matter only over a constrained link.

For correctness debugging rather than performance, see [Debugging](debugging.md).
