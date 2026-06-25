---
title: FAQ
description: Frequently asked questions about PatrolBot — the two-machine split, why RViz shows nothing, why a change didn't take effect, the laser orientation, and more.
---

# FAQ

Short answers with links to the full story.

## Architecture

**Why are there two computers?**
: The Pioneer base only speaks the legacy ARIA C++ library; modern Nav2 runs on ROS 2 Jazzy.
  PatrolBot keeps ARIA + hardware on the **SBC** and the whole ROS 2 stack on the **Pi**, joined by
  one TCP socket. See [Overview → Design philosophy](architecture/overview.md#design-philosophy).

**Does the SBC run ROS 2?**
: No. It runs one C++ ARIA program (`patrolbot_server`) and streams text over TCP. The Pi is the
  only ROS 2 machine. See [Communication Architecture](architecture/communication-architecture.md).

**Can I `ros2 topic echo` something from the SBC?**
: No — there are no SBC topics. The SBC's data only becomes ROS topics after the bridge republishes
  it on the Pi (`/odom`, `/scan`, `/sonar`, `/battery`, `/diagnostics`).

**Why TCP text instead of a ROS 2 bridge over DDS?**
: Debuggability (`nc 172.20.87.231 7272` prints readable lines), no schema/versioning burden, and
  the SBC needs no DDS at all. See
  [Communication → why a TCP text protocol](architecture/communication-architecture.md#why-a-tcp-text-protocol-and-not-a-ros-2-bridge).

## "It's not working"

**RViz shows nothing / "Frame map does not exist."**
: On the LAN: set **Fixed Frame = `map`**, and make sure `/scan` is flowing and you've set a *2D
  Pose Estimate*. From home over a VPN: it's a **transport** problem — multicast discovery doesn't
  cross the VPN. See [Remote Operation](deployment/remote-operation.md) and
  [Debugging](development/debugging.md#frame-map-does-not-exist--blank-map-in-rviz).

**I changed `patrolbot-launch` and nothing happened.**
: The mobile base runs from `~/build_backup/patrolbot-launch/`, not `src`. Update the `build_backup`
  copy. See [Updates](deployment/updates.md#the-mobile-base-deployment-step).

**The robot localizes but won't drive to a goal.**
: Walk the `cmd_vel` chain. Most often the `teleop_velocity_smoother` isn't `active` (the
  `lifecycle_mgr.py` step), or a goal was sent before navigation finished activating (~2.5 min). See
  [Debugging](development/debugging.md#robot-wont-move-under-navigation-but-localization-is-fine).

**The laser scan looks mirrored.**
: Known, unresolved. The live launch applies `roll=π` to un-mirror; older notes say `yaw=π`. Needs a
  visual RViz check. See [Known Gaps](known-gaps.md#laser-transform-orientation).

**`/odom` and `/scan` stopped.**
: The SBC link dropped. The bridge reconnects every 3 s. If the SBC was physically rebooted,
  re-set the pose with *2D Pose Estimate* (odometry reset to 0,0,0). See
  [Debugging](development/debugging.md).

**Nav2 restarted itself.**
: Expected if `nav2_container` died — the launch tears down and systemd restarts a fresh stack (a
  respawn would come back empty). See
  [Software Architecture](architecture/software-architecture.md#crash-handling-tear-down-dont-respawn).

## Operations

**How do I see what's going on?**
: `ssh ubuntu@patrolbot-ros.qatar.cmu.edu ./patrolbot-logs.sh status` (health), `... topics` (rates), `... scan` (laser),
  `... nav` (Nav2 logs). See [Debugging](development/debugging.md).

**How long until the robot is ready after boot?**
: Map + pose estimate in a few seconds; full navigation (goals) in ~2.5 min. See
  [Startup Sequence](internals/startup-sequence.md).

**Does the robot keep running if I lose my RViz/SSH connection?**
: Yes — autonomy continues; a running goal isn't interrupted. The joystick is the local override.

**Why is the robot capped at 0.26 m/s?**
: An indoor-patrol safety choice (DWB `max_vel_x`), not a hardware limit. Raising it means re-tuning
  accel limits and re-checking the `base_shift_correction: False` assumption. See
  [Actuators](devices/actuators.md#scalability--tuning-notes).

**Why is the battery percentage `NaN`?**
: This base has no real state-of-charge sensor; only `voltage` is meaningful. See
  [Sensors → Battery](devices/sensors.md#battery).

## Development

**Why are there two velocity smoothers?**
: One inside `nav2_container` (smooths DWB output to `cmd_vel_smoothed`) and one in the mobile base
  (`teleop_velocity_smoother`, the final stage before the bridge). The overlapping topic names come
  from remaps. See
  [Software Architecture → cmd_vel chain](architecture/software-architecture.md#the-cmd_vel-arbitration-chain).

**Why can't I just run Nav2 as separate processes?**
: The Pi's `ulimit -n = 1024` makes ~13 DDS participants exhaust shared-memory port locks. One
  composed container is mandatory. See
  [Software Architecture](architecture/software-architecture.md#the-composed-nav2_container-and-why-composition-is-mandatory).

**Is `rosaria2` used?**
: No — it's the legacy direct-ARIA driver, kept as a fallback. Don't run it alongside the bridge.
  See [rosaria2](packages/rosaria2.md).

**What's unverified in this documentation?**
: Everything SBC-side (the SBC wasn't reachable), plus a handful of doc/source mismatches. See
  [Known Gaps](known-gaps.md).
