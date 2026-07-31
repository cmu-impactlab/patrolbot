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
: Readable protocol lines, no schema/versioning burden, and the SBC needs no DDS at all. Port 7272
  is single-client, so never connect `nc` while the Pi bridge is active. See
  [Communication → why a TCP text protocol](architecture/communication-architecture.md#why-a-tcp-text-protocol-and-not-a-ros-2-bridge).

## "It's not working"

**RViz shows nothing / "Frame map does not exist."**
: Raw DDS is intentionally loopback-only, so laptop RViz sees nothing on both LAN and VPN. For a
  local visualization path, verify `/scan` and provide a pose only through an authorized
  interface. See [Remote Operation](deployment/remote-operation.md) and
  [Debugging](development/debugging.md#frame-map-does-not-exist--blank-map-in-rviz).

**I changed `patrolbot-launch` and nothing happened.**
: On the main Pi 5, restart `patrolbot-bringup` after editing an existing
  bind-mounted launch/param/script file. Rebuild the image when adding files or
  changing package metadata.
  The `~/build_backup/` path referenced in older notes was removed — the service now uses
  `ros2 launch patrolbot-launch bringup.xml`. See [Updates](deployment/updates.md).

**The robot localizes but won't drive to a goal.**
: Walk the `cmd_vel` chain. Most often the `teleop_velocity_smoother` isn't `active` (the
  `lifecycle_mgr.py` step), or a goal was sent before navigation finished activating. See
  [Debugging](development/debugging.md#robot-wont-move-under-navigation-but-localization-is-fine).

**The laser scan looks mirrored.**
: The live static TF applies `roll=π` at `x=0.037, z=0.2` to un-mirror the flipped SICK scan.
  Confirmed from live TF data (quaternion `x≈1, w≈0`). If the scan still looks wrong,
  re-check the `LaserFlipped` param in `patrolbot-sh.p` or confirm walls align in RViz.

**`/odom` and `/scan` stopped.**
: The SBC link dropped. The bridge reconnects every 3 s. If the SBC was physically rebooted,
  keep localization invalid until an operator provides a fresh pose (odometry reset to 0,0,0). See
  [Debugging](development/debugging.md).

**Nav2 restarted itself.**
: Expected if `nav2_container` died — the launch tears down and Docker restarts
  a fresh Pi 5 service container (a component-container respawn would come back empty). See
  [Software Architecture](architecture/software-architecture.md#crash-handling-tear-down-dont-respawn).

## Operations

**How do I see what's going on?**
: `ssh robot-pi2 'cd /home/ubuntu/patrolbot-repo && ./docker/status.sh'`
  checks readiness; `docker logs patrolbot-navigation` and `docker logs
  patrolbot-bridge` show the main runtime. See [Debugging](development/debugging.md).

**How long until the robot is ready after boot?**
: The map/localization services load first; navigation services follow after staged activation.
  Localization does not become valid until an operator supplies a pose. See
  [Startup Sequence](internals/startup-sequence.md).

**Does the robot keep running if I lose my dashboard/SSH connection?**
: A running goal is not canceled merely because the observer disconnects. The robot's onboard
  safety chain remains authoritative; normal operation is still prohibited until the physical
  E-stop/joystick/sensor commissioning gates pass.

**Why is the robot capped at 0.26 m/s?**
: An indoor-patrol safety choice in the Nav2 smoother; RPP's desired cruise speed
  is 0.22 m/s. It is not a hardware limit. Raising it means
  re-tuning the lookahead distance and accel limits and re-checking the
  `base_shift_correction: False` assumption. See
  [Actuators](devices/actuators.md#scalability--tuning-notes).

**Why is the battery percentage `NaN`?**
: This base has no real state-of-charge sensor; only `voltage` is meaningful. See
  [Sensors → Battery](devices/sensors.md#battery).

## Development

**Why are there two velocity smoothers?**
: One inside `nav2_container` (smooths RPP output to `cmd_vel_smoothed`) and one in the mobile base
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

**What can't the robot do?**
: It senses obstacles in a single laser plane, so low and overhanging objects are invisible to
  it, and Nav2 never plans around anything sonar sees. Rear protection is four sparse sonar
  transducers. Undocking refuses without a measured hardware profile, and E-stop recovery is
  physical. See [Limitations](reference/limitations.md).
