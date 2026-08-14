---
title: Limitations
description: Constraints of the deployed PatrolBot system — what it refuses to do, what it cannot sense, and what stops working when a machine goes away.
---

# Limitations

This page records constraints of the system **as deployed**. Each entry describes behavior an
operator or developer will actually observe. Nothing here is a defect report or a work item.

The key words **MUST**, **MUST NOT**, and **SHOULD** are to be interpreted as described in
[RFC 2119](https://www.rfc-editor.org/rfc/rfc2119).

## The SBC is a single point of failure for all base data

The SBC owns the Pioneer base and the SICK laser and is not a ROS node. Every piece of base
data reaches ROS through one TCP socket, so when the SBC is unreachable the following go
silent — no error, no stale-data fallback:

| Topic | Type | Source |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | SBC `ODOM` lines |
| `/scan` | `sensor_msgs/msg/LaserScan` | SBC `LASER` lines |
| `/sonar`, `/battery` | `sensor_msgs/msg/Range`, `sensor_msgs/msg/BatteryState` | SBC `AUX` lines |
| `/patrolbot/base_state`, `/patrolbot/dock_readiness` | PatrolBot interfaces | SBC state reporting |

The `odom` → `base_link` transform stops with them, so Nav2 cannot localize or plan. Nodes
stay up and the container stays healthy; only the data stops. A silent `/odom` therefore means
"check the link and the SBC", not "check Nav2".

## Undocking refuses without a commissioned hardware profile

Contact separation is an exclusive SBC-owned operation. An undock request is refused with
`HARDWARE_PROFILE_UNCOMMISSIONED` unless `undock_profile.conf` supplies a measured hardware
profile. The profile is site-specific — it encodes charge-release timing, E-stop signal
semantics, rear-sonar indices and near-field floor, and the idle fault and stall flags behind
`critical_fault_mask` — and is deliberately **not** shipped with the image. A robot moved to a
new dock starts refusing again until its profile is remeasured.

This refusal is the designed behavior, not a failure state.

## Generic Nav2 docking is not wired

The Nav2 `docking_server` is absent from the running graph. Its direct velocity path writes
straight to the base and would bypass the priority mux and the safety chain, so PatrolBot uses
its own guarded undock action instead.

`/input/docking` exists as an input to `nav_command_mux` at priority 9 and has **zero
publishers**. Publishing to it would outrank normal path following, though the command would
still pass through the collision monitor. It **MUST** remain publisher-free unless the
publisher has been reviewed against the [safety chain](../safety/safety-chain.md).

## Planning sees one horizontal plane; the rear is sonar-only

Two different sensor sets protect the robot, and they do not cover the same things.

| Protection | Source | Covers |
|---|---|---|
| Costmaps (global and local) | `/scan` only | Path planning and the inflation layer |
| `front_velocity_stop` polygon | `/scan`, `min_points: 8` | Forward and rotating motion |
| `rear_velocity_stop` polygon | `/sonar`, `min_points: 1` | Reverse motion |

Consequences:

- **Nav2 never plans around anything sonar sees.** Sonar is a collision-monitor source only, not
  a costmap observation source. It can stop the robot; it cannot shape a path.
- **Anything outside the laser plane is invisible to planning.** Low kerbs, cables, feet,
  castors, and open drawers pass under it; table edges, shelf lips, and wall fixtures overhang
  it. The costmaps will happily plan straight through all of them.
- **Reverse protection is four sparse transducers.** `min_points: 1` means one credible echo
  stops reverse motion — deliberately twitchy, because the alternative is not stopping. Sonar
  cannot resolve narrow or sound-absorbing obstacles, so a clear rear polygon is not proof the
  path behind is clear.

PatrolBot **MUST NOT** be operated near low obstacles without a human watching.

## E-stop recovery is physical

There is one hardware E-stop. It cuts motor power directly; software cannot override it, and
software cannot clear it. Recovery requires a physical `PWR ON` action at the robot — no
service call, topic, or dashboard control restores motor power. Motors additionally require an
explicit enable after recovery; the robot does not silently return to a drivable state.

## The SBC platform is end-of-life

The SBC runs Ubuntu 16.04 on i686. It receives no security updates and cannot be brought to a
supported release in place. Access is key-only with no root login, which limits exposure but
does not change the platform's support status.

Two consequences are visible in normal operation:

- **The clock is wrong.** The SBC has no working RTC battery, so its clock does not survive a
  cold boot. Timestamps in SBC logs **MUST NOT** be compared against Pi timestamps without
  accounting for the offset.
- **There is no graceful shutdown on power loss.** The serial-to-TCP topology does not expose
  the base's power-fail signal to Linux, so the SBC cannot know it is losing power. Cutting
  power is always an abrupt stop.

## `rosaria2` and the bridge are mutually exclusive

The legacy `rosaria2` package publishes the same transforms and consumes the same velocity
topics as `patrolbot_bridge`. Running both produces conflicting TF and duplicated velocity
commands. Exactly one **MUST** be running; the deployed configuration uses the bridge.

## Raw LAN DDS is not access-controlled

The current Compose policy sets `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` for every
runtime container. RViz and ROS CLI on the multicast-capable LAN can discover
the graph, but DDS does not distinguish observation from control: command
topics, services, actions, and `/initialpose` are exposed too.

The one `x-patrolbot-config.ros_discovery_range` scalar in
`docker/docker-compose.yml` switches every service between `SUBNET` and
`LOCALHOST`. Set it to `LOCALHOST` whenever raw LAN discovery is unnecessary.

<!-- SPDX-License-Identifier: Apache-2.0 -->
