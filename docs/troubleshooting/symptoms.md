---
title: Troubleshooting
description: Symptom-first diagnosis — what the operator observes, what actually causes it, and what to do about it.
---

# Troubleshooting

Organised by what you observe, not by which component is at fault. For tool-oriented
investigation techniques, see [Debugging](../development/debugging.md).

Before anything else, run the readiness report:

```bash
cd ~/patrolbot-repo && ./docker/status.sh
```

## The robot does not move

### Motors are disabled

The most common cause. Motors do not enable themselves, and a `DRIVE` command never enables
them. Check `/diagnostics` for the motors-enabled flag, then call `/patrolbot/motor_enable`.

### The E-stop is latched

Software cannot see past it and cannot clear it. Recovery requires a physical `PWR ON` at the
robot, followed by an explicit motor enable. A latched E-stop has been mistaken for a software
fault more than once — check it early.

### The safety watchdog is holding a stop

The watchdog claims mux priority 10 and nothing else can drive while it does. It trips on
stale `/scan` (0.5 s), stale `/odom` (0.5 s), stale base state (0.75 s), stale `/battery`
(1.0 s), battery at or below 23.0 V, an asserted E-stop, or a base fault.

Because the SBC supplies almost all of those, **losing the SBC link trips several at once**.
That is the intended response, not a bug.

### The robot is docked

Charging contact inhibits motion. Leaving the dock is the undock operation's job — see
[Running the Robot](../operations/running-the-robot.md).

## The robot freezes mid-doorway

Check that `use_collision_detection` is still `false` on the controller. With `true`, RPP
checks its lookahead arc using the *inscribed* radius; in a doorway the inflation band covers
the whole opening, so the check never clears and the robot stops with no error message.

## Every Nav2 node died at once

The Nav2 stack is composed into a single container process, so one uncaught exception kills all
of them together.

The classic trigger is `base_shift_correction` set to `True` on the collision monitor. After a
bridge or SBC reconnect there is a brief TF gap; a scan arrives stamped earlier than the
rebuilt TF cache, and the resulting `tf2::BackwardExtrapolationException` is not caught. It
**MUST** stay `False`.

## Nav2 came back empty after the SBC returned

Containers were up, but no Nav2 nodes were in the graph.

Costmap activation needs the `base_link` → `odom` transform. With the SBC down the bridge
publishes no odometry, so with a short activation timeout the costmap fails to activate, the
lifecycle manager aborts bringup, and the container stays alive but empty — so nothing restarts
it. `transform_tolerance: 1.0` widens that window. If it happens anyway, restart the navigation
container once the bridge is publishing.

## Goals abort with "Failed to make progress"

Historically caused by an untuned DWB controller scoring a near-stationary trajectory best —
commanding about 0.05 rad/s, below the base deadband, so the robot never moved. The deployed
controller is RPP, which rotates in place to face the path first. If this reappears, confirm
the controller plugin is actually RPP.

## Goals abort with "Costmap timed out waiting for update"

The local costmap is updating more slowly than the controller runs. Deployed values are 5 Hz
for the local costmap against a 5 Hz controller; if the costmap has been lowered to 1 Hz, this
is the result.

## Goals are rejected immediately

Either the Nav2 lifecycle nodes are not active, or the robot has no valid pose. A silent
`/odom` means the SBC link is down, not that Nav2 is broken.

## `/odom`, `/scan`, `/sonar` and `/battery` all went silent together

The SBC link is down. All base data crosses one TCP socket at `10.0.0.1:7272`, so it fails as a
unit. Nodes stay up and containers stay healthy — only the data stops.

Check the link, then the SBC. The bridge reconnects on its own every 3.0 s.

!!! warning "Do not open a second client"
    Only one bridge client is expected on port 7272. Opening another to "check the protocol"
    disturbs the session the robot is using.

## The robot drifts or jumps while undocking

Expected while docked. The dock structure fills the laser's near field, the AMCL match score
drops, and the filter starts guessing elsewhere. A narrow docked-only allowance covers this
while the SBC reports mated plates and the pose already agrees with the surveyed dock pose.

`max_position_variance` **MUST NOT** be raised to work around it — that value still gates
general navigation.

## Undock refuses with `HARDWARE_PROFILE_UNCOMMISSIONED`

`undock_profile.conf` has no measured profile for this dock. The profile is site-specific and
is not shipped with the image, so a robot moved to a new dock refuses until remeasured. This is
designed behavior, not a fault.

## Undock stops immediately with `SENSOR_BLOCKED`

Reverse motion is governed by `rear_velocity_stop`, sourced from four sonar transducers at
`min_points: 1`. One credible echo stops it. Sonar also reflects off dock structure and
sound-absorbing surfaces unpredictably — check what is actually behind the robot before
assuming a sensor fault.

## The dock state is stuck at `FAULT_STOPPED`

It is cleared only by an explicit `/patrolbot/dock_reset`. Nothing clears it automatically.

## RViz or `ros2` CLI on a laptop sees nothing

Working as configured. Every container sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so no
process outside the Pi can discover the graph.

Discovery **MUST NOT** be widened to `SUBNET` to fix this — that exposes an unauthenticated
command path to the whole LAN. Observe over SSH or through the dashboard.

## The robot reports front bumper contact while docked

Normal. The dock holds the two centre-front bumper panels (`0x1800`) active.

Note also that `bumper_enforcement_enabled` is `False`, so bumper contact does not stop the
robot in any case.

## A deployment change had no effect

Three usual causes:

1. The `install/` tree was not rebuilt — editing `src/` alone changes nothing at runtime.
2. The container was not restarted, so it is still running the old launch, params, or scripts.
3. The image tag in `docker/.env` still points at the previous build.

Confirm with the revision and source-tree digest reported by `./docker/status.sh`.

<!-- SPDX-License-Identifier: Apache-2.0 -->
