---
title: Running the Robot
description: Day-to-day operation — bringing the stack up and down, checking readiness, driving manually, undocking, and running a navigation goal.
---

# Running the Robot

This page covers normal operation of a commissioned robot. Every command runs **on the Pi**
(`robot-pi2`) unless stated otherwise.

The key words **MUST**, **MUST NOT**, and **SHOULD** are to be interpreted as described in
[RFC 2119](https://www.rfc-editor.org/rfc/rfc2119).

Read [Safety Chain](../safety/safety-chain.md) and [Limitations](../reference/limitations.md)
before commanding motion for the first time. The robot cannot see low obstacles, and bumper
contact does not stop it.

## Check readiness first

```bash
cd ~/patrolbot-repo && ./docker/status.sh
```

This is read-only. It reports container liveness and health, the deployed image revision and
source-tree digest, and whether the SBC socket at `10.0.0.1:7272` is reachable.

Do not skip it. Every container can be `running` while the robot has no base data at all,
because the SBC is a separate machine — see [Limitations](../reference/limitations.md).

## Bring the stack up

```bash
cd ~/patrolbot-repo
docker compose -f docker/docker-compose.yml --env-file docker/.env up -d
```

The stack is four services:

| Container | Role |
|---|---|
| `patrolbot-bringup` | Mux, smoother, teleop, watchdog |
| `patrolbot-bridge` | The SBC TCP link — owns `/odom`, `/scan`, `/sonar`, `/battery` |
| `patrolbot-navigation` | Nav2, composed into one container process |
| `patrolbot-web-bridge` | Dashboard telemetry |

All use `network_mode: host`, `restart: unless-stopped`, and a 20 s stop grace period.

Bring the **bridge up before navigation** when starting from cold. Nav2's costmaps need the
`odom` → `base_link` transform to activate; without it, activation fails and the container
stays alive but empty. The `transform_tolerance: 1.0` setting exists to widen that window —
see [Configuration](../reference/configuration.md).

## Bring the stack down

```bash
docker compose -f docker/docker-compose.yml --env-file docker/.env down
```

Stopping the stack does not stop the robot moving if it is already moving — it removes the
source of new commands, and the base's own command timeout brings it to rest. To stop a moving
robot, use the E-stop.

## Verify the data path

After any restart, confirm the telemetry actually flows:

```bash
docker exec patrolbot-navigation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic hz /scan'
```

Expect 20–25 Hz on `/scan`, and about 5 Hz on `/sonar` and `/battery`. Also confirm both
transforms exist:

```bash
docker exec patrolbot-navigation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros tf2_echo odom base_link'
```

## Manual driving

The joystick has mux priority 8 and outranks Nav2 (priority 5), so picking up the controller
takes ownership mid-goal. Releasing it hands control back after the 0.5 s input timeout.

Manual commands are bounded by the teleop smoother at 0.3 m/s and 0.7 rad/s, but they do
**not** pass through the collision monitor. Nothing stops the robot for you while driving
manually except the E-stop.

## Enabling motors

Motors do not enable themselves, and a `DRIVE` command never enables them.

```bash
docker exec patrolbot-navigation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 service call /patrolbot/motor_enable \
   patrolbot_interfaces/srv/MotorEnable "{request_id: \"<canonical-uuid>\"}"'
```

`request_id` **MUST** be a caller-generated canonical lowercase UUID. Resending the same id is
safe — the SBC recognises the replay and does not act twice.

Motor enable is permitted only where dock contact is impossible: dock state `CLEAR_CONFIRMED`
or `UNKNOWN`, with the E-stop clear. Enabling motors in order to leave the dock is the undock
operation's job, not this one's.

## Undocking

The controller Start-button, localization-independent variant is documented in
[Controller manual undocking](controller-undocking.md). It is separate from the
dashboard operation below and requires a matched, reviewed release before use.

Contact separation is an exclusive SBC-owned operation exposed as an action:

| Interface | Server | Purpose |
|---|---|---|
| `/patrolbot/undock` (action) | `patrolbot_dock_manager` | Guarded undock — the operator entry point |
| `/patrolbot/initialize_dock_pose` (service) | `patrolbot_dock_manager` | Seed localization from the surveyed dock pose |
| `/patrolbot/hardware_undock` (action) | `patrolbot_bridge` | Lower-level hardware undock |
| `/patrolbot/charge_release` (service) | `patrolbot_bridge` | Release charging contacts only |
| `/patrolbot/motor_enable` (service) | `patrolbot_bridge` | Enable motors |
| `/patrolbot/dock_reset` (service) | `patrolbot_bridge` | Clear the SBC's `FAULT_STOPPED` dock state |

Use `/patrolbot/undock` for normal operation. `/patrolbot/hardware_undock` is the layer beneath
it and bypasses the dock manager's guards.

An undock refuses with `HARDWARE_PROFILE_UNCOMMISSIONED` unless `undock_profile.conf` supplies
a measured profile for this dock. That refusal is designed behavior.

During undock the robot reverses, so the `rear_velocity_stop` polygon governs — four sparse
sonar transducers with `min_points: 1`. A single echo halts the motion.

After an undock reaches `HANDOFF_READY`, Nav2 takes over. Do not send a navigation goal before
the handoff completes.

## Navigation goals

Localization **MUST** be valid before any goal. After a restart, seed the pose from the
surveyed dock pose with `/patrolbot/initialize_dock_pose`; never accept a guessed pose and
never force the map origin.

```bash
docker exec patrolbot-navigation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose \
   nav2_msgs/action/NavigateToPose "<goal>"'
```

| Action | Use |
|---|---|
| `/navigate_to_pose` | Single destination |
| `/navigate_through_poses` | Ordered multi-pose run |
| `/follow_waypoints` | Patrol-style waypoint list |

Nav2 motion is capped at 0.22 m/s cruise (0.26 m/s ceiling) and passes through the collision
monitor's `front_velocity_stop` polygon.

## Monitoring

With the current `SUBNET` YAML policy, `ros2` CLI and RViz on the same
multicast-capable LAN can discover the full graph using `ROS_DOMAIN_ID=0`.
This exposure includes unauthenticated command endpoints. SSH and the dashboard
remain available, and changing the one Compose scalar to `LOCALHOST` restores
Pi-only discovery.

```bash
docker logs -f patrolbot-bridge
docker exec patrolbot-navigation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /diagnostics'
```

`/diagnostics` carries the base flags — fault, stall, motors enabled, and E-stop state.

<!-- SPDX-License-Identifier: Apache-2.0 -->
