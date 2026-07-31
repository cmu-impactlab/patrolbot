---
title: Quickstart
description: Bring PatrolBot up to a stationary, fail-closed readiness state before any physical commissioning.
---

# Quickstart

The current safe quickstart ends at **stationary readiness**. The software starts
automatically, but localization, joystick, bumpers, dock pose, successful motor
enable, and physical motion still require supervised commissioning. Do not turn
a healthy container status into permission to move the robot.

## 1. Power on

The SBC autostarts its systemd services and Docker restores the Pi 5 containers.
The SBC server connects to the base and laser, enables sonar, and keeps motors
disabled. The Pi connects as the sole TCP 7272 client.

```mermaid
flowchart LR
    PWR["power on"] --> SBC["SBC: socat + patrolbot_server"]
    PWR --> PI["Pi: bringup + bridge + navigation"]
    SBC -. "single TCP :7272 session" .-> PI
    PI --> READY["stationary telemetry and safety readiness"]
```

## 2. Confirm deployed health

```bash
ssh robot-pi2 'cd /home/ubuntu/patrolbot-repo && ./docker/status.sh'
```

The status report distinguishes software/container health from physical
commissioning. `OVERALL=degraded reason=ros-readiness` is expected while the
joystick, localization, dock pose, or bumper mapping is unavailable. It is not
safe to bypass those gates.

For a bounded telemetry-rate check that does not leave a hidden subscriber:

```bash
ssh robot-pi2 "docker exec patrolbot-bridge bash -lc \
  'source /opt/ros/\$ROS_DISTRO/setup.bash; for topic in /odom /scan; do \
     timeout --signal=INT --kill-after=2 6 ros2 topic hz \"\$topic\" || true; \
   done'"
```

Expected stationary invariants include:

- exactly one established Pi-to-SBC TCP 7272 connection;
- fresh `/scan`, `/odom`, `/sonar`, `/battery`, and `/patrolbot/base_state`;
- exactly one `/cmd_vel` publisher (`teleop_velocity_smoother`) and bridge subscriber;
- zero linear/angular odometry while no motion is authorized;
- motors disabled while charging and hardware fault flags zero.

## 3. Operator access

Raw DDS discovery is loopback-only. A laptop on the LAN or VPN will not discover
the ROS graph with RViz, even when the robot is healthy. Use SSH for observation
or the separately reviewed dashboard bridge. See
[Remote Operation](../deployment/remote-operation.md).

Dashboard telemetry does not itself authorize commands. Initial pose, goals,
charge release, motor enable, and velocity are separate state-changing
operations.

## 4. Localization

AMCL no longer assumes that every restart occurred at map origin. A valid pose
must come from an operator-authorized interface, or from
`/patrolbot/initialize_dock_pose`, which publishes the surveyed dock pose and
returns once AMCL converges. The dock pose is surveyed and configured
(`dock_pose_configured: true`); were it not, the initializer would reject
rather than publish a guessed pose.

Do not send a navigation goal while localization is invalid.

## 5. Before commanding motion

The robot senses obstacles in a single laser plane and cannot see anything below or above it,
so first motion **MUST** be supervised with a hand on the E-stop. Read
[Limitations](../reference/limitations.md) before commanding anything.

Motors do not enable themselves. A `DRIVE` line cannot enable them: motor power requires the
E-stop to be released and an explicit enable, and after an E-stop trip a physical `PWR ON` at
the robot is the only way back.

Any proposed velocity test **MUST** state the exact:

- publication topic and source;
- linear and angular magnitude;
- direction;
- maximum duration or distance;
- operator/E-stop arrangement;
- stop and rollback conditions.

Only explicit operator approval after those bounds are stated authorizes the
publication. Dock/undock and guarded reverse remain separate later phases.

## If something is wrong

| Symptom | Go to |
|---|---|
| Status reports stale scan/odom or no SBC session | [Debugging](../development/debugging.md) |
| LAN RViz sees nothing | [Remote Operation](../deployment/remote-operation.md) — expected with loopback-only DDS |
| Localization invalid | inspect telemetry and provide an authorized pose; never restore forced origin |
| Motors disabled | inspect charging, E-stop, faults, and physical readiness; `DRIVE` cannot enable motors |
| A deployment change did not take effect | [Updates](../deployment/updates.md) |
