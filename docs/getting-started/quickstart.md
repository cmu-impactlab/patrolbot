---
title: Quickstart
description: Bring PatrolBot up from power-on, set an initial pose, and send a navigation goal — plus how to drive it manually with the gamepad.
---

# Quickstart

The fastest path from power-on to a moving robot. It assumes the SBC and main Pi 5
are [built](building.md) and [deployed](../deployment/robot-deployment.md). If you're
starting cold, skim [Installation](installation.md) first.

## 1. Power on — the robot starts itself

The SBC autostarts its systemd services and Docker restarts the Pi 5 containers. You
do **not** need to launch ROS manually in normal operation. Confirm the Pi 4 rollback
services are stopped before enabling motion.

```mermaid
flowchart LR
    PWR["power on"] --> SBC["SBC: socat + patrolbot_server (:7272)"]
    PWR --> PI["Pi: bringup → bridge → navigation"]
    SBC -. ":7272" .-> PI
    PI --> READY["map + pose in seconds;\ngoals after staged activation"]
```

## 2. Confirm health

```bash
ssh robot-pi2 'cd /home/ubuntu/patrolbot-repo && ./docker/status.sh'
# Expect: three healthy containers and OVERALL=ready

ssh robot-pi2 "docker exec patrolbot-bridge bash -lc \
  'source /opt/ros/\$ROS_DISTRO/setup.bash; for topic in /odom /scan; do \
     timeout --signal=INT --kill-after=2 6 ros2 topic hz \"\$topic\" || true; \
   done'"
# Expect: /odom and /scan around 20–25 Hz (about 25 Hz observed live)
```

If the containers, topics, lifecycle nodes, and TF links are individually healthy
but the deployed status command reports a lifecycle false negative, use the
direct `GetState` checks in [Debugging](../development/debugging.md) and deploy the
latest `docker/status.sh` with the next image revision.

If `/odom` and `/scan` aren't flowing, the SBC link is unavailable — the bridge will keep retrying every
3 s. See [Debugging](../development/debugging.md).

## 3. Open RViz (on the LAN)

```bash
export ROS_DOMAIN_ID=0
rviz2
```

- Set **Global Options → Fixed Frame = `map`**.
- You should see the map within seconds of the navigation service starting. If you see "Frame map
  does not exist," the most common cause is the Fixed Frame — then a missing initial pose.

!!! note "From home (VPN)?"
    Default discovery doesn't cross a VPN, so remote RViz sees nothing until the discovery server is
    enabled. See [Remote Operation](../deployment/remote-operation.md).

## 4. Set the initial pose

AMCL needs to know where the robot starts.

1. Click **2D Pose Estimate** in RViz.
2. Click the robot's real location on the map and drag in its facing direction.
3. The map and robot should snap into alignment; `map → odom` starts publishing.

This works **almost immediately** after boot (localization activates in seconds) — it does **not**
require the full navigation stack.

## 5. Send a navigation goal

Once the navigation half is active (expected around ~70 s after the boot-time network-wait fix):

!!! warning "Supervise every motion test"
    Clear the robot's path, keep the gamepad available, and be ready to release the
    deadman or use the physical emergency stop before sending a goal.

1. Click **Nav2 Goal** in RViz.
2. Click a destination and drag for the final heading.
3. The robot plans a path and drives to it, avoiding obstacles.

```bash
# Equivalent from the command line:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

If a goal is **rejected immediately**, the navigation half isn't active yet — wait and retry. See
[Actions](../ros2/actions.md).

## 6. Drive manually (gamepad)

The Logitech gamepad (USB on the Pi, switch on **X** for Xinput) overrides autonomy at any time:

- **Hold RB** (deadman) + **left stick Y** = forward/reverse.
- **Left stick X** = turn; D-pad axes are the fallback.
- Release RB; the command ramps to zero, then navigation resumes after the mux timeout.

Defaults are 0.35 m/s / 0.7 rad/s under teleop. See [Interfaces](../devices/interfaces.md#logitech-gamepad-manual-override).

## If something's wrong

| Symptom | Go to |
|---|---|
| RViz shows nothing | [Debugging](../development/debugging.md#frame-map-does-not-exist--blank-map-in-rviz) |
| Goal rejected / aborts | [Actions](../ros2/actions.md#failure-modes) |
| Robot localizes but won't drive | [Debugging](../development/debugging.md#robot-wont-move-under-navigation-but-localization-is-fine) |
| Scan looks mirrored | [Sensors](../devices/sensors.md#sick-lms-200-laser) |
| A change didn't take effect | [Updates](../deployment/updates.md) |

## After a physical SBC reboot

A physical SBC power-cycle resets wheel odometry to `0,0,0`. The robot reconnects automatically, but
AMCL's pose is now wrong — **re-do step 4 (2D Pose Estimate)**.
