---
title: Interfaces
description: PatrolBot's human and host interfaces — the Logitech gamepad on the Pi, the SBC TCP telemetry socket, and the not-integrated PTZ camera.
---

# Interfaces

"Interfaces" here means the human-input and machine-to-machine interfaces that are not strictly
sensors or actuators: the gamepad, the SBC↔Pi socket, and the (unused) PTZ camera.

## Logitech gamepad (manual override)

The **only** device wired to the Pi, and the operator's manual-override interface.

| Field | Value |
|---|---|
| **Hardware** | Logitech gamepad in **Xinput** mode (X/D switch on **X**) |
| **Host machine** | **Pi 5** (USB, main runtime) |
| **Connection** | USB → Linux `joydev` → `/dev/input/js*` |
| **ROS interface** | `joy_node` → `/joy` (`sensor_msgs/Joy`) → `patrolbot_joy_teleop` → `input/joy` |
| **Update rate** | event-driven (on input change) |

### Controls

| Control | Axis/Button | Action |
|---|---|---|
| Deadman | **RB** (button 5) | must be held for any motion (safety interlock) |
| Drive | left-stick **Y** (axis 1), D-pad Y (axis 7) fallback | forward / reverse, maximum 0.30 m/s |
| Turn | left-stick **X** (axis 0), D-pad X (axis 6) fallback | rotate, default maximum 0.7 rad/s |
| Deadzone | — | 0.12 |
| Speed trim | A/Y and X/B | adjust linear and angular maximums within their hardware-safe caps |

While RB is held, the teleop publishes its acceleration-ramped command at 30 Hz,
including zero when the sticks are centered, at twist_mux **priority 8** (above
navigation's 5). A 0.4 s `/joy` watchdog drops the deadman if the controller stream
disappears. The rising RB edge cancels any active Nav2 goal. On RB release it ramps
to zero, publishes one final zero, and goes silent after cancellation is confirmed;
the canceled goal never resumes automatically. See
[Nodes → patrolbot_joy_teleop](../ros2/nodes.md#patrolbot_joy_teleop).

The linear maximum is pinned to the SBC's ordinary `DRIVE` ceiling. Raising it
above 0.30 m/s makes the hardware server reject the command and force zero
velocity, which feels like intermittent or chunky control rather than a clean
speed clamp.

### Failure conditions

| Condition | Symptom | Effect |
|---|---|---|
| Controller unplugged | no `/dev/input/js*`, no `/joy` | 0.4 s watchdog drops the deadman and ramps to zero; a canceled goal stays canceled |
| Switch on **D** (DirectInput) | wrong axis/button map | sticks do nothing useful — set the switch to **X** |
| Deadman not held | sticks ignored | by design (interlock) |

## SBC telemetry socket (machine-to-machine)

The defining interface of the whole robot: a single TCP socket joining the two machines.

| Field | Value |
|---|---|
| **Endpoint** | SBC `10.0.0.1:7272` (server); Pi bridge is the client |
| **Protocol** | plain-text lines: `ODOM|LASER` (nominal ~20 Hz; ~25 Hz observed live), `AUX` (~5 Hz), `DRIVE` (on demand) |
| **Framing** | newline-delimited; the Pi splits on `\n` and dispatches by prefix |
| **Security** | none — LAN-local plaintext (assumes a trusted robot network) |

This is documented in full on [Communication Architecture](../architecture/communication-architecture.md).
You can inspect it from any machine on the LAN:

```bash
# Read the raw stream (do NOT do this while the bridge is the active client —
# the server is single-client; use a dev/test instance instead).
nc 10.0.0.1 7272
```

## PTZ VCC4 camera (not integrated)

| Field | Value |
|---|---|
| **Hardware** | Canon VCC4 pan-tilt-zoom camera |
| **Status** | **referenced in `patrolbot-sh.p` only; not integrated** |

The ARIA hardware profile mentions a PTZ VCC4 camera, but **nothing in the active stack drives or
reads it** — there is no ROS node, topic, or service for it. It is listed here so its absence from
the live system is explicit. Adding it would be net-new work (a camera/PTZ driver on the SBC or
Pi, plus a ROS interface), not a wiring change.

## Operator interfaces (software)

Not a device, but worth naming as the human interface to the running system:

| Interface | Where | Purpose |
|---|---|---|
| **Reviewed dashboard bridge** | Pi/application service | telemetry; pose/goal only when its separate command mode and physical procedure are authorized |
| **`docker compose logs` / `docker exec`** | Pi 5 (SSH) | live container logs and ROS 2 inspection |
| **Container-local ROS tools** | Pi 5 (SSH) | bounded graph/topic diagnostics without exposing DDS to the LAN |

Raw RViz discovery is intentionally unavailable on both LAN and VPN — see
[Remote Operation](../deployment/remote-operation.md).
