---
title: Hardware Architecture
description: PatrolBot's two physical compute units (SBC and Raspberry Pi), what sensors and actuators are wired to each, and the serial/USB/network links between them.
---

# Hardware Architecture

PatrolBot has **two compute units** and a set of peripherals split deliberately between them.
This page documents the physical topology: what is plugged into what, on which port, at which
rate. The logical/software view is in [Software Architecture](software-architecture.md); the
protocol on the wire between the machines is in
[Communication Architecture](communication-architecture.md).

!!! info "SBC truth source while down"
    The SBC is currently down, so this page follows `SKILLS/sbc-architecture.md` from the source
    workspace for SBC service, port, watchdog, and wire-protocol details.

## Hardware topology

```mermaid
flowchart TB
    subgraph CHASSIS["Pioneer PatrolBot-SH chassis"]
        BASEMCU["Base microcontroller\n(differential drive, encoders)"]
        SONAR["4 rear sonar sensors"]
        LMS["SICK LMS-200 laser\n(mounted flipped)"]
        BATT["Battery pack"]
    end

    subgraph SBCBOX["SBC — robot main PC (172.20.87.231)"]
        TTYS0["/dev/ttyS0 @ 9600 baud"]
        TTYS2["/dev/ttyS2 @ 38400 baud"]
        ARIA["ARIA / patrolbot_server"]
        TTYS0 --> ARIA
        TTYS2 --> ARIA
    end

    subgraph PIBOX["Raspberry Pi (Ubuntu, ROS 2 Jazzy)"]
        ETH["Ethernet / Wi-Fi"]
        USB["USB"]
        ROS["ROS 2 stack"]
        USB --> ROS
        ETH --> ROS
    end

    GAMEPAD["Logitech gamepad\n(Xinput mode)"]

    BASEMCU -- "serial" --> TTYS0
    SONAR -- "base bus" --> BASEMCU
    BATT -- "base bus" --> BASEMCU
    LMS -- "RS-232/422" --> TTYS2
    SBCBOX <== "TCP :7272 (LAN)" ==> PIBOX
    GAMEPAD -- "USB" --> USB
```

## Ownership: what is attached to which machine

The guiding principle: **heavy sensing and the base live on the SBC; compute lives on the Pi.**

| Peripheral | Physically attached to | Port / link | Rate | Reaches ROS via |
|---|---|---|---|---|
| Pioneer base (drive + encoders) | **SBC** | `/dev/ttyS0` @ 9600 (through a boot-time `socat` → TCP:7000 shim) | 20 Hz odom | bridge → `/odom`, TF `odom→base_link` |
| SICK LMS-200 laser | **SBC** | `/dev/ttyS2` @ 38400 | ~20 Hz | bridge → `/scan` |
| Sonar (4 rear sensors) | **SBC** (via base) | base bus, read by ARIA | ~4–5 Hz | bridge → `/sonar` |
| Battery / charge state | **SBC** (via base) | base bus, read by ARIA | ~4–5 Hz | bridge → `/battery` |
| Base flags / faults / stall | **SBC** (via base) | base bus, read by ARIA | ~4–5 Hz | bridge → `/diagnostics` |
| Logitech gamepad | **Pi** | USB | event-driven | `joy_node` → `/joy` |
| PTZ VCC4 camera | (in `patrolbot-sh.p`) | — | — | **Not integrated** (config-only) |

Key consequence: **the Pi never talks to the laser or the base directly.** It receives all
sensor data over the single TCP stream from the SBC and emits all motion as `DRIVE` commands
back to the SBC. The only peripheral wired to the Pi is the gamepad.

## The Pioneer PatrolBot-SH base

A differential-drive research platform configured by the ARIA parameter file `patrolbot-sh.p`:

- **Footprint:** modeled in Nav2 as an octagon from `RobotLength 510 mm`, `RobotWidth 425 mm`, and
  the 0.29 m swing radius.
- **Drive:** two driven wheels + casters, controlled by the base microcontroller. Top speed is
  capped in software to **0.26 m/s** linear (RPP `desired_linear_vel`) for indoor patrol.
- **Sonar:** the base physically carries **4 rear-facing sonar sensors** (ARIA param file
  `patrolbot-sh.p` defines a generic 16-unit ring, but 12 positions are unpopulated and always
  report max-range 5000 mm — those are filtered out in `patrolbot_server.cpp` so `/sonar` carries
  only real detections). ARIA reports each valid return in robot-frame coordinates; the bridge
  republishes them as a `/sonar` point cloud in `base_link`.
- **Sensing for safety:** stall and fault flags from the base feed `/diagnostics`. Note that the
  bumper bit-fields are reported **raw, for reference only** — on this PatrolBot-SH a reserved bit
  reads high even when idle, so bumpers do not drive the alarm level (only fault flags and motor
  stalls do). See [Devices → Controllers](../devices/controllers.md).

Full device pages: [Actuators](../devices/actuators.md) (base drive),
[Sensors](../devices/sensors.md) (laser, sonar, battery).

## The SICK LMS-200 laser

- **Attached to the SBC** on `/dev/ttyS2` at 38400 baud, read by ARIA's `ArLaserConnector`.
- The bridge publishes the scan as `/scan` with a **180° forward** field of view
  (`angle_min = -π/2`, `angle_max = +π/2`), `range_min` 0.25 m, `range_max` 8.0 m.
- **Mounting:** the SICK is mounted **flipped** (`LaserFlipped=true` in `patrolbot-sh.p`), and
  ARIA returns its readings in flipped order, so the scan arrives mirrored left↔right. The fix is
  a static transform that **rolls `laser_frame` 180° about the forward axis** (`roll = π`),
  re-correcting left/right while keeping front as front.

!!! tip "Laser orientation confirmed 2026-06-29"
    Live TF from the Pi: `base_link → laser_frame` at `x=0.037, z=0.2`, quaternion `(x≈1, y=0, z=0, w≈0)` — that is **`roll = π`**, confirmed. Older notes claiming `yaw = π` were wrong. `LaserFlipped=true` in `patrolbot-sh.p` is consistent: the flip corrects ARIA's mirrored scan order, and the roll corrects the spatial orientation. See [Devices → Sensors](../devices/sensors.md#sick-lms-200-laser).

## The two compute units

### SBC (main PC)

The robot's original onboard PC. It is the **only** machine that can run ARIA and the only one
wired to the base and laser. It runs no ROS 2 — just the `socat` serial shim and the
`patrolbot_server` C++ binary. Reachable on the LAN at **172.20.87.231**, serving TCP port
**7272**. Details: [`patrolbot_hw_server`](../packages/patrolbot_hw_server.md).

### Raspberry Pi

The production navigation computer is a Raspberry Pi 4 running ROS 2 Jazzy as user `ubuntu` (home
`/home/ubuntu`). A Raspberry Pi 5 (`robot-pi2`, hostname `patrolbot-rpi5`, Ubuntu 24.04.4 LTS,
aarch64) is provisioned as the Docker migration target but is not yet production. The Pi 4's
resource constraints shape the software:

- **`ulimit -n = 1024`** — forces Nav2 composition into one container (see
  [Software Architecture](software-architecture.md#the-composed-nav2_container-and-why-composition-is-mandatory)).
- **Limited RAM** — forces single-threaded map decode (`MAGICK_THREAD_LIMIT=1`,
  `OMP_NUM_THREADS=1`) and motivates keeping the global costmap coarser than the static map.

## Network and power

- **SBC ↔ Pi:** Ethernet/Wi-Fi LAN, TCP only. On the same subnet, ROS 2 DDS multicast discovery
  works for tools like RViz; across a VPN it does not (see
  [Remote Operation](../deployment/remote-operation.md)).
- **Gamepad:** USB on the Pi, in **Xinput** mode (the X/D switch must be on X).
- **Power:** the base battery powers the chassis and onboard electronics. A **physical SBC reboot
  resets wheel odometry to zero** — an operational caveat, not a fault; the operator re-localizes
  with *2D Pose Estimate* afterward.
