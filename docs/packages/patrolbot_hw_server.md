---
title: patrolbot_hw_server (SBC)
description: The SBC's standalone C++ ARIA server — the hardware data source that streams telemetry to the Pi over TCP.
---

# patrolbot_hw_server (SBC)

The SBC's entire robot software: one C++ program, `patrolbot_server`, that speaks ARIA to the
hardware and a plain-text protocol to the Pi. It is **not a ROS package** — just a `Makefile`
project linked against ARIA.

!!! info "SBC source and live verification"
    On 2026-07-22 the system units, user server unit, single TCP client,
    deployed source/binary checksums, and live stationary telemetry were
    revalidated. The commissioned server keeps motors disabled at startup and
    exposes separate, idempotent charge-release and motor-enable operations.

| | |
|---|---|
| **Deploys to** | **SBC** (robot main PC; Pi endpoint `10.0.0.1:7272`, lab SSH may be `172.20.87.231`) |
| **Build** | `Makefile` (`g++ -I/usr/local/Aria/include -lAria -lArNetworking -lpthread`) |
| **Binary** | `patrolbot_server` (32-bit i386 executable; deployed atomically from a tested candidate) |
| **Source** | `hardware/patrolbot_hw_server/` in this repository; deployed to `~/patrolbot_hw_server/` on the SBC |
| **Runs ROS 2?** | **No** |

## Purpose

Be the robot's hardware data source. Connect ARIA to the Pioneer base and the SICK laser, enable
sonar, keep motors disabled until a deliberate control request succeeds, stream telemetry to the
Pi, and accept drive and bounded hardware-control requests back. Nothing on the SBC participates
in navigation — that all lives on the Pi.

## Interfaces

This program's "public interface" is the [TCP wire
protocol](../architecture/communication-architecture.md), not ROS:

| Direction | Line | Rate |
|---|---|---|
| SBC → Pi | `ODOM:x,y,th,vx,vth|LASER:r1,...,rN` | ~20 Hz |
| SBC → Pi | `AUX:SONAR=..|BATT=..|FLAGS=..|DOCK=..` | ~4–5 Hz (every 5th nav frame) |
| SBC → Pi | `HELLO:PROTOCOL=2:FEATURES=UNDOCK` | once, on accept |
| Pi → SBC | `DRIVE:linear:angular` | on demand |
| Pi → SBC | `CHARGE_RELEASE` / `MOTOR_ENABLE` / `DOCK_RESET` `:request_uuid` | deliberate zero-motion request only |
| Pi → SBC | `UNDOCK:uuid:clearance:speed:timeout:validation` | the exclusive dock-departure operation |
| Pi → SBC | `UNDOCK_CANCEL:target_uuid` | names the operation being cancelled |
| SBC → Pi | `ACK:...` / `RESULT:...` | correlated control response |
| SBC → Pi | `UNDOCK_EVENT:...` / `UNDOCK_RESULT:...` | undock progress and terminal frame |

## Dock departure

The SBC owns physical separation from the charger outright: charger release, motor enable,
reverse velocity, encoder displacement, charge-state recovery, stall detection, rear sonar and
bumper safety, and the completion decision. While an `UNDOCK` runs, ordinary `DRIVE` is refused
and the maintenance commands are refused; only cancellation and E-stop are accepted.

The machine lives in `undock.cpp`/`undock.hpp`, which include **no ARIA headers and perform no
I/O** — every decision is a pure function of an explicit hardware snapshot, so the whole thing is
unit-testable off the robot. `patrolbot_server.cpp` is the only translation unit that maps
`ArRobot` onto those structures and applies the resulting effects.

Configuration lives in `undock_profile.conf`, deployed alongside the binary. A production undock
is refused with `HARDWARE_PROFILE_UNCOMMISSIONED` until the rear-sonar indices, E-stop semantics
and fault masks in that file have been physically measured. See
[State machines](../internals/state-machines.md) for both diagrams.

## Commissioning probe

`patrolbot_commission_probe` (built with `make probe`) fills in those unmeasured facts.

**Its default mode is strictly passive**: it reads ARIA telemetry at 20–50 Hz and writes CSV,
including each sonar's geometry, range and age, so the rear transducer indices can be identified.
It never sends a velocity command, never enables or disables motors, and never sends a charging
command.

`--allow-single-charge-release` is an **active** mode, not a read-only one. It issues exactly one
µARCS `CHARGE 0` after a logged baseline, and only when reported motion is stably zero, the motors
are already disabled, the confirmation token is supplied, and no production server is running.
There is no motor-enable and no motion path in any mode, and `CHARGE 1` is never sent.

!!! danger "Single-owner hardware connection"
    `socat` runs without `fork` and only one process may hold the base connection, so the probe
    **cannot** run while `patrolbot_server` is active — it refuses to start if it detects one. To
    use it, stop the service with the robot stationary and motors disabled, run the probe, then
    restart the service:

    ```bash
    ssh robot-sbc 'systemctl --user stop patrolbot-server.service'
    ssh robot-sbc 'cd /home/ros/patrolbot_hw_server && ./patrolbot_commission_probe \
      --seconds=120 --out=/tmp/commission.csv -rh 127.0.0.1 -rrtp 7000'
    ssh robot-sbc 'systemctl --user start patrolbot-server.service'
    ```

    The Pi bridge drops and reconnects automatically. Do not restart `socat-boot.service`.

Units are converted from ARIA's mm/deg to m/rad **before** sending.

## Internal architecture

```mermaid
flowchart TB
    subgraph SBC["SBC"]
        SOCAT["socat (boot service)\n/dev/ttyS0 → TCP:7000"]
        SERVER["patrolbot_server"]
        BASE["Pioneer base"]
        LASER["SICK LMS-200\n/dev/ttyS2"]
        BASE -- "serial" --> SOCAT
        SOCAT -- "TCP:7000" --> SERVER
        LASER -- "ARIA direct" --> SERVER
    end
    SERVER <-->|"TCP :7272"| PI["Pi bridge"]
```

- **ARIA connections:** `ArRobotConnector` to the base (routed through the socat TCP bridge with
  `-rh 127.0.0.1 -rrtp 7000`), `ArLaserConnector` to the SICK on `/dev/ttyS2`. Sonar is enabled and
  motors are explicitly disabled at startup.
- **Telemetry build:** each section is assembled defensively under `robot.lock()`. The `AUX` line
  is emitted as a **separate, independent** line every 5th nav frame so an AUX build/parse problem
  can never disturb the navigation-critical `ODOM|LASER` line.
- **Server loop:** single-client. An outer `while (robot.isRunning())` loop wraps `accept()`, so
  the server re-accepts automatically after each Pi disconnect.
- **Self-healing:** a consecutive-`EAGAIN` guard (~3 s) plus `SO_KEEPALIVE` + `TCP_USER_TIMEOUT
  = 5000 ms` detect a silently-gone Pi and break to re-`accept()` promptly (instead of spinning on
  a full send buffer for minutes).
- **Command watchdog:** if no `DRIVE` arrives for 750 ms, the server commands zero velocity. It also
  stops the base on Pi disconnect before accepting the next client.
- **Motor policy:** `DRIVE` never enables motors. Only a canonical-UUID `MOTOR_ENABLE` request can
  call ARIA `enableMotors()`, and it is rejected unless the robot is stationary, charge state is
  zero, the E-stop is clear, telemetry is valid, and fault flags are zero.
- **Charge release:** `CHARGE_RELEASE` disables motors if necessary, confirms them stably off,
  sends µARCS command 68/argument 0 exactly once, and confirms stable charge state zero. It never
  sends velocity or enables motors. Reusing the same UUID replays the cached result instead of
  repeating the hardware action.
- **Stale laser handling:** stale cached SICK readings are not streamed as live data. When the laser
  is disconnected or stale, the server emits an empty `LASER:` field so the Pi safety path detects
  sensor loss.
- **Diagnostics:** the `FLAGS` section includes motors-enabled and e-stop state; a latched hardware
  e-stop cannot be overridden in software.

## Why the serial bridge (`socat`)?

The base and the ARIA server both wanted `/dev/ttyS0`, which is a single-holder device. A boot-time
`socat-boot.service` opens `/dev/ttyS0` once and exposes it as `TCP:7000`; the ARIA server then
reaches the base over that socket (`-rrtp 7000`) rather than contending for the serial device. This
resolves the serial conflict cleanly.

## Deployment

| Service | Type | Role |
|---|---|---|
| `patrolbot-wired-ip.service` | systemd **system** | keep `10.0.0.1/24` applied to `enp2s5` with `Restart=always` |
| `socat-boot.service` | systemd **system** | bridge `/dev/ttyS0` → TCP:7000 at boot |
| `patrolbot-server.service` | systemd **user** | run `patrolbot_server -rh 127.0.0.1 -rrtp 7000` at boot |

The user service needs `loginctl enable-linger ros` once to start without a login. The architecture
notes record this as done.

## Building (on the SBC)

```bash
cd ~/patrolbot_hw_server
make -B test               # mandatory: the SBC RTC can make mtimes unreliable
make -B patrolbot_server   # g++ ... -lAria -lArNetworking -lpthread
```

Do not execute a candidate beside the live server: port 7272 and the ARIA/socat path are
single-owner resources. Deployment uses a checksum-backed backup, atomic binary replacement, and
a restart of only `patrolbot-server.service`.

## Where to read more

- The protocol in full: [Communication Architecture](../architecture/communication-architecture.md).
- The Pi end: [`patrolbot_bridge`](patrolbot_bridge.md).
- Current open verification items: [Known Gaps](../known-gaps.md).
