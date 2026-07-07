---
title: Sensors
description: PatrolBot's sensors — the SICK LMS-200 laser, the rear sonar sensors, and the battery monitor — with connection, protocol, ROS interface, rates, calibration, and failure conditions.
---

# Sensors

All sensing hardware is wired to the **SBC** and reaches ROS 2 only through the bridge's TCP
stream (see [Communication Architecture](../architecture/communication-architecture.md)). Each
sensor below lists its host, connection, ROS interface, rate, calibration, and failure modes.

## SICK LMS-200 laser

The primary perception sensor: localization and obstacle avoidance both depend on it.

| Field | Value |
|---|---|
| **Hardware** | SICK LMS-200 planar laser scanner |
| **Host machine** | **SBC** |
| **Connection** | `/dev/ttyS2` @ 38400 baud, read by ARIA `ArLaserConnector` |
| **Protocol (to ROS)** | SBC parses ARIA ranges → `LASER:r1,...,rN` text line → bridge → `/scan` |
| **ROS topic** | `/scan` (`sensor_msgs/LaserScan`), frame `laser_frame` |
| **Field of view** | 180° forward: `angle_min = -π/2`, `angle_max = +π/2` |
| **Range** | `range_min = 0.25 m`, `range_max = 8.0 m` |
| **Update rate** | ~20 Hz |

### Calibration / mounting

- **Position:** `base_link → laser_frame` static TF at `x = 0.037 m` (ARIA `LaserX`), `z = 0.2 m`.
- **Orientation:** the SICK is mounted **flipped** (`LaserFlipped=true` in `patrolbot-sh.p`) and
  ARIA returns readings in flipped order, so the scan arrives mirrored left↔right. The live launch
  corrects this with **`roll = π`** about the forward axis (front stays front; left/right swap is
  undone).
- **Footprint-clearance cutoff:** the bridge forces any return `< 0.25 m` to `+inf`. Do not raise
  this much higher; nearby real obstacles can appear in the 0.25-0.40 m range.

!!! success "Orientation confirmed"
    `roll = π` is the correct static TF for the flipped SICK mount. It is consistent with
    `LaserFlipped=true` in `patrolbot-sh.p`; older `yaw = π` notes are stale.

### Failure conditions

| Condition | Symptom | Handling |
|---|---|---|
| Laser unplugged / SBC can't open `/dev/ttyS2` | `LASER:` field empty or absent | bridge publishes no/short `/scan`; costmaps clear, AMCL can't update → `map→odom` stops |
| SBC down | no `/scan` at all | bridge reconnects every 3 s; resumes on return |
| Scan appears mirrored in RViz | walls on wrong side | re-check `LaserFlipped` and the `roll=π` static TF |
| Phantom obstacle hugging the robot | nav refuses to move | check `SCAN_RANGE_MIN`; run `./patrolbot-logs.sh scan` |

## Sonar

| Field | Value |
|---|---|
| **Hardware** | 4 rear-facing sonar sensors on the Pioneer base |
| **Host machine** | **SBC** (read by ARIA; enabled at startup via `robot.enableSonar()`) |
| **Connection** | base bus → ARIA → `AUX:SONAR=x,y;...` text line |
| **ROS topic** | `/sonar` (`sensor_msgs/PointCloud2`), frame `base_link` |
| **Update rate** | ~4–5 Hz (every 5th nav frame) |
| **Calibration** | geometry from ARIA `patrolbot-sh.p`; the SBC computes each valid echo as a point in `base_link` |

The sonar feeds **visualization/monitoring**, not the costmaps — obstacle avoidance is laser-based.
The ARIA param file defines a generic 16-position ring, but the robot physically has 4 rear sensors
and the 12 unpopulated entries sit at max range. The SBC publishes only valid echoes
(`getRange() < 4335 mm`), so `/sonar` width is the live detection count, often 0-2.

**Failure conditions:** a malformed `SONAR` section is dropped in isolation (the `AUX` line is
parsed section-by-section), so a sonar glitch never disturbs `/scan` or `/odom`. If the base does
not report sonar, `/sonar` simply stops; nothing else is affected.

## Battery

| Field | Value |
|---|---|
| **Hardware** | Pioneer base battery (no true state-of-charge sensor) |
| **Host machine** | **SBC** (ARIA battery readings) |
| **Connection** | base bus → ARIA → `AUX:BATT=volt,soc,chargeState,temp` |
| **ROS topic** | `/battery` (`sensor_msgs/BatteryState`) |
| **Update rate** | ~4–5 Hz |

Field mapping in the bridge:

- `voltage` — `getRealBatteryVoltageNow()`; **the meaningful field.**
- `percentage` — `NaN` if the base reports no state-of-charge (`soc = -1`), which is the case here.
- `power_supply_status` — `CHARGING` if ARIA charge state > 0 (on dock), else `DISCHARGING`.
- `temperature` — `NaN` if unavailable.

**Failure conditions:** treat `percentage` as unavailable on this base — monitor `voltage`. A
malformed `BATT` section drops only `/battery`.

## Sensor data path (summary)

```mermaid
flowchart LR
    L["SICK LMS-200\n/dev/ttyS2"] -->|ARIA| S["SBC server"]
    SO["sonar ring"] -->|ARIA| S
    B["battery"] -->|ARIA| S
    S -->|"LASER:"| BR["bridge"]
    S -->|"AUX:SONAR/BATT"| BR
    BR -->|/scan| NAV["AMCL + costmaps"]
    BR -->|/sonar| RV["RViz"]
    BR -->|/battery| MON["monitoring"]
```

See [Controllers](controllers.md) for the base status/diagnostics path and
[Actuators](actuators.md) for the drive side.
