---
title: Wire Protocol
description: The complete line grammar of the SBC-to-Pi TCP link — telemetry frames, control requests, undock progress and result frames, and every status code.
---

# Wire Protocol

The SBC and the Pi meet at exactly one TCP socket. This page is the formal reference for what
crosses it. For why the seam exists at all, see
[Communication Architecture](../architecture/communication-architecture.md).

The key words **MUST**, **MUST NOT**, and **SHOULD** are to be interpreted as described in
[RFC 2119](https://www.rfc-editor.org/rfc/rfc2119).

## Transport

| Property | Value |
|---|---|
| Endpoint | `10.0.0.1:7272` (bridge parameters `server_ip`, `server_port`) |
| Role | The SBC listens; `patrolbot_bridge` on the Pi connects |
| Encoding | ASCII, newline-terminated lines |
| Field separators | `:` between frame fields, `\|` between telemetry sections, `,` within a section |
| Nominal rate | 20 Hz source loop; 20–25 Hz observed |
| Connect timeout | 5.0 s |
| Receive timeout | 3.0 s |
| Reconnect delay | 3.0 s |
| Control timeout | 6.0 s |

The link carries no authentication and no encryption. It **MUST** remain on the dedicated
point-to-point Ethernet segment between the two machines.

Only one bridge client is expected at a time. Opening a second client to the same port
disturbs the session the robot is using, so read-only investigation **MUST NOT** create one.

## Frames from the SBC

### `HELLO` — capability banner

```text
HELLO:PROTOCOL=<n>:FEATURES=<token>,<token>,...
```

Sent when the SBC accepts a new client. `FEATURES` advertises optional capabilities; the
`UNDOCK` token declares that this SBC can execute the hardware undock. Without it the bridge
refuses undock goals outright rather than sending a command the SBC will never answer.

A malformed `HELLO` field is an error. Unknown feature tokens are ignored.

### `ODOM` / `LASER` — navigation telemetry

```text
ODOM:x,y,th,vx,vth|LASER:r1,r2,...,rN
```

| Section | Fields | Units |
|---|---|---|
| `ODOM` | pose `x`, `y`, heading `th`, linear velocity `vx`, angular velocity `vth` | m, m, rad, m/s, rad/s |
| `LASER` | `N` range readings, −90° to +90° | m, valid 0.25–8.0 |

Publishes `/odom` (`nav_msgs/msg/Odometry`), `/scan` (`sensor_msgs/msg/LaserScan`,
BEST_EFFORT), and the `odom` → `base_link` transform at 50 Hz.

A line missing either the `|` separator or either section label is rejected as malformed.

### `AUX` — auxiliary telemetry

```text
AUX:SONAR=x,y;x,y;...|BATT=volts,soc,charge_state,temp|FLAGS=flags,fault,stall,motors,estop|DOCK=...
```

| Section | Fields | Publishes |
|---|---|---|
| `SONAR` | `x,y` pairs separated by `;` | `/sonar` (`sensor_msgs/msg/PointCloud2`, BEST_EFFORT) |
| `BATT` | volts, state of charge, charge state, temperature | `/battery` (`sensor_msgs/msg/BatteryState`) |
| `FLAGS` | flags, fault, stall, motors enabled, E-stop pressed | `/diagnostics` |
| `DOCK` | 10 fields, see below | `/patrolbot/base_state`, `/patrolbot/dock_readiness` |

`DOCK` fields, in order:

`state`, `phase`, `phase_name`, `active`, `attempts`, `rear_range_m`, `rear_usable`,
`inhibited`, `inhibit_remaining_ms`, `profile_ok`

`DOCK` is emitted only by protocol 2 servers. Unknown sections are skipped in both directions,
so the two halves stay compatible across versions.

!!! info "Sections are parsed in isolation"
    Each `AUX` section is parsed and published independently. A malformed or missing section
    skips only its own topic and never disturbs another — and never touches `/odom` or
    `/scan`, which travel on a separate queue entirely. This isolation is deliberate and
    **MUST** be preserved when adding sections.

### `ACK` — request accepted

```text
ACK:<command>:<request_id>:<ACCEPTED|REPLAYED>
```

Exactly four fields. `REPLAYED` means the SBC recognised a repeated `request_id` and did not
perform the operation twice — the controls are idempotent by design.

### `RESULT` — terminal reply

```text
RESULT:<command>:<request_id>:<OK|ERROR>:<code>:<message>:<charge>:<motors>:<estop>
```

Exactly nine fields. `message` uses `_` in place of spaces on the wire. `charge` is an integer
charge state; `motors` and `estop` are `0` or `1`.

### `UNDOCK_EVENT` — phase transition

Twelve fields, one per phase transition during an undock:

`request_id`, `phase`, `phase_name`, `distance_traveled`, `lateral_error`, `heading_error`,
`charge_state`, `motors_enabled`, `minimum_rear_range`, `release_attempts`, `dock_state`

### `UNDOCK_RESULT` — terminal undock frame

Seventeen fields, adding `success`, `code`, `message`, `estop_pressed`, and `physically_clear`
to the event fields above.

`UNDOCK_EVENT` and `UNDOCK_RESULT` use distinct leading verbs rather than overloading `RESULT`,
so a bridge that predates them ignores them instead of logging a parse error on every frame.

## Frames to the SBC

```text
DRIVE:<linear>:<angular>
CHARGE_RELEASE:<request_id>
MOTOR_ENABLE:<request_id>
DOCK_RESET:<request_id>
UNDOCK:<request_id>:<clearance_m>:<speed_mps>:<timeout_s>:<validation_mode>
UNDOCK_CANCEL:<target_request_id>
```

`DRIVE` carries the command after Nav2, collision monitoring, the mux, and final smoothing —
see [Safety Chain](../safety/safety-chain.md).

!!! warning "`DRIVE` never enables motors"
    Motor enable is a separate, deliberate operation with stationary, released, E-stop-clear,
    fault-free, fresh-telemetry preconditions. A `DRIVE` line sent to disabled motors does
    nothing.

### Request identifiers

Every control request except `DRIVE` carries a caller-generated **canonical lowercase UUID**.
Non-canonical forms are rejected rather than normalized, because an ambiguous identifier
would break replay detection. Resending a request with the same id is safe and yields
`ACK:...:REPLAYED`.

`UNDOCK_CANCEL` is the exception to the id rule: its payload is the **target** operation's id,
not a new one. The SBC dispatches it outside its request tracker, which keys on command name
and would otherwise call a cancel of a pending `UNDOCK` "busy".

### Undock request bounds

`clearance_m`, `speed_mps`, and `timeout_s` **MUST** be finite and positive. Values are sent as
given: the SBC rejects anything outside its hard caps rather than clamping, so a bad goal
fails loudly instead of quietly becoming a different goal.

## Status codes

`UNDOCK_RESULT` codes, mirroring `patrolbot::UndockCode`:

| Code | Name | Meaning |
|---:|---|---|
| 0 | `OK` | Completed |
| 1 | `INVALID_REQUEST` | Malformed or out-of-bounds request |
| 2 | `LINK_UNAVAILABLE` | Link to the base is unusable |
| 3 | `PRECONDITION_FAILED` | A required precondition did not hold |
| 4 | `BUSY` | Another operation is in progress |
| 5 | `TIMEOUT` | Exceeded the requested timeout |
| 6 | `SESSION_CHANGED` | Hardware session generation changed mid-operation |
| 7 | `PROTOCOL_ERROR` | Frame-level violation |
| 8 | `RELEASE_FAILED` | Charge release did not take effect |
| 9 | `MOTOR_STATE_FAILED` | Motors did not reach the required state |
| 10 | `MOTION_FAILED` | Commanded motion did not occur |
| 11 | `SENSOR_BLOCKED` | Rear sensing blocked or unusable |
| 12 | `SAFETY_FAULT` | A safety condition tripped |
| 13 | `CANCELED` | Cancelled by request |
| 14 | `HARDWARE_PROFILE_UNCOMMISSIONED` | No measured `undock_profile.conf` — see [Limitations](limitations.md) |

## State vocabularies

Dock states:

`UNKNOWN`, `DOCKED_CONFIRMED`, `DEPARTING`, `CLEAR_CONFIRMED`, `FAULT_STOPPED`

Undock phases:

`IDLE`, `PREPARING_UNDOCK`, `RELEASING`, `ENABLING_MOTORS`, `BACKING_OUT`,
`VERIFYING_CLEARANCE`, `HANDOFF_READY`, `STOPPING`

`FAULT_STOPPED` is cleared only by an explicit `DOCK_RESET`. Motor enable is permitted only
where dock contact is impossible — dock state `CLEAR_CONFIRMED` or `UNKNOWN`, and E-stop not
pressed. The SBC independently enforces the same rule against its own dock observer, so the
Pi-side check is defence in depth rather than the only guard.

## Parsing rules

Frame field counts are **fixed**. A frame with a different count is malformed, not a frame with
optional trailing fields. Parsers **MUST** reject rather than guess.

An unknown leading verb returns "not mine" rather than raising, which is what lets old and new
halves of the system share a link without one spamming parse errors about the other's frames.

<!-- SPDX-License-Identifier: Apache-2.0 -->

### Scan acquisition age (optional navigation extension)

Navigation frames may append `|LASER_AGE_MS:<milliseconds>` after `LASER:`.
The SBC sends the oldest ARIA reading age across the complete scan, measured
while holding the laser lock. Every beam must be present and have age 0–500 ms;
otherwise it sends no ranges and age `-1`. Using the oldest reading also covers
ARIA's alternating deinterlaced acquisition timestamps.

The bridge subtracts this age and its measured local receive-queue delay from
its ROS clock. SBC/Pi wall clocks need not agree. Transport/send-buffer delay
and the individual beam acquisition offsets remain unmeasured: this corrects
known age, not an exact synchronized acquisition timestamp. Malformed,
duplicate, negative, or over-limit metadata suppresses scan publication while
odometry continues. Missing metadata is legacy-compatible with a throttled
warning; it is not acquisition-time acceptance. Empty laser payloads remain
silent. Existing bridges ignore the added section.
