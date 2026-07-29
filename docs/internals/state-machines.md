---
title: State Machines
description: The state machines that govern PatrolBot — Nav2 lifecycle node transitions, the bridge connection state machine, the cmd_vel arbitration states, and the joystick interlock.
---

# State Machines

Several parts of PatrolBot are best understood as state machines. This page collects them: the
Nav2 lifecycle, the bridge's connection logic, the twist_mux arbitration, and the joystick
interlock.

## Nav2 lifecycle node

Every node in `nav2_container` is a **managed lifecycle node**. The lifecycle managers (patched
with `bond_timeout: 0.0`) drive them with `autostart: True`, and `lifecycle_mgr.py` drives the
mobile-base `teleop_velocity_smoother` the same way.

```mermaid
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Inactive: configure
    Inactive --> Active: activate
    Active --> Inactive: deactivate
    Inactive --> Unconfigured: cleanup
    Active --> Finalized: shutdown
    Inactive --> Finalized: shutdown
    Unconfigured --> Finalized: shutdown
    Finalized --> [*]
```

- **autostart** walks every managed node `Unconfigured → Inactive → Active` at launch.
- **`bond_timeout: 0.0`** disables the bond watchdog that would otherwise abort a node whose
  configure/activate takes too long — necessary because inflating the large map is slow. See
  [Software Architecture](../architecture/software-architecture.md#the-large-map-problem).
- Drive a transition by hand with `ros2 lifecycle set /<node> activate` or the `change_state`
  service ([Services](../ros2/services.md)).

## Bridge connection state machine

```mermaid
stateDiagram-v2
    [*] --> Connecting
    Connecting --> Connected: connect() ok
    Connecting --> Backoff: connect() fails
    Backoff --> Connecting: after 3 s
    Connected --> Receiving: recv loop
    Receiving --> Receiving: line parsed (ODOM|LASER / AUX)
    Receiving --> Backoff: recv timeout (3 s silence) / peer closed / error
```

- **Connecting:** open socket, set `SO_KEEPALIVE` + `RECV_TIMEOUT = 3 s`.
- **Receiving:** accumulate bytes, split on `\n`, dispatch each line.
- **Backoff → Connecting:** 3 s of silence (timeout), a closed peer, or any error closes the socket
  and retries after 3 s.

This is what makes the SBC link self-healing without operator action — detailed on
[Communication Architecture](../architecture/communication-architecture.md#self-healing-hardened-on-both-ends).

## SBC server accept state machine

```mermaid
stateDiagram-v2
    [*] --> Listening
    Listening --> Serving: accept() one Pi
    Serving --> Serving: send HELLO, stream ODOM|LASER/AUX, read DRIVE
    Serving --> Listening: EAGAIN guard (~3 s) / keepalive dead-peer → break & re-accept
```

The server is single-client: an outer `while(robot.isRunning())` loop wraps `accept()`, so a gone
Pi is detected (sustained `EAGAIN` + TCP keepalive/`TCP_USER_TIMEOUT`) and the server returns to
`Listening` for the next connection. Losing the client stops the base, **disables the motors
unconditionally**, and invalidates any operation in flight; nothing resumes on reconnect.

## SBC dock observer (persistent)

Decides whether the robot may be driven at all. It survives client reconnects and starts at
`UNKNOWN` — nothing is assumed at process start.

```mermaid
stateDiagram-v2
    [*] --> UNKNOWN: process start
    UNKNOWN --> DOCKED_CONFIRMED: charging debounced OR front-contact bits
    UNKNOWN --> CLEAR_CONFIRMED: charge 0 debounced AND contacts clear AND stationary
    DOCKED_CONFIRMED --> CLEAR_CONFIRMED: same explicit clear verification
    DOCKED_CONFIRMED --> DEPARTING: UNDOCK accepted
    DEPARTING --> CLEAR_CONFIRMED: full physical clearance proof
    DEPARTING --> FAULT_STOPPED: failure or cancellation
    CLEAR_CONFIRMED --> DOCKED_CONFIRMED: charge RISING EDGE after a confirmed zero,\nstationary, plus contact bits or a dwell
    FAULT_STOPPED --> UNKNOWN: explicit DOCK_RESET once the condition clears
```

Ordinary `DRIVE` is permitted **only** in `CLEAR_CONFIRMED`. `charge_state` is an input to this
observer, never an interlock in its own right, so a stale re-asserted charge level after a proven
departure does not block navigation — and the only latch that survives such a level is
`DEPARTING → CLEAR_CONFIRMED`, which requires measured displacement plus clear bumpers and rear
sonar. A stale level alone can never move the observer back to `DOCKED_CONFIRMED`.

## SBC undock operation (bounded)

Runs only between `DEPARTING` entry and exit, at 20 Hz in the main loop, supervised from outside
by an independent `ArASyncTask` command-lease watchdog.

```mermaid
stateDiagram-v2
    [*] --> PREPARING_UNDOCK
    PREPARING_UNDOCK --> RELEASING: motors confirmed off, stationary
    RELEASING --> ENABLING_MOTORS: falling edge or one valid charge-zero sample
    RELEASING --> RELEASING: bounded retry, rate-limited CHARGE 0
    ENABLING_MOTORS --> BACKING_OUT: motors confirmed in a LATER snapshot
    BACKING_OUT --> BACKING_OUT: charge re-asserted but motors live and\nprogress healthy → keep reversing
    BACKING_OUT --> RELEASING: motors dropped OR no motion (bounded)
    BACKING_OUT --> VERIFYING_CLEARANCE: projected reverse progress ≥ clearance
    VERIFYING_CLEARANCE --> BACKING_OUT: front contact persists, budget remains
    VERIFYING_CLEARANCE --> HANDOFF_READY: stationary, bumpers clear, rear clear
    HANDOFF_READY --> [*]: RESULT OK, MOTORS LEFT ENABLED, authority → DRIVE
    PREPARING_UNDOCK --> STOPPING: any safety trip
    RELEASING --> STOPPING: attempts exhausted
    BACKING_OUT --> STOPPING: bumper/sonar/stall/e-stop/fault/timeout
    STOPPING --> [*]: RESULT ERROR, motors DISABLED, dock state FAULT_STOPPED
```

Two terminal outcomes, deliberately different:

| | Velocity | Motors | Authority | Dock state |
|---|---|---|---|---|
| Success (`HANDOFF_READY`) | 0, confirmed | **enabled** | transferred to `DRIVE` | `CLEAR_CONFIRMED` |
| Failure / cancel (`STOPPING`) | 0, confirmed | **disabled** | `NONE` | `FAULT_STOPPED` |

Success leaves the motors live because the generic drive gate refuses motion without them:
handing off any other way would strand the robot. Charge re-assertion while the reverse is
healthy does **not** stop the robot — stopping on every re-latch would let the contacts settle
back in — it only triggers a rate-limited `CHARGE 0`. Completion uses signed reverse progress
projected onto the departure heading, with lateral-drift and heading-error limits, so forward or
sideways motion never counts as progress.

## `cmd_vel` arbitration (twist_mux)

Not a classic FSM, but a priority selection that behaves like one over time:

```mermaid
stateDiagram-v2
    [*] --> Navigation
    Navigation --> Joystick: RB held → cancel goal + input/joy (prio 8 > 5)
    Joystick --> [*]: RB released → zero + 0.5 s timeout
    note right of Navigation
        Default: collision_monitor → input/navi (prio 5)
    end note
    note right of Joystick
        Override: p3dxJoyTeleop → input/joy (prio 8)
    end note
```

The joystick (priority 8) preempts navigation (priority 5) when RB is held and
cancels the active Nav2 goal. On release the teleop ramps to zero and stays at
priority 8 until cancellation is confirmed, then goes silent and times out
after 0.5 s. A new autonomous goal must be sent explicitly. Configured-but-unused
inputs (safety 10, teleop 8, switch 6) would slot into this ordering if a
publisher appeared.

## Joystick interlock (deadman)

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Driving: RB held AND stick past deadzone
    Driving --> Driving: publish Twist (input/joy)
    Driving --> Stopping: stick centered OR RB released
    Stopping --> Idle: publish one zero, then silent
```

The teleop publishes **only** while commanded or while waiting for takeover
cancellation, so an idle-but-connected controller never blocks autonomy. The
explicit zero on release prevents coasting; silence begins only after the prior
Nav2 goal cannot resume. See
[Nodes → patrolbot_joy_teleop](../ros2/nodes.md#patrolbot_joy_teleop).
