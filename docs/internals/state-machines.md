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
    Serving --> Serving: stream ODOM|LASER/AUX, read DRIVE
    Serving --> Listening: EAGAIN guard (~3 s) / keepalive dead-peer → break & re-accept
```

The server is single-client: an outer `while(robot.isRunning())` loop wraps `accept()`, so a gone
Pi is detected (sustained `EAGAIN` + TCP keepalive/`TCP_USER_TIMEOUT`) and the server returns to
`Listening` for the next connection.

## `cmd_vel` arbitration (twist_mux)

Not a classic FSM, but a priority selection that behaves like one over time:

```mermaid
stateDiagram-v2
    [*] --> Navigation
    Navigation --> Joystick: joystick moves (input/joy, prio 8 > 5)
    Joystick --> Navigation: joystick released → 1 s timeout
    note right of Navigation
        Default: collision_monitor → input/navi (prio 5)
    end note
    note right of Joystick
        Override: p3dxJoyTeleop → input/joy (prio 8)
    end note
```

The joystick (priority 8) preempts navigation (priority 5) the instant a stick moves; on release
the teleop goes silent and twist_mux times the joy input out after 1 s, handing control back to
navigation. Configured-but-unused inputs (safety 10, teleop 8, switch 6) would slot into this
ordering if a publisher appeared.

## Joystick interlock (deadman)

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Driving: RB held AND stick past deadzone
    Driving --> Driving: publish Twist (input/joy)
    Driving --> Stopping: stick centered OR RB released
    Stopping --> Idle: publish one zero, then silent
```

The teleop publishes **only** while commanded, so an idle-but-connected controller never blocks
autonomy. The single explicit zero on release ensures the robot stops promptly rather than coasting
on a stale command; then silence lets the twist_mux timeout return control to navigation. See
[Nodes → patrolbot_joy_teleop](../ros2/nodes.md#patrolbot_joy_teleop).
