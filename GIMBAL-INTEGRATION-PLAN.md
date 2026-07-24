# Gimbal Integration Plan

Bringing the SimpleBGC 32-bit gimbal and FLIR Duo Pro R into the PatrolBot ROS 2 stack and
the dashboard, driven by the existing Logitech F710 and by on-screen controls.

**Status:** plan only. Nothing here is implemented yet.

Decisions already settled: the ROS package lives in this repo's workspace; the SimpleBGC
protocol is ported to Python; dashboard arrows are hold-to-move; gimbal commands require
the same command lease as driving.

---

## 1. Goal and hard constraints

Add camera pan/tilt/roll control that can be driven from the F710 or the dashboard, without
being able to affect robot motion under any circumstance.

| Constraint | Why | How it is enforced |
|---|---|---|
| **Cannot affect `/cmd_vel`** | The velocity chain is safety-reviewed and commissioned | No gimbal source file may reference `cmd_vel` or `input/*`; asserted by a contract test |
| **Cannot destabilise Nav2** | The Pi is already at its file-descriptor limit with one DDS participant | Separate process, separate launch, opt-in Compose profile, never added to `nav2_container` |
| **Cannot steal the F710 from driving** | The same pad drives the robot | Gimbal uses only axes and buttons the drive teleop leaves free |
| **Fails closed** | A lost link must stop the camera, not freeze the last command | Deadman, joy-loss watchdog, and a driver-side command timeout |
| **Runs without the gimbal attached** | The stack must not degrade when the hardware is absent or unplugged | Driver reports disconnected and holds; nothing else blocks on it |

### Isolation is the whole point

The existing `cmd_vel` chain has exactly one arbiter (`twist_mux`) and one final publisher.
The gimbal deliberately builds a **parallel, self-contained chain** that shares nothing with
it except the `/joy` topic it only ever reads:

```
                    ┌─────────────── robot motion (UNCHANGED) ───────────────┐
/joy ──┬──▶ patrolbot_joy_teleop ──▶ input/joy ──▶ twist_mux ──▶ /cmd_vel ──▶ bridge ──▶ SBC
       │
       └──▶ gimbal_joy_node ──▶ gimbal/input/joy ──┐
                                                    ├──▶ gimbal_mux ──▶ gimbal/cmd ──▶ gimbal_driver ──▶ serial ──▶ SBGC
web bridge ──────────────▶ gimbal/input/web ───────┘
```

`/joy` is the only shared resource, and the gimbal node is a subscriber only. Everything
else lives under the `/gimbal/` namespace.

---

## 2. Package layout

`ros2_ws/src/patrolbot_gimbal/` — a `ament_python` package, matching the other Python
packages in this workspace.

```
patrolbot_gimbal/
├── package.xml
├── setup.py
├── patrolbot_gimbal/
│   ├── sbgc_protocol.py      pure codec — no ROS, no serial
│   ├── sbgc_transport.py     pyserial wrapper, isolated for faking
│   ├── gimbal_policy.py      pure joy/limit/rate logic — no ROS
│   ├── keybinds.py           pure keybind resolution and validation
│   ├── driver_node.py        owns the serial port; the only hardware node
│   ├── joy_node.py           /joy → gimbal/input/joy
│   └── mux_node.py           arbitrates joy over web
├── config/
│   ├── gimbal_params.yaml
│   └── keybinds.default.yaml
├── launch/
│   └── gimbal.launch.py      standalone; NOT included by nav bringup
└── test/
    ├── test_sbgc_protocol.py     vendor hex-dump assertions
    ├── test_gimbal_policy.py     rate mapping, deadzone, limits
    ├── test_keybinds.py          defaults, overrides, conflict detection
    ├── test_isolation_contract.py  the safety guarantee
    └── test_driver_transport.py  driver against a pty fake
```

Interfaces go in the existing `patrolbot_interfaces` package, alongside the current
messages, rather than creating a second interfaces package.

---

## 3. Interfaces

**Messages** (`patrolbot_interfaces/msg/`):

```
GimbalCommand.msg
  uint8 MODE_HOLD=0, MODE_RATE=1, MODE_ANGLE=2
  uint8  mode
  float32 pan          # deg/s in RATE, deg in ANGLE
  float32 tilt
  float32 roll
  bool    valid        # false = treat as hold; the deadman lives here
  string  source       # "joy" | "web" — for audit, never for arbitration

GimbalState.msg
  bool    connected        # serial link up
  bool    motors_on
  float32 imu_pan/tilt/roll        # ground truth from the board
  float32 target_pan/tilt/roll
  float32 pan_min/pan_max          # limits, so the UI can render them
  float32 tilt_min/tilt_max
  float32 roll_min/roll_max
  string  fault
```

**Services:** `/gimbal/set_motors` (`std_srvs/SetBool`), `/gimbal/home`, `/gimbal/level`
(`std_srvs/Trigger`).

**Topics:** all under `/gimbal/` — `input/joy`, `input/web`, `cmd`, `state`. The driver also
contributes a `DiagnosticStatus` to `/diagnostics` under hardware_id `patrolbot/gimbal`,
which is read-only participation in an existing topic and cannot affect motion.

### Why a mux instead of two subscribers

The driver could subscribe to both inputs, but then "who wins" is implicit in message
arrival order. A tiny explicit mux makes precedence a stated property (`joy` outranks
`web`, always) and gives one place to test it. It mirrors `twist_mux` conceptually without
touching it.

---

## 4. Controller mapping

The F710 in XInput mode. Current drive teleop uses **axes 0, 1, 6, 7** and **buttons 0, 1,
2, 3, 5**. The gimbal uses only what is left, so the two never contend:

| Input | Index | Drive use | Gimbal use |
|---|---|---|---|
| Left stick | axes 0, 1 | drive | — |
| D-pad | axes 6, 7 | drive | — |
| RB | button 5 | **drive deadman** | — |
| **Right stick** | axes 3, 4 | free | **pan / tilt** |
| **LT / RT** | axes 2, 5 | free | **roll ccw / cw** |
| **LB** | button 4 | free | **gimbal deadman** |
| Back | button 6 | free | recenter (home) |

Two independent deadmen is the key decision: you can pan while parked, and you can drive
without the camera drifting. Holding both at once drives and looks simultaneously, which is
the actual patrol use case.

Safety behaviour mirrors `patrolbot_joy_teleop` exactly, because that logic is already
commissioned: LB released → one zero-rate command then silence; `/joy` stale for 0.4 s →
treated as deadman release.

---

## 5. TDD plan

Every phase is **write the failing test first, then implement**. Phases 0–2 need neither ROS
nor hardware, so they run in plain `pytest` on any machine.

### Phase 0 — Isolation contract (before any feature code)

`test_isolation_contract.py`, modelled on the existing `test_safety_contract.py`, which
already does static assertions over source and config. Written first so the guarantee
exists before there is anything to violate it:

- no file under `patrolbot_gimbal/` contains `cmd_vel`, `input/joy` (unprefixed),
  `input/navi`, `input/teleop`, or `twist_mux`
- the gimbal is absent from `patrolbot_navigation` bringup and from any lifecycle manager
- every topic name in the package is `/gimbal/`-prefixed or `/diagnostics`
- `gimbal.launch.py` launches no Nav2 or mobile-base node
- the Compose service is behind a non-default profile
- gimbal joy indices do not intersect the drive teleop's indices — *parsed from both source
  files*, so if someone remaps the drive teleop later, this test fails and tells them

That last one is the test I most want. It converts "we chose non-conflicting buttons" from a
comment into an enforced invariant.

### Phase 1 — `sbgc_protocol.py`

Port the byte-verified codec from the standalone C tool. **The tests port first**, including
the four official BaseCam `CMD_CONTROL` hex dumps and the spec's `read Profile2` framing
example, so the Python implementation must reproduce the exact bytes the C one does. Also
carries over: unit conversions (45° → 2048, 5 °/s → 41, 90 °/s → 737), checksum wrapping,
parser resynchronisation after garbage, and rejection of a corrupted body checksum.

This is the lowest-risk phase — the answers are already known and verified.

### Phase 2 — `gimbal_policy.py` and `keybinds.py`

Pure functions, no ROS, mirroring `joystick_policy.py`'s style:

- `stick_to_rate()` — deadzone, scaling, inversion per the axis map
- `clamp_to_limits()` — soft limits with a "was clamped" signal
- `deadman_state()` — hold/release edges and the joy-loss timeout
- `resolve_keybinds()` — defaults merged with user overrides, conflict detection, rejection
  of unknown actions

Tests cover the boring-but-fatal cases: missing axes, an empty `/joy` message, a deadzone at
exactly the boundary, limits with min > max, a keybind bound to two actions.

### Phase 3 — Driver node

The serial layer is faked with a **pty pair**, so tests exercise real reads and writes with
no hardware: open a pty, point the driver at the slave, assert the exact frames appearing on
the master, and feed synthetic `CMD_GET_ANGLES` responses back.

Tests: correct frame for a rate command; **command timeout stops the gimbal** (the
fail-closed guarantee); disconnect mid-session is reported and does not crash the node; a
malformed response is dropped rather than misparsed; soft limits are enforced *server-side*
even when a client asks for something out of range.

That last point matters — a client is not trusted. The dashboard could send anything.

### Phase 4–5 — Joy node and mux

`launch_testing` integration: publish synthetic `sensor_msgs/Joy`, assert what appears on
`gimbal/input/joy`. Explicit test that **no message is ever published on any `cmd_vel`-family
topic**, by subscribing to them and asserting silence.

Mux tests: joy wins while valid; web resumes when joy goes invalid; both silent → hold.

### Phase 6 — Launch and deployment

`gimbal.launch.py` standalone. A separate Compose service on an opt-in profile, matching how
the `web-bridge` service was made optional. Plus the udev rule for a stable device name.

Contract test asserts the profile is not in the default set — the gimbal must never start
with the navigation stack.

---

## 6. Dashboard pipeline

### ROS → server

`patrolbot_web_bridge` gains a `/gimbal/state` subscription and a `gimbal/input/web`
publisher. It already owns the socket to the server, so no new transport.

### Protocol

Extend `shared/schemas/protocol.md`, `server/app/protocol/messages.py`, and
`frontend/src/types/protocol.ts` together:

- `CommandType` gains `"gimbal_rate"`, `"gimbal_angle"`, `"gimbal_home"`, `"gimbal_motors"`
- new telemetry frame `gimbal.state`

### Command authority

Gimbal commands go through the **existing lease** just committed — same authority as
driving, every command audited. Concretely this means `OperatorLease` is consulted before a
gimbal command is accepted, and refusals are audited exactly like navigation refusals.

**A rate command needs a server-side deadman.** A browser can disconnect mid-slew. The
server holds a short expiry (~300 ms) per gimbal rate command; the widget re-sends while a
button is held, and if frames stop arriving the server emits a hold on its own. This is the
same fail-closed shape as the joy watchdog, moved up a layer, and it is why hold-to-move is
safe over a network.

---

## 7. Widget UX

`frontend/src/widgets/GimbalWidget.tsx`, registered in `registry.ts` like the others.

```
┌────────────── Camera ──────────────┐
│                 ▲                  │   ▲ ▼ ◀ ▶  hold to pan/tilt
│            ◀   ⌂   ▶               │   ⌂        recenter (home)
│                 ▼                  │   ↺ ↻      roll
│         ↺              ↻           │
│                                    │
│  pan  −12°  ▓▓▓▓▓▓░░░░░░  tilt −5° │   live angles from /gimbal/state,
│  speed ──────●────── 45 °/s        │   bar shows position within limits
│  ⌨ keybinds        ⏻ motors  ●on   │
└────────────────────────────────────┘
```

Interaction rules:

- **Hold to move, release to stop.** `pointerdown` starts, `pointerup` stops.
- **Safety net for lost pointers.** `pointercancel`, `pointerleave`, window `blur`, and tab
  visibility change all stop motion. A dragged-off or lost pointer event must not leave the
  camera slewing. This is the widget's most important behaviour and gets explicit tests.
- **Disabled without command authority**, with the reason shown — consistent with how
  `NavControlsWidget` already handles the lease, so a control never looks broken.
- **Limits are visible.** The position bar shows where the camera is within its travel, so
  hitting a limit looks intentional rather than like a failure.
- **Live angles** come from `/gimbal/state` — the board's IMU, not dead reckoning.

> **Note on precedent.** `NavControlsWidget` states it deliberately has *no* joystick or
> velocity control, because the robot has no interaction model for it. That reasoning is
> about driving. Aiming a camera is a different risk class: it cannot collide with anything,
> its travel is mechanically bounded, and PTZ hold-to-move is the convention operators
> expect. The prohibition on driving the *base* from the browser stays untouched.

### Keybinds

Defaults ship in `keybinds.default.yaml` and mirror the widget: arrows for pan/tilt, `,`/`.`
for roll, `Home` to recenter, `Space` to stop. Customisation lives in a settings panel;
overrides persist per user. Conflict detection rejects a key already bound elsewhere, and
"reset to defaults" is always one click away.

Keybinds are inert unless the widget has focus, so typing in another field never moves the
camera.

---

## 8. Delivery order

Each phase is independently reviewable, and the stack is deployable after any of them.

| # | Phase | Deliverable | Verified by |
|---|---|---|---|
| 0 | Isolation contract | contract test | passes trivially, then keeps passing |
| 1 | Protocol port | `sbgc_protocol.py` | vendor hex dumps |
| 2 | Pure policy | policy + keybinds | pytest, no ROS |
| 3 | Driver node | serial node | pty fake |
| 4 | Joy node | F710 control | launch_testing + real pad |
| 5 | Mux | arbitration | launch_testing |
| 6 | Launch/deploy | opt-in Compose service | contract test |
| — | **Hardware checkpoint** | **first real motion** | **balance and PID-tune in the GUI first** |
| 7 | Bridge relay | telemetry + command path | bridge tests |
| 8 | Server protocol | lease-gated commands | server tests |
| 9 | Widget | on-screen control | vitest incl. pointer-loss cases |
| 10 | Keybinds UI | customisation | vitest |

Phases 0–6 are in `patrolbot-repo`; 7–10 span both repos.

---

## 9. Risks

| Risk | Mitigation |
|---|---|
| **Gimbal not balanced/tuned** | Nothing in software fixes an oscillating gimbal. GUI tuning is a precondition for the hardware checkpoint, not a step in it |
| `CMD_GET_ANGLES` response layout unverified | Only the request side was byte-verified; the parser validates length and reports a mismatch rather than misparsing. First thing to check against real hardware |
| Browser disconnect mid-slew | Server-side rate expiry (~300 ms) plus the widget's pointer-loss handlers |
| Pi resource pressure | Separate process, opt-in, never in `nav2_container`. Measure fd and CPU at the hardware checkpoint |
| Serial port contention | The SimpleBGC GUI holds the port exclusively; documented, and the driver reports the failure clearly instead of hanging |
| Scope creep into video | The Duo Pro R has **no USB video** — HDMI or analog capture only. Video is explicitly out of scope for this plan |

## 10. Open questions

1. **Does the gimbal share the Pi with the nav stack, or get its own board?** Assumed same
   Pi 5 for now. If fd pressure shows up at the hardware checkpoint, this is the escape.
2. **Two-axis or three?** The plan carries roll throughout. If the mount is 2-axis, roll
   drops out of the widget and the policy, which simplifies things.
3. **Should `/gimbal/state` be recorded in the dashboard's telemetry history**, or is live-only
   enough? Affects the DB schema, so worth deciding before phase 8.
