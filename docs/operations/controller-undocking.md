---
title: Controller manual undocking
description: Start-button charger departure without AMCL or Nav2 readiness.
---

# Controller manual undocking

**Development implementation, not a deployed or physically commissioned release.**
The dashboard's existing guarded Undock action is unchanged.

## Controls and sequence

On the Logitech F710 in **XInput** mode:

1. Release every button, center both sticks and the D-pad, and release both triggers.
2. While stationary and confirmed docked/charging, **hold Start for one second**.
   RB must remain released. Holding Start never repeats the operation.
3. The SBC disables motors for charger release, observes release, enables motors,
   reverses **0.53 m at 0.10 m/s**, stops and verifies physical clearance, then
   turns **180° counterclockwise** using encoder heading. The turn uses at most
   **0.70 rad/s**, with the existing final-smoother angular acceleration/braking
   bounds (1.5/2.0 rad/s²), a 20 s turn deadline and measured-yaw completion.
4. Success leaves the robot stopped with motors enabled. Release all controls,
   then make a **new RB press** to drive manually. No navigation goal resumes,
   and no old localization seed is restored.

After releasing the initiating Start hold, **press Start again to cancel**.
Releasing the initial hold does not cancel. Controller disconnection by itself
does not cancel this automatic operation. Reconnecting with Start held is not a
fresh cancellation press; release it, then press again. The physical E-stop and
existing explicit motor-disable operation remain available.

Ordinary joystick input and the LiDAR-power gesture are suppressed during the
operation. Attempts while another undock, tuning, navigation, hardware-control or
in-flight/unknown map/lifecycle mutation owns the robot are refused, not queued.

## Independence and safeguards

The sequence requires **neither AMCL, a selected map, map→odom, the Nav2 behavior
server, nor a Nav2 readiness/idle service**. It does require the real SBC link,
known encoder origin, fresh finite hardware/odometry and scanner evidence, and a
commissioned undock hardware profile. Missing sensor data is never treated as clear.

The existing release, reverse-distance, sonar, E-stop, fault, stall, watchdog,
timeout and clearance checks remain. The independent SBC command-lease watchdog
also supervises the new turn. A missing hardware link or frozen control loop still
stops/disables; this is not a bumper/E-stop-only bypass.

The new turn retains the existing front laser rotation stop zone and point count
and supervises rear sonar locally on the SBC. It does not need a costmap. This does
not add sensors, repair the documented rear-right sonar coverage gap or prove
collision avoidance in unobserved regions.

An initially dock-held front bumper panel is tolerated only until that individual
panel releases. A newly pressed panel stops the manual operation. During a turn,
either bumper group stops it.

### Motor-on recovery is conditional

After **proven full separation**, a confirmed recoverable bumper/obstacle stop or
Start cancellation leaves already-enabled motors on if hardware conditions permit.
The robot holds zero velocity until controls are released and RB is pressed again.
An asserted front bumper blocks forward/turn commands; an asserted rear bumper
blocks reverse/turn commands. Manual escape is only away from the pressed group.

The system **never re-enables disabled motors merely to provide recovery**.
E-stop, critical faults, wheel stalls, stale/invalid hardware, uncertain outcomes,
or failure/cancellation **before full dock clearance is proven** keep fail-safe
motor handling. Partial departure is not permission to drive against possibly
engaged charging plates. Inspect and use the existing explicit recovery workflow;
do not blindly repeat Start or clear an unknown result.

## Implementation and deployment boundary

- `manual_undock_policy.py`: release-armed, timestamp-checked Start gesture.
- `manual_undock.py`: bridge-owned exclusive admission, one-shot dispatch,
  cancellation, terminal confirmation, input replay barrier and unknown-result hold.
- `manual_telemetry.py`: raw encoder/scanner evidence independent of AMCL publication
  ordering. It never publishes a pose/TF or acknowledges an AMCL reset.
- SBC protocol: `MANUAL_UNDOCK_V1` capability and fixed-profile `MANUAL_UNDOCK`
  request; existing `UNDOCK_EVENT`, `UNDOCK_RESULT` and `UNDOCK_CANCEL` tracking.
  Normal `UNDOCK` remains reverse-only and its dashboard turn remains in Nav2.
- `manual_control.launch.py`: joystick, manual teleop, feedback and watchdog now
  run in **bringup**, outside the Nav2 container's crash/restart-coupled launch.
  Standard Compose input-device access, status and healthchecks move with them.

This needs a **matched SBC server and Pi image/launch/Compose update**. A bridge
connected to an older SBC refuses Start rather than falling back to a less guarded
command. Update/recreate roles together to avoid duplicate `joy_node` publishers;
an old navigation container plus new bringup would be an invalid mixed deployment.
There is no new ROS message/action definition or dashboard protocol change.

Local ROS graph provenance checks require a stable sole expected publisher and
fresh source timestamps. Jazzy dictionary metadata is supported; newer per-sample
publisher IDs are checked when supplied. This is **not cryptographic authorization**:
untrusted DDS participants require the existing ROS access/network policy.

## Verification boundary

**Recovery checkpoint (2026-10-26):** the original uncommitted implementation was
restored from the retained OpenCode snapshot into a durable feature worktree.
The recovered snapshot tree was `e9a6d8d1dda78225c94c3a63b42d82f0dffad192`, based
on robot main `f710cc8`. Portable CI now includes the new manual-input tests and
SBC controller/protocol checks. The launch-source inventory follows the moved
manual-control nodes. Recovery and merge are not a robot deployment or physical
acceptance claim; a matched SBC/Pi update is still required as described above.

Portable tests exercise Start arming/hold/cancel/reconnect, raw evidence freshness
and origin continuity, resource admission, single dispatch, no retries, retained
unknown ownership, and fresh manual handoff. C++ fake-base tests exercise release,
reverse, the encoder half-turn, obstacle/bumper stops, fault paths and unchanged
normal-undock zero-angular output. An isolated ROS/DDS + loopback scripted-SBC
test exercises the real bridge without AMCL/Nav2 processes. A scripted SBC is not
a native ARIA server or a physical robot.

Local checkpoint (2026-10-24): **278 targeted Python checks passed**, **1,135
manual-controller C++ assertions passed**, and **116 protocol assertions passed**.
The four affected ROS packages build in an isolated network-none container;
the composed bringup launch parses and the real DDS/loopback test passes without
AMCL/Nav2 nodes. New Python modules/tests pass the project's flake8 rule set;
standard Compose configuration and whitespace validation pass.

A broader ROS-container run passed 918 tests with 42 opt-in skips, but its old
pytest invoked two unchanged tuning helper fixtures as xunit setup (27 setup
errors). Those 27 tests pass under the workstation pytest and are included in
the targeted count above. The separate contracts run was blocked by missing
`jsonschema` in that test environment; no contracts were changed.

The unchanged `main` C++ suite also has existing lateral-drift and post-release
yaw test failures (six assertions across two tests). They were reproduced against
the unchanged baseline; this feature must not be presented as fixing them. The
manual variant additionally checks reverse drift/yaw itself.

Before release: compile the complete SBC against the installed ARIA API, validate
the matched native Pi/Compose launch with no duplicate publishers, review the
fault/recovery boundary, and separately authorize supervised physical release,
clearance, turn, bumper/E-stop, cancellation and manual-handoff acceptance.
No live deployment, motion, motor, pose or undock operation is authorized by this page.
