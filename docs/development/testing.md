---
title: Testing
description: How PatrolBot is tested today — ament lint tests and the manual freeze/resume resilience tests that validated the self-healing.
---

# Testing

PatrolBot's testing is pragmatic: ament lint tests and a set of **manual
freeze/resume resilience tests** that are the real validation of the system's
fault tolerance. There is no automated integration suite or checked-in SBC
simulator today; this page documents what exists and how to run it.

## Unit / lint tests

Each package carries the standard ament test templates:

```bash
cd ~/ros2_ws
colcon test --packages-select patrolbot_bridge patrolbot-launch
colcon test-result --verbose
```

| Package | Tests |
|---|---|
| `patrolbot_bridge` | `test_control_protocol.py`, `test_undock_protocol.py`, `test_safety_telemetry.py`, `test_fake_sbc_integration.py`, plus lint |
| `patrolbot_dock_manager` | `test_contracts.py`, `test_manager_integration.py`, plus lint |
| `patrolbot_navigation` | `test_safety_contract.py`, `test_safety_policy.py`, `test_joystick_policy.py`, `test_physical_calibration.py`, `test_collision_monitor_integration.py` |
| `patrolbot_docking_plugins` | `test_charge_state.cpp` (gtest) |
| `patrolbot_gimbal` | seven behaviour suites incl. `test_isolation_contract.py` |
| `patrolbot-launch` | `test_copyright.py`, `test_flake8.py`, `test_pep257.py` |
| `rosaria2` | `ament_lint_auto` / `ament_lint_common` |

The two integration suites are skipped unless `PATROLBOT_RUN_INTEGRATION=1` is set, because they
create real rclpy nodes and must run in an isolated ROS domain:

```bash
PATROLBOT_RUN_INTEGRATION=1 colcon test \
  --packages-select patrolbot_bridge patrolbot_dock_manager
```

## SBC server tests

The SBC has its own suites, and **neither links ARIA**, so both build and run on a workstation:

```bash
cd hardware/patrolbot_hw_server
make -B test        # runs tests/protocol_test and tests/undock_test
```

`tests/undock_test.cpp` drives the dock observer and the undock controller through scripted
hardware snapshots — a simulated base with configurable charge-release latency, charge
re-assertion, wheel lock, motor drop-out, rear obstacles and bumper events. That covers the
release-retry loop, no-motion detection, projected-displacement completion, cancellation,
duplicate request IDs, the redock inhibition and restart behaviour without a robot.

Because the pure modules are 32-bit-clean C++11, verify them against the target toolchain before
deploying — ideally on the SBC itself, which is the only place g++ 5.4 exists:

```bash
g++ -m32 -Wall -Wextra -Wpedantic -O2 -std=c++11 -c protocol.cpp -o /tmp/protocol.o
g++ -m32 -Wall -Wextra -Wpedantic -O2 -std=c++11 -c undock.cpp   -o /tmp/undock.o
```

## Resilience tests (the important ones)

The self-healing behavior was validated by deliberately breaking the link on the live robot and
confirming automatic recovery. These are the canonical acceptance tests for any change near the
seam; re-run them after touching the bridge, the SBC server, or the lifecycle/launch logic.

| Scenario | How | Expected |
|---|---|---|
| **SBC freeze** | `SIGSTOP` the SBC server | Bridge logs "SBC telemetry timed out. Reconnecting…" within 3 s; on `SIGCONT`, `/odom` `/scan` resume at ~25 Hz |
| **Pi/bridge freeze** | `SIGSTOP` the bridge | SBC logs "Pi unresponsive (send stalled)…"; on resume, server re-accepts |
| **Reconnect TF skew** | repeat a ~12 s bridge freeze | Nav stays active, `NRestarts` stays 0, only graceful "Message Filter dropping message" INFO logs (this is what `base_shift_correction: False` fixed) |
| **Nav node/container crash** | kill `nav2_container` | Launch tears down → Docker restarts the Pi 5 service container with a fresh launch |
| **Service autostart** | reboot Pi 5 / SBC | Docker and the lingered SBC service restore the runtime |

```bash
# Observe during a test
ssh robot-pi2 'cd /home/ubuntu/patrolbot-repo && ./docker/status.sh'
ssh robot-pi2 'docker logs --tail 100 patrolbot-navigation'
```

## What is **not** covered

- No automated end-to-end navigation test (plan → follow → reach goal). Validate goals manually in
  RViz against the active map.
- No automated end-to-end navigation test on hardware; validate goals manually in RViz.
- The undock hardware profile (rear-sonar indices, E-stop semantics, fault masks) cannot be
  measured offline. Until `patrolbot_commission_probe` supplies them, a production undock is
  refused by design.
- Encoder fidelity on the charging plates, and the causal ordering of motor stopping during a
  charge re-latch, need a supervised physical session.

## Suggested additions (future)

If you invest in test infrastructure, the highest-value targets, in order:

1. A pytest suite around the bridge's `_parse_telemetry` / `_parse_aux` (pure functions over
   strings — easy to unit test, high payoff given they handle untrusted wire input).
2. A checked-in mock SBC plus a launch-testing smoke test that asserts `/odom`,
   `/scan`, and `/cmd_vel` flow.
3. A scripted version of the freeze/resume resilience matrix.

See [Profiling](profiling.md) for performance measurement and [Debugging](debugging.md) for the
diagnostic tooling these tests rely on.
