---
title: Testing
description: Offline Python and C++ regressions, separate native tests, and supervised hardware acceptance.
---

# Testing

Portable regression checks, native ROS/fake-SBC integration and supervised
hardware acceptance are separate evidence levels. Passing offline tests does
not establish that a feature is deployed or commissioned.

## Portable regression runner

In an isolated development environment, install `contracts/requirements-test.txt`,
then run:

```bash
python -B docker/run-portable-review-tests.py
```

This is the offline CI entry point. It covers contracts, portable bridge/tuning,
navigation safety, dock-manager policy, Docker helper units, reset/preflight
helpers, localization diagnostics, status-report
and patch-artifact checks, then all five ARIA-free hardware C++ binaries.
Separate pytest processes isolate test-local ROS stubs; the runner places its
own checkout first on `PYTHONPATH`.

`--list` prints the selection without running it. Native ROS message/runtime
tests, opt-in live-stack/Docker integration (including deployed-odometry tests)
and ament lint templates are explicitly excluded **before collection**, even if
ROS is installed. The runner disables inherited integration opt-ins; it does not
start a ROS domain, container, robot connection or deployment. C++ failures stay
failures, and each built binary runs independently so one failure cannot hide
later checks. No safety expectation is relaxed to make a baseline green.

The broader native and fake-SBC suites remain separate gates.

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
| `patrolbot-launch` | `test_copyright.py`, `test_flake8.py`, `test_pep257.py` |
| `rosaria2` | `ament_lint_auto` / `ament_lint_common` |

The fake-SBC and dock-manager integration suites are skipped unless `PATROLBOT_RUN_INTEGRATION=1` is set, because they
create real rclpy nodes and must run in an isolated ROS domain:

```bash
PATROLBOT_RUN_INTEGRATION=1 colcon test \
  --packages-select patrolbot_bridge patrolbot_dock_manager
```

## SBC server tests

The SBC has five suites; **none links ARIA**, so they build and run on a workstation:

```bash
cd hardware/patrolbot_hw_server
make -B test        # protocol, undock, scan timing, LiDAR power, encoder capture
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

## Supervised resilience acceptance

Historical self-healing checks deliberately broke the link on the live robot
and observed recovery. They are not acceptance evidence for the current checkout.
Re-running this matrix requires a separately authorized, supervised hardware
session after changes near the bridge/SBC/lifecycle seam; the offline runner
does not execute any of it.

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

- Portable units do not replace the separate native closed-loop/fake-hardware
  navigation tests or supervised hardware navigation acceptance.
- The undock hardware profile (rear-sonar indices, E-stop semantics, fault masks) cannot be
  measured offline. Until `patrolbot_commission_probe` supplies them, a production undock is
  refused by design.
- Encoder fidelity on the charging plates, and the causal ordering of motor stopping during a
  charge re-latch, need a supervised physical session.

## Suggested additions (future)

If you invest in test infrastructure, the highest-value targets, in order:

1. Extend existing protocol/odometry tests with additional malformed-input cases.
2. Add supervised evidence capture for the freeze/resume resilience matrix.

See [Profiling](profiling.md) for performance measurement and [Debugging](debugging.md) for the
diagnostic tooling these tests rely on.
