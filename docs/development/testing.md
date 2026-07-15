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
| `patrolbot_bridge` | `test_copyright.py`, `test_flake8.py`, `test_pep257.py` |
| `patrolbot-launch` | `test_copyright.py`, `test_flake8.py`, `test_pep257.py` |
| `patrolbot_navigation` | `ament_lint_auto` / `ament_lint_common` |
| `rosaria2` | `ament_lint_auto` / `ament_lint_common` |

These are **lint/style** checks, not behavior tests. Treat them as a formatting gate.

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
- No simulation harness or checked-in mock SBC. Integration testing is currently on hardware.
- The SBC server has no unit tests; it is validated by the resilience scenarios above.

## Suggested additions (future)

If you invest in test infrastructure, the highest-value targets, in order:

1. A pytest suite around the bridge's `_parse_telemetry` / `_parse_aux` (pure functions over
   strings — easy to unit test, high payoff given they handle untrusted wire input).
2. A checked-in mock SBC plus a launch-testing smoke test that asserts `/odom`,
   `/scan`, and `/cmd_vel` flow.
3. A scripted version of the freeze/resume resilience matrix.

See [Profiling](profiling.md) for performance measurement and [Debugging](debugging.md) for the
diagnostic tooling these tests rely on.
