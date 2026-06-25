---
title: Testing
description: How PatrolBot is tested today — ament lint tests, the mock SBC server and laser probe, and the manual freeze/resume resilience tests that validated the self-healing.
---

# Testing

PatrolBot's testing is pragmatic: ament lint tests in CI-style runs, two dev tools for exercising
the bridge without a robot, and a set of **manual freeze/resume resilience tests** that are the
real validation of the system's fault tolerance. There is no large automated integration suite —
this page documents what exists and how to run it.

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

## Testing the bridge without a robot

The Pi carries two dev tools under `~/yousef/` (dev tools — **not** production):

| Tool | Purpose |
|---|---|
| `mock_sbc_server.py` | Emulates the SBC's TCP server on :7272 — streams `ODOM|LASER`/`AUX`, accepts `DRIVE` |
| `test_laser.py` | Raw serial probe for the SICK LMS-200 |

```bash
# Terminal 1 — fake SBC
python3 ~/yousef/mock_sbc_server.py
# Terminal 2 — bridge against it
ros2 run patrolbot_bridge bridge_node
# Terminal 3 — verify
ros2 topic hz /odom /scan
ros2 topic echo /diagnostics
```

This lets you validate parsing changes, new `AUX` fields, and reconnect behavior on a desk.

## Resilience tests (the important ones)

The self-healing behavior was validated by deliberately breaking the link on the live robot and
confirming automatic recovery. These are the canonical acceptance tests for any change near the
seam; re-run them after touching the bridge, the SBC server, or the lifecycle/launch logic.

| Scenario | How | Expected |
|---|---|---|
| **SBC freeze** | `SIGSTOP` the SBC server | Bridge logs "SBC telemetry timed out. Reconnecting…" within 3 s; on `SIGCONT`, `/odom` `/scan` resume at ~25 Hz |
| **Pi/bridge freeze** | `SIGSTOP` the bridge | SBC logs "Pi unresponsive (send stalled)…"; on resume, server re-accepts |
| **Reconnect TF skew** | repeat a ~12 s bridge freeze | Nav stays active, `NRestarts` stays 0, only graceful "Message Filter dropping message" INFO logs (this is what `base_shift_correction: False` fixed) |
| **Nav node/container crash** | kill `nav2_container` | Launch tears down → systemd restarts → localization + map back in ~4 s |
| **Service autostart** | reboot Pi / SBC | all services come back (linger enabled) |

```bash
# Observe during a test
ssh ubuntu@patrolbot-ros.qatar.cmu.edu ./patrolbot-logs.sh           # all three services
ssh ubuntu@patrolbot-ros.qatar.cmu.edu ./patrolbot-logs.sh status    # health + recent errors
```

## What is **not** covered

- No automated end-to-end navigation test (plan → follow → reach goal). Validate goals manually in
  RViz against the active map.
- No simulation harness. Testing is on hardware or against the mock SBC.
- The SBC server has no unit tests; it is validated by the resilience scenarios above.

## Suggested additions (future)

If you invest in test infrastructure, the highest-value targets, in order:

1. A pytest suite around the bridge's `_parse_telemetry` / `_parse_aux` (pure functions over
   strings — easy to unit test, high payoff given they handle untrusted wire input).
2. A launch_testing smoke test that brings up the stack against `mock_sbc_server.py` and asserts
   `/odom`, `/scan`, and `/cmd_vel` flow.
3. A scripted version of the freeze/resume resilience matrix.

See [Profiling](profiling.md) for performance measurement and [Debugging](debugging.md) for the
diagnostic tooling these tests rely on.
