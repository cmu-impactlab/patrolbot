---
title: Coding Standards
description: The conventions PatrolBot's code follows — ament linters, ROS 2 idioms, the defensive parsing style in the bridge, and the documentation-as-you-go expectation.
---

# Coding Standards

PatrolBot is a small codebase maintained by a small team. The standards below are partly enforced
by tooling (ament linters) and partly conventions visible in the existing code. New code should
read like the code around it.

## Language and framework conventions

| Area | Convention |
|---|---|
| **Pi nodes** | Python (`rclpy`) for the bridge, teleop, and lifecycle manager; C++ (`rclcpp`) only in the legacy `rosaria2` |
| **SBC** | C++ against ARIA; built with `make` |
| **ROS 2 version** | Jazzy idioms (composable nodes, lifecycle, `geometry_msgs/Twist` velocity) |
| **Frames** | `map` → `odom` → `base_link` → `laser_frame`; never invent parallel frame names |
| **Topics** | keep the documented [topic graph](../ros2/topics.md); the `cmd_vel` chain naming is load-bearing |

## ament linters (the enforced part)

The `ament_python` packages ship the standard test templates
(`test_copyright.py`, `test_flake8.py`, `test_pep257.py`) and the `ament_cmake` packages use
`ament_lint_auto` + `ament_lint_common`. So:

- **Python:** PEP 8 (flake8) and PEP 257 docstrings.
- **C++:** the ament lint set (uncrustify-style formatting, cpplint).
- Run them with `colcon test` — see [Testing](testing.md).

## Defensive-parsing style (the bridge)

The bridge sets the tone for how PatrolBot handles untrusted input from the wire. Follow it when
touching any parsing path:

- **Never let a bad line crash the node.** Every parse path is wrapped so a malformed message is
  skipped, not fatal.
- **Isolate independent data.** Navigation (`ODOM|LASER`) and auxiliary (`AUX`) data are parsed
  independently; a sonar/battery glitch must never affect `/odom` or `/scan`.
- **Decouple timing from delivery.** TF is published on its own 50 Hz timer, not when a scan
  arrives, so consumers always have a transform buffered first.

```python
# The house style: guard, skip, keep running.
if 'SONAR' in sections:
    try:
        self._publish_sonar(sections['SONAR'], stamp)
    except Exception:
        pass   # a bad SONAR section drops only /sonar
```

## Configuration and "magic numbers"

- Robot-specific tunables live in **config**, not code: `nav2_params.yaml`, `mux.yaml`,
  `smoother.yaml`. Prefer adding a parameter to hard-coding.
- Where a constant *is* in code (e.g. the bridge's `RECV_TIMEOUT`, `SCAN_RANGE_MIN`, SBC endpoint),
  give it a name and a comment explaining the *why*, as the existing code does.
- Any value with a non-obvious reason (e.g. `base_shift_correction: False`, `bond_timeout: 0.0`,
  `MAGICK_THREAD_LIMIT=1`) **must** carry a comment explaining the failure it prevents. These
  comments are the difference between "looks removable" and "load-bearing."

## Comment the failure modes, not the obvious

The existing config files are heavily commented where a setting prevents a specific failure
(costmap timeout, container SIGABRT, bond starvation, OOM). Keep this up: a future maintainer who
deletes `base_shift_correction: False` because it "looks like a default" will reintroduce a crash.

## Keep written docs in sync with the live system

A recurring problem in this project is **docs drifting from the running code** (the laser TF, the
map resolution, the launch mode). When you change a structural fact:

1. Update the relevant package `README.md`.
2. Update the architecture knowledge base (`SKILLS/*.md`) if you maintain it.
3. Update this site under [`docs/`](../index.md).

The [Known Gaps](../known-gaps.md) page exists largely because of past drift — don't add to it.

## Git hygiene

- `patrolbot_navigation/` and `rosaria2/` have their own `.git/` — commit changes in the correct
  repository.
- Remove editor temp files (`*~`, `#file#`) and dead experiments rather than committing them;
  several are already flagged as [legacy/dead](../internals/legacy-components.md).
