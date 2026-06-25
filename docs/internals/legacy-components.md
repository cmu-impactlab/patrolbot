---
title: Legacy Components
description: The single appendix for PatrolBot's "Maybe" components (kept but inactive) and "Known Dead Code" (cleanup candidates). Deliberately one line each.
---

# Legacy Components

This is the **one** place PatrolBot's inactive components are written up. Per the project's
documentation rule, "Maybe" components get a single line here and nowhere else, and "Not Used"
components are listed once as cleanup candidates — they are never described elsewhere as if they are
part of the live system. The full classification table is on
[Component Breakdown](component-breakdown.md).

## "Maybe" — kept, not active

Real components retained as fallbacks or references, but not part of the running stack:

- **`rosaria2` package** — superseded direct-ARIA driver for the Pioneer base; kept as a fallback
  to the SBC-server-plus-bridge architecture. Conflicts with the bridge if run; full detail on its
  [package page](../packages/rosaria2.md). (11 files: `rosaria2_node.cpp`, `laser_publisher.cpp`,
  `test.cpp`, four headers, `BumperState.msg`, `CMakeLists.txt`, `package.xml`.)
- **`lms200_sanitizer.py`** — fixes SICK LMS-200 header fields (`/bad_scan → /good_scan`); no
  `/bad_scan` publisher exists in the current stack, so it has nothing to consume.
- **`patrolbot_navigation/scripts/twist_mux.yaml`** — a reference velocity-priority config; the
  active twist_mux runs from `patrolbot-launch`'s `mux.yaml`.
- **`patrolbot_nav.rviz`** — operator RViz layout for manual visualization.
- **`maps/cmuq_1st_floor.{yaml,pgm}`** — an older CMU-Q floor map, not loaded by the active launch.
- **`second_map_original_0.1.{pgm,yaml}.bak`** — pre-downsample map backups, retained as the revert
  path for the 0.2 → 0.1 m/px change.
- **`ARIA/maps/`, `ARIA/bin/`** — ARIA reference map data and utility binaries.

## Known Dead Code — cleanup candidates { #known-dead-code--cleanup-candidates }

Listed once, here, as removal candidates. Not documented anywhere else:

- **`patrolbot-launch/launch/rosaria2.xml`, `joy.xml`** — commented out of `bringup.xml`.
- **`patrolbot-launch/launch/#joy.xml#`, `*.xml~`, `readlidar.py~`** — editor temp/backup files
  accidentally retained.
- **`patrolbot-launch/launch/teleop-key*.xml`, `rosaria2.py`, `readlidar.py`,
  `launch_teleop_keyboard.bash`, `Aria.log`** — dev experiments and stray runtime artifacts.
- **`ekf_test.yaml` reference in `bringup.launch.py`** — the `ekf_config_file` variable is defined
  but the file was already removed and is never used in the launch body; safe to delete the line.
- **`patrolbot_navigation/log/`** — a stale colcon build log committed into the package.

## Why this page exists

Documentation that treats dead code as load-bearing is worse than no documentation — a maintainer
who can't tell the live system from the cruft will hesitate to delete safely, or worse, "fix"
something that isn't running. Concentrating all of it here keeps the rest of the site describing
**only** what actually runs.

If you remove any of the dead code above, also update
[Component Breakdown](component-breakdown.md). If you *promote* a "Maybe" component to active, give
it real documentation in the appropriate section and remove its line here.
