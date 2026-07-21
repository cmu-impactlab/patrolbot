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
  [package page](../packages/rosaria2.md).
- **`lms200_sanitizer.py`** — fixes SICK LMS-200 header fields (`/bad_scan → /good_scan`); no
  `/bad_scan` publisher exists in the current stack, so it has nothing to consume.
- **`patrolbot_navigation/scripts/twist_mux.yaml`** — a reference velocity-priority config; the
  active twist_mux runs from `patrolbot-launch`'s `mux.yaml`.
- **`patrolbot_nav.rviz`** — operator RViz layout for manual visualization.
- **`maps/cmuq_1st_floor.{yaml,pgm}`** — an older CMU-Q floor map, not loaded by the active launch.

## Removed dead code { #known-dead-code--cleanup-candidates }

Dependency review removed the empty component container, obsolete
`readlidar.py`, the unused `ekf_test.yaml` reference, and the unreferenced
`rosaria2`, duplicate joystick/keyboard, obsolete smoother, and standalone
LiDAR launch experiments. Git history and verified deployment backups provide
rollback; none of these files remains installed in the production image.

## Why this page exists

Documentation that treats dead code as load-bearing is worse than no documentation — a maintainer
who can't tell the live system from the cruft will hesitate to delete safely, or worse, "fix"
something that isn't running. Concentrating all of it here keeps the rest of the site describing
**only** what actually runs.

If you *promote* a "Maybe" component to active, give it real documentation in
the appropriate section and remove its line here.
