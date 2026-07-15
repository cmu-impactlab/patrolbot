---
title: Known Gaps
description: Current open documentation and code-hygiene items for PatrolBot, updated 2026-07-16.
---

# Known Gaps

This page lists only current open work as of **2026-07-16**. Verified runtime facts
belong in the relevant architecture, deployment, and reference pages.

## Code Hygiene

Minor cleanup items from source review:

- **Package metadata/dependencies.** `patrolbot_bridge` and `patrolbot-launch`
  still carry scaffold-default TODO fields in `setup.py`; `rosaria2/package.xml`
  has placeholder metadata. The bridge manifest omits `nav_msgs`, and the launch
  manifest does not declare all of its runtime dependencies.
- **Dead `ekf_config_file` reference.** `bringup.launch.py` defines an unused path to
  `ekf_test.yaml`.
- **Dead launch and experimental files** remain in `patrolbot-launch/launch/`; see
  [Legacy Components](internals/legacy-components.md#known-dead-code--cleanup-candidates).
