# First CI remediation tranche — 2026-11-20

This candidate starts from fixes-only local main `4d28339`. It does not merge
the separate bloat cleanup or enable the deferred Save/Apply commissioning
feature. No production runtime behavior is changed.

## Changes

- The ROS test recipe supplies the Nav2 patch files read by selected tests.
- Rosdep input includes navigation and its local docking-plugin dependency.
  The plugin remains a source fixture, not a newly built/launched package.
- The test image installs pytest-timeout for the suite's existing timeout
  configuration.
- Four bridge test factories use explicit `make_*` names instead of the
  deprecated nose-compatible `setup` hook name. Assertions are unchanged;
  nose compatibility is not globally disabled.
- Seven portable regressions check materialized recipe inputs, dependencies,
  build/test selection, Docker-context allowance and helper names.

## Validation

- Full portable runner: **1,300 Python tests and six subtests passed**.
- Patch/header artifact validation passed.
- All five ARIA-free C++ suites passed, including **13,084 undock checks**.
- **42 targeted native-environment tests passed** under Ubuntu Python 3.12 and
  pytest 7.4.4 with default nose discovery. They used the recipe's materialized
  fixture layout and a cached ROS toolchain, with pytest-timeout installed from
  a cached wheel into a disposable target directory.
- Independent local Codex review found no actionable issues.

Evidence is retained in `/tmp/opencode/ci-fixes/`, including
`core-portable-tests.log`, `core-native-focused.log` and `review-core-result.md`.

## Limits

Docker external DNS is unavailable. No clean apt/rosdep rebuild of the revised
ROS image was established; cached-toolchain tests are not that evidence.
Complete native lint, new hosted native gates, production-image acceptance,
patch provenance, target architectures and hardware commissioning remain outside
this tranche. No hosted run, main merge, push, robot connection or deployment
was performed.

`rosaria2` remains source-only/nondeployed, automatic dock-in remains parked, and
no gimbal implementation is restored.
