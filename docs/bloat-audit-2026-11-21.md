# Narrow core bloat cleanup — 2026-11-21

Local candidate based on `771c5294961eb09783b656764b6d5cea4b2e552f`.
This records implementation and local evidence, not approval or native acceptance.

## Audited scope and evidence

Read `/tmp/opencode/bloat-cycle/core-audit.md` and the parked proposal
`git diff e5fee26^..8b9ab66`. The proposal was reference only. Searched repository
references before removing helpers and traced the affected packaging, launch,
parser, mux and map-selection paths. This is a focused cleanup, not a new audit
of every file or proof that arbitrary external consumers do not exist.

- Navigation launches `joy` with `patrolbot_joy_teleop.py`, not
  `teleop_twist_joy`; its map/localization path uses the catalog and AMCL.
  The removed direct SLAM/teleop dependencies had no launch consumers.
- `patrolbot_navigation/scripts/twist_mux.yaml` explicitly said
  **LEGACY / NOT LOADED**. `patrolbot-launch/launch/mobile_base.xml` loads
  `param/defaults/mux.yaml`; `config/nav2_params.yaml` defines the separate
  active `nav_command_mux`.
- `_digest_for`, `_revision_generated_digest` and the service-local
  `_reject_constant`/`_no_duplicates` had definitions only. The service's
  `strict_json_loads` delegates to `tuning_json.loads`. The contract validator's
  separate, active `_reject_constant` remains untouched.
- `undock_turn.py` creates the `/input/docking` publisher, forwards owned
  behavior commands there, and forwards collision-filtered output through
  `/input/undock_turn`. Automatic dock-in remains parked and unlaunched.
- `bringup.launch.py` reads `MapCatalog` and `ActiveMapStore`; Floor 1 and
  Floor 2 remain selectable catalog entries, with Floor 2 the default.

## Actual changes

- Removed direct `slam_toolbox` and `teleop_twist_joy` dependency declarations
  from navigation and Docker install lists, and updated active setup docs.
  Kept `joy`, `twist_mux`, custom joystick and all other runtime packages.
  No image-size or transitive-package-removal claim is made.
- Deleted only the legacy navigation `scripts/twist_mux.yaml` and its active
  documentation references. Kept both running mux configurations unchanged in
  value, including docking and guarded-turn inputs.
- Removed only the four private helpers above. No executable parser,
  persistence, safety or activation behavior was changed.
- Corrected docking publisher claims across architecture, safety, topic,
  parameter, limitation and component docs and the stale YAML comment.
  Corrected the navigation README's Floor 1 status and joystick output topic.
- Added regressions for retained public exports and `plan=` injection,
  `require_initialized` readiness/refusal behavior, shared strict parsing,
  private helper absence, retained sanitizer packaging, dependency removal,
  custom joystick launch and both active mux/docking paths. Existing strict
  JSON rejection and operation-plan allowlist tests remain.

## Retained and deferred

`FixedOperationPlan`, `HOST_SERVICES`, `__all__`, coordinator `plan=` injection,
`RevisionStore.require_initialized` and shipped `lms200_sanitizer.py` are retained
for compatibility. Local zero consumers do not authorize external breakage.

Contracts, canonical maps and map validator, safety guards/timeouts/footprints,
authorization, epoch/origin/receipt identities and outcomes, persistence checks,
Save/Apply deferred branches and native patches are unchanged. Retained:
source-only nondeployed `rosaria2`, the real SBC ARIA server, automatic dock-in
prototype without launching it, public/manual recovery and diagnostics,
optional launch alternatives, historical evidence and Docker overlays.
Gimbal stays removed. CI/protection fields are unchanged. No robot data,
recordings, volumes or credentials were touched.

## Local verification

- Focused run: **138 passed**, covering tuning activation, store, service,
  ROS boundary parser checks and navigation safety/source contracts.
  Full output: `/tmp/opencode/bloat-cycle/core-focused.log`.
- Read-only scope self-check passed: bridge ASTs differ only by the four
  authorized private removals; navigation YAML values are identical to base;
  shipped sanitizer bytes are identical; protected path diff exclusions pass.
  Output: `/tmp/opencode/bloat-cycle/core-scope-review.log`.
- `git diff --check` passed.

Commands use `PYTHONDONTWRITEBYTECODE=1`, Python `-B` and pytest
`-p no:cacheprovider`. Focused tests used
`/home/yousef/Projects/repos/patrolbot-repo/.venv/bin/python` with the current
checkout's bridge source on `PYTHONPATH`. The isolated
`/tmp/opencode/bloat-cycle/venv/bin/python` initially lacked `pytest`; a later
check confirmed `pytest`, `yaml` and `jsonschema` were available after parent
preparation. This worker installed nothing into either environment.

First full runner command:

```sh
PYTHONDONTWRITEBYTECODE=1 /home/yousef/Projects/repos/patrolbot-repo/.venv/bin/python -B docker/run-portable-review-tests.py
```

Exit **1**, solely because contract collection lacked `jsonschema`; no contract
tests ran in that invocation. Other groups passed: bridge **646** (+4 subtests),
navigation **207**, dock manager **112**, Docker tests **258**, system reset
**25**, localization diagnostics **24** (+2 subtests), Docker root **5**.
Patch artifact verification passed. All five offline ARIA-free C++ binaries
built and exited successfully: protocol **116 checks**, undock **13,084 checks**,
scan timing **12 checks / 0 failures**, LiDAR power checks and encoder capture.
Full output: `/tmp/opencode/bloat-cycle/core-portable.log`.

Complete runner rerun with the now-ready isolated environment:

```sh
PYTHONDONTWRITEBYTECODE=1 /tmp/opencode/bloat-cycle/venv/bin/python -B docker/run-portable-review-tests.py
```

Exit **0**: **1,477 tests passed**, plus **6 subtests**. Group counts:

| Group | Passed |
|---|---:|
| contracts | 200 |
| bridge | 646 (+4 subtests) |
| navigation | 207 |
| dock manager | 112 |
| Docker tests | 258 |
| system reset | 25 |
| localization diagnostics | 24 (+2 subtests) |
| Docker root | 5 |

Patch artifact verification and all five rebuilt offline ARIA-free C++ binaries
also passed with the same check counts above. No new failures were filtered or
assertions weakened. Full output:
`/tmp/opencode/bloat-cycle/core-portable-isolated.log`.
The full candidate diff, including this report, is saved at
`/tmp/opencode/bloat-cycle/core-cleanup.diff`.

### Packaging test placement correction

The full portable results and diff snapshot above predate this correction;
the original logs are retained unchanged. That portable pass did not expose
the navigation regression's unconditional read of
`docker/Dockerfile.floor-validation`, which is absent from the native test
image's explicit support-file COPY set in `docker/Dockerfile.ros-tests`.

Moved both Dockerfile assertion loops unchanged into
`contracts/test_native_build_sources.py`, selected by the portable contracts
group. Navigation retains the package dependency, sanitizer installation,
custom joystick and active mux/docking assertions against native-available
sources. No production files, CI/image contents, assertions or test exclusions
were changed by this correction.

Focused verification after the correction:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTEST_ADDOPTS='' /tmp/opencode/bloat-cycle/venv/bin/python -B -m pytest -p no:cacheprovider -v contracts/test_native_build_sources.py ros2_ws/src/patrolbot_navigation/test/test_safety_contract.py
```

Exit **0**: **58 passed in 1.28s**, covering both modified test files in full.
Separate output: `/tmp/opencode/bloat-cycle/core-packaging-review-focused.log`.
`git diff --check` also passed; separate output:
`/tmp/opencode/bloat-cycle/core-packaging-review-diff-check.log`.
The complete portable runner and native CI were not rerun after this correction.
No other concrete issue was found in this focused review.

## Remaining gates

Parent and independent reviewer must review the final diff and test evidence.
The complete portable pass is local evidence only. Run parent-owned native
image/build and functional checks for joystick takeover,
both mux paths, guarded undock and both floor selections. Required CI must pass
before the parent decides publication or merge. No native/ROS/Docker/robot,
network, installation, deployment, commit, push, PR or merge was performed here;
the portable runner's offline ARIA-free C++ tests are the only hardware-source
execution authorized for this task.
