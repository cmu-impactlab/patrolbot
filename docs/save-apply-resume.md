# Save/Apply: parked feature branch — resume here

**2026-10-24. Work-in-progress checkpoint, not deployment approval.**

Both repositories use `feat/save-apply-commissioning`:

- Robot: `cmu-impactlab/patrolbot`, based on main `f710cc8` (plus the earlier
  handoff commit `412eda5`).
- Dashboard: `cmu-impactlab/patrolbot-dashboard`, based on main `c168dce`.

The operator requested that all current work be committed to these branches and
parked for later. No implementation is merged or deployed. No production host
helper was installed; runtime Apply defaults to off and the example host policy
has `enabled: false`. The SBC was reported down and has not been contacted.
Do not interpret this checkpoint, test goals or disposable-container restarts
as live authorization. The [commissioning plan](save-apply-commissioning-plan.md)
and its separate operator gates remain authoritative.

## Settled scope — keep it small

- Separate explicit **Save changes** and **Apply changes**. Save persists one
  complete CAS batch; it never activates anything.
- Apply exactly the reviewed saved revision. Proven hot values stay hot; **any**
  restart-required Nav2 value selects the whole fixed `navigation` container.
- Do not restart core bridge, bringup, dashboard or SBC for these parameters.
  No parameter-only image build, per-node lifecycle schedule or silent wider
  fallback. Earlier broader prototypes are retained only as offline material.
- Reject new goals/input while Apply owns admission, in hot and restart modes.
  No application goal queue, delayed replay, automatic arm or patrol resume.
- Apply does not require odometry/localization/seed, motors off, or a selected
  map for unrelated edits. Keep fresh hardware/ownership checks and every normal
  driving/watchdog gate. Command idle is not measured physical standstill.
- Verify actual consumed values before ACTIVE promotion. Unknown outcomes retain
  durable holds and require explicit recovery; no automatic mutation retry.
- Ordinary startup uses committed ACTIVE. A reviewed replacement boot consumes
  candidate permission once. Whole-Nav2 replacement requires a fresh operator
  seed; hot-only edits preserve unaffected localization.

## What is implemented locally

- Dashboard drafts, explicit Save, conflict/unknown retention, frozen Apply and
  restore review, request binding and three-second observation freshness.
- Production `SimpleApplyBackend`, pinned mutability classification, core/manual/
  composed-action input blocking, post-maintenance timestamp barriers, cached
  policy handshake, full fixed-consumer verification and durable job outcomes.
- Fixed navigation-only host helper, private core versus boot-only sockets,
  exact-job/image/volume/launcher checks and one-use replacement boot. It never
  promotes configuration or commands motors. Host/core restarts invalidate old
  candidate permission. Standard Compose keeps existing reset wiring.
- Replacement navigator/F710 reacquisition, lower-trim restoration, map/seed
  barrier verification, read-only native capability collection and freshness.
- Native patches and disposable fixtures for cached-value behavior, direct
  action rejection, pending work, cancellation, ownership races and real Docker
  ACTIVE -> candidate-once -> ordinary ACTIVE boot behavior.

Main code: `tuning_simple.py`, `tuning_simple_plan.py`, `tuning_mutability.py`,
`tuning_backend.py`, `tuning_nav2.py`, `tuning_startup.py`, `tuning_run.py`,
`bridge_node.py`, `epoch_runtime.py`, `docker/tuning-host/navigation_supervisor.py`,
and `docker/nav2-patches/0022-*` / `0023-*`. Dashboard companion notes are in its
`docs/save-apply-resume.md`. [Operations/mutability matrix](save-apply-phase-c-operations.md)
and [historical local evidence](save-apply-local-progress.md) provide detail.

## Remaining work — not a completed or safe-to-deploy Apply

1. Demonstrate the entire signed dashboard -> core -> real native consumers ->
   verification -> commit -> release cycle for hot and replacement modes in an
   isolated fake-hardware environment. Current component and boot tests do not
   prove that complete cycle. Do not fabricate cache or source-identity evidence.
2. Verify target ROS source-publisher metadata, complete production launch/image/
   volume/helper setup, seed recovery and interruption/crash/timeout boundaries.
   Older test-image rclpy metadata lacks publisher GIDs and must remain
   inadmissible for source-bound motion evidence. Keep unknown jobs held.
3. Review the branch changes, run all applicable checks/lint, then follow robot
   contract PR -> green CI -> merge before dashboard parity/PR -> green CI ->
   merge. This WIP checkpoint does not claim green PR CI or full lint acceptance.
4. Only after those gates and separate operator authorization, deploy merged
   revisions and perform live commissioning with the SBC available.

## Evidence and resuming tests

Final parked-branch rerun passed **924 portable robot contract/tuning/bootstrap tests**
(two opt-in native/Docker tests skipped in that selection), 278 portable safety
checks, 116 native epoch/map/hardware/producer checks, two native web checks,
281 server tests, 103 web adapter/input/manual tests, 500 frontend units and 72
Chromium/WebKit browser cases. The final helper/real-Docker rerun passed **35**,
including the legacy-job ordinary-startup assertion. The frontend's 500 units,
281 server tests and 103 web adapter/input/manual tests were rerun successfully
before committing, along with preview/e2e TypeScript, bundle budget, contract
parity, standard Compose configuration and whitespace checks. These
counts have different scopes and are not summed into an end-to-end claim.

All eight shared tuning artifacts must remain byte-identical across repositories.
Registry/compiler/default history was not migrated. No private keys, production
state, build output or Docker images belong in Git.

Portable test environment (ephemeral, recreate if missing):

```sh
python3 -m venv /tmp/opencode/save-apply-tests
/tmp/opencode/save-apply-tests/bin/python -m pip install -r contracts/requirements-test.txt -r docker/tuning-host/requirements-test.txt
PYTHONPATH=ros2_ws/src/patrolbot_bridge /tmp/opencode/save-apply-tests/bin/python -m pytest contracts/ ros2_ws/src/patrolbot_bridge/test/test_tuning_*.py ros2_ws/src/patrolbot_bridge/test/test_config_bootstrap.py -q -p no:cacheprovider
```

Native fixture image: `patrolbot-tests:save-apply-action-gate`, last recorded
manifest `sha256:d23ae3fe8335f450ac0d3bf84cdd8eb84e22d0a00a588f8e4c2ce8347dd5f1c1`.
Build instructions are in `docker/Dockerfile.tuning-scoped-native`; use only the
explicit local Docker socket, network-none, isolated ROS domain, no devices,
production volumes or Docker socket mounted into test containers. The pinned
Nav2 archive is under `/tmp/opencode/save-apply-native-source` when retained;
temporary files and local images are not guaranteed to survive cleanup.

UI-only preview, with no fake/real SBC or actual Docker restart:

```sh
# In patrolbot-dashboard/frontend
npm run preview:advanced
# http://127.0.0.1:4180/advanced-preview.html?advanced=1&scenario=scoped
# http://127.0.0.1:4180/advanced-preview.html?advanced=1&scenario=docker
```

The latest frontend build and both preview Save/Apply reviews were checked.
The preview blocks external API/robot traffic. Do not substitute its synthetic
success responses for real Apply evidence.
