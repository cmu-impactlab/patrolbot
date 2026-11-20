# Fixes-only review backport to main

Base: `c661ed4ce086cc78afde7dfa47e81c143955f802` (`origin/main`).
Branch: `fix/review-main-only-20261120`.
Paired dashboard base: `18a4ea195b06c47f48256788fbf3d5c8f73dde66`.

## Scope

The user requested independent safety/maintenance fixes only. Do not merge the
parked Save/Apply commissioning feature or the earlier broad integration branch.
Main's newer manual-undocking/obstacle-recovery and reset boot-directory changes
are preserved. Existing tuning behavior is not replaced or enabled.

Applicable review fixes cover undock deviation safety, pending navigation Stop,
live command authority, emergency reset exemptions, protected transport replies,
unknown motion ownership, passive recording/manual input, unsupported gimbal
removal, timestamp/retention correctness, and verified dead-code/test cleanup.
The Save/Apply-specific R7 recovery work remains on its separate feature-based
integration branch and is deliberately deferred.

The robot has no gimbal. Remove its code, interfaces and deployment service
rather than adding support. Preserve shared tuning/teleop contracts and map
validator parity, original hardware constants and all current-main manual
undocking and fault-recovery gates.

## Acceptance boundary

Before any main merge: inspect the exact main-to-candidate diff, prove that the
WIP feature did not enter, run combined portable/native/browser coverage, and
review compatibility with the current main changes. Previous broad-branch test
counts are not evidence for this new candidate. No robot connection, physical
operation, push, deployment, runtime enablement or commissioning is authorized.

## Source disposition

- R1 was partially guarded on current main for manual BACKING_OUT only. The
  shared safety gate now guards all post-fixture samples through clearance and
  normal handoff, while manual turning after clearance retains its separate
  odometry bounds. Main's new manual-undock/obstacle recovery remains intact.
- R9/R10: complete unsupported gimbal removal with four absence regressions.
- R12: verified unused state/helper removal, current-policy helper fixtures,
  broad portable CI and runner-orchestration tests. No tuning implementation
  edits. The empty component container was already absent on current main.
- Applicable dashboard command/authority/transport/history and recording fixes
  are in the paired branch; Save/Apply-specific recovery remains deferred.

Reviewed implementation commit: `3eb4e60`. Paired dashboard implementation:
`c36f360` and `1f603d5`. These are selective backports, not merges of the old
feature-based integration branches.

## Scope audit

The parent independently checked that production tuning modules, tuning/teleop
contracts, epoch runtime, joystick controls, Dockerfile recipes, Nav2 patches,
host service directory and interface services remain unchanged from main.
Interface generation only removes the two Gimbal messages. Parked WIP commits
`37c4eff`/`a189858` are not ancestors of the new candidates. Eight shared
tuning/teleop artifacts, map validator and map catalog files match between repos.

## Completed portable evidence

- Robot Python: 1,293 passed plus six subtests across separate offline groups.
- All five ARIA-free C++ suites passed; undock alone ran 13,084 checks.
- Schema validation and whitespace/diff checks passed.
- Paired dashboard: 1,052 server tests (two external tests excluded), 340
  portable web-adapter tests (three environment skips), and 498 frontend unit/
  component tests passed. Frontend typechecks, build and 175 kB bundle policy
  passed (174.4 kB gzip initial JavaScript).

The undock suite also passed AddressSanitizer/UndefinedBehaviorSanitizer.

## Native and browser evidence

In a disposable `--network=none` container, all six selected ROS packages built
from this candidate. Functional suites passed:

- Core bridge: 784 passed, one intentionally gated test skipped.
- Dock manager: 162 passed.
- Navigation: 206 passed.
- Final paired web adapter: 341 passed, two server-app cases excluded only from the
  ROS image (those run with real server imports on the host).

Full paired six-project browser matrix: 738 passed, six skipped. Playwright's
saved result says `passed` with no failed tests. Mock robot: 26 passed.
No parked Save/Apply feature was needed for these results.

Native lint was explicitly excluded; old Ubuntu pytest's nose plugin was
disabled to avoid treating ordinary `setup()` factories as legacy hooks.
Existing rclpy teardown/Destroyable and missing pytest-timeout configuration
warnings were observed. These checks do not establish a production Nav2 image,
external PostgreSQL migration acceptance or physical commissioning.

## Independent review

Local Codex reviewed `c661ed4..3eb4e60` read-only and found no actionable safety,
correctness or merge-blocking issues. It specifically traced shared guard
capture/order, `manual_clear_`, turn/recovery/handoff behavior, gimbal removal,
deleted-helper callers, tests and CI selection. Parent retains merge ownership.
The dashboard review separately identified an Undock Stop/result-subscription
ordering case. It is fixed in `cc32d88`, with eleven regression cases covering
Stop, original-ID cancel, motor-disable and callback failure orderings. Final
host/native adapter reruns passed. Codex rechecked that delta, confirmed the P1
closed and reported no remaining actionable findings.

## Main merge acceptance

After completed tests and reviews, origin main was re-fetched and remained at
the reviewed base. Both fixes-only candidates passed a clean merge simulation,
ancestry check and renewed feature-exclusion/parity audit. Accepted for local
fast-forward into main only. No push, deployment or feature enablement is part of
this acceptance; parked Save/Apply branches remain separate and unmerged.
