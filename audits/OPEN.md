# PatrolBot — open work

## First deployment problem — dashboard UI restoration DEPLOYED October 15

**Problem:** served UI lacked previously merged uncertainty display and
the independent motor-disable fallback.

**Progress:** deployed from reviewed main `6b3b785` (312 frontend tests pass
including eight new disable-guard refusals; build emits the accepted entry
bundle). Server image `d2a78dc3` recreated and healthy; public asset
byte-identical to accepted `8299aab5`; uncertainty text present; health
healthy/robot online with zero write losses. Isolated UI-only candidate
`9a9235d3` and its 28/28 layout rerun remain preserved history. No database
migration and no nginx/Pi/SBC restart; no motor/pose/goal/undock command sent.

**Remaining:** fresh live AMCL display and actual motor-response acceptance when
the SBC returns under separate authorization; operator usability follow-up for
#47. Dashboard release manifest recorded on `deployed/dashboard-20261015`.

**Blocked on:** SBC availability and separately authorized physical
acceptance — not software. Other map/joystick/undock/override/planning defects
remain open below. No issue/count closure by this deployment.

Superseded candidate history (preserved, not deleted): the isolated UI-only
branch restored four runtime/type files exactly from reviewed main against the
old deployment; eleven widget contracts failed before and 72 passed after, and
the 28/28 layout rerun confirmed the probe timeouts were contention. The
production-derived candidate `9a9235d3` was never activated; main already
carried the fix, so deployment went from main instead. See
[evidence](2026-10-clean-state/UI-RESTORATION-20261015.md).

## 1. Reproducible source and release provenance — PB020

**Problem:** live code came from dirty forks/device copies and inherited image
labels, not a single published reproducible release. Dashboard source and shipped
assets regressed against main; robot/dashboard Floor 1 revisions differ.

**Progress:** exact observed sources are preserved on `deployed/robot-20261015`
and dashboard `deployed/dashboard-20261015`; read-only verifiers match 136 robot
and 62 dashboard/web files, running images and SBC executable. The dashboard
rebuild matches all 15 shipped static files. Review candidates are separate.
Private device snapshots are verified and archived off-device.

**Remaining:** independently rebuild ARM/native/SBC
artifacts from pinned inputs; restore missing merged frontend work and reconcile
map identity in separately reviewed releases. Adapt per-role rollout/rollback to
off-device policy before activation. Do not use the legacy shared-image apply.

**Blocked on:** review/build/acceptance of those releases, not SSH access.
All11 preservation/deployment branches have been published and their exact
remote refs verified; both canonical checkouts are clean.

## 2. Unmerged software — deliberately not activated

**Problem:** undock admission/result truthfulness, unknown lifecycle outcomes,
joystick ownership, navigation paths and stationary planning have overlapping
unfinished repairs. Advanced override admits requests but lacks final per-goal
robot authority. Current manual callback contains a dangling return.

**Progress:** all original canonical/fork dirty changes are committed on named
review branches. The handshake was recovered from its evidence image; candidate
contracts do not substitute for its missing original 22 native scenarios.
Production-backport and dashboard-device source snapshots are preserved separately.

**Remaining:** review branch inventory, reconcile shared ownership generations,
recover/replace full native coverage, test exact ARM role/interface/import hashes.
Approve override specification before implementing missing authority. Preserve
the deployed XML compatibility fix. See [inventory](2026-10-clean-state/REPORT.md).

**Blocked on:** software review/specification and exact-artifact acceptance, then
separately authorized supervised tests. No unfinished feature was merged here.

## 3. Physical acceptance and hardware

**Problem:** accepted software or parked tests do not prove motion safety or
physical map alignment. Current pose/goal failures and map mismatch remain.

**Progress:** scoped parked endurance, link continuity, motor-disable, bumper
mapping and FLASH readbacks remain accepted historical evidence. SBC executable
`1767110d…` and actual profile `ba38f3b7…` are now read back successfully.

**Remaining:** issue-specific #6/#7/#8/#9/#11/#12/#13/#15/#18/#21/#24/#25/#26/
#34/#39/#48/#49/#50/#59/#65/#68 criteria: geometry, moving accuracy, Stop/E-stop,
bumpers/rear sensing, dock/undock, supply/thermal and electrical power controls.
Gimbal code exists but hardware/dashboard integration is deferred. Licensing #53,
dashboard hardening #41/#47/#54, DDS #20 and deferred power #64 remain separate.

**Blocked on:** actual measurements/equipment and explicit current-session
operator authorization/supervision. Do not repeat completed calibration without
new contradictory evidence. No physical acceptance was added by cleanup.

## 4. Tracker and historical records

**Problem:** old bodies/checkpoints still say implementation/deployment is missing
where code exists, or claim completed guarantees contradicted by current artifacts.

**Progress:** all 27 open robot issues reviewed; no open issue meets full closure.
Dashboard has zero open issues. Proposed corrections include closed robot #5
(undock regression), #38 (provenance), #42 (gimbal implementation), and open
#18/#41/#47/#48/#53/#65/#68. No issue was modified or closed.

**Remaining:** operator reviews proposed tracker updates; reconcile remaining
historical architecture/runbook statements. Original narratives are archived
privately under `audits/history/20261015-pre-cleanup-*`; raw evidence was not erased.

**Blocked on:** review of tracker semantics and actual acceptance evidence, not
an assumption that clean Git means done.

## Mandatory warnings

- do not widen `max_position_variance` — it also gates general navigation;
- do not flip `allow_reversing` — mutually exclusive with the enabled `use_rotate_to_heading`;
- do not install a graceful-shutdown daemon with no real power-fail input;
- do not change footprint and inflation together;
- do not restore DDS discovery to `SUBNET`;
- do not raise `SCAN_RANGE_MIN` to make the stop box see nearer.

Do not guess physical values, bypass unknown outcomes, restore background
resampling, auto-enable motors, reuse moved poses or resume old goals.
