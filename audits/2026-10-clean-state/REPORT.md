# Clean-state and deployment reconciliation — October 15, 2026

## Outcome and scope

Existing source was preserved, not discarded. Current deployments are recorded
on dedicated branches; unfinished work is separately reviewable. Main branches
were not reset, merged or overwritten. No robot command, service restart or
behavioral deployment was performed. The dashboard host's existing dirty source
was committed as metadata-only preservation; running files were unchanged.

Subagents preserved the two dirty worktrees and issue matrix before an account
usage limit interrupted them. Parent completed verification and archive migration.

## Branch inventory

| Repository | Branch | Preserved scope |
|---|---|---|
| Robot | `deployed/robot-20261015` | Exact observed core/navigation/native-patch/SBC source, captured profile and read-only manifest. |
| Robot | `review/undock-rejection-20261015` | Original canonical diagnostics/result patch and tests (`4d0c30f`); older baseline, not a production replacement. |
| Robot | `review/navigation-goal-dirty-snapshot-20261015` | Entire original dirty fork (`bba5f61`), including pending patch0018/native/test/config work. |
| Robot | `review/joystick-handshake-incomplete-20261015` | Recovered candidate (`72e3d8c`); original22 native cases and exact role builds remain missing. |
| Robot | `review/production-backport-incomplete-20261015` | Historical exact-production backport (`a18fb89`); controller provisional and localization merge incomplete. Original cross-repo paths are archival, not a new package placement. |
| Dashboard | `deployed/dashboard-20261015` | Actual backend/maps and Pi web source; frontend source reproduces all15 shipped assets. |
| Dashboard | `review/20261015-canonical-undock-rejection-test` | Original isolated test (`9a4ee1b`). |
| Dashboard | `review/20261015-canonical-override-admission-incomplete` | Original incomplete admission-only override (`699d84a`). |
| Dashboard | `review/20261015-dashboard-work-navigation-recovery-incomplete` | Original dirty nested dashboard clone (`a9575aa`). |
| Dashboard | `review/dashboard-host-source-20261015` | Captured pre-reconciliation device checkout (`6ad18a4`), including older frontend regressions. |
| Dashboard | `review/dashboard-device-checkout-20261015` | Device-created source-preservation commit (`cf301e6`), same source tree as preceding branch. |

Existing robot `db4e797`/`9fd409f` motor fixes are ancestors of deployed/review
branches. Every preexisting dashboard branch was already an ancestor of main.
Branch preservation is not test acceptance or approval to merge. Do not replay
all branches in order: several deliberately overlap or conflict.

**Publication verified:** all11 deployed/review refs are on GitHub; both original
main refs remain unchanged. Source baseline commits are robot `7bc58c8` and
dashboard `749cb18`; the robot closeout commit adds this final verification only.
Both primary checkouts and preserved active worktrees are clean. Three stale
registrations for missing /tmp dashboard worktrees were pruned after their
branch ancestry was checked. See [exact refs/checks](verification.json).

## Verified deployed state

| Component | Immutable identity |
|---|---|
| Bringup / bridge | `2d3df1c3c6bfe857f256be1a947892e78e942802c633f727b194331ffe73355f` |
| Navigation XML leaf | `607d352411ed16c59ab16fb39323534f69d0b42068dad09b503e72f8bea845c8` |
| Pi web bridge | `0da4b7b6810de6b96ce17defc1e5c20dc698e704f2a9c33085522f40b873e4b6` |
| Dashboard server | `04a464df6668b2197795ed965f4c1704f180a7b08bf733dcd2086af47e9240bc` |
| Running SBC executable | `1767110d175d1f8a4d785bd0939dbbb19db7fe3c32b51ee43c5946809da8fdf6` |
| Actual commissioned profile | `ba38f3b74eed1554ab746fe5ec5fcd0d73cf9a83159380604fb96c3a749c047d` |
| Public/rebuilt entry JS | `92453189385baeb86d01b5100a1fbc9a0b5ad7af9dc6d950405591cbaf1aa664` |

Read-only manifests verify136 robot files,62 dashboard/web files, selected
running images/executable and15 public assets. Dashboard private-copy build
`npm run build` (Vite7.3.6) matches all15 installed static files byte-for-byte.
The verifier's two offline regressions pass. Native/hardware candidate suites
were not rerun and are not relabelled passed by source preservation.

## Known deployed defects / pending release order

1. Robot Floor1 revision22872ec1 differs from dashboard f5e10f78. Preserve the
   currently consumed map/persisted identity; package matching dashboard data in
   a reviewed release rather than bypass identity admission.
2. Missing merged frontend uncertainty and separate always-visible disable UI
   are real current regressions. Motor toggle/disable command itself still exists.
3. Patch0017/path-aware Pi web behavior is installed, but corresponding current
   dashboard frontend/server goal-liveness contract is not fully deployed.
4. Undock truthful-result and unknown-lifecycle-write repairs remain candidates.
5. Installed joystick callback's extra return remains. The handshake candidate
   has partial20/21/11 checks, not the unavailable original22 native suite.
6. Patch0018 stationary planning/goal-owned recovery guards and newer interface
   fields are not deployed; do not copy its XML into the existing plugin image.
7. Advanced recovery override is admission-only and paused for specification/
   robot-owned per-goal authority design. It is not an end-to-end motion fix.

Source-first reconciliation does not restore these features automatically.
Current image labels are historical/noncommit identifiers: exact independent
rebuild/pinned artifact retrieval and reviewed per-role rollback remain open.

## Issues reviewed — no full closure justified

Tracker readback: robot27open/34closed; dashboard0open/24closed. No issue was
changed by this cleanup. Proposed updates, retaining original acceptance:

| Issue | Keep open / correct stale scope |
|---|---|
| #6 | Both real-robot recovery trees need physical acceptance; synthetic results are not motion proof. |
| #7 | Sonar consumers exist; rear-obstacle stop/dropout and coverage remain. |
| #8 | Scaffold exists; low-obstacle hardware/wiring/geometry remain. |
| #9 | Panel mapping/FLASH readback exist; empirical firmware/motion cutoff remains. |
| #11 | Footprint needs bumper-inclusive measurement/doorway acceptance. |
| #12 | Explicit controls exist; enabled-motor E-stop power removal remains. |
| #13 | Chrony works; RTC replacement/cold boot remain. |
| #15 | Holds exist; real electrical power-fail input/shutdown remain. |
| #18 | Not only physical tests: current callback/ownership/native-suite defects remain. |
| #20 | SUBNET remains deliberately exposed; containment decision/verification remains. |
| #21 | 0.10m software deployed; reflection/costmap/stop acceptance remains. |
| #24 | Physical height datum/mount measurement remains. |
| #25 | Hardened EOL i686 platform still needs replacement. |
| #26 | Reuse completed distance/turn calibration; prove physical gyro contribution. |
| #34 | Acquisition-age implementation deployed; latency/TF/wall agreement remain. |
| #39 | Actual profile commissioned flag is not all-criteria physical proof. |
| #41 | Much hardening implemented; frontend regression and remaining command-boundary/Stop/takeover criteria remain. |
| #47 | Implemented/previously accepted, currently missing from public UI; restore and accept usability. |
| #48 | 90-minute docked substep passed; moving/geometry/patrol/undock remain. |
| #49 | Docked endurance does not prove clear-of-dock/moving freshness. |
| #50 | Per-axis fix exists; full positive/negative age/covariance/motion matrix remains. |
| #53 | Metadata fixed; rosaria2 MIT/GPLv2 provenance remains. |
| #54 | Logger fix deployed; authorized actual override-path test remains. |
| #59 | Parked thermal results do not close representative PSU/load integrity. |
| #64 | Deferred physical shutdown/wake, not the implemented software reset. |
| #65 | Software merges/endurance exist; wall relocation, map parity, moving accuracy/navigation remain. |
| #68 | Software and FLASH change exist; electrical/boot/F710/retention/fresh-localization acceptance remains. |

Closed #5 should be reviewed for reopening after the current undock regression;
closed #38 should reflect the new provenance regression (PB020 reopened locally).
Closed/deferred #42 must stop saying no gimbal implementation exists: July
`0a34992`/`ca66321` contain it. Keep #56/#58/#66 scoped historical closures and
#10/#29/#43/#51/#60 supersession links; do not create duplicate implementation work.

## Backup migration / remaining cleanup

Thirty-six exact snapshot directories plus six stray Pi backup files were
privately archived, streamed/checksummed, member/content/metadata validated,
rechecked and removed. See [sanitized archive index](archives.json). Pi's
historical consolidation text/root symlink remain; dashboard/SBC backup roots
are empty. Active binds, runtime data/secrets, images and testing evidence were
not deleted. SBC root-process inspection was unavailable; application-owned
processes, effective system services and mounts showed no snapshot references.

Workstation archives still contain older nested repositories, private credentials
and unique evidence. They are not a safe bulk-delete target. Useful active
candidates are now branches; additional historical deduplication is separate.
Never blanket-add backups or full device homes to Git. Three missing /tmp Git
worktree registrations were pruned after branch preservation. No live bind
directory should be removed merely because its name looks temporary.

Warnings: do not widen `max_position_variance` — it also gates general navigation;
do not flip `allow_reversing` — mutually exclusive with enabled `use_rotate_to_heading`;
do not install graceful shutdown without real power-fail input; do not change
footprint/inflation together; do not restore DDS to SUBNET; do not raise
SCAN_RANGE_MIN to make the stop box see nearer. Never bypass unknown outcomes,
guess physical values, auto-enable motors or resume old goals.
