# Save/apply local implementation checkpoint

**Parked 2026-10-24:** [resume handoff](save-apply-resume.md) is the current entry
point for the paired feature branches. Earlier checkpoints below retain their
original test scope and are not deployment approval.

**Design:** [reduced Apply and mutability matrix](save-apply-phase-c-operations.md).
The earlier 2026-10-22 checkpoint below is historical, not current support status.

**Final branch checkpoint (2026-10-24):** 924 portable robot contract/tuning/
bootstrap tests passed, two opt-in tests skipped in that selection. The separate
real-Docker/helper selection passed 35. Frontend 500, server 281, web adapter/
input/manual 103 were rerun and passed. Type checks, bundle budget, all eight
contract artifacts, Compose config, and whitespace checks passed. See the paired
resume handoffs for unfinished integration/commissioning gates. All source,
tests and handoffs are being committed to `feat/save-apply-commissioning` in
both repositories; earlier “no implementation commits/push” notes are historical.

## 2026-10-23 reduced-policy checkpoint

## 2026-10-24 navigation-only integration follow-up

- Added the fixed `navigation_supervisor.py` helper and `tuning_nav2.py` socket /
  exact-job validation adapter. No new signing system or generic execution API.
  The existing core authenticates and journals Apply; host independently checks
  that running reviewed job, immutable candidate, ACTIVE baseline, image, launcher
  and read-only state-volume identity before one `patrolbot-navigation` restart.
- Standard Compose mounts separate read-only core-control and boot-only socket
  directories into only their respective containers. Existing reset wiring stays
  intact. The service/policy are templates only; host policy is disabled, no
  installation or runtime enablement has occurred.
- Host dispatch and boot permission are consumed before their possible side
  effects. Missing replies never retry a restart or a candidate boot. `tuning_run`
  clears inherited candidate paths and validates the one replacement boot against
  immutable job/revision data; an ordinary subsequent restart uses committed ACTIVE.
- A real disposable, network-none Docker fixture exercised the actual Unix socket,
  helper, launcher and startup resolver: ACTIVE first, exact candidate once after
  restart, then ACTIVE again on a separate ordinary restart. Only uniquely named
  test containers were created/restarted/removed; no production name/device/state,
  host networking or Docker socket was mounted into a test container. This is
  boot-path acceptance, not full production graph/image/volume commissioning.
- Transport now reacquires replacement inputs, restores only lower F710 trims,
  persists the exact new map/fresh-seed barrier, and verifies boot/seed evidence
  before promotion. Native trim restoration passes using the current interfaces.
  An older fixture image silently lacked `payload_json`; capability collection now
  refuses that ABI explicitly rather than claiming a working policy handshake.
- Capability collection uses the existing retained worker: actual cache versions,
  composed membership/no duplicate standalone consumers, native action input,
  physical/web policy protocols, final publisher graph, session/origin and full
  three-second read age. Missing host support leaves verified hot-only plans
  available. Actual Apply still needs the runtime flag plus fresh admission.
- Current focused helper/startup/transport/capability/manual selection: **265 passed**;
  expanded verifier/trim/capability selection: **165 passed**; web adapter/input/
  maintenance selection: **103 passed**; native producer/epoch/map gate selection:
  **12 passed**; full native manager/cache/action/AMCL suite passes. The broader
  contract/tuning/bootstrap selection then passed **919 tests, 2 skipped** after
  updating the retained worker fixture. Native epoch/map/hardware/F710/DDS tests:
  **116 passed**. Native web maintenance: **2 passed**. Portable teleop/safety:
  **278 passed**. Server tuning: **281 passed**. All eight shared artifacts remain
  byte-identical; standard Compose config and whitespace checks pass.
- Final host-identity checks additionally require the canonical entrypoint and
  private read-only boot mount. Pending boot permission is bound to the current
  host boot and still-running core-container identity; an ordinary host/core
  restart never revives that candidate permission. These last assertions are
  included in the final focused helper/real-Docker acceptance rerun.
- Latest pinned native image manifest:
  `sha256:d23ae3fe8335f450ac0d3bf84cdd8eb84e22d0a00a588f8e4c2ce8347dd5f1c1`.

**Not commissioned:** target-native whole-cycle fake-hardware/source-identity
acceptance, production image/volume/launch/failure acceptance, PR/green CI, live
operator authorization. The older ROS image's missing source GIDs still refuse
motion evidence; do not invent them. SBC remains reported down and untouched.
No live Apply, deployment, pose, motion, reset or motor operation has occurred.

The 2026-10-23 sections below are historical checkpoints; their statements that
the navigation adapter/capability collector were unimplemented are superseded here.

### Follow-up: shared composed-action input boundary

- Added `0023-tuning-composed-action-input.patch`: one `nav2_util_core` gate,
  reused through the existing navigator maintenance service. No new per-node
  services, new signing protocol or host orchestration. Rebuilt every fixed
  template consumer in the native recipe; patch application uses zero fuzz.
- Direct-action goal admission and maintenance share one mutex. Current/pending
  goals and unfinished execution callbacks block Apply admission. New goal
  handlers reject while held; cancel remains usable. Peer-token release cannot
  drop the core hold. Production transport requires exact compiled gate evidence
  and resolves core topic/service ordering using reads, never repeated acquire.
- Native probe passes direct Spin/FollowPath action protocols with nonmoving
  callbacks, pending goals, worker tails, foreign release, release gap,
  cancellation, new explicit goals after release and 1,000 admission races.
- Added the F710 producer release timestamp barrier: middleware-queued old
  release/press/trim samples cannot rearm or alter trims after maintenance.
  Focused portable source/input/verifier selection: **141 passed**. Native F710
  and core DDS barrier selection: **3 passed**. Broader contract/tuning/bootstrap
  selection: **864 passed, 1 skipped** (the ROS-only test run separately).
  Portable teleop/safety selection: **272 passed**. Two further core-hold-ACK
  refusal assertions passed in the latest **28-test** input/source selection.
- Full isolated native manager/cache/AMCL/navigator/direct-action regression run
  passed using `patrolbot-tests:save-apply-action-gate` and the current test script
  mounted read-only. The initial run caught a DDS first-response discovery race
  in the fixture. Navigator readiness now uses the same read-only reply probing
  as the manager tests; mutations still execute once and are never retried.
  Native image manifest before the test-script-only rebuild:
  `sha256:3ca0789d0903134925381d625f5bb12bfc1ee1cca67bfcbaa637268a951cff5f`.
- Whole-navigation restart/replacement boot and capability collection remain
  unfinished and disabled. Legacy/unrecognized producers still need acceptance;
  a process-local action gate is not an arbitrary-network action firewall.
  In particular, DDS-buffered requests whose handlers first run after release
  remain outside this probe's guarantee. Do not treat this as completed
  end-to-end no-replay commissioning.

- Production wiring selects `SimpleApplyBackend`: verified hot batches, otherwise
  one complete `navigation` restart; no per-node lifecycle choreography or silent
  three-container fallback. Shared runtime contract and synthetic preview agree.
- Core DRIVE closes before installation; physical/web input is blocked and
  discarded. Core source-timestamp and web ingress/drain barriers reject queued
  pre-maintenance input after release. Stop/cancel/release/emergency disable remain.
- Source-audited mutability classifies AMCL particle/KLD resets, Spin bounds and
  RPP approach scaling distance as restart-required; lifecycle-dependent hot
  callbacks stay hot only in supported states. Derived values participate.
- Fixed `0022-amcl-hot-cache-readback.patch` against post-0021 source. It applies
  with **zero fuzz**. Rebuilt `patrolbot-tests:save-apply-hot-audit` successfully
  (local image manifest `sha256:0c42e7c0fdd2ed322555f31fff2a410fa88930040fe24d82a800de3c800617aa`).
- The isolated network-none native run passes manager/cache regressions, 1,000
  navigator admission races, active AMCL model/threshold/laser-input changes
  without particle-filter recreation, and the inactive AMCL stale-cache boundary.
  No map, odometry or operator seed was supplied to the AMCL hot test.
- Full portable contract/tuning/bootstrap selection: **854 passed, 1 skipped**
  (native ROS-only barrier test, exercised separately below). Latest focused
  executor/input-adapter selection: **28 passed**, including two subsequent
  support-withdrawal assertions. Input-adapter/verifier selection: **66 passed**.
  These offline I/O fixtures are not end-to-end runtime acceptance.
- Portable teleop/safety regressions: **271 passed**. Native epoch/map/hardware,
  actual F710 post-set block/acquire, and real DDS queue-barrier selection:
  **116 passed**. The recording fake SBC received no replay of a velocity sample
  published before release; new samples still obey normal raw-odom gates.
- Native F710 testing caught/fixed a policy setter that still demanded released
  buttons after acknowledged ingress closure. New button pressure is now ignored
  only under a held job lease with drained buffers/no pending local operation.
  Initial admission still needs release; lower operator trims are preserved.
- Read-only ACTIVE inspection remains available after the backend switch and
  checks native Nav2 caches. Native readback age starts before the first RPC,
  rather than being renewed by a slow last reply. Source-timestamp handling
  accepts object/dictionary middleware metadata; missing source GID still refuses
  sample-identity evidence rather than borrowing the sole graph publisher.
- Web adapter/input/maintenance/manual tests: **103 passed**; real ROS web
  acquire/block/observe/release tests: **2 passed**. Server tuning suites:
  **281 passed**, including rehashed-but-invalid wider restart scope refusals.
- Frontend: **500 unit tests**, preview/e2e TypeScript, production build and bundle
  budget passed (**173.2 kB gzip / 175 kB**). Isolated Chromium/WebKit browser
  acceptance: **72 passed** across six desktop/phone/tablet projects. The read-only
  checkout needed an ephemeral `.vite-temp` tmpfs; no dependencies were deleted.
- All eight shared contract artifacts are byte-identical. Core-generated hot,
  Spin restart and derived-angular restart snapshots passed the dashboard server
  validator in isolated temporary stores without any runtime mutation.
- Older scoped manager and signed host/candidate-boot prototypes remain retained,
  not enabled or substituted into the new policy.

**Outstanding:** direct-action/legacy producer rejection through release; actual
native capability collection; fixed navigation-only host restart and exact
replacement boot; producer reacquisition and fresh-seed barrier; full final
regressions and PR/CI. `simple_capabilities()` remains unavailable and
`restart_navigation()` deliberately refuses. No real Apply is enabled. The SBC
is reported down and has not been contacted; no live work, implementation commit,
push, PR, merge or deployment has occurred.

The older ROS test image exposes timestamp dictionaries without publisher GIDs.
That ABI cannot supply the required sample-to-publisher proof; capability work
must verify the actual target ABI, not enable support from graph names alone.

## Historical 2026-10-22 checkpoint

**2026-10-22 — not commissioned, not deployment-ready.**

Continues the [approved handoff](save-apply-commissioning-plan.md) and
[Phase A design](save-apply-phase-a-design.md). Both repositories remain on local
`feat/save-apply-commissioning` branches. No implementation PR, merge, deployment,
runtime enablement or real robot/SBC/save/apply/reset/motion/pose/motor operation
has occurred. No signing secrets or production state were inspected or changed.

## Phase status

| Phase | Current local status |
| --- | --- |
| A | Source inventory and dependency/state design recorded. |
| B | Explicit Save, frozen Apply/restore review, conservative unknown handling, synthetic scenarios and Chromium acceptance implemented. WebKit acceptance remains blocked by local browser dependencies. |
| C | Read-only compiled dependency planner, signed review binding, command-idle coordinator predicate, durable plan/mutation journal and full-revision proof boundary implemented/tested offline. **Native scoped execution and ownership integration are not implemented/accepted.** |
| D | **Not implemented.** Docker impact exists only as a synthetic UI scenario. There is no new host helper or candidate-boot permission. |
| E | Portable/unit, contract parity, frontend checks/build/budget and Chromium acceptance exercised. Native ROS/container tests, WebKit and green PR CI remain outstanding. |
| F | No live authorization requested or exercised; all separate operator gates remain in force. |

## Implemented locally

Dashboard paths are relative to `/home/yousef/Projects/repos/patrolbot-dashboard`:

- `frontend/src/components/TuningPanel.tsx`: separate **Save changes** and
  **Apply changes**. Blur, Enter, slider movement/release/cancel and timers do not
  save. Dirty count, saved/pending targets and independent live observations stay
  separate. Disconnected drafts remain visible, disabled and exact. Conflicts
  display saved/draft values and require explicit rebase or discard.
- `frontend/src/stores/tuningStore.ts`: one canonical CAS batch across all dirty
  groups, complete/coupled validation, fresh administrator/store/SBC admission,
  single-flight intent and per-edit versions. Confirming a submitted version
  cannot delete a later edit. Unknown outcomes retain their original request and
  block resubmission/discard until status resolution; reconnect does not replay.
  Save is independent of odometry, localization, navigation activity and motor
  enable. Apply and last-good restore require a frozen core-provided plan and
  send only the reviewed digest/hash, never a browser-selected execution scope.
- `frontend/src/api/tuning.ts`: bounded mutation wait without retries; full HTTP
  round-trip age is charged to observations on reads **and mutation responses**.
  The existing three-second freshness/session boundary is unchanged.
- `frontend/dev/advancedPreview.tsx`, `frontend/e2e/save-apply.spec.ts`: isolated
  explicit Save, refused/unknown Save, scoped Apply (motors on/off, no odometry),
  reviewed Docker disruption/seed loss, pre-mutation refusal and unknown Apply.
  All API/socket operations terminate in the simulation. No native execution is
  claimed by these browser scenarios.
- Server/web-bridge validators preserve the signed `review` binding and reject
  malformed hashes or arbitrary scope arguments. Server validation checks preview
  canonical hashes, revisions/session, fixed node/service lists and logical values.

Robot paths are relative to
`ros2_ws/src/patrolbot_bridge/patrolbot_bridge/`:

- `tuning_plan.py`: compares full compiled ACTIVE/candidate parameter plans,
  including derived values and F710 policies. Speed-only closure does not include
  AMCL or imply seed loss. Preview hashes bind revision/digests, old ACTIVE,
  desired revision, session/hardware generation, compatibility, operations,
  verification scopes and state loss. Missing native/cache support is explicit
  non-executable evidence, not inferred from service availability.
- `tuning_service.py`: exposes read-only Apply/restore previews, reusing each
  immutable revision already loaded for a snapshot. Saving still only persists
  desired state. The mutation request accepts a bounded signed review binding.
- `tuning_activation.py`: removes the tuning coordinator's odometry/stationarity
  admission requirements, uses truthful command-idle wording, fingerprints the
  review, rejects changed/unreviewed plans, journals ownership before reservation
  and records exact scope/mutation boundary. Every definitive post-reservation
  refusal before reload unwinds acknowledged ownership, including the final
  pre-mutation recheck. Unknown reservations/releases/partial changes stay held.
  Unsettled jobs cannot be overwritten even if a caller omitted startup recovery.
- Reviewed execution cannot call the old all-node `reload()`. It needs distinct
  exact-job execute/verify/committed-binding hooks. The proof boundary requires
  both scopes, all fixed-node parameter readbacks, policy-cache attestations,
  graph/protected-invariant/hold attestations, and exact job/revision/digest/hash/
  hardware generation. Incomplete/foreign/stale proof never promotes. Commit or
  committed-binding uncertainty remains durable/held; no automatic retry/fallback.
- **The native `LifecycleBackend` supplies none of those reviewed hooks or native
  support evidence. Its previews are not executable, and real activation is not
  advertised. Changing an enable flag alone cannot make the reviewed path run.**
  Its old broad reload is retained for offline regression coverage, not used as
  the new reviewed executor. Its final tuning-only odometry checks were removed.

`contracts/robot-tuning.schema.json`, `robot-tuning-runtime.json` and shared tests
carry review protocol version 1. The review field is shape-optional for reading
legacy requests, but the real native backend requires it at execution admission.
Legacy `redeploy` cannot select an unreviewed Docker action. All eight vendored
artifacts remain byte-identical; registry/compiler/default bytes were not changed,
so this checkpoint does not reinterpret or migrate existing revision history.
Robot contract must land before dashboard parity checks against merged main.

## Verification and environment notes

- Robot portable contract/tuning/bootstrap suite: **575 passed**; existing
  portable teleop/navigation safety regressions: **249 passed**.
  Reviewed-coordinator tests fabricate certificates
  **only in named offline fixtures** to test rejection/binding/persistence; they
  are not proof of native policy consumption or manager behavior.
- Server tuning/integration suites: **273 passed**; web-bridge tuning adapter:
  **24 passed**. Actual core-generated Apply/restore snapshots also passed the
  dashboard validator in an isolated temporary store; ACTIVE remained revision 1.
- Chromium Advanced/tuning/save-apply acceptance: **36 passed** across desktop,
  Android tablet and Android phone (24 new save/apply cases plus 12 regressions).
  Used `/usr/bin/chromium` via the existing executable override. Results directory:
  `/tmp/opencode/save-apply-chromium-final`.
- WebKit projects could not launch: installed WebKit requires missing
  `libicudata.so.74`. Default pinned Chromium headless shell is also absent on the
  workstation. Those failed launches are **not** passing browser acceptance.
  No checks were skipped in source or freshness windows increased to hide this.
- Existing Vite `node_modules/.vite` contains files not writable by this user.
  They were not deleted/chowned. The optional `PATROLBOT_VITE_CACHE_DIR` supports
  an isolated cache; local runs used `/tmp/opencode/save-apply-vite-e2e`.
- One pre-existing map geometry test consumed its hardware fixture's 500 ms
  window under concurrent load. Its test-only monotonic clock is now controlled,
  with a new explicit expired-fixture refusal assertion. Production freshness
  and map/motion code are unchanged; no retry was used to mask the failure.
- Frontend units: **500 passed / 39 files**. Production build, preview/e2e
  TypeScript and bundle budget passed; initial static JS **173.1 kB gzip / 175 kB**.
  The synthetic preview is not a production build entry. All eight shared
  artifacts match byte-for-byte; both repository diffs pass whitespace checks.

Reproduce the browser checkpoint from dashboard `frontend/`:

```sh
PLAYWRIGHT_CHROMIUM_EXECUTABLE=/usr/bin/chromium \
PATROLBOT_VITE_CACHE_DIR=/tmp/opencode/save-apply-vite-e2e \
npm run test:e2e -- e2e/save-apply.spec.ts e2e/advanced.spec.ts e2e/tuning.spec.ts \
  --project=desktop-chromium --project=android-phone --project=android-tablet \
  --output=/tmp/opencode/save-apply-chromium-final
```

## Next work — do not mistake the checkpoint for completed C/D

1. Refactor the **real bridge evidence producers**: no-map and quiet
   inactive-awaiting-seed are not mutation activity. The legacy bridge predicate
   still uses navigation/map readiness shortcuts; removing only coordinator
   odometry checks is not complete apply admission.
2. Add job-specific acknowledged exclusion across physical/web/legacy switch,
   navigator/behavior/undock/map/lifecycle and hardware mutation paths. The
   navigator's current Boolean reservation is not an ownership token that may
   safely release an unrelated seed hold. Preserve emergency disable/failsafes.
3. Separate EpochRuntime command maintenance from localization invalidation.
   `_set_tuning_hold_source` still clears accepted seed/readiness; do not restore
   those fields by copying/replaying a pose. Preserve localization in place for
   unrelated changes while keeping all normal driving freshness gates.
4. Implement the native exact-plan executor, supported manager-subset exclusion,
   actual consumed-cache/policy readback and exact-job candidate/committed policy
   handshakes. Proof dictionary validation alone is **not** evidence collection.
   Exercise pinned callback/constructor behavior with fake hardware in an isolated
   ROS domain/network before advertising executable support.
5. Implement/test the restricted host-owned Docker helper and crash-safe candidate
   boot handoff, retaining active-only ordinary restart, history, job/hold and
   explicit recovery. Keep standard Compose reset socket/group and all merged
   features intact; no dashboard Docker privilege or automatic host/SBC action.
6. Finish WebKit/native/container/failure-injection acceptance, then PR -> green
   CI -> merge. Deployment and every real commissioning scenario still need the
   separate named operator authorization in Phase F.
