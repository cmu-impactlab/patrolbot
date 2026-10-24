# Save/apply commissioning: fresh-context implementation handoff

**Prepared:** 2026-10-22. **Status:** agreed specifications; local Phase B and
Phase C and navigation-only restart are locally wired but uncommissioned.

**Parked 2026-10-24:** resume from [save-apply-resume.md](save-apply-resume.md)
on the paired `feat/save-apply-commissioning` branches. The handoff below retains
historical source findings; the resume document identifies current state and gaps.

**Operator simplification, 2026-10-23:** Apply has two choices: verified hot
updates, or one restart of the complete `navigation` container if any changed
Nav2 value needs reconstruction. Close input admission during installation and
discard incoming commands; never queue/replay them. This supersedes the earlier
per-node lifecycle and three-container fallback designs, not the live gates.
See the current [operations and mutability matrix](save-apply-phase-c-operations.md).

Current checkpoint and outstanding gates:
[save-apply-local-progress.md](save-apply-local-progress.md). Nothing is commissioned
or deployment-ready; all live authorization gates below remain unchanged.

Local Phase A findings, complete editable-field dependency matrix and proposed
state/protocol design: [save-apply-phase-a-design.md](save-apply-phase-a-design.md).
This is a source-reviewed design candidate, not operator/native acceptance or
authorization for a live operation. Historical handoff baselines below are retained.

This document is a self-contained handoff for a new coding session. It records
operator decisions, source/deployment baselines, implementation phases and
acceptance gates. It is not evidence that runtime activation is commissioned,
and is not permission to issue live apply/redeploy, motion or hardware commands.

## 1. Task and authority

Implement and commission explicit **Save changes** and **Apply changes** on the
Advanced page. Preserve the existing durable configuration store, administrator
authorization and robot safety architecture. Work locally first, then use
branch -> PR -> green CI -> merge -> deploy the merged revision.

The operator approved the specifications below and requested this plan before
starting from fresh context. This session created documentation only. Do not
interpret approval of the design as authorization to execute a live activation,
Docker redeploy or physical acceptance trial. Obtain a separate operator go at
the corresponding live gate. Do not re-ask already settled specification choices.

## 2. Approved specifications

1. **Save is explicit.** Replace numeric-field blur/Enter and slider debounce/
   release autosaves with draft editing and a dedicated **Save changes** button.
   Save validates a complete batch and persists desired targets on the robot.
   It never changes running parameters or reloads services.
2. **Apply is separate and explicit.** A dedicated **Apply changes** button opens
   a review of the exact saved revision, changes and reload impact. Confirmation
   applies that reviewed revision, never an unreviewed newer revision. Save and
   Apply are two buttons, not a combined Save & Apply action.
3. **Save admission:** authenticated signed administrator, initialized writable
   store and fresh SBC<->RPi link. Motors, odometry, localization, stationarity
   and navigation idle are not prerequisites for saving targets.
4. **Apply does NOT require fresh odometry or localization.** Neither absence nor
   staleness of raw odometry, AMCL state, seed or pose is by itself an apply refusal.
   This overrides the old tuning-specific measured-stationary admission design.
5. **Motors may be ON or OFF during apply.** Motor-enable state is not an apply
   prerequisite. Apply must not send enable/disable commands, automatically
   restore motor power, or weaken independent hardware failsafes.
6. **Apply still requires command idle and fresh non-odom evidence:** connected
   hardware/link, healthy applicable hardware/fault/dock state, no navigation,
   undocking, map mutation, competing lifecycle work or other mutation owner;
   physical/web manual inputs explicitly released; zero command buffers and an
   acknowledged exclusive hold across all motion command paths. Unknown required
   ownership/release/hardware evidence refuses admission.
7. **Use hot updates only where consumed without restart.** Compute the dependency
   closure, including derived limits and cached producer policies, against the
   pinned implementation and current lifecycle state. Successful ROS setters and
   getters alone are not evidence of consumption. Hot-only edits preserve
   unaffected localization and do not restart nodes or containers.
8. **Any restart-required Nav2 value selects the whole Nav2 stack.** Review and
   restart exactly Compose service `navigation` / container `patrolbot-navigation`,
   including AMCL and its co-located producers. Do not hot-write that old stack
   first. Core bridge, bringup, dashboard and SBC remain up. No per-node lifecycle
   choreography or silent wider fallback. Show seed loss before confirmation.
   Parameter-only Apply does not rebuild an image. Stop accepting commands during
   installation and discard input rather than queueing/replaying it.
9. **Localization after apply:** preserve existing valid localization when its
   dependencies are unaffected. If a necessary reload/redeploy invalidates it,
   navigation remains blocked until a fresh operator-supplied seed is accepted.
   Never replay an old pose or inject a seed automatically. Activation success
   and navigation readiness are separate statuses.
10. **Verify before success:** read back affected parameters and dependent runtime
    policies, validate the remaining revision evidence, then mark the exact
    revision active. Saving alone or receiving an HTTP success is not verification.
11. **Failure/timeout:** report refused/failed/unknown honestly. A pre-mutation
    refusal must not leave an orphan hold; a possibly partial mutation retains a
    durable safety hold and requires explicit recovery. Never blindly retry an
    ambiguous operation or launch an automatic redeploy after an unknown outcome.
12. **Persistence:** ordinary restart loads the last verified committed active
    configuration, never merely saved/unapplied desired targets. Keep immutable
    history, compare-and-swap revision checks, request deduplication and audits.
13. **No unintended action:** never automatically arm manual input, replay held
    input, resume a goal/patrol, enable motors, undock, set a pose or reset a host.

### Meaning of idle without odometry

Command idle is **not proof of physical standstill**. With motors on and no fresh
odometry, software cannot claim measured zero motion. UI text and evidence must
say command idle/maintenance, not "stationary verified". Explain this operational
limitation without silently reintroducing an odometry prerequisite or requiring
motors off. Do not fabricate fresh odometry or remove odometry gates from normal
driving/navigation/watchdog functions; this change is confined to tuning admission.

## 3. Non-negotiable invariants

- Ordinary-drive hard envelope: **0.30 m/s and 0.70 rad/s**. Preserve lower
  producer-specific ceilings; do not raise them to make a test pass.
- Mux priority: safety **10** > undock **9** > F710 **8** > web **7** > nav **5**.
- `/cmd_vel_out` has only `cmd_vel_mux`; `/cmd_vel` has only
  `teleop_velocity_smoother` as publisher. Activation helpers must not become new
  velocity publishers or bypass the existing motion owner.
- No weakening of E-stop, collision, watchdog, docking, ordinary-drive raw-origin
  or manual-drive freshness protections.
- Keep live observations independent of saved and verified revisions. Their
  three-second live freshness/session-generation boundary remains unchanged.
  Expired values may be labelled last-known, never used to authorize motion.
- Keep signing secrets server/core only. Browser and web bridge receive no key.
- Standard Docker deployment must retain all merged capabilities, especially reset
  socket/group wiring. No optional feature overlay may be needed to restore them.
- No arbitrary shell, service names, filesystem paths, Docker socket or SSH
  credentials exposed through the dashboard/web bridge to implement fallback.

## 4. Source state at handoff

Both repositories were clean on main before creating this documentation branch:

| Repository | Path | Merged main |
| --- | --- | --- |
| Robot | `/home/yousef/Projects/repos/patrolbot-repo` | `f710cc814fe1cd0cb4ebb9b7aa1a4810a96ee18e` |
| Dashboard | `/home/yousef/Projects/repos/patrolbot-dashboard` | `c168dce244af8adb40aa5d0fc10d51822fe968cd` |

- Robot PR **#78** merged the snapshot read-cost fix, reading each immutable
  revision once per response without caching validation across requests. Local
  benchmark was about 280 ms -> 56 ms; 39 service/store tests passed; contract CI green.
- Dashboard PR **#37** merged the compact Advanced layout, corrected label,
  live cap markers, pending badges and single-flight polling. All **25 PR checks**
  passed, including all six browser shards; 487 frontend unit tests passed locally.
- That PR still has autosave commit triggers: the new explicit-Save behavior is
  future work. Its Apply button currently says **Apply saved changes**.
- WebKit CI needed test-only startup-race fixes: wait for the header before
  desktop/mobile branching; include teleop capabilities in every fixture startup
  snapshot rather than relying on a message to StrictMode's discarded socket.
  No production safety gates were changed by those fixes.
- **Neither #37 nor #78 has been deployed.** A merge is not runtime evidence.
- The local synthetic preview is launched with `npm run preview:advanced` in the
  dashboard `frontend/`, at
  `http://127.0.0.1:4180/advanced-preview.html?advanced=1`. It may need restarting.
  It blocks real API/robot traffic and is excluded from production build output.

### Last recorded deployment, not a fresh live inspection

- Public dashboard: `https://patrolbot-dashboard.qatar.cmu.edu`.
- Core runtime source `964d76cf507b6b8d970065f893c6c867dd44fddc`;
  image `sha256:ce223e07453d396dc74403874241df52e09a00612dd73b033b22d4fe432d3ea4`.
- Robot web bridge image
  `sha256:12e7d93d38c4fdb0e7696115d394f9bba267ff44db0b1663303e06027af218f4`.
- Dashboard source `cc1d006efbda1db303308477bdecb8e12ec916cd`;
  image `sha256:5549235330f88ee7cf4c0dfad487f6b99c465876f652357a12a20532d8f942c3`.
- Standard Compose reset wiring is from robot PR #77, revision
  `6e0f6f3d8a3efac883518cc30ee50880b971746f`. Supervisor socket
  `/run/patrolbot-system-reset/supervisor.sock`, dedicated GID recorded as **988**,
  read-only mount. No software reset was executed.
- `PATROLBOT_TUNING_ENABLED=1`, `PATROLBOT_TUNING_RUNTIME_ENABLED=0`.
  Signing key provisioned privately, 64 bytes/mode 0600; initialized robot store
  `/var/lib/patrolbot/config`, last verified active revision recorded as **1**.
  Desired revisions may have changed since then: read current state before use.
- Last observation: all 37 fields live, 0.30/0.70 caps; runtime apply unsupported
  intentionally. Navigation inactive awaiting an operator seed; never restore it.
- Historical evidence (ignored workstation files):
  `backups/20261021-worker-release/release-notes.md` and
  `backups/20261021-standard-reset-release/deployment-notes.md`.
- Last recorded robot rollback snapshots:
  `/home/ubuntu/backups/20261021T182834Z-pre-readback-worker-retry` and
  `/home/ubuntu/backups/20261021T185213Z-standard-reset-wiring`.
  Dashboard snapshot:
  `/srv/patrolbot-dashboard/backups/20261021T120731Z-pre-readback-cc1d006`.

Recheck these facts during an authorized preflight. Older readiness documents
contain obsolete deployment/SBC status and the superseded fresh-odom apply rule.

## 5. Code map and known implementation gaps

Robot paths below are relative to `ros2_ws/src/patrolbot_bridge/patrolbot_bridge/`:

- `tuning_service.py`: read/save/apply dispatch, authorization boundary and snapshots.
- `tuning_store.py`: immutable revisions, manifests, promotion, startup artifacts,
  journals, audits and request deduplication. `promote()` currently requires all
  scopes: scoped activation needs a sound atomic full-revision proof, not merely
  relaxing this check.
- `tuning_activation.py`: `Evidence`, `evaluate_evidence`, coordinator and durable
  holds. `REQUIRED_FRESH` still includes `raw_odom_fresh` and `stationary`; it
  reloads all `SCOPES`. These are not the newly approved admission/scoping rules.
- `tuning_runtime.py`: explicit physical/web releases, publisher identity health,
  generation invalidation and final command-buffer observations.
- `bridge_node.py`: `_tuning_evidence`, `_tuning_save_admission`, transport/backend
  wiring, retained observation worker and navigation admission recovery. Inspect
  all transitive evidence producers for hidden odom/localization prerequisites.
- `tuning_backend.py`: fixed-node lifecycle transport, generated parameter plan,
  exact readback and map/seed handling. The backend exists already; do not replace
  it with a dummy or assume activation is just flipping an environment flag.
  Current reload always cleans both managers/all tuning nodes, requires a selected
  map even for speed changes, and unconditionally requires a new seed. Its final
  verification also requires raw odometry/stationarity. Refactor all these stages.
- `tuning_policy.py`, `config_bootstrap.py`, `tuning_run.py`,
  `tuning_recover.py`: active-only startup, bounded cached drive policies, consumer
  exclusion and explicit offline recovery. Preserve their safety properties.
- Existing `EpochRuntime` tuning hold, navigator admission service and smoother
  maintenance owner must remain the only authorities for their respective work.

Other robot paths:

- `ros2_ws/src/patrolbot_navigation/`: F710 cached policies, final smoother
  management, navigation lifecycle integration and tests.
- `docker/docker-compose.yml`, `docker/system-reset/`: standard deployment and
  existing restricted supervisor patterns; do not conflate reset with tuning apply.
- `contracts/robot-tuning.*`, `contracts/test_deployment_features.py`: shared
  protocol/registry and standard-deployment regression tests.

Dashboard paths relative to `/home/yousef/Projects/repos/patrolbot-dashboard`:

- `frontend/src/components/TuningPanel.tsx`: explicit Save migration, field drafts,
  confirmation/review/status UI; `AdvancedPage.tsx`, `ManualDrive.tsx`,
  `SpeedGauges.tsx`: layout and read-only feedback/safety integration.
- `frontend/src/stores/tuningStore.ts`: group commit, CAS/draft conflict handling,
  exact reviewed revision/job status and pending derivation. Migrate autosave to
  an explicit batch without replaying drafts on reconnect or losing failed drafts.
- `frontend/src/lib/tuning.ts`, `useTuningReadback.ts`, `frontend/src/api/tuning.ts`:
  normalization, freshness/session binding, validation and API transport.
- `server/app/tuning/`: signed administrator requests, validation and relay.
- `ros2_ws/src/patrolbot_web_bridge/`: ROS/WebSocket/API transport integration.
- `frontend/dev/advancedPreview.tsx`, `frontend/e2e/fixtures.ts`,
  `frontend/e2e/{advanced,tuning}.spec.ts`: synthetic preview and browser acceptance.
- `contracts/`: vendored shared artifacts; keep all eight artifacts byte-identical
  when modifying the shared contract. Robot-side contract lands before dashboard
  parity checks against main.

## 6. Implementation phases and exit gates

### Phase A — establish local baseline and refine the design

1. Read repository/ancestor `AGENTS.md` if present, inspect Git status and current
   mains, and preserve all unfamiliar work. Start implementation topic branches
   from the latest merged main, not this documentation branch by accident.
2. Inspect the code map and inventory each editable field, derived consumers,
   lifecycle/parameter mutability, manager coupling and required verification.
   Do not infer hot-reload support from successful ROS SetParameters alone.
3. Write a reviewed dependency/activation matrix: field -> actual changed values
   -> consumer closure -> smallest supported operation -> state lost -> readback.
   Speed-ceiling closure must cover core clamp, mux/final-smoother semantics,
   F710 cache, web sessions and affected Nav2 producers, but not unrelated AMCL.
4. Specify structured refusal/status reasons and a previewable activation plan,
   including exact revision/digest, node/service allowlist and possible seed loss.
   If execution scope changes, require a new review instead of silently escalating.
5. Define command-idle evidence independent of odom/localization. Include inactive
   navigation awaiting a seed as a potentially valid idle state; do not make a
   selected map a universal prerequisite for speed-only tuning.

**Exit:** documented state transitions and dependency matrix with no unresolved
claim that a fresh odometry sample or localization is needed for apply.

### Phase B — explicit Save and truthful UI (local first)

1. Remove network save triggers from blur, Enter and slider debounce/release.
   Preserve precise numeric drafts, bounds and coupled-field validation.
2. Add Save changes, dirty count and clear distinction between unsaved draft,
   durably saved/pending target and verified active/observed values. Disable Apply
   while unsaved/conflicted drafts would make the review ambiguous.
3. Save a complete valid batch with administrator signing, CAS and fresh link
   admission. Keep invalid/conflicted/unknown-result drafts visible. Acknowledge
   confirmed saves only; never auto-resend on reconnect, refresh or timer ticks.
4. Extend synthetic preview to cover explicit save, apply review, refused/unknown
   outcomes and node-versus-Docker impact; it must still never contact a robot.

**Exit:** tests prove editing/focus loss/slider release/Enter never causes a save;
one explicit click makes one canonical request; motors off and no localization/
odom do not block saving, but disconnected/stale SBC evidence does.

### Phase C — independent apply admission and scoped backend

1. Replace only tuning-specific raw-odom/stationary/localization prerequisites,
   including post-reload/commit checks, with approved command-idle admission.
   Keep all normal-drive safety gates and fresh non-odom evidence intact.
2. Journal ownership before remote reservations; atomically reserve navigation and
   input admission; hold all command sources. Recheck after writer/owner admission
   and before mutation. Existing goals refuse apply rather than being silently
   canceled. New goals/manual sessions cannot enter during the maintenance hold.
3. Execute the exact allowlisted dependency plan. Refresh only changed producer
   policies; use supported native mutation or scoped lifecycle/container reload
   as appropriate. Managers must not silently restart unplanned siblings.
4. Do not touch AMCL/map state for unrelated changes. If affected AMCL/map state
   must be recreated, journal navigation-seed admission separately from the job.
5. Verify changed consumers plus trustworthy unchanged revision evidence before
   atomic promotion. Do not report a whole revision active after partial proof.
   Hardware session/generation changes invalidate old proof even though a fresh
   odometry sample is not needed.
6. Release only this job's holds on verified success. Keep unrelated holds, require
   a new input arm/deadman press, and leave any navigation-seed hold until the
   operator supplies a new seed. Keep motor-enable state unchanged.

**Exit:** isolated tests pass with motors on/off, absent/stale odometry and no
localization; scope tests prove only required nodes/policies are touched.

### Phase D — restricted Docker fallback and restart/recovery

1. Add or extend a host-owned, fixed-operation allowlisted helper if required.
   Keep Docker privileges out of the web bridge/browser. Verify signatures,
   revision/digest, scope, local authorization, lock ownership and volume paths.
2. Choose fallback during preflight for a known unsupported scoped operation.
   Do not catch an arbitrary timeout/partial failure and silently redeploy.
3. Define the durable handoff when redeploy restarts its own coordinator: the job,
   command hold and exact staged candidate must survive process/container loss.
   Ordinary startup must still use committed ACTIVE, not DESIRED. Any temporary
   candidate boot requires a distinct exact-job/digest authorization and retained
   hold until verification/promotion; a leftover staging file is not permission.
4. Authenticate and verify the replacement runtime before committing active state.
   On interruption, partial startup, corrupt artifacts, lost volume or unknown
   result, retain the hold and offer explicit status/recovery rather than retry.
5. Preserve last-active/last-good artifacts and operator-reviewed recovery. Never
   reinitialize the store, discard history, or reset defaults to resolve a failure.
6. Keep fallback in standard Compose with regression coverage for tuning, maps,
   manual drive, LiDAR/motor control and software-reset wiring. A tuning fallback
   must not execute software reset or modify the SBC unless separately necessary.

**Exit:** crash/restart/failure injection proves durable holds and exact-revision
recovery; ordinary restart with an unapplied desired change loads only ACTIVE.

### Phase E — validation, PRs and local acceptance

- Portable/unit suites for robot tuning store, authorization, activation, runtime
  evidence, backend dependency plans, startup/recovery and bridge/navigation safety.
- Frontend store/component/API tests, server/web-bridge tests, TypeScript checks,
  production build/bundle budget and synthetic browser acceptance on Chromium and
  WebKit across desktop/tablet/mobile. Do not skip a failing safety assertion to merge.
- Native ROS integration in an isolated domain/network, fake hardware only, no
  production devices/state/secrets. Exercise actual pinned lifecycle/parameter
  behavior, managed-node coupling and callback progress under concurrent readback.
- Container tests for the host fallback, standard Compose prerequisites,
  restart recovery and sole-publisher graph. Treat native/image behavior as
  unverified until tested, regardless of fake-backend success.
- PRs -> all green checks -> merge. Coordinate contract parity across repositories.
  Test-only startup-race fixes are acceptable; do not mask failures with retries
  or increase safety freshness windows to get a green result.

### Phase F — operator-authorized release and commissioning

1. Obtain explicit deployment authorization. Re-inventory real image/config/store
   state read-only; record fresh rollback snapshots and exact merged revisions.
   Do not overwrite existing desired revisions or reuse historical state blindly.
2. Build/deploy merged revisions only, RPi for robot ROS changes and dashboard
   server as required; SBC only if an actual SBC change is required. Preserve all
   merged capabilities through standard Docker. Do not combine this release with
   unrelated motion, reset, seed or motor-enable operations.
3. Runtime apply remains off until prerequisites/native evidence are reviewed and
   the operator authorizes enabling commissioning. Enabling a flag alone is not
   a pass. Preflight must not require fresh odometry/localization or motors off.
4. Obtain explicit authorization for each real save/apply/redeploy acceptance
   scenario, naming the exact parameter, reviewed revision, scope and impact.
   Start with a benign reversible reviewed change; do not choose values silently.
5. Verify scoped update, genuine restart-only fallback, readback and restart
   persistence. If localization is invalidated, let the operator seed explicitly.
   No old goal/input resumes; no motor command or test movement is implied.
6. Record images, revisions, before/after values, actual nodes restarted,
   command/publisher evidence, job journal and recovery proof. Physical motion
   trials need separate explicit authorization and are not required to claim a
   successful no-motion parameter reload.

## 7. Minimum acceptance matrix

| Scenario | Required result |
| --- | --- |
| Edit, blur, Enter, slider release, refresh/reconnect | Draft only; zero implicit save/apply requests |
| Save with motors off, missing odom/localization, navigation active | Allowed if administrator/store/link checks pass; no runtime change |
| Apply with motors ON or OFF, missing/stale odom, no seed | Allowed if non-odom command-idle evidence is valid |
| Missing/stale link, hardware identity, release or idle ownership | Clear refusal; never infer permission from missing messages |
| Goal/manual/undock/map/lifecycle activity; concurrent apply | Refusal without stealing/canceling the active operation |
| New goal/input racing apply admission | Exactly one owner wins; no command bypass |
| Speed-only or other unrelated-to-AMCL change | Minimal supported dependency reload; no AMCL/map reset/seed demand |
| Coupled AMCL field edits | Batch validates atomically; only necessary localization dependencies reload |
| Full Docker fallback known necessary | Shown before confirmation; fixed scope; durable job/hold across restart |
| Lost response, crash or timeout after possible mutation | Unknown result, durable hold, status query, no blind retry/fallback |
| Verified apply invalidates localization | Parameters active, navigation awaiting operator seed; no automatic pose/goal |
| Desired saved but not applied, then restart | Prior verified ACTIVE config runs; pending desired remains pending |
| Corrupt/missing store/artifact or changed session | Fail closed; no YAML-default substitution or fabricated verification |
| Cap change | Derived producers/caches verified; hard 0.30/0.70 and mux/publisher invariants preserved |
| Invalid, unauthorized, unsigned or conflicting revision request | Refused/conflict, no runtime mutation or silent overwrite |

## 8. Fresh-session starter prompt

> Read `docs/save-apply-commissioning-plan.md` in
> `/home/yousef/Projects/repos/patrolbot-repo` completely. It contains the approved
> specifications and implementation/commissioning gates. Dashboard repository:
> `/home/yousef/Projects/repos/patrolbot-dashboard`. Inspect current Git status and
> source baselines, preserve all unfamiliar work, and start with local Phase A.
> Implement explicit separate Save changes and Apply changes. Apply must NOT
> require fresh odometry/localization or motors off; motors may remain on while
> command idle. Use minimal affected-node reload with a reviewed full robot
> Docker redeploy fallback where necessary. Preserve all safety/authorization,
> active-only startup, exact-revision verification and durable recovery rules.
> Do not deploy, enable runtime activation, issue live apply/redeploy/reset,
> motion, goal, pose or motor-enable commands without separate operator approval.
> Follow branch -> PR -> green CI -> merge -> authorized deployment. Do not
> delegate to subagents unless the operator or applicable instructions ask for it.
