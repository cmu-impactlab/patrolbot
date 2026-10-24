# Save/apply: Phase A local design and dependency inventory

**Recorded:** 2026-10-22. **Status:** historical Phase A source-reviewed design
candidate. Subsequent implementation: [local progress](save-apply-local-progress.md).
Operator review and native acceptance are not claimed. The approved
[commissioning handoff](save-apply-commissioning-plan.md)
remains authoritative; this document refines implementation, not authorization.

**Superseded execution policy (2026-10-23):** the per-node lifecycle (`L`/`A`)
and broad Docker fallback choices below are historical. The current policy is
verified hot updates or one whole-navigation-container restart, with input
discarded during installation. Use the current
[operations/mutability matrix](save-apply-phase-c-operations.md) and
`tuning_mutability.py`; this historical inventory does not enable runtime support.

No robot inspection, live save/apply, activation enablement, deployment, reset,
motor command, motion, goal or pose operation was performed. No subagents used.

## 1. Local baseline

No ancestor/repository `AGENTS.md` was found in either checkout. Both working
trees were clean. `git fetch origin main` confirmed these latest merged mains:

| Repository | Source baseline | Local topic branch |
| --- | --- | --- |
| Robot | `f710cc814fe1cd0cb4ebb9b7aa1a4810a96ee18e` | `feat/save-apply-commissioning` |
| Dashboard | `c168dce244af8adb40aa5d0fc10d51822fe968cd` | `feat/save-apply-commissioning` |

Both topic branches were created explicitly from `origin/main`. The existing
handoff-only commit `08055f4` was carried onto the robot topic as `412eda5`.
Other branches/worktrees and ignored local evidence were left alone. Neither
topic was pushed. This phase changes documentation only, not shared contracts,
runtime code, deployment settings or secrets.

## 2. Source findings that constrain implementation

Robot module names here are relative to
`ros2_ws/src/patrolbot_bridge/patrolbot_bridge/`.

| Source | Current behavior / required change |
| --- | --- |
| `tuning_service.py`, `tuning_store.py` | Save already supports an atomic batch, administrator admission, desired-revision CAS and durable request deduplication. Maximum batch 32 fits all **27 editable fields** (37 total). Keep this path; Save must never call the backend. |
| `tuning_activation.py` | Requires raw odometry/stationarity, calls `reload(sorted(SCOPES), ...)`, and returns “stationary and idle”. Replace tuning-only predicates and use an exact reviewed execution plan. Pre-mutation refusals must unwind acknowledged reservations, including the last recheck before `reload`. |
| `tuning_backend.py` | `ready()` requires every fixed service, preflight requires a selected map, reload cleans both managers/all nodes, restores the map, always requests a seed, and checks odometry/stationarity again. Split readiness, preflight, mutation, verification and seed impact by plan. |
| `bridge_node.py::_tuning_evidence` | Explicit odometry predicates; `maps_clear` uses `epoch.map_hold`, which also means **no selected map**; `autonomy_idle` rejects `_tuning_nav_pending`. Use tuning-specific ownership predicates, not normal navigation-readiness predicates. |
| `epoch_runtime.py::_set_tuning_hold_source` | Any job **or policy** hold clears `_pending_pose`, `_accepted_seed_stamp` and `_navigation_ready`. A non-localization maintenance hold must block commands without invalidating already accepted localization. Do not “restore” these values by replaying a saved pose. |
| `epoch_runtime.py::tick`, `map_runtime.py` | Hold stops ordinary lifecycle work; fresh navigator idle service polling does not require odometry. Post-seed admission is a separate durable record. Keep normal driving/map/seed gates; distinguish a quiet wait-for-seed from an in-flight recovery mutation. |
| `tuning_runtime.py`, bridge publisher checks | Explicit physical/web/session release observations, 0.5 s health/release windows, generation invalidation and retained final zeros already exist. Final zero alone is not acknowledgment from every upstream producer or competing owner. |
| `tuning_policy.py` | Core clamp reads committed startup/ACTIVE only. A candidate-policy handshake under a job hold is needed to verify changed caches before promotion; Save must never feed this cache. |
| Navigation `scripts/patrolbot_joy_teleop.py` | Has validation **and post-set** cached-policy callbacks, `tuning_policy_version`, idle acknowledgment and release-before-rearm. Parameter getters alone do not expose actual lower operator trims/cached execution values; add bounded verification evidence. |
| Dashboard web bridge `teleop.py`, `tuning.py` | Web sessions stop on hold or policy generation change. `safety()` correctly requires odometry/motors for **driving**; do not reuse it for tuning admission. Keep policy freshness/session checks and never restart a session automatically. |
| Core hardware operation admission | `_hardware_operation_lock` / `_control_lock`, LiDAR pending work and control futures are outside today's tuning evidence. Integrate mutation exclusion; do not bypass emergency disable or independent failsafes. |
| `tuning_store.py::promote`, `tuning_startup.py`, `tuning_run.py` | Promotion requires both consumer scopes; restart resolves committed ACTIVE only. Keep whole-revision promotion and consumer exclusion. Scope of execution is not scope of proof. |
| `tuning_store.py::load_revision` | Registry/default/compiler digests are compatibility boundaries. Changing the registry, compiler or packaged defaults needs an explicit history-preserving migration design, not reinitialization or new defaults. |
| Dashboard `TuningPanel.tsx`, `tuningStore.ts` | Blur/Enter/slider timers/release call `commitGroup`; Apply uses the old redeploy dialog and inaccurate stationary/scoped-only language. Review captures revision/digest, but displayed diffs use the latest snapshot. Freeze the entire review and bind it server/core-side. |

### Pinned native source inspection (not a native test)

Inspected Navigation2 **1.3.12**, matching the intended source in
`docker/Dockerfile.nav2-fixes`. Downloaded archive SHA-256 was verified against
the recipe: `bcdf575c86250d63007595c992b9c5785c3ec907e897b79abfb4fc7eacbd1129`.
The inspected files matched that archive byte-for-byte. References below use
[this upstream tag](https://github.com/ros-navigation/navigation2/tree/1.3.12).
Local copies are under `/tmp/opencode/save-apply-phase-a-upstream/`.

- `nav2_amcl/src/amcl_node.cpp`: dynamic handler is installed on activate and
  removed on deactivate. Alpha updates rebuild the motion model; update thresholds
  update members; beam/laser updates rebuild filters. Particle bounds / `pf_err` /
  `pf_z` call `initParticleFilter()`, which initializes from retained `init_pose_`
  and covariance. **Do not use that callback as a safe pose-preserving particle
  reset.** Use a guarded reset requiring a new operator seed, with old-pose reuse
  disabled and tested against the existing AMCL patches.
- `nav2_regulated_pure_pursuit_controller/src/parameter_handler.cpp`: callback
  updates cruise/base cruise, lookaheads, rotation, approach minimum and regulated
  minimum speed. `approach_velocity_scaling_dist` is read at construction but is
  **absent** from this callback: a successful SetParameters can leave it cached old.
- `nav2_velocity_smoother/src/velocity_smoother.cpp`: active callback updates the
  complete max/min velocity arrays; callback is removed on deactivate. Setting
  parameters while merely inactive then activating is not proof of refreshed
  cache; cleanup/configure or proven active callback is required.
- `nav2_behaviors/plugins/spin.cpp`: `onConfigure()` caches both rotational
  maximum and minimum; no dynamic update is present there.
- `nav2_lifecycle_manager/src/lifecycle_manager.cpp`: `node_names_` is cached at
  construction, manager transitions iterate all children. Editing `node_names`
  via SetParameters is not a supported subset operation.

The standard `docker/Dockerfile` uses unpinned apt Nav2 and does not build this
patched overlay. Neither source inspection nor existing fake transports prove
the installed image's behavior. Phase E must test the exact intended image,
patch set and package versions, including standard deployment capability parity.

## 3. Dependency / activation matrix

The plan compares **compiled full ACTIVE and candidate artifacts**, not only
logical edited IDs or top-level restart scopes. Complete typed ROS array values
are compared/written; scalar ROS types follow the catalog (a numeric `0` must
not accidentally become an integer parameter). Unchanged effective values are
verified but not rewritten. Logical target changes still become pending history
even when their current effective value is cap-clamped.

Notation (operations below are design targets, **not commissioned support**):

- **H**: native atomic parameter batch with a source-backed callback, only in a
  proven lifecycle state, under acknowledged maintenance and zero command buffers.
  Requires consumer/cache evidence, not just ROS parameter acceptance/readback.
- **L**: scoped deactivate if active -> cleanup if configured -> set coherent
  parameters -> configure -> return to the prior allowed lifecycle state. Never
  activate a previously inactive navigation stack merely to apply configuration.
- **P**: exact-job candidate cache refresh/acknowledgment under the existing hold;
  committed ACTIVE binding must be checked before release. No arm/drive effect.
- **A**: guarded AMCL reset (L with existing no-old-seed safeguards); localization
  becomes unavailable until a **new operator seed**, never an internal pose replay.
- **V**: exact parameter readback plus actual callback/cache evidence, lifecycle
  state/identity checks and unchanged full-revision verification (§6).

H is selected only after pinned native evidence proves semantics. Otherwise use
the smallest proven L; if that cannot be coordinated safely, select the reviewed
robot Docker fallback **before confirmation**, or refuse unsupported activation.
Missing odometry/localization/motor enable is never the reason to choose fallback.

“Prior allowed lifecycle state” does not add an indirect odometry/TF prerequisite:
if normal navigation activation cannot currently pass its independent readiness
gates, verify the configured parameter/cache state and leave navigation inactive
with a separate readiness status. Do not require navigation to become ready to
declare the parameter revision verified, and do not remove its normal gates.
This must not demand a new seed when accepted localization was unaffected.

### All 27 editable fields

Rows name the logical ID suffix within the indicated group. `C` is the candidate
linear cap and `W` the candidate angular cap; `target(k)` is its saved logical
target (otherwise its reviewed source value). Min operations below retain lower
producer-specific targets; they never raise a target to match a global cap.

| Field | Actual changed values / consumer closure | Smallest source-backed candidate operation | State lost / required readback |
| --- | --- | --- | --- |
| `max_linear_m_s` | Final smoother `max_velocity[0]=C`, `min_velocity[0]=-C`; core clamp; F710 linear cap/trim/minimum; web policy/session; affected RPP cruise, approach/regulated minima; Nav2 smoother forward/reverse bounds | H or L on affected smoothers/controller + P core/web + F710 atomic cached-policy update | Only L cache state and explicitly disarmed inputs; **no AMCL/map/seed loss**. V all caps, symmetric bounds, core/F710/web caches and mux semantics |
| `max_angular_rad_s` | Final smoother `max_velocity[2]=W`, `min_velocity[2]=-W`; core clamp; F710 angular cap/trim/minimum; web policy/session; affected RPP rotation target; Nav2 smoother angular/reverse bounds; Spin max and derived min | H/L affected smoothers/controller; L behavior if compiled Spin bounds change; P core/web/F710 | Behavior plugin/cache state if L; disarmed inputs; **no AMCL/map/seed loss**. V all angular producers including Spin cache |
| `amcl.alpha1` | `amcl.alpha1`; motion-model cache | H active, otherwise L/A if required by proven state | H rebuilds motion model/reference, not particle filter; V model coefficients/reference semantics; L/A loses seed |
| `amcl.alpha2` | `amcl.alpha2`; motion-model cache | Same as alpha1 | Same as alpha1 |
| `amcl.alpha3` | `amcl.alpha3`; motion-model cache | Same as alpha1 | Same as alpha1 |
| `amcl.alpha4` | `amcl.alpha4`; motion-model cache | Same as alpha1 | Same as alpha1 |
| `amcl.update_min_d` | `amcl.update_min_d`; distance update threshold | H active, otherwise proven L/A | H threshold only; V consumed threshold; L/A loses seed |
| `amcl.update_min_a` | `amcl.update_min_a`; angle update threshold | H active, otherwise proven L/A | H threshold only; V consumed threshold; L/A loses seed |
| `amcl.min_particles` | `amcl.min_particles`; particle-filter allocation; validate min <= max as a batch | A, not unguarded dynamic particle reset | Particle/filter/localization state; V bounds/allocation and new-seed admission barrier |
| `amcl.max_particles` | `amcl.max_particles`; particle-filter allocation; validate min <= max | A | Same as min_particles |
| `amcl.max_beams` | `amcl.max_beams`; laser/filter cache | H active only with proven filter recreation, otherwise A | Laser/filter cache; preserve valid particle state only if proven; V consumed beam count; A loses seed |
| `amcl.pf_err` | `amcl.pf_err`; particle-filter KLD population error | A | Particle/filter/localization state; V consumed KLD value and seed barrier |
| `amcl.pf_z` | `amcl.pf_z`; particle-filter KLD quantile | A | Same as pf_err; positive quantile, not a probability capped at 1 |
| `amcl.sigma_hit` | `amcl.sigma_hit`; laser-model cache | H active with proven cache recreation, otherwise A | Laser/filter cache; V consumed sigma; A loses seed |
| `amcl.z_hit` | `amcl.z_hit`; laser-model cache; coherent sum with z_rand | One coherent H batch active, otherwise A | Laser/filter cache; V both weights and sum; A loses seed |
| `amcl.z_rand` | `amcl.z_rand`; laser-model cache; coherent sum with z_hit | Same as z_hit | Same as z_hit |
| `nav2.controller.desired_linear_vel` | `FollowPath.desired_linear_vel=min(target,C)`; base-cruise cache; derived approach minimum and `regulated_linear_scaling_min_speed` | H controller, otherwise scoped L controller | H policy only; L also controller plugin/local-costmap execution state. V cruise/base cruise and both derived minima |
| `nav2.controller.lookahead_dist` | `FollowPath.lookahead_dist`; RPP cache | H controller, otherwise L | V cached value and coherent lookahead tuple; L resets controller-local state |
| `nav2.controller.min_lookahead_dist` | `FollowPath.min_lookahead_dist`; validate min <= max and base <= max | H coherent controller batch, otherwise L | Same as lookahead_dist |
| `nav2.controller.max_lookahead_dist` | `FollowPath.max_lookahead_dist`; incident lookahead constraints | H coherent controller batch, otherwise L | Same as lookahead_dist |
| `nav2.controller.lookahead_time` | `FollowPath.lookahead_time`; RPP cache | H controller, otherwise L | V cached value; L resets controller-local state |
| `nav2.controller.rotate_to_heading_angular_vel` | `FollowPath.rotate_to_heading_angular_vel=min(target,W)`; RPP cache | H controller, otherwise L | V rotation cache and downstream angular bound; not a bound on all curved-path output |
| `nav2.controller.min_approach_linear_velocity` | Minimum of saved approach target, C and effective cruise | H controller, otherwise L | V consumed approach minimum (>0.01), <= cruise; L resets controller-local state |
| `nav2.controller.approach_velocity_scaling_dist` | `FollowPath.approach_velocity_scaling_dist`; construction-time RPP cache | **L controller**, not H on inspected source | Controller plugin/local-costmap state; V recreated plugin cache and geometry-dependent bound; preserve AMCL/map server |
| `nav2.velocity_smoother.max_velocity_x` | `velocity_smoother.max_velocity[0]=min(target,C)`, symmetric `min_velocity[0]`; incident cruise constraint | H active, otherwise L smoother | V complete arrays and consumed cache; L resets smoothing history, not localization |
| `nav2.velocity_smoother.max_velocity_theta` | `velocity_smoother.max_velocity[2]=min(target,W)`, symmetric `min_velocity[2]` | H active, otherwise L smoother | Same as smoother x |
| `nav2.behavior.max_rotational_vel` | `behavior_server.max_rotational_vel=min(target,W)`; `min_rotational_vel=min(0.4,effective max)`; Spin plugin cache | **L behavior_server** | Behavior plugin state; V max/min in actual Spin cache; preserve AMCL/map server |

No other fields become editable. Keep `amcl.alpha5`, protected safety gates,
laser model and topology read-only. In particular, neither raw-odom/watchdog
timeouts nor collision freshness nor mux priorities are tuning workarounds.

### Derived closure and manager coupling

- Compiler formulas in `tuning_spec.py`: all speed producers are bounded by the
  global cap; RPP approach minimum is also bounded by effective cruise;
  `regulated_linear_scaling_min_speed=min(0.10,effective cruise)`; Spin minimum
  is bounded by its maximum; F710 minima are `min(0.05,C)` / `min(0.1,W)`.
  Preserve lower operator trims when refreshing F710 limits.
- Core P and web P must cover policy values **and** generation/session binding.
  End/disarm every existing web manual session; never recreate it. Unchanged
  policies may retain values, but their evidence must be bound to the new proof.
- Mux is a verification dependency, not an editable speed parameter or an excuse
  to restart it: safety 10 > undock 9 > F710 8 > web 7 > switch 6 > nav 5;
  `/cmd_vel_out` only `cmd_vel_mux`, `/cmd_vel` only `teleop_velocity_smoother`.
- Localization manager children: `map_server`, `amcl`. Navigation manager children:
  `controller_server`, `smoother_server`, `planner_server`, `route_server`,
  `behavior_server`, `velocity_smoother`, `collision_monitor`, `bt_navigator`,
  `waypoint_follower`. Controller cleanup also owns its local-costmap/plugin
  subtree. A manager-wide transition is **not** a single-node operation.
- For L/A, extend the existing manager authority with fixed-subset maintenance
  ownership/acknowledgment, or prove direct transitions safe under its exclusion.
  Do not introduce a competing coordinator, silently restart siblings, or change
  its cached `node_names` parameter. If subset coordination is unsupported on the
  pinned image, preview a proven broader operation/fallback or refuse it.
- AMCL reset does not automatically require map-server reload. Re-consume a
  retained selected map when necessary; restoring map-server state is only an
  explicit dependency of an operation that actually destroyed that state.

Synthetic compiler-only checks (not selected live values) confirmed linear cap
0.20 changes final/Nav2 smoothers, RPP cruise and F710; angular cap 0.50 additionally
changes Spin and RPP rotation. Neither changes any AMCL parameter. A synthetic
cruise 0.06 changes both derived RPP minima as well as cruise; that delta alone
is **not** a valid one-field Save batch when the saved approach target is higher.

## 4. Admission: command idle, independent of odometry/localization

**Save:** signed current administrator, initialized writable store, session/CAS
match and fresh SBC link (0.5 s), rechecked after writer admission. Navigation
may be active. Motors, odometry, seed, map and localization are not prerequisites.

**Apply:** fresh non-odom evidence plus exclusive ownership. Admission and final
checks must not call `commands_allowed`, `seed_tf_ready`, manual-drive `safety()`,
or a generic navigation/map readiness predicate. Remove tuning-only
`raw_odom_fresh`/`stationary` checks at **all** stages, not just the first probe.

| Evidence / authority | Required tuning interpretation |
| --- | --- |
| Hardware/link | Fresh flags/charge and authenticated current SBC/session identity, no applicable faults/E-stop/bumper fault, healthy dock state. Existing `guard.ready` establishes origin identity from an announcement, **not** a fresh odometry sample. Do not synthesize or relax origin checks. |
| Dock | Keep fresh authoritative dock health (current implementation requires CLEAR_CONFIRMED). Hardware dock confirmation has its own independent safeguards; do not drive/undock/reset the dock to satisfy tuning. No new ROS odometry test is added. |
| Navigation | Fresh authoritative idle evidence and atomic reservation against **all** goal/action paths; no current goal, patrol/waypoint/behavior/undock owner. Configured inactive navigation waiting for an operator seed is potentially valid idle. Never cancel an existing goal to win admission. |
| Map / localization | No in-flight map mutation, map transaction owner, ambiguous map operation or conflicting lifecycle work. No selected map or missing/stale localization is **not** busy by itself. Existing pending seed barrier can remain while applying unrelated parameters. A map identity is required only by a plan step that consumes/restores that map. |
| Physical/web input | Explicit fresh released physical input, released web evidence and inactive web session, each bound to current producer identity; acknowledged producer hold and zero internal target/ramp buffers. Unknown or missing release refuses. |
| Other mux paths | Include `/input/switch` (legacy/manual), navigation/behavior, undock and existing safety producers in the fixed graph/ownership inventory. No publisher/message is not a fabricated release. Unsupported/unidentified producers refuse admission; require an authoritative disabled-path/empty-buffer proof or their own acknowledged hold. |
| Output buffers | Both final topics zero from the sole allowlisted publishers, with same-generation publisher health and post-reservation checks. Retained zeros are invalidated on identity/generation change. Include upstream/mux buffered work; zero final output alone does not prove all sources held. |
| Lifecycle / other mutations | No competing manager/epoch-recovery transition, hardware/LiDAR operation, reset or mutation owner. Obtain mutually exclusive reservations under their existing authorities, not a check-then-act Boolean. Explicit emergency disable/failsafes always retain precedence. |
| Motors | ON and OFF both admissible. Never call enable/disable or restoration from tuning. Unexpected external safety/session changes invalidate evidence; independent hardware failsafes are not suppressed to preserve motor state. |

Keep 0.5 s release/publisher/hardware checks (existing dock window 1 s) distinct
from **3 s live parameter observation** freshness. Do not increase either to pass
tests. Raw odometry, AMCL, seed and pose may be absent/stale throughout apply.
Normal driving/navigation/watchdog freshness gates remain unchanged afterward.

Required UI language: **“Command-idle maintenance; not measured standstill.
Motors may remain on. Without fresh odometry, software cannot verify physical
zero motion.”** Parameter activation and navigation readiness are separate.

## 5. Exact review, protocol and state transitions

### Reviewable plan (new protocol surface; not implemented)

Add an explicit read-only activation preview with a server/core-generated bounded
plan. It must be capability/version negotiated; old clients cannot bypass review
by calling the legacy `apply`/`redeploy` shape. Keep all eight shared contract
artifacts byte-identical when changing them; robot contract lands before dashboard
checks against main. Do not add shell/node/path selection to browser requests.

Freeze and hash a canonical plan containing:

- robot ID, core tuning session, compatibility/registry/compiler/default digests;
  exact desired revision + generated digest and base committed ACTIVE revision +
  digest; expected hardware generation and fixed service/node implementation IDs;
- logical before/after targets and compiled effective/derived before/after values;
  consumer closure, ordered fixed operations, node/service allowlist, state loss,
  required readbacks/cache acknowledgments and prior lifecycle-state policy;
- execution kind (`scoped_parameters`, `scoped_lifecycle`, `robot_docker`), explicit
  container list where applicable, seed/localization impact, disruption, whether
  any image change is actually required, and structured refusal reasons.

The confirmation sends only exact revision/digest + opaque plan ID/hash and a new
request ID within the signed administrator intent. Core rederives/validates the
same plan under writer/owner admission. Changed desired/base ACTIVE, generation,
compatibility, selected dependency identity, implementation support or execution
scope means **conflict / review required**, never “apply latest” or escalation.
Review creates no reservation and no permission to execute later without rechecks.
Its displayed diff/impact never follows polling to a newer revision.

### State transitions

| State / event | Next state / invariant |
| --- | --- |
| Edit, blur, Enter, slider change/release, refresh/reconnect | Draft only, zero mutation requests. Preserve precise text and invalid drafts. Count dirty values against exact saved targets. |
| Explicit Save click | Validate **all dirty fields as one coherent batch** against captured desired/session; send one canonical request with durable request ID. No activation/hold. |
| Save confirmed for exact request/batch/revision | Acknowledge only submitted draft versions; preserve edits made while request was in flight. Saved/pending is distinct from verified active and observed live. |
| Save conflict/refusal/failure/unknown | Keep drafts, base/session and request identity visible. Unknown blocks resubmission until an explicit status query resolves intent; no timer/reconnect replay. Explicit conflict review/rebase or discard, never silent overwrite. |
| Apply with unsaved/conflicted/in-flight/unknown drafts | Disabled; review cannot ambiguously mix unsaved inputs and saved targets. |
| Apply changes click | Fetch/freeze exact preview; show review only. Cancel is side-effect free. |
| Confirm review | Reauthenticate/sign; validate plan/CAS/dedup; record durable pending job **before any remote reservation**. Acquire local ownership, navigator/input/manager reservations in fixed order and recheck fresh evidence. |
| Definitive refusal before runtime mutation | Release only this job's acquired reservations; journal cleanup acknowledgment and refused outcome. No orphan hold and no localization invalidation. |
| Reservation/cleanup acknowledgment lost | Unknown, durable hold; query/recover explicitly. Do not presume no remote owner exists just because parameter mutation has not begun. |
| All reservations/zero buffers acknowledged | Journal running/mutation boundary; execute only the reviewed allowlist, recording per-step outcomes. |
| Successful runtime changes | Verify full candidate proof (§6), then atomically commit whole ACTIVE/startup. Verify committed policy bindings before releasing this job's holds and reporting succeeded. |
| Possible partial mutation, readback mismatch, timeout, crash or commit ambiguity | Unknown with a structured failure/uncertainty reason, durable hold and explicit recovery. No blind retry, no automatic Docker fallback. Startup/status must preserve exact job, scope and evidence. |
| Verified activation, localization dependencies unaffected | Preserve accepted localization in place and prior lifecycle state. Release maintenance only; no goal/patrol/manual session resumes. Normal freshness/readiness can still block navigation independently. |
| Verified activation with necessary localization loss | Job succeeded / parameters active; navigation awaiting **fresh operator seed** (or map/seed prerequisites). Persist seed admission separately; do not label the activation failed just because there is no seed. |

Existing job states (`pending`, `running`, `succeeded`, `failed`, `unknown`) can
remain, with bounded `stage`, `mutation_started`, reservation ownership and
verification metadata. `failed` may release a hold only when non-mutation or an
explicit recovery is proven. A possibly partial “failed” operation stays held.
Dedup fingerprints must include the entire reviewed intent, actor and plan hash.

Structured reasons should be stable data, with bounded explanatory text:

- authentication/store: `not_administrator`, `signature_invalid`,
  `store_uninitialized`, `store_unwritable`, `store_corrupt`, `link_stale`;
- conflict/support: `session_changed`, `desired_revision_changed`,
  `active_revision_changed`, `plan_changed`, `implementation_unverified`,
  `scope_unsupported`, `map_dependency_unavailable` (only applicable plans);
- ownership/evidence: `hardware_evidence_unknown`, `hardware_fault`,
  `dock_state_unsafe`, `navigation_busy`, `map_mutation_busy`, `undock_busy`,
  `lifecycle_busy`, `mutation_owner_busy`, `physical_input_not_released`,
  `web_input_not_released`, `producer_identity_unknown`, `buffers_not_zero`,
  `hold_not_acknowledged`;
- execution: `reservation_outcome_unknown`, `mutation_outcome_unknown`,
  `verification_mismatch`, `generation_changed`, `commit_outcome_unknown`,
  `recovery_required`.

Return reason evidence/source, stage, request/job ID, exact revision/digest/plan
hash, mutation/hold status and allowed next action (`review`, `status`, or
`explicit_recovery`). Do not parse reason prose as permission. No
`odometry_stale`, `localization_missing`, `not_stationary` or `motors_off` refusal
is introduced for tuning admission. Navigation-readiness reasons remain separate.

## 6. Full-revision proof and durable fallback boundary

Prefer a **fresh complete readback** of the small fixed tuning parameter set for
the first scoped implementation rather than trusting inherited observation
caches. Execution may touch one node while proof covers both consumer scopes.

1. Under the acknowledged hold, pin candidate, old ACTIVE, artifacts, identities,
   hardware/origin generation and plan hash. Verify protected hard envelope,
   publisher graph, manager exclusion and command-idle evidence.
2. Verify changed native consumer caches as well as their parameter values.
   For constructor-only caches, prove cleanup/configure and reread/ack the new
   execution policy; GetParameters alone cannot certify Spin/RPP cache refresh.
   Add fixed read-only cache/policy evidence or pinned native proof-backed
   generation acknowledgments; no motion is required to read those policies.
3. Stage only affected core/F710/web candidate policies through exact-job,
   digest-bound maintenance admission. Verify safe caps, lower trims, cleared
   buffers and inactive sessions. Candidate policies never authorize driving and
   never become ordinary startup config. Saved DESIRED is not a policy source.
4. Read all unchanged tuning consumers and relevant protected/derived values in
   the same proof interval. A mismatch cannot be “fixed” by claiming an untouched
   scope active; refuse before mutation if found early, otherwise retain the hold.
   Recheck generations and ownership immediately before promotion.
5. Produce a complete proof for both scopes, immutable revision/digest and exact
   job. `promote()` must require that proof; **do not relax its all-scopes guard**
   to accept only the executed node list. Commit startup/manifest atomically or
   fail closed on an interrupted commit as today; history and last-good survive.
6. Refresh/check committed ACTIVE policy binding under the retained hold, journal
   verified success, and acknowledge release of only this job's reservations.
   Release uncertainty is unknown/held. Keep policy/fault/map/seed holds belonging
   to other authorities. Release never injects seed/input/goal or enables motors.

Ordinary live observations remain independent and expire at 3 s/current session,
even when a durable verification record exists. The observation worker must stay
bounded and must not block the callbacks needed to finish service calls.

For Phase D, known unsupported scoped mutation may select `robot_docker` only
if a tested restricted host helper exists. Initial fixed core-container scope is
Compose `bringup`, `bridge`, `navigation` (actual containers `patrolbot-bringup`,
`patrolbot-bridge`, `patrolbot-navigation`). Additional robot web-bridge restart
must be necessary and explicitly in the plan; dashboard/SBC, gimbal and host reset
are not automatically included. Do not reuse software-reset operations for tuning.

The helper validates signed exact-job authorization, digest/scope, local policy,
durable locks and fixed volume identities; Docker privilege stays host-only.
Restarting the coordinator requires durable job/hold plus a separately authorized
candidate-boot capability for that exact job/digest. A staged file or DESIRED
pointer alone never permits candidate boot. Replacement runtime must authenticate
and verify before promotion. Ordinary restart loads only last verified committed
ACTIVE. Lost/corrupt state fails closed; never reinitialize or silently retry.
A reconnect may trigger independent hardware failsafes; never counteract them or
restore motor power. Disruption and fresh-seed consequences belong in the review.

## 7. Local validation and remaining gates

Baseline checks (no production state/devices/secrets):

- Robot contract + all `test_tuning_*.py` + `test_config_bootstrap.py`:
  **526 passed**.
- CI's ten portable teleop/navigation safety test files: **249 passed**.
- Dashboard `frontend/`: `npm test -- --run`: **487 passed / 38 files**.
- Dashboard `scripts/check_tuning_contract.py --core-repo` against this robot
  checkout: **8/8 shared artifacts byte-identical**.
- All 27 editable IDs inventoried; synthetic compiled cap/cruise dependency deltas
  inspected; pinned upstream archive checksum/source comparison passed.

System Python had no pytest; the pre-existing robot `.venv` lacked jsonschema.
Neither environment was modified. Robot tests used an isolated
`/tmp/opencode/save-apply-phase-a-venv` installed from
`contracts/requirements-test.txt` (Python 3.14; CI specifies 3.12).
These are baseline tests of the **old behavior**, not acceptance of this design.

Phase A's local design artifacts now cover all fields, state transitions,
preview/review binding and non-odom admission. Operator design review is not
claimed. Before claiming Phase C/E support, implement and prove:

- explicit Save/no autosaves; single-flight batch and in-flight edit preservation;
  no replay after reconnect, exact frozen review and conservative unknown handling;
- non-localization maintenance that does not clear/replay accepted localization;
  no-map / inactive-awaiting-seed admission; fresh non-odom evidence at every stage;
- atomic ownership races including hardware operations, legacy switch input,
  independent actions and manager subsets; refusal cleanup at every reservation
  boundary; existing goals never canceled to admit tuning;
- complete candidate/unchanged/cache proof, active-only startup, migration
  compatibility, crash-safe commit and job-specific hold release;
- pinned native tests for H/L/A and constructor-only cache refresh, inactive-node
  behavior, manager sibling coupling, sole-publisher graph and callback progress;
- restricted Docker handoff and fault injection, standard Compose parity including
  reset socket/group wiring, then PR -> all green CI -> merge.

No ROS/native/container/browser commissioning was performed in Phase A. Runtime
activation stays off; every live gate in handoff Phase F still requires its own
operator go. There is no unresolved design requirement for fresh odometry,
localization, a universal selected map or motors off to admit apply.
