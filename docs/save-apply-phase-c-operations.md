# Apply: verified hot updates or one whole-Nav2 restart

**2026-10-23. Focused operational specification, not commissioning evidence.**

This describes the current phase only: **apply an already saved configuration
revision to the necessary running consumers, verify it, and finish safely.**
The approved [handoff](save-apply-commissioning-plan.md) remains authoritative.
This document does not change the agreed requirements or authorize live work.
The operator's 2026-10-23 simplification supersedes the older per-node lifecycle
and three-container fallback designs. The SBC is reported down; local testing can continue,
but a real Apply must refuse without fresh link/hardware evidence.

## 1. What this phase is, and is not

The operation is:

> Review saved revision R -> confirm R -> reserve command maintenance -> update
> hot consumers OR restart all Nav2 -> verify R -> commit R as ACTIVE -> release this job's
> maintenance, leaving input disarmed and unrelated safety holds intact.

It is **not** saving drafts, rebuilding images for each parameter change,
restarting the SBC/core/bringup/dashboard, commissioning physical movement, or
creating a general-purpose robot administration/execution framework.

The two fixed choices are `scoped_parameters` (hot, no lifecycle restart) and
`nav2_redeploy` (exactly Compose service `navigation`, container
`patrolbot-navigation`). One restart-required value makes every changed value
inside that container load together at replacement boot. The final bringup
smoother stays up and may receive a verified hot batch. Its unavailable/inactive
callback is a refusal, not permission to restart bringup or the core.

Earlier scoped-lifecycle and three-container host prototypes remain offline
regression material; production selects `SimpleApplyBackend`, never those as a
silent fallback. The navigation-only replacement path is now wired locally to
the fixed host helper and one-shot boot resolver. Host policy and runtime enable
flags remain off; it is not installed, deployed or commissioned. Unsupported or
stale native evidence must remain visibly unavailable.

## 2. Operator-visible starting state

There are three different kinds of values:

1. **Draft:** text/slider edits in the browser, not yet saved.
2. **Saved desired revision:** durable targets on the robot, possibly unapplied.
3. **Verified ACTIVE / live observations:** committed configuration versus what
   is currently observed. A stale observation is not current runtime evidence.

Apply uses (2), not (1). Save never invokes Apply. Merely opening/closing the
review performs no runtime mutation and reserves no motion authority.

## 3. Prerequisites

### Required before executing a confirmed Apply

| Requirement | Why it is required |
| --- | --- |
| Current authenticated administrator, signed request verified by core | The browser cannot grant itself configuration authority. |
| Initialized, compatible store; valid immutable candidate and previous ACTIVE artifacts; writable journal/commit storage | Do not apply corrupt, reinterpreted, or unrecordable configuration. Never substitute defaults. |
| Exact reviewed revision, digest and operation scope still match | Another save/session/scope change must not replace what was approved with “latest”. |
| No unresolved competing apply or ambiguous previous mutation | Do not overwrite ownership of an operation whose outcome is unknown. |
| Fresh SBC link and current hardware identity/session; healthy applicable fault, E-stop, bumper and dock evidence | Do not mutate running configuration using stale/disconnected or unsafe hardware evidence. Preserve existing protections. |
| No active navigation/direct behavior, undock, map mutation, conflicting lifecycle transition or other mutation owner | Apply refuses rather than stealing/canceling somebody else's operation. |
| Physical input explicitly released; web manual session inactive/released; command targets, ramps and relevant pipeline buffers zero | An output zero alone cannot rule out a queued or held command. Missing messages are not release acknowledgments. |
| Existing core command gate closes; navigation and manual producers acknowledge this job | Discard racing/new input during installation, rather than queueing it for release. |
| The exact implementation supports the planned update and verification | A parameter service existing or returning success does not prove the execution code consumed a value. |

The public review can show why admission is unavailable. Actual checks are
repeated at execution admission; an earlier green UI snapshot is not permission.

### Explicitly NOT required

- Motors off. **Motors ON and OFF are both admissible.**
- Fresh raw odometry, measured stationary velocity, a current AMCL pose, valid
  localization, or an accepted seed.
- A selected map for a change that does not consume/recreate map state.
- Navigation ready to drive. Configured/inactive navigation quietly waiting for
  a seed can be idle and admissible.
- Permission to deploy, rebuild, reset, enable motors, drive, resume a patrol,
  undock, or set a pose.

A map/seed wait alone is not a competing mutation. An actual in-flight map or
lifecycle operation is. Unknown required ownership/hardware evidence still
refuses; none of these exclusions allows missing evidence to count as healthy.

## 4. Execution, step by step

### Step A — prepare the exact review

Compare the immutable saved candidate against committed ACTIVE. Derive actual
changed consumer values, including coupled/derived limits and cached policies.
Show the changed targets, affected nodes, supported operation, disruption and
whether localization will be invalidated. Freeze that review.

Why: a ceiling can affect more consumers than the edited field's own ROS node,
while a controller-only setting must not automatically affect localization.

Expected result: a read-only plan bound to R and its baseline. No hold, reload,
parameter write, Docker action, or motor operation has happened.

### Step B — admit and reserve maintenance

On explicit confirmation, core verifies authorization and the exact review,
checks current evidence, and durably records the job before remote reservations.
Use existing authorities to exclude new conflicting work; recheck admission
after ownership is acquired. Do not cancel goals or terminate an active manual
session in order to make an inadmissible operation pass.

Ownership does not mean a new lease on every component. The reduced boundaries are:

- Existing core owner for core hardware/map/undock/lifecycle admission.
- Navigation/action admission for autonomous and direct behavior commands.
- Existing physical and web producers for their local input/ramp/session state.
- Existing core DRIVE boundary for every ordinary velocity source, including
  manual and undock paths; keep independent safety commands effective.

After acknowledged closure, new buttons/goals are ignored rather than causing
installation to fail or being called “released”. Hardware/fault/ownership and
zero-buffer checks remain required. On release, require new middleware velocity
samples and a new manual arm/deadman gesture. Web ingress rejects commands and
drain-time barriers reject queued pre-maintenance commands. Stop, cancel, release
and emergency disable remain available. No input is saved for replay.

Independent safety stops remain effective; a maintenance job must not acquire
the power to suppress a hardware failsafe. One job identifier prevents releasing
another job's reservation; it is not a new general distributed lock system.

Expected result: acknowledged command-idle maintenance, with no running parameter
changes yet. If admission fails, unwind this job's acknowledged reservations.
If a reservation/release response is ambiguous, retain a recorded hold rather
than inventing a clean refusal.

### Step C — hot update, or restart the whole Nav2 container once

Execute the reviewed two-mode operation:

- Use a proven native parameter update when it actually refreshes the execution
  cache in the current node state.
- If any Nav2 value requires reconstruction, restart the complete `navigation`
  container once. Do not first hot-write some of its values into the old stack.
  Do not restart the coordinator/core, bringup, dashboard or SBC.
- A replacement must boot the exact reviewed immutable candidate under retained
  input maintenance. Ordinary/unsolicited restart still loads committed ACTIVE,
  never merely DESIRED; a stale/interrupted candidate launch is not ordinary boot.
- Parameter-only Apply performs no image build/pull or host reset.
- A previously inactive navigation consumer must not be activated simply to
  demonstrate that tuning succeeded. Independent normal readiness gates remain.
- Refresh changed producer policies where required, under the retained hold.
  Unchanged policy values do not need staging/rewrite merely because another
  setting changed; their verification/binding must still be trustworthy.

Expected result: candidate values are installed in affected execution state.
They are not declared ACTIVE yet. No motor enable/disable command, input arm,
goal/patrol resume, pose injection or host operation is part of this step.

### Pinned parameter mutability matrix

This audit is for Navigation2 **1.3.12**, archive SHA-256
`bcdf575c86250d63007595c992b9c5785c3ec907e897b79abfb4fc7eacbd1129`,
plus the local F710 post-set callback. “Hot” is source-backed behavior, not an
automatic runtime capability or commissioning claim. Unknown keys, versions or
transitional states are unsupported. Derived changes count just like edited ones.

| Consumer / fields | Current-state behavior and Apply choice |
| --- | --- |
| AMCL `alpha1..4`, `update_min_a`, `update_min_d` | Hot while active. `dynamicParametersCallback()` updates thresholds or calls `initOdometry()` to refresh the motion model; no particle-filter recreation. Inactive/unconfigured selects whole Nav2 restart. |
| AMCL `max_beams`, `sigma_hit`, `z_hit`, `z_rand` | Hot while active. Callback clears old laser models and rebuilds message filters; subsequent laser creation uses new model inputs. Filter generation must remain unchanged. Inactive selects whole Nav2 restart. |
| AMCL `min_particles`, `max_particles`, `pf_err`, `pf_z` | Whole Nav2 restart. The active callback exists but calls `initParticleFilter()` and resets filter state, so it is not an acceptable localization-preserving hot path. |
| RPP `FollowPath.desired_linear_vel`, lookahead distance/min/max/time, rotate-to-heading velocity, approach minimum, derived regulated minimum | Hot while configured (active or inactive); `ParameterHandler::dynamicParametersCallback()` refreshes consumed fields, including base cruise. Unconfigured selects whole Nav2 restart. |
| RPP `FollowPath.approach_velocity_scaling_dist` | Whole Nav2 restart: constructor-cached, absent from dynamic callback. |
| Nav2 smoother `max_velocity`, `min_velocity` arrays | Hot only while active; callback is installed on activate and removed on deactivate. Inactive/unconfigured selects whole Nav2 restart. |
| Final `teleop_velocity_smoother` arrays (bringup) | Same hot callback while active. Otherwise unsupported: it is outside the fixed navigation restart scope. |
| Spin `max_rotational_vel`, derived `min_rotational_vel` | Whole Nav2 restart: `Spin::onConfigure()` caches bounds. An angular ceiling edit can trigger this through derived Spin values. |
| F710 caps/minima/maxima | Hot with the verified running post-set callback. Preserve lower operator trims and require release/new deadman after maintenance. |

Source files are `nav2_amcl/src/amcl_node.cpp`, RPP
`src/parameter_handler.cpp`, smoother `src/velocity_smoother.cpp`, Spin
`plugins/spin.cpp`, and local `patrolbot_joy_teleop.py`. Machine policy is
`tuning_mutability.py`; planner tests cover mixed/derived batches and fixed scope.
Native cache tests demonstrate stale RPP/Spin/inactive-smoother behavior and
active versus inactive AMCL consumption without map, odometry or a seed.

### Step D — verify before committing success

Read back the affected values and actual consumed caches/policies. Verify the
remaining configuration evidence sufficiently to commit the whole revision;
do not claim an untouched scope verified merely by passing its name in a list.
Recheck command ownership and hardware/session identity before commit.

The initial implementation can use fresh complete readback of the small fixed
tuning set rather than a complicated inherited-proof cache. Reading unchanged
consumers is not permission to reload or rewrite them.

Why: native tests demonstrate that RPP, Spin and an inactive smoother can report
new ROS parameter values while their execution caches remain old. Neither an
HTTP 200 nor successful SetParameters is sufficient verification.

Expected result: a job-bound verification record identifying what was actually
observed. Missing/mismatching/stale evidence after possible mutation leaves the
operation unverified and held, not reported as success.

### Step E — commit and finish

Atomically promote the exact verified revision through the existing ACTIVE /
startup store path. Confirm any changed core/web policy bindings refer to that
committed revision. Release only this job's maintenance ownership.

Expected result:

- ACTIVE and ordinary restart refer to R; history remains intact.
- Saved but unapplied targets are never activated by ordinary restart.
- Existing valid localization remains untouched when its dependencies did not
  change. Do not clear and then restore a saved pose to achieve this.
- If a necessary localization reload invalidated it, report **parameters active;
  navigation awaiting a fresh operator seed**. Seed admission stays separate.
- Old input, goals and patrols do not resume. A new manual arm/deadman gesture or
  new navigation request must be explicit and pass normal driving gates.
- Apply sends no motor-state command. Independent hardware failsafes or a separate
  operator action may change motor state; Apply must not counteract or restore it.

## 5. Outcomes and what each means

| Outcome | Configuration state | Ownership / next action |
| --- | --- | --- |
| Review canceled | No runtime change | No reservation was created. |
| Review/CAS conflict | No runtime change by this request | Read/review the new exact revision or scope. |
| Definitive pre-mutation refusal | No runtime change | Release this job's acquired reservations; explain the specific blocker. |
| Reservation/release result uncertain | Runtime parameters may still be unchanged, but ownership is uncertain | Record unknown and retain safety ownership; status/recovery, not blind resend. |
| Mutation or verification fails after possible partial change | Running consumers may be mixed/unverified | Retain a durable hold and prior verified startup config unless promotion was already verified/committed. Explicit recovery is required. |
| Commit/finish outcome uncertain | Do not guess whether commit/release completed | Retain durable job evidence/hold; query status and recover explicitly. |
| Verified success, localization unaffected | R is ACTIVE | Release this job only. Normal readiness can still block driving independently. |
| Verified success, localization invalidated | R is ACTIVE | Release job maintenance, retain navigation-seed admission until a new operator seed is accepted. |

No failure launches an automatic Docker fallback, retries an ambiguous mutation,
restores defaults, reinitializes the store or enables motors.

## 6. Assumptions and limitations

We assume the existing core is the final ordinary-drive gate, the store and
administrator signing boundary remain authoritative, and the supported native
image/package behavior is identified and tested. Compatibility/version changes
must invalidate unsupported assumptions, not silently inherit them.

We do **not** assume:

- Software command idle proves physical standstill. With motors on and no fresh
  odometry, it does not. External forces, slopes and hardware behavior are not
  measured by this operation; existing physical safety measures remain necessary.
- Topic silence proves release, or one zero proves every buffer is drained.
- A goal/session cannot arrive while a Boolean idle check is being evaluated.
- A matching parameter getter proves a constructor-cached value changed.
- A lost response proves the request did not execute.
- An existing localization seed may be replayed after a reset.
- Component tests or mocked verification dictionaries prove the complete native
  command path safe.
- The SBC is available. It is currently reported down, so live admission is blocked.

This feature does not redesign DDS/network security, guarantee physical
standstill without sensors, or add per-component signing secrets. Unknown or
unrecognized producers/configurations refuse support; they are not guessed safe.

## 7. What is implemented versus missing today

**Implemented/tested components:** explicit Save/frozen review; reduced two-mode
planner/executor; exact-job binding and durable unknown holds; non-odom admission;
localization-preserving maintenance; core hardware exclusion and post-release
middleware timestamp barrier; physical/web block/observe protocols and ingress
discard; changed-policy-only staging; complete fixed-consumer verifier with
native cache/policy, final publisher identity and protected mux checks. Native
AMCL hot/inactive, RPP/Spin/smoother cache and navigator race tests pass locally.
Verifier unit I/O and executor fixtures remain explicitly offline evidence.

Native producer tests exercise actual F710 post-set updates while new button
pressure is blocked, without calling that pressure “released”. Real DDS tests
exercise a command published before release but delivered afterward. Read-only
ACTIVE inspection preserves the three-second boundary including native RPC time.
Middleware timestamp dictionaries are supported; an ABI without publisher GIDs
still cannot supply exact source-identity evidence and must remain unsupported.

**Follow-up integration (2026-10-24):** read-only native capability collection now
binds fixed composed-node membership, absence of duplicate standalone consumers,
audited consumed-cache/action-gate versions, manual policy protocols, final
publishers and current hardware/origin generation. Its three-second age includes
the full read interval. An absent host helper disables restart plans, not genuine
hot-only plans. The actual native service ABI must include held policy payloads.

`restart_navigation()` now uses the fixed navigation-only host helper, not the
old three-container dispatcher. It durably records seed loss before dispatch,
waits for the one replacement boot receipt, reacquires new navigator/F710 input
holds once, restores only lower trims and binds the new AMCL map identity.
Final verification requires that exact boot, unchanged final publishers, actual
candidate caches and an unseeded pending navigation barrier before promotion.
No Docker success alone marks ACTIVE or releases input.

**Still required before commissioning:** a complete target-native Apply cycle
with real source-identity telemetry/fake hardware, production image/volume and
standard-launch acceptance, failure/recovery acceptance, green PR CI, and the
separate live authorization. Older local rclpy test images without publisher
GIDs cannot supply source-bound motion evidence and remain inadmissible. Runtime
enablement and the example host policy are still disabled; no live Apply is claimed.

**Direct-action follow-up:** `0023-tuning-composed-action-input.patch` now reuses
the navigator's existing maintenance endpoint and one shared `nav2_util_core`
input boundary for the fixed composed Nav2 process. Every fixed
`SimpleActionServer` consumer is rebuilt, including controller, behavior,
planner, route, smoother, waypoint and navigator servers. It counts accepted
goals (including pending/preemption slots) and callbacks still running after
their result; busy work refuses admission instead of being canceled to get a
hold. New goal handlers reject while held, and cancel stays available. A new
process starts closed until the core hold topic is observed. Releasing the job
token cannot clear the still-held core input boundary.

The real native probe exercises the existing endpoint, direct Spin/FollowPath
action protocols with nonmoving callbacks, pending goals, callback tails,
foreign release, the core/token release gap, cancellation and 1,000 admission
races. Production block/verification now requires that evidence; the older
navigator-only token cannot satisfy it. This is evidence for the fixed composed
process, **not** a claim of exclusion for arbitrary standalone action servers or
unrecognized legacy producers. Native capability collection must still bind
that deployment. It does not enable Apply or the host adapter.

The operator clarified that goals are simply rejected while Apply owns admission,
for both hot and restart modes. There is no application-level goal queue or
post-Apply replay/retry. The native probe covers handlers invoked during the hold
and already accepted pending work; it is not a claim about arbitrary delayed
network traffic or unrecognized external action servers.

F710 also discards old header-stamped release/press/trim samples delivered after
its last maintenance hold ends; only newly generated release/new-deadman input
can rearm. This closes producer-side replay, not just final DRIVE buffering.

Passing native component tests is useful evidence, but not a substitute for those
missing connections. The current mux maintenance header is unfinished source,
not an installed or accepted ownership implementation.

## 8. Reduced complexity to preserve

These implement the operator's revised scope without weakening safety:

| Keep | Simplify or defer |
| --- | --- |
| One existing coordinator and fixed allowlisted consumers | No generic execution framework or arbitrary caller-selected node/service/path. |
| Exact revision/scope review and administrator signing | No independent signing protocol/key for each ROS consumer. |
| Acknowledged exclusion at real command/admission boundaries | Do not equate safety with adding an RPC and lease to every node. Reuse the authority that already owns each path. |
| Readback of consumed execution values | Prefer small fixed diagnostics and complete readback over a generic certificate/inherited-proof subsystem. |
| Refresh changed policies | Do not stage unchanged core/web policies for every unrelated AMCL/controller edit. Current executor staging is broader than necessary and should be narrowed. |
| Durable record and hold after ambiguous partial mutation | No automatic rollback/retry/redeploy orchestration in this phase. Explicit existing recovery is sufficient for Phase C. |
| Verified hot updates and one fixed Nav2 restart | No per-node lifecycle schedule, automatic wider fallback or image rebuild. |
| Exact navigation-only replacement boot | Keep the coordinator alive; do not revive the old three-container coordinator handoff. |

The essential complexity is **ownership, actual cache consumption and honest
failure state**. Generalizing beyond the fixed robot architecture, adding duplicate
ownership layers or reviving the older whole-robot fallback is avoidable scope
growth. A successful Docker exit alone never proves the replacement consumed R.

## 9. Phase C completion test

In an isolated native environment with fake hardware, demonstrate a complete
review -> input closure -> hot/restart installation -> real readback -> commit -> release cycle:

- With motors ON and OFF, no fresh odometry, no localization/seed, and no selected
  map for an unrelated setting; all required non-odom evidence is nevertheless real.
- With unchanged AMCL/map/localization state for unrelated changes, and a separate
  new-seed barrier only when necessary localization dependencies were invalidated.
- With only reviewed hot consumers changed, or one complete navigation replacement
  when any value needs restart; core/bringup/dashboard/SBC stay up, lower ceilings
  remain unchanged, no image build is performed.
- With racing goals/manual input producing exactly one admitted owner, no goal
  cancellation to obtain tuning ownership, and no replay on release.
- With definitive refusal leaving no orphan reservation, and partial/unknown
  mutation remaining held through restart without automatic retries.
- With ordinary restart loading only verified committed ACTIVE, and sole publishers,
  hard envelope 0.30 m/s / 0.70 rad/s and mux priorities unchanged.

Demonstrate exact replacement boot, producer re-blocking, fresh-seed admission and
interrupted/unknown restart behavior separately in isolated containers. No live
robot, host reset, physical motion or automatic seed is needed for these tests.
Live authorization gates remain separate even after local acceptance passes.
