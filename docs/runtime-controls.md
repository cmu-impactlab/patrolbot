# Runtime Controls v1

Manual controls use an explicit runtime-only submit path. Authenticated operators
and administrators can select 0.05–0.30 m/s and 0.10–0.70 rad/s, bounded further
by the commissioned producer maxima and the existing lower active policy.
The selected ceiling applies symmetrically to trusted web manual input and F710
ordinary driving. Physical trims can reduce it. Autonomous and exclusive undock
speed profiles are unchanged. Zero is not a settings value or Stop.

The browser keeps numeric drafts as strings and sends exactly one POST on Update.
It disarms an armed local manual panel, creates no session or velocity, and never
retries or rearms. GET/polling/reconnect only inspect. Robot connection epochs,
browser transport, hardware generation and a three-second evidence deadline
invalidate observations and pending reads. Requested values, actual per-producer
ceilings/effective trims, and partial/unknown outcomes are separate.

## Fixed trusted path

* `POST /api/robots/{robot_id}/runtime-controls`: fresh session role, strict fixed
  fields, shared command admission, durable audit before send and replay check.
* `runtime_controls.request`: a server-signed envelope with a distinct runtime
  HMAC purpose, actor, robot, request ID, state generation and short expiry.
* `/patrolbot/runtime_controls` (`RuntimeControls.srv`): verifies the signature
  and current state, checks fresh hardware/zero buffers/released input/known idle
  owners, and acquires a separate core runtime reservation under the epoch lock.
* `/bt_navigator/tuning_hold`: only the existing native mutex-backed navigator
  admission gate is reused. It rejects when navigation already owns the mux and
  serializes new goals. The core tuning hold, tuning topic, lifecycle operations,
  accepted seed and `_navigation_ready` state are not changed.
* `/patrolbot/web_runtime_manual` and `/patrolbot/joy_runtime_manual`
  (`RuntimeManual.srv`): fixed read/prepare/commit/release/reconcile protocol. Every mutating
  phase has a separate signed core delegation bound to producer, incarnation,
  hardware generation, request ID, values and phase. Missing keys fail closed.
  F710 uses native `set_parameters_atomically` for only max_linear/max_angular;
  its existing policy validation remains mandatory. Readback includes the cache,
  physical trim, bounds and process incarnation, not merely parameter storage.

Both producers prepare before either changes. A definite motion-first refusal
leaves the existing producer forwarding path intact. The core gates nonzero DRIVE
only after both preparations and before any setting effect; uncertain preparation
also retains that gate. Each commit is generation checked,
read back, and checked against every prepared peer. The core reservation gates
new motion and hardware effects; the producer reservations share their actual
input/session admission. Stop, motor disable, cancellations and watchdogs remain
independent. No settings phase publishes velocity, enables motors or starts goals.
A fresh released F710 report and a fresh explicit web start are required afterward.

Runtime mode is an immutable startup choice: set
`PATROLBOT_RUNTIME_CONTROLS_ENABLED=1` in both core and web bridge to opt in.
Source Compose passes this variable with default **0**. Default/disabled installs
retain existing admission and commissioned defaults even without the optional web
producer or signing key. Discovery never enables or disables this mode. In an
enabled epoch, missing producer evidence retains a core hold and DRIVE block;
service/key disappearance never switches web enforcement back to legacy limits.
Keys authorize mutations, not readback of actual caps. A missing mutation key can
remove update permission while fresh current policy remains enforceable.

Enabled startup reads discover surviving producer reservations without replay.
Actual surviving reservations remain held. Subsequent loss never clears a hold on
service return or timeout. A definite prepare refusal with verified complete
cleanup preserves the previous usable policy: the response outcome is rejected,
but current state remains default/confirmed. Failure before acquiring ownership
cannot invent an unresolved write. Readback failure after verified final release
can report an unknown request outcome without inventing a surviving reservation.
Consumer reset/external generation changes retire old requested/confirmed status;
fresh per-consumer readback is displayed as current/default, never confirmation
of the old request. The UI labels default as current/default/external state.

### Explicit ownership reconciliation

An operator/admin may press **Reconcile runtime ownership**, which POSTs the fixed
operation `reconcile`, fresh UUID request ID, current state generation,
`target: "manual"`, and `values: {}`. The same origin, role, short-lived signature,
replay, audit-before-send and server motion admission checks apply. Only this
operation may enter through the server's own uncertain-runtime barrier; native,
physical, web, hardware, localization and other motion owners must still be idle.

Core obtains current hardware/policy and both producer generations, actual
selected/effective caps and existing reservation IDs, checks native navigator idle
under its mutex gate, then sends producer-specific signed reconcile requests under
each actual input/session lock. Each must prove fresh neutral/idle buffers and
match its exact current generation/hardware/reservation. Reconciliation releases
ownership only, rotates producer evidence to fence delayed old prepares/commits,
and reads back both unchanged states before releasing native/core holds. Failed,
stale, busy or uncertain recovery remains held; it never unlocks on a deadline.
No parameter restore, upward rollback, setting replay, velocity, motor enable,
goal cancel/resume, localization reset or input rearm occurs.

Partial unequal values remain visibly unequal per consumer. Successful ownership
recovery returns `outcome: ok`, `status: default`, no requested unified setting,
and the actual per-consumer ceilings. It does **not** confirm the failed setting
or choose an unseen higher fallback. Each producer keeps enforcing its own current
ceiling, with independent lower policy caps retained. Changing ceilings afterward
requires a separate explicit fresh set request. Physical rearm requires a neutral
sample acquired strictly after local release cutoff, received within JOY_TIMEOUT,
with no future timestamp; the neutral callback itself produces no motion. Web
requires a new explicit start. Merely clearing a hold cannot rearm either input.

Policy polling runs in the trusted server path and the browser connection path,
independently of the settings panel. Concurrent GET readers share a bounded source
read. Refresh atomically replaces a snapshot only after validation, retaining
still-fresh same-transport evidence meanwhile. Failed refresh does not extend its
source age. Expiry or robot/transport/hardware retirement invalidates applicability.
No read retries or refreshes mutate settings. Replay ledgers remain bounded to
4096 operations per process and fail closed when full.

## Pairing and commissioning

Use matching core/dashboard sources and rebuild/install `patrolbot_interfaces`
including both new services. Both native consumers must run. Existing old browser
clients can read; older robot interfaces do not advertise this implementation.
`runtime_controls_policy_v1` requires fresh trusted policy for web manual motion;
losing mutation capability does not fall back to a higher legacy envelope.
`runtime_controls_manual_v1` is advertised only after actual producer evidence.

Use the existing protected signing-key provisioning: dashboard server setting
`tuning_auth_key_file`, and robot `PATROLBOT_TUNING_AUTH_KEY_FILE`, with identical
robot IDs. Core, F710 and web producer verify purpose-separated signatures. The
source Compose wiring mounts the existing secret volume read-only into those
robot processes. This change does not provision, inspect or copy any real key,
enable commands, or run containers. Missing/mismatched keys remain unavailable.
Rebuild/native isolated commissioning is required before deployment; offline fake
services do not establish DDS scheduling or actual hardware behavior.

## Navigation implementation is next

`route_server.AdjustSpeedLimit` is a real configured publisher. In pinned Nav2
1.3.12 it sends percentage SpeedLimit on entry to an edge, defaults missing
metadata to 100%, and does not publish a current-value service, source generation,
usable timestamp or freshness heartbeat. The controller accepts last-message-wins
SpeedLimit and provides no application acknowledgement.

A safe min arbiter would require separate explicitly routed route/safety/operator
inputs, immutable commissioned baseline for percentage/absolute conversion,
source-incarnation binding, retained restrictions, a sole controller-facing
publisher, and a motion-admission interlock while any independent source state
is unknown. A restarted arbiter/controller cannot infer the restriction for an
already occupied edge from this event-only route publisher. Publisher counts
cannot fix that race. Reconstructing it requires additional source-state and
lifecycle/admission integration scheduled in the approved navigation blueprint.

In this manual-fix snapshot no navigation mutation is enabled, no competing publisher exists,
no desired_linear_vel workaround is used, and zero is never emitted as reset.
Missing acknowledgement alone would permit published-unconfirmed, but missing
safe restriction reconstruction does not. No Nav2 patch/fork or zone subsystem.

Pinned source:
[route operation](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_route/src/plugins/route_operations/adjust_speed_limit.cpp),
[controller](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_controller/src/controller_server.cpp),
[RPP](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp).

## Paired CI handoff

Dashboard `portable` now explicitly selects runtime schema tests alongside every
old tuning test. Existing `parity` still compares old tuning artifacts with main
unchanged. A distinct fixed `cmu-impactlab/patrolbot` checkout compares runtime
schemas, response schema, fixtures, schema tests, all three validator copies and
producer protocol/auth implementations. Missing artifacts fail; internal mirror
and per-artifact drift/missing regressions are portable tests.

Parent must replace the deliberately non-resolving 40-zero SHA in
`.github/workflows/tuning-contract.yml` (Reviewed runtime canonical checkout)
with the reviewed/committed **full immutable core feature SHA**, push that core
commit and the paired dashboard workflow, then collect both required jobs. Do not
merge either PR to pair them. The placeholder intentionally fails checkout until
this handoff; no local/CI acceptance is claimed for that unresolved remote gate.
Local pairing uses `python scripts/check_runtime_contract.py --core-repo ../core`.
