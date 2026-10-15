# Exact-production Python backport — merge staging, not deployment approval

Status: **READY FOR LOCALIZATION MERGE; controller files PROVISIONAL**.
No production files, services, robot commands or Git state were changed.
Parent owns canonical audits and final installed-image/native/ARM validation.

## Provenance and boundaries

`source/` began as the eight byte-verified running modules captured at
`backups/20261014T205841Z-combined-undock-controller-audit/production-baseline/`.
`baseline-manifest.json` preserves capture paths and full hashes.
Initial edit backup: canonical
`backups/20261014T211104Z-production-integration/`.

Running core: `sha256:2d3df1c3c6bfe857f256be1a947892e78e942802c633f727b194331ffe73355f`.
Running web: `sha256:0da4b7b6810de6b96ce17defc1e5c20dc698e704f2a9c33085522f40b873e4b6`.
Python only: no interfaces, native plugins, maps, Nav2 parameters, BT XML,
launch files or undeployed path/planning features are included.
`map_transaction.py` is unchanged and protected by the manifest.

## Module deltas

- `epoch_runtime.py`: read-only `local_motion_mutations_settled`; typed
  lifecycle-reply validation before token turnover; retained unknown-write
  tombstones containing the exact future, issuing phase, token and session.
  Only a readable reply from that SAME future can settle it. Unrelated reads,
  new/refused writes, new tokens and pose requests do not clear it. Failed
  submission without a future identity requires explicit stack recovery.
  Unknown flags with no tombstone fail closed. Unknown owners also gate normal
  `commands_allowed` and stop the coordinator from issuing new writes.
- `map_runtime.py`: matching map-write validation/tombstones; all fail/cancel
  sites retain unanswered writes; read-only probes may be dropped. Unknown
  outcomes reject new operator-localization admission. Failed map and active
  undock hold issue no further map writes. Settled map failures can be bypassed
  for hardware-only motion, not ordinary navigation.
- `undock_turn.py`: hardware-only reverse admission with actionable rejection
  logs; rechecks the shared predicate through `hardware_problem`, including
  reverse execution, turn acceptance/execution and ongoing turn permission.
  All other method bodies (including r4 precision/drift rules) are unchanged.
- `dock_manager.py`: only definitive pre-acceptance refusal/unavailable child
  becomes REJECTED, zero distance, no manual recovery. Unknown-child waiter
  is unchanged and continues holding ownership.
- `commands.py`: six semantic wording corrections (STOPPING has two callsites),
  no future path feedback, capabilities or gate changes.
- `bridge_node.py` / `manual_active.py`: first controller backport only, without
  future BaseState fields. **NOT finalized:** reviewer raw-odometry/provenance/
  READY-map-failure findings belong to the controller agent's revised patch.

## Quiescence claims and recovery

A retained same-write ACK proves that original service call answered; its old
FSM is NOT advanced and old map requests are NOT resumed. A retained pending
future, RPC exception, malformed reply or missing future identity proves no
quiescence. Such outcomes keep reverse, turn and controller admission held.
No read-only state probe, operator pose, new session or unrelated request is
treated as proof an old server-side write cannot still land.
Whole coordinated stack recovery must establish that old server-side owners
cannot execute before discarding unresolved ownership. This source package
does not perform that recovery and does not claim a bridge-only restart does so.

## Validation / handoff

Offline inert real-method tests:

```sh
python3 -B -m unittest discover -s tests -v
```

The tests execute parsed production method bodies with inert futures/peers;
they are not installed-image, physical or controller-provenance acceptance.
`baseline-failing-before.log` preserves concrete lost-future failures on the
exact captured baseline. `source-manifest.json` records all output hashes;
`patches/` contains per-module baseline-to-candidate unified patches.

`stage_image_overlay.py` verifies **every selected original hash** plus candidate
hash before writing anything, then rehashes changed modules. It defaults to
read-only; --apply requires an explicit build-container guard. Parent's final
leaf images must derive from the exact running core/web images, override stale
revision/source labels with new manifest provenance, and retain interface ABI.
Do not build from the newer fork's Dockerfiles or native/BT patches.

## Necessary bind/provenance work (parent only)

Core modules are baked installed Python; apply to captured installed paths,
not stale deployed host source. Existing source binds can mask baked code:
inspect effective imports and hashes in every role before activation.
Web imports `/opt/web_bridge`; its source bind must select this backported
`commands.py`, not future dashboard-work or a stale host copy. Do not remove
bindings blindly. Preserve environment/state/maps/interface/native images.

## Localization merge conflict map

The separate agent's changes must be rebased onto this exact production tree.
Likely overlap: map `_map_accept`, `_map_call`, `_fail_map_transaction`,
`tick_map`, stage transitions; epoch `tick`, `_request`, reply consumption,
`_reject_localization` and new shared predicate. Preserve unknown-owner checks
at operator pose admission and never clear tombstones on new request/readback.
No localization-agent patch has been invented or merged here.
