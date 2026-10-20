# Advanced tuning: contract and read-only runtime foundation

Slice 1 adds development-only contract artifacts under `contracts/`. Subsequent
work adds a bounded YAML revision store, active-only startup resolver, policy
backstop, activation state machine and read-only `/patrolbot/tuning` inspection.
The coordinator now has a fixed-node lifecycle backend with exact readback and
explicit enablement. DDS actor claims alone remain insufficient: mutations require
a current server signature bound to robot, request and authenticated actor.
Actual write/apply capabilities require their keys, initialized store, supporting
services and fresh safe-idle evidence. Defaults remain disabled.

**Starting values:** use the current packaged YAML configuration, not library
example defaults or guessed input ranges. Existing saved state takes precedence
after initialization. User edits must meet Nav2 Jazzy documented domains and
cross-parameter constraints as well as the robot's narrower edit policy. See
[parameter rules and references](tuning-parameter-rules.md).

Do not set `PATROLBOT_TUNING_ENABLED=1` on an uninitialized deployment: startup intentionally
fails closed. A missing/corrupt previously expected active configuration holds
motion; it must never silently restore faster defaults. Saved desired revisions
are not active startup configuration. This topic-branch work is not deployed or
physically accepted functionality; see the code/unit readiness record.

The guarded root-local `tuning_initialize`/`tuning_recover` tools are implemented;
they require exact reviewed digests and exclusion of running guarded consumers.
See [code/unit readiness](tuning-code-readiness.md). Native/deployment and physical
acceptance tests are deferred by the operator's hard rule, not inferred successes.

The next-slice diagnostic is available as
`python -m patrolbot_bridge.config_bootstrap` or the packaged
`tuning_bootstrap_preview` entrypoint. It reads explicit registry/state paths and
reports candidate provenance without creating state, acquiring a writer lock or
promoting startup configuration. See [bootstrap preflight](tuning-bootstrap.md).

Baseline-aware persistence is implemented as an internal, unit-tested library
path: [baseline import](tuning-baseline-import.md). It requires complete,
representable observations and refuses unpreservable lower settings. This does
not provide a browser initialization endpoint or permit unsigned ROS mutations.

The paired `patrolbot-dashboard` repository contains the cross-stack source
inventory, threat model and verification record:

- `docs/advanced-tuning-inventory.md`
- `docs/advanced-tuning-threat-model.md`
- `docs/advanced-tuning-slice1-verification.md`
- `docs/advanced-tuning-runtime-verification.md`

See `contracts/README.md` for the portable validator and fixture tests. Shared
artifacts must match the dashboard's vendored copies byte-for-byte before a
matched release; passing either repository's tests alone is not parity evidence.

## Evidence boundary

The commissioned ordinary-drive envelope remains **0.30 m/s and 0.70 rad/s**.
These are upper bounds, not motion requests, minimum useful speeds, or permission
to increase lower producer-specific settings. Source defaults are not runtime
readback. Usable lower speed bounds and installed/native parameter mutability
remain unverified; no candidate is approved for activation by this work.

Further activation work must use the existing core mutation owner,
preserve emergency release/watchdog behavior and arbitrate against both physical
and web manual evidence. Unknown ownership, hardware or odometry is not safe idle.
Startup must use committed **active**, never merely **desired**, configuration.
No edit, read, save, apply, restart or restore may re-arm teleop, resume a goal,
enable motors, undock or publish a pose.

No robot contact or deployment inventory is authorized by this document. Native
behavior proofs belong in an isolated lab with pinned artifacts; physical tests
require separate operator authorization.
