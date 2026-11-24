# Advanced tuning: contract and read-only runtime foundation

`/patrolbot/tuning` supports read-only inspection and historical job status.
Save, apply, redeploy, and restore are not runtime operations. A legacy signing
key or enablement flag cannot re-enable them. Existing revisions are not changed
or migrated: boot still resolves the committed ACTIVE revision, existing motion
policy remains enforced, and an unsettled historical job still holds motion.

**Starting values:** use the current packaged YAML configuration, not library
example defaults or guessed input ranges. Existing saved state takes precedence
after initialization. The retained registry describes Nav2 Jazzy parameter
domains and the robot's narrower policy for the versioned configuration. See
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
not provide a browser initialization endpoint or permit ROS mutations.

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

Startup must use committed **active**, never merely **desired**, configuration.
Inspection does not re-arm teleop, resume a goal, enable motors, undock or publish
a pose.

No robot contact or deployment inventory is authorized by this document. Native
behavior proofs belong in an isolated lab with pinned artifacts; physical tests
require separate operator authorization.
