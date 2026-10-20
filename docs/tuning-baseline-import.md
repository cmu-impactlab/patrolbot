# Baseline-aware initialization (unit-tested foundation)

This slice adds a **local library path**, not a deployed initialization endpoint.
`RevisionStore.initialize_from_baseline(bundle, actor=..., reason=...)` imports
an explicit baseline only when every supplied consumer configuration is exactly
representable by the reviewed generator. It never substitutes faster packaged
defaults for missing observations.

No production command, WebSocket operation or ROS mutation is enabled by a
baseline import. Separate signed mutations and the opt-in lifecycle backend are
implemented, but their keys, store and runtime gates remain mandatory.
Tests use synthetic bundles and temporary directories only; they do not prove
that a real robot has been observed or is safe to initialize.

## Explicit baseline bundle

The version-1 JSON bundle has fixed fields:

- `contract: "patrolbot-tuning-baseline"`, strict integer `version: 1`.
- Canonical UUID `capture_id` and `source_session_id` for provenance.
- `registry_digest` and `defaults_digest`, bound to the reviewed inputs.
- `observed_values`: every editable, applicable, non-protected logical value.
- `observed_artifacts`: complete `bringup.yaml` and `navigation.yaml` consumer
  configurations, including source-specific limits and protected configuration.

The parser rejects unknown/missing fields, malformed types, non-finite numbers,
duplicate JSON/YAML keys, invalid logical values and incompatible provenance.
Each complete observed consumer document must match generated configuration;
partial observations cannot silently inherit packaged values. A slower source
setting that this generator cannot preserve is **refused**, not raised or
silently normalized. Supporting additional baseline shapes is later work.

This is content validation, not proof of freshness, administrator identity or
live readback. A bundle cannot authorize a write merely by containing an actor,
session UUID or digest. A future authenticated local owner must supply authority
and verify the observation against the deployment before exposing initialization.

## Persistence behavior

The caller supplies an administrator actor and an explicit bounded reason at the
trusted local API. Invalid bundles or actors are refused before state creation.
Valid imports use the existing single-writer lock, immutable revision files,
checksums, fsynced atomic writes and audit path.

- Import initializes revision 1 using the supplied logical values, not registry
  defaults. Exact consumer artifacts and baseline provenance are recorded.
- Generation and packaged-default digests are rechecked before writing revision
  contents, so source drift cannot silently change the reviewed candidate.
- Existing initialized state is never overwritten, even by a repeated import.
- Corrupt or partially written state is not reset. An interrupted initialization
  remains an explicit recovery problem, not permission to restore defaults.
- Reopening uses the committed ACTIVE startup pointer. Later saved DESIRED
  changes remain pending until a separate approved activation.
- No node is reconfigured, no motor is enabled, no goal or pose is published and
  no teleop input is armed. Committed startup artifacts are not live readback.

The `initialize()` primitive remains for controlled clean-install/fixture
construction and now reads starting values from the current packaged YAML,
rather than from copied registry defaults. It is not a production migration
route; using packaged values over an unknown existing configuration is not
authorized. Neither initializer overwrites existing saved state.

## Verification boundary

The focused Python unit tests cover baseline parsing/compatibility, lower caps
and cruise preservation, input immutability, reopen/ACTIVE-vs-DESIRED behavior,
administrator refusal, interrupted initialization and competing writers. They
use no Docker, native ROS graph, robot or deployed environment.

```sh
PYTHONPATH=ros2_ws/src/patrolbot_bridge python -m pytest \
  ros2_ws/src/patrolbot_bridge/test/test_tuning_baseline.py \
  ros2_ws/src/patrolbot_bridge/test/test_tuning_baseline_store.py -q
```

Read-only diagnostics remain separate: [bootstrap preview](tuning-bootstrap.md).
Deployed baseline capture, ownership/freshness verification and actual
initialization remain later acceptance gates, not inferred unit-test successes.
