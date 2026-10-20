# Tuning bootstrap preflight (read-only)

`patrolbot_bridge.config_bootstrap` is a **diagnostic**, not a production
bootstrap. It answers one question offline: if the persistent tuning store were
initialised on this host, which current packaged-YAML revision and generated config
digests would be proposed, and is the state directory in a truly clean,
uninitialised condition?

It never initialises the store, creates a directory, takes a writer lock,
promotes startup config, imports ROS or contacts a node. It does not fabricate a
live parameter/odometry readback or claim migration eligibility.

## Command

```bash
python -m patrolbot_bridge.config_bootstrap \
    --registry contracts/robot-tuning.registry.json \
    --state-dir /var/lib/patrolbot/config
```

Both paths are explicit; no default is invented. Exit code `0` means the state
is clean and a preview was produced; exit code `2` means a refusal report was
produced. The JSON report is always printed on stdout.

After package installation the equivalent entrypoint is
`ros2 run patrolbot_bridge tuning_bootstrap_preview --registry FILE --state-dir DIR`.
This command does not use a ROS graph. Adding the entrypoint is not evidence
that a deployed image already contains it.

## Report

The bounded JSON report (schema `patrolbot-tuning-bootstrap/1`) always carries:

- `write_performed: false`, `activatable: false`, `live_readback: false`,
  `migration_eligible: false`.
- `initialization_supported: false` with
  `initialization_blocked_reason` explaining that this preview cannot write;
  the separate local initializer requires explicit review and stopped consumers.
- `bootstrap_status`: `clean_uninitialized`, `already_initialized`, `damaged` or
  `refused`.
- `state_parent_available`: whether the durable volume parent exists; if not, a
  note records that initialization is impossible on this host.
- `summary`: a short multiline human summary whose **second line** states the
  status and the two false booleans.
- On a clean state, `candidate_defaults`: revision `1`, the editable parameter
  count, `source: packaged_yaml_starting_values`, `defaults_digest` (packaged
  YAML provenance) and `config_digest`
  (the generated consumer artifacts), plus `validation` and `reasons`.

## Fail-closed behaviour

- A present `initialized` sentinel plus manifest is reported as
  `already_initialized` and refused; it is **never** reset.
- Missing sentinel next to real state, an unreadable/corrupt manifest or sentinel,
  and a symlinked state directory are reported as `damaged` and refused.
- The registry must be a regular file, read within a hard byte budget without
  following a final symlink; FIFOs are rejected without waiting for a writer, and
  parsed with the strict bounded JSON codec (duplicate keys and oversize are
  rejected); the logical contract is validated through `tuning_spec`.
- All of these checks reuse `RevisionStore.state()` rather than a bespoke walk,
  so the preview and the mutating path agree on what “clean” means.
- Missing packaged defaults, malformed registry structure and oversized reports
  produce refusals rather than a successful clean preview.

## Honesty notes

- Existing YAML speed producers that sit above the global ceiling (for example
  `nav2.velocity_smoother.max_velocity_theta`) are emitted at `min(cap, target)`.
  They are reported under `clamped_producers`, not as candidate defects; the
  preview does not run the patch dependency check over every default, which
  would falsely reject them.
- Candidate digests describe YAML starting values merged onto the **locally
  packaged** configuration. They are provenance hints, not observed runtime state, and
  are only meaningful once the same baseline is confirmed by commissioning.
- Unknown deployments are not automatically migrated: the separate root-local
  `tuning_initialize` requires clean-install confirmation and exact reviewed
  digests, because committing packaged YAML as ACTIVE over unknown lower settings
  would be unsafe. See `docs/tuning-code-readiness.md` for the guard/recovery path.
- The [baseline-import foundation](tuning-baseline-import.md) now supports exact
  representable bundles in local unit tests. It is deliberately not wired to
  this CLI or to production mutation transports.
- This slice is topic-branch review work: nothing here is deployed or physically
  accepted, and no robot contact is authorised.
