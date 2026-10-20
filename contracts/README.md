# Robot tuning contract (slice 1, offline)

Canonical, cross-repo contract for the planned `robot_tuning_v1` capability:
persistent, administrator-only speed / AMCL / Nav2 tuning. **The portable contract
remains non-authorizing.** Guarded runtime, persistence, signed mutation and
fixed-scope reload code now exists. Missing keys, initialization, explicit runtime
enablement or fresh evidence keep the corresponding operations disabled.

The boundaries and evidence below describe the original **slice 1 artifacts**,
not every module now present in the repository. `runtime_implemented: true` records
source implementation, **not native/deployed verification or permission to apply**.
See `docs/tuning-code-readiness.md` for the current boundary and deferred gates.

## Boundary — what this slice does and does not do

In scope:

- `robot-tuning.schema.json` — Draft 2020-12 shapes for
  `read` / `save` / `apply` / `redeploy` / `status` / `restore` requests plus a
  `tuning_state_snapshot` response.
- `robot-tuning.registry.json` — the curated catalog: two speed ceilings, a
  selected AMCL and Nav2 candidate set, and the protected safety gates.
- `robot-tuning.fixtures.json` — valid and invalid request/snapshot examples
  with the exact expected rejection reason.
- `validate_robot_tuning.py` — portable offline validator and consistency checks
  (strict bounded JSON, dependency-closure coherence, optional authoritative
  revision/session context, snapshot honesty, registry structural invariants).
- `test_robot_tuning.py` — portable pytest suite (schema, registry, fixtures,
  strict JSON semantics, dependency closure, mutability matrix, snapshots).
- `test_robot_tuning_sources.py` — robot-repo-only source-consistency tests.

Outside the portable validator itself (historical slice-1 boundary):

- **No runtime authorization.** The static descriptor remains
  `capability.advertised = false` and every registry parameter is
  `activatable: false`; actual runtime admission is separate.
- **No new endpoints.** No HTTP, WebSocket, ROS service, action or topic is
  added or changed.
- **No YAML write interface.** There is no writer, store or file path in this
  slice; a test asserts the modules never call a YAML dump.
- **No ROS import.** The portable modules have no `rclpy`/ROS dependency.
- **No store, no UI.**

## Honest evidence limitations

- Only the local core-repo source tree was read. No live robot/SBC inventory was
  taken, so **no installed package version is claimed**. `package_version_evidence`
  is `unverified` everywhere.
- No native ROS node was executed, so **no hotload/live mutability is proven**.
  Unverified parameters fall back to `restart_required`; structural read-only
  entries are already restart-classified. Every entry is `live_verified: false`.
- The commissioned usable **lower** performance bound for either speed ceiling is
  unknown and recorded as `null` (`lower_bound_status: unknown_uncommissioned`).
  No lower bound is invented.
- AMCL/Nav2 ranges are `candidate_unverified` proposals for review, not approved
  commissioning ranges. They exist so requests can be exercised offline; they
  must not be read as approved limits.
- Two upstream inconsistencies are **recorded, not silently clamped**:
  `nav2.controller.rotate_to_heading_angular_vel = 0.9` and
  `nav2.velocity_smoother.max_velocity_theta = 1.0` both exceed the commissioned
  `0.70 rad/s` cap. The downstream final smoother still clamps, but the registry
  marks `exceeds_effective_ceiling: true`.
- Only the fixed **upper** envelope (`0.30 m/s`, `0.70 rad/s`) is source-backed,
  matching `smoother.yaml` and `contracts/web-teleop.json`.

## What the validator does and does not prove

Honesty about the validator itself:

- **Strict, bounded JSON.** A request is parsed with an explicit byte limit
  (`REQUEST_MAX_BYTES`, 16 KiB) and artifacts (schema/registry/fixtures) with a
  separate larger limit (`ARTIFACT_MAX_BYTES`). Nesting depth and item count are
  bounded before parsing; duplicate keys are rejected; `NaN`/`Infinity` literals
  are rejected and so are float results that overflow to infinity (`1e9999`),
  and integer literals beyond a fixed magnitude. Every in-memory payload is
  scanned — *before* schema validation — for non-finite numbers anywhere
  (snapshots included), illegal types, cycles, oversize, and an aggregate
  **byte budget** (`REQUEST_MAX_BYTES`), so an oversized in-memory payload never
  reaches expensive validation. Only a `str` is accepted as raw text; bytes or
  other types and lone-surrogate/invalid-encoding strings become structured
  errors instead of unbounded encoding work. Revisions must be true integers:
  integral floats (`0.0`) and booleans are rejected on `base_revision`,
  `expected_revision`, `restore_revision` and on snapshot revisions (`null` is
  allowed only for snapshot revisions).
- **No external schema references.** An explicit `schema_ref` must be an
  internal `#/$defs/<name>` pointer that exists in the document; external or
  unresolved references are rejected with a structured error, and the static
  schema itself contains only internal `$ref`s, so `jsonschema` can never be
  asked to fetch anything over the network.
- **Dependency closure.** When a `save` touches a parameter, **every dependency
  edge incident to it (either endpoint)** is evaluated against the merged
  defaults + supplied `base_values` + the whole valid patch. A one-sided edit
  therefore cannot bypass a constraint (e.g. lowering only `max_particles`,
  changing only one of `z_hit`/`z_rand`, or lowering a speed cap without bringing
  its producers in line). Edges unrelated to the patch are left alone, so the
  known upstream `0.9`/`1.0` vs `0.70` angular mismatch never blocks an unrelated
  edit — but an edit that touches the angular component must resolve it.
- **Scope of the closure.** Closure covers only the dependency edges declared in
  the registry. Producers outside the registry (for example the manual web/F710
  teleop path or the SBC velocity classifier) are *not* modelled, which is one
  more reason the capability stays `activatable: false`.
- **No runtime status inference.** Durable pending changes can remain known even
  when runtime readback is unknown. The draft's boolean `pending` cannot express
  genuinely unknown manifest/promotion status; the persistence slice must model
  that explicitly before wiring this draft to a runtime or UI.
- **Shape-only unless the context is complete.** `validation_only` is `true` and
  `authorization_evaluated` is `false` unless the caller passes a **complete and
  valid** authoritative `context={"current_revision": int, "session_id": uuid}`.
  An empty, partial, extra-field or wrong-typed context is itself an error and
  performs **no** contextual checks (so it cannot silently look "checked"). This
  is a standalone shape checker: it **cannot** by itself see the robot's current
  revision or session. Only a complete context rejects a stale compare-and-swap
  revision or a foreign/stale session epoch. It never authorizes anything;
  `activatable` is always `false`.
- **`generation.session_id` is an opaque epoch token**, not the desired
  revision. Future runtime compares it to the authoritative session; the
  compare-and-swap is `base_revision`/`expected_revision` against the
  authoritative revision. Only canonical lowercase UUIDs are accepted so case
  variants cannot alias the same token.
- **Restore binds an immutable target.** `restore` carries both
  `expected_revision` (the reviewed current state) and `restore_revision` (the
  exact last-known-good revision). A moving "latest" target is rejected.
- **Snapshot honesty.** `safe_apply.allowed` is `const false` in slice 1 (no
  activation exists), `safe_apply.reason` must be non-empty, an unknown runtime
  cannot carry *known* active values, `active_known: true` requires a concrete
  `active`, a known runtime cannot have a null active revision, duplicate field
  IDs are rejected, each field's type/classification is checked against the
  registry, and `desired` is bounds-checked. An `active` readback is deliberately
  **not** bounds-checked: real drift outside the current policy must be
  representable rather than rejected as impossible.
- **Pending is durable, not a runtime inference.** `pending` is a persisted
  promotion status owned by the robot manifest. A saved-but-unactivated change
  stays `pending: true` across a reboot even while `runtime` is `unknown` and
  `active` is `null`; unknown runtime therefore does **not** clear a truthful
  pending flag or get conflated with it.

## Mutability matrix — every unverified ability held disabled

The whole point of slice 1 is to describe the shape without pretending the
service exists. `test_robot_tuning.py` enforces the invariants below (never the
exact counts, which may grow with review):

| Group | count | mutability | live_verified | activatable | accepts_edits |
|-------|-------|------------|---------------|-------------|---------------|
| `speed_ceiling` (`max_linear_m_s`, `max_angular_rad_s`) | 2 | unverified | false | false | true (shape only) |
| `amcl` editable candidates | 14 | unverified | false | false | true (shape only) |
| `amcl.alpha5` not applicable | 1 | unverified | false | false | false |
| `nav2_controller` | 8 | unverified | false | false | true (shape only) |
| `nav2_velocity_smoother` | 2 | unverified | false | false | true (shape only) |
| `nav2_behavior` | 1 | unverified | false | false | true (shape only) |
| `protected_safety` | 7 | unverified | false | false | false |
| `structural_read_only` | 2 | restart_required | false | false | false |

No parameter claims `live`, and the snapshot `classification` enum has no
`active`/`live` member. `desired` and `active` are independent: `active` is
meaningful only when `active_known` is true and `runtime` is `known`; an
unknown runtime is a first-class, busy/unsafe state and is never shown as zero.

`accepts_edits: true` means the *request shape* accepts a well-formed change so
the contract can be exercised offline. It does **not** mean the value can be
activated — nothing in this slice is activatable.

## Which files are vendored cross-repo

The dashboard repo vendors **exact copies** of the portable set:

- `robot-tuning.schema.json`
- `robot-tuning.registry.json`
- `robot-tuning.fixtures.json`
- `validate_robot_tuning.py`
- `test_robot_tuning.py`
- `requirements-test.txt`

`test_robot_tuning_sources.py`, `requirements-test.lock.txt` and this README are
robot-repo-only (they depend on, or describe, the core source tree).

## Running the tests

```bash
python -m pip install -r contracts/requirements-test.txt
python -m pytest contracts/ -q          # portable + source consistency
python contracts/validate_robot_tuning.py   # standalone schema/registry/fixture check
```

The source-consistency tests read only local YAML/JSON/launch files; they never
contact a robot, open SSH, publish ROS traffic or change runtime state.

## Continuous integration

`.github/workflows/robot-tuning-contract.yml` runs on every push to `main` and
every pull request (no `paths` filter) so that edits to the pinned source files
(`smoother.yaml`, `mux.yaml`, `nav2_params.yaml`, `contracts/web-teleop.json`)
cannot drift past the source-consistency checks. It runs the standalone
validator, the contract tests, and a selected set of existing portable
teleop/safety tests with `PYTHONPATH=ros2_ws/src/patrolbot_bridge`. Those tests
install a mocked ROS graph; there is no native ROS, no robot contact and no graph
introspection.
