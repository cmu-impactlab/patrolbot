# Advanced tuning: code/unit readiness, not deployment acceptance

Updated 2026-10-20. The current work implements the guarded runtime path on
`feat/advanced-tuning-contract` in both repositories. **The operator's hard rule
is code and unit tests first; deployment/environment tests are deferred.** No
robot contact, Docker validation, native ROS trial or physical acceptance is
authorized by this record. Latest operator report: **SBC down**.

## Implemented paths

- Current YAML starting values and documented Nav2 input constraints. Existing
  saved state is never reset or replaced with a registry fallback.
- Robot-owned immutable revisions, fsynced manifest/startup files, compare-and-
  swap saves, durable request deduplication, job journal and audit records.
- Server-derived administrator identity plus short-lived HMAC authorization,
  bound to robot, actor and exact canonical request. An unsigned DDS actor claim
  never authorizes a mutation. Reads remain unsigned; the key never traverses ROS.
- Separate save and activation capability gates. A durable desired save does not
  activate a node or widen the running policy.
- Fixed-node lifecycle reload, complete parameter readback and immutable digest
  checks. AMCL/RPP are restart-required, not claimed hot-loadable. No arbitrary
  shell/service/node/path API is exposed.
- Existing EpochRuntime ownership, fresh hardware/odom, source-checked explicit
  deadman releases, actual mux/smoother observations, navigator idle service,
  manual-session acknowledgments and persistent smoother-manager maintenance.
- Atomic navigator admission in patch `0018`, so an existing goal refuses tuning
  ownership and new goals cannot enter while maintenance owns navigation.
- F710 cached caps update under idle maintenance and require release/new press;
  web sessions are disarmed. No input is replayed after a configuration change.
- Reload restores the selected map to AMCL without injecting an old pose.
  Navigation stays gated until a fresh operator seed and exact global-costmap
  map-consumption acknowledgment; manual localization-independent control keeps
  its existing separate safety boundary.
- Navigation admission pending after reload is journalled independently of the
  completed configuration job, survives a bridge-only restart, and is never
  silently released. Policy-unavailable holds are independently recovered;
  recovering policy cannot clear a durable job hold.
- Explicit root-local clean-install initialization with three reviewed digests,
  retained consumer-process locks and an external durable initialization marker.
  Corrupt/deleted prior state cannot silently become a clean install.
- Explicit root-local offline last-good recovery for unresolved jobs, while all
  guarded consumers are stopped. Desired/history remain intact; runtime
  verification is reported unknown, never fabricated.
- Startup, registry and compiler provenance are checked. Once initialized,
  losing an enable flag cannot restore packaged defaults; a changed compiler
  requires explicit compatibility review rather than reinterpreting old YAML.
- Unit-tested measured-motion gauges, precise input fields, autosave commit
  behavior, truthful saved/active labels and exact-revision activation dialogs.

## Configuration remains opt-in

No key or enabled flag is created automatically. The server and robot core need
the same private 32–4096-byte key at `PATROLBOT_TUNING_AUTH_KEY_FILE`, mounted
read-only and absent from the browser/web bridge. Key permissions are private;
missing, symlinked, weak or permissive files refuse mutations.

`PATROLBOT_TUNING_ENABLED=1` makes startup use committed ACTIVE artifacts.
`PATROLBOT_TUNING_RUNTIME_ENABLED=1` opts into the fixed-node reload backend.
Neither flag is enabled by this work or is proof of deployed compatibility.
Services, publisher identities, joystick callback support and live evidence must
also be present before runtime admission can succeed.

The portable catalog's `runtime_implemented=true` describes source code only.
Its static `advertised=false` / `activatable=false` entries are not authorization
or claims of installed/native verification. Actual capabilities are derived from
the signed transport, initialized store and current runtime snapshot.

## Local operator tools (implementation only; do not run on the robot yet)

All tools are packaged under `patrolbot_bridge`:

- `tuning_bootstrap_preview`: side-effect-free diagnostic.
- `tuning_initialize`: defaults to preview; a root-local write additionally
  requires `--initialize --confirm-clean-install`, a reason, and exact registry,
  defaults and generated digests from review. Never use it to reset old state.
- `tuning_run --role bringup|navigation`: fixed-role launch retaining a shared
  initialization/recovery exclusion lock.
- `tuning_recover`: exact job/last-good revision/digest with explicit stopped
  confirmation. It writes only startup/journal state and never restarts services.

## Unit-stage verification

The current local unit pass reports **543 core/contract/manual checks** and
**201 navigation/joystick/safety-policy checks**, plus clean bridge-package lint
and docstrings. The paired dashboard reports **851 server tests** (2 external
tests deselected), **468 frontend unit tests**, and **26 mock tests**. These
counts describe source/unit verification, not native ROS or deployed behavior.
All **8 shared contract artifacts** match byte-for-byte between repositories.

The branch is prepared for **draft review**. No merge, activation enablement,
image build, deployment test or physical acceptance is implied by that stage.

## Deferred gates — do not mistake unit success for these

1. Finish review, matched commits/PRs, and unit CI in both repositories. The core
   contract must land before the paired dashboard parity job can pass.
2. The new Nav2 patch and ROS integration have **not** been compiled or exercised
   in a native environment in this unit-only continuation. Validate the pinned
   service/parameter/lifecycle behavior later when separately authorized.
3. Image/package/secret/volume inventory, matched image builds, sole-publisher
   graph and configuration provenance must be verified later. No live defaults,
   image digest or package versions are inferred from these unit tests.
4. Browser/native acceptance and physical trials remain separate later gates.
   A draft PR runs unit CI only; making it ready for review triggers the existing
   browser acceptance workflow. No merge/deploy-ready claim is made while those
   gates remain deferred.
5. Actual deployment and physical motion require a new explicit operator go.

The intended ordinary-drive envelope stays **0.30 m/s / 0.70 rad/s**. Do not widen
it, weaken watchdogs/footprints/collision/origin gates, auto-enable motors, reuse
old poses or automatically resume a goal to make any acceptance check pass.
