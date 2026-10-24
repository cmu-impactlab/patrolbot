# Tuning host helpers — local implementation, not commissioned

## Current navigation-only path

`navigation_supervisor.py` is the reduced fixed helper used by
`RosParameterTransport.restart_navigation()`. It does not use the older
three-container dispatcher described below, add a second signing authority, or
restart the coordinator. Core authenticates the administrator request and
journals the reviewed job; the helper independently revalidates that exact
running job, immutable revision, plan hash, ACTIVE baseline and fixed scope.

- Its only mutation is `docker --host unix:///var/run/docker.sock restart --time
  10 patrolbot-navigation`. No build, pull, arbitrary argv, host/SBC reset,
  dashboard/bridge/bringup restart, promotion, input release or pose is available.
- The core socket is mounted only into the core bridge. The separate boot-only
  socket is mounted only into navigation. Neither socket nor the Docker daemon
  is exposed to the web bridge/dashboard. Standard Compose includes these
  read-only directory mounts and retains existing reset wiring.
- The host operator pins the existing local image ID and real state-volume
  directory in root-owned `/etc/patrolbot/tuning-host/navigation.json`, with a
  matching registry beside it. `navigation.example.json` is disabled by default.
  The actual container image, Compose labels, launcher command and read-only
  state-volume identity must match before support is advertised.
- Root-owned `/var/lib/patrolbot-tuning-host/navigation` journals a dispatch
  before Docker is invoked. The expected container must have a new start time
  before the boot socket can return the exact candidate. Boot permission is
  consumed durably before reply; a lost reply or helper crash never retries
  Docker or authorizes a second candidate boot. Subsequent ordinary boots load
  committed ACTIVE through the existing startup resolver. Permission also binds
  the host boot ID and unchanged core-container start identity: restarting the
  host or coordinator cannot revive an interrupted Apply as a candidate boot.
- `tuning_run` clears any inherited candidate path, retains its initialization
  guard, and validates a boot reply against the immutable job/revision. No
  DESIRED lookup, environment flag or staged artifact alone chooses a candidate.
- Core retains input blocking, reacquires the replacement navigator/F710 once,
  restores only lower operator trims, verifies consumed caches and unchanged
  final publishers, and retains the exact map/fresh-seed barrier before success.

`patrolbot-navigation-apply.service` is a provisioning template, **not an
installation action**. The host needs Python/PyYAML plus the reviewed bridge
Python package/source; runtime directories/control/policy must be root protected.
The directory inodes are preserved across helper restarts so read-only container
mounts still reach newly created sockets. The service runs as root; its sockets
are root-owned mode 0660 for the existing root-running core/navigation containers.
No key is provisioned or new RPC endpoint exposed to the browser.

Portable tests exercise refusal, one-shot persistence, exact boot binding and
the actual transport sequence with offline I/O. The explicit
`test_tuning_nav2_docker.py` acceptance uses a uniquely named disposable,
network-none, no-devices container and the real helper/socket/`tuning_run` boot
path. It verifies candidate-once then ACTIVE on ordinary restart. Its fixture
driver is not production image/volume or full robot command-path commissioning.
Do not install/enable the service or runtime Apply without the separate live go.

## Historical broader fallback prototype — not selected

The portable modules live in `patrolbot_bridge/tuning_{handoff,host_auth,host,candidate_boot}.py`.
They are **not** a daemon, web API, installed host service, or enabled fallback.
`FixedDockerDriver.candidate_runtime_integrated` is false and refuses even when
local policy is enabled. The example policy is documentation, not a loader or
permission file. No host installation/enablement is performed by this tree.

## Boundaries

* `tuning_handoff.stage()` requires an already journalled, running exact reviewed
  `robot_docker` job with fixed services `bringup`, `bridge`, `navigation`, full
  verification scopes and explicit seed loss. The coordinator must first obtain
  real command/owner hold acknowledgments. A staged JSON pointer grants nothing.
* A separate host Ed25519 key authorizes execution **and**, distinctly, candidate
  boot. Both capabilities bind job, revision, generated digest, plan hash, handoff
  hash, store generation, host boot, approved config hash and volume identity.
  They expire within 300 seconds. Runtime consumers receive only the public key.
  Keep private keys outside images, robot state volumes, core/dashboard signers,
  browser and web bridge. No key provisioning is implemented here.
* The protected host control directory consumes an exact job before dispatch.
  A crash before/after dispatch is status/recovery-only; a new signature does not
  permit repeating the same job. The immutable stage/revision history remains.
* The only Docker argv targets the local Unix daemon and fixed root-approved
  Compose/environment files and three services, with no dependencies, builds or
  pulls. It never selects a shell, reset, SBC, dashboard, gimbal or web bridge.
  Root ownership/non-writability, fixed container/image/volume identities and the
  approved configuration digest are checked before the mutation boundary.
* Docker return code is **not** runtime verification. No host code promotes
  ACTIVE, releases any command/seed hold, enables motors or resumes input/goals.
  Readback/identity uncertainty remains held. Explicit existing offline recovery
  invalidates stale candidate grants through the settled job state.

## Integration still required before executable support

1. Native coordinator supplies a reviewed Docker plan *before* confirmation and
   authoritative acknowledged admission through `HostDispatcher.admission`.
2. Provision/root-protect the separate control directory, approved deployment
   snapshot, public verification key and host-only signing key. Bind a persistent
   host-issued volume UUID to the actual local Docker volume inventory. Mount only
   the public key/boot receipt/capability read-only into core consumers, **never**
   into web bridge. The current standard Compose remains unchanged, including all
   reset socket/GID wiring. No Docker socket is added to any container.
3. Integrate `tuning_startup.resolve_candidate()` into a distinct fixed candidate
   launcher, with authenticated per-launch runtime identity and real retained
   command maintenance/candidate core/F710 policy consumption. **Ordinary**
   `resolve()`/`tuning_run` must remain ACTIVE-only; environment flags, staged files
   or DESIRED are never candidate permission. The explicit resolver returns a
   hold-required binding, not permission to drive.
4. Authenticate the replacement runtime and collect the existing complete
   exact-job/revision/plan proof; atomically promote and verify committed policy
   binding before releasing only that job's hold. Seed admission stays separate.
5. Native/container crash, sole-publisher and actual volume/config acceptance in
   an isolated environment. Unit fake drivers prove **only** protocol/persistence.

No native Docker mutation was run for this foundation. No live authorization is
implied. Do not remove the driver's unsupported gate based on fake tests.
