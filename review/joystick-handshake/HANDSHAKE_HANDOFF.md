# 🟨 BLOCKED — candidate handshake, NOT deployment admission

`physical_acceptance=false`. No production activation, SSH, Pi writes, robot
connection, deployment/restart, Git operation, motor/charge/undock/goal/pose or
live velocity command occurred. Work is isolated here. Canonical audits were
read only; the parent must update canonical OPEN/STATUS with this partial work.
An initial offline test generated a parent-level Python bytecode cache; only
that generated file/empty cache directory were removed. Subsequent tests use -B.

## Problem

Teleop's independent zero drain can be ready at 0.533 s while the bridge's drain
at 0.534 s has only 0.499 s. Sending nonzero then clears the bridge's joy zero
history before it latches. Increasing valid DDS source timestamps keep arriving,
but the recovery manual bypass remains unowned indefinitely. Publisher names
were already verified; they were not changed to guess around this race.

## Implementation and provenance

All prior recovered source, logs, manifests and handoff remain unmodified under
`../` and the copied archive `evidence-recovered/`. The recovered archive image
is `sha256:bf0e16db385777d1e2f4c398d27312875bc9ea652d0b74e10751b221bede4199`.
It was never started. `results/archive-verification.log` verifies all entries of
its original `RECOVERY-ARTIFACTS.sha256`. This establishes recovered bytes, not
missing original-controller acceptance provenance.

Before-edit snapshots are under `backups/20261015T054434Z-pre-readiness/`,
`backups/pre-response-timestamp/`, `backups/pre-final-contracts/`, and
`backups/pre-unavailable-final/` (additional test snapshots also retained).
`handshake-only-r3.patch` compares the pre-handshake candidate to current source.
The earlier P0 reverse admission and P1 stale-ACK remediation is retained.

* Bridge adds `/patrolbot/manual_ready`, existing `std_srvs/srv/Trigger`. Query
  success records only a one-shot readiness snapshot; it does NOT set Bool,
  drain ownership, motion ownership, motor state, goals or socket output.
* Snapshot binds origin/session/recovery/token/TF generation, drain revocation
  generation, stable endpoint GIDs and map identity/stage. Shared mutation
  generation records intervening lifecycle/map/unknown-write admission,
  navigation/action activity and reverse reservation, even if it settles before
  the reply is consumed. These additions necessarily change the correlated
  epoch/map/undock modules beyond their recovered f0b7/d8f3/37f65 bytes; do not
  assemble a mixed-generation role from those earlier hashes.
* Readiness requires the bridge's OWN zero drain, stable sole source graph,
  fresh raw origin/hardware/LiDAR safety, idle navigation/actions and strictly
  settled mutation ownership. Missing/unknown ownership still rejects.
* Only a fresh trusted Bool may consume the snapshot within 1/30 s, under the
  epoch lock. Bridge revalidates all current lease conditions at DRIVE. A service
  caller alone cannot open the bypass. False revokes/stops the admitted manual
  command immediately through the existing socket path.
* Teleop initially accepts READY only for Bool + priority-8 ZERO. It ramps only
  after a fresh OWNED reply confirms bridge consumption. It polls while driving;
  absent/late/exception/canceled/unavailable/stale/wrong-generation replies give
  zero and False. Navigation cancel acceptance retains zero until a subsequent
  terminal/idle status, including after release. No old-goal auto-resume.
* A response carries its bridge ROS timestamp. Request-to-reply ARRIVAL must be
  <= one tick; consumption must be <= one tick since the bridge timestamp and
  local receipt. Bridge independently enforces a monotonic one-shot deadline.
  This fixes timer jitter without extending reply lifetime, the 0.5 s drain,
  any physical threshold or any physical limit. Callback only records receipt;
  only the current tick consumes the current future/context.

Initial request-age-only implementation was inadequate: the first 120-tick
integration recorded only 1 nonzero. That source and logs are retained in
`backups/pre-response-timestamp/` and `results/installed-handshake-r3.log`.
It is superseded, NOT acceptance. Final strengthened graph check requires >=60
accepted ticks. The corrected r2 candidate gave 102 and 100. The final r3 also
fails closed immediately if the service disappears/throws despite a prior fresh
reply; its three runs gave 100, 83 and 81 over 120 ticks each. Missed one-tick
deadlines remain zero/False, not an excuse to widen the threshold.

## Evidence — NEW tests, not the original 22

Frozen source SHA-256: `results/handshake-sources-r3.json` (same contents as
`results/handshake-sources.json`). Runtime source was unchanged between the last
two installed runs. All eight installed imports matched exact source hashes;
no implementation source bind masked imports.

* Unmodified final boundary regression against the backed-up original: fails
  before (premature `0.02333333333333333` ramp at 0.533 s), passes after.
  `results/offline-before-boundary-r3.log` and
  `results/offline-after-handshake-r3.log`.
* 20 NEW handshake contracts pass offline and against actual installed methods.
  Includes the exact 0.533/0.534 boundary plus synchronized one-command control,
  original indefinite-unowned reproduction, delayed/stale/unavailable/exception/
  canceled service replies, old future identity, false/stale lease, unknown
  mutations, owner/action/context/source GID changes, stale final/buffered zero,
  cancellation until terminal, and held ordinary navigation.
* 21 retained ownership contracts pass offline and installed. These are the
  recovered NEW ownership contracts, not the original 22 native scenarios.
* Real Jazzy DDS + Trigger fake graph: nine denial/positive checks plus actual
  installed teleop tick/client integration and a genuinely delayed ROS service
  check (11 checks total). One positive-control nonzero, 201
  zeros before the separate 120-tick integration, NO additional nonzero during
  denial checks. Final teleop integration: 81 accepted nonzeros / 120 ticks;
  preceding same-source runs: 100 and 83 / 120. A service that sleeps 40 ms then
  returns a freshly timestamped response still results in zero/False and no
  nonzero socket output: the original arrival deadline is retained. Inert safety collaborators/recording
  socket sink, not a plant, full Nav2 stack or hardware acceptance.
* Actual Jazzy callback dictionary keys: `source_timestamp`,
  `received_timestamp`, `publication_sequence_number`,
  `reception_sequence_number` (the latter was null in the captured sample).
  There is NO callback GID metadata. Actual stable endpoint GIDs are captured
  separately in the graph log. This is graph provenance, not DDS security.

Authoritative installed log: `results/installed-handshake-r3-final-delay.log`.
The preceding same-source strengthened graph runs are
`results/installed-handshake-r3-final-a.log` and
`results/installed-handshake-r3-final-b.log`. Each has zero exit codes for all suites.

Final diagnostic immutable ID:
`sha256:441470f84264595217dcc839ac59934975dc030c2e8c3831683638c8d3d43ca6`
tag `patrolbot:joystick-handshake-diagnostic-20261015-r3`.
The earlier handshake IDs `ab5adabc...` and `73cae1f8...`, and ALL prior 03f304/a5b544/f4b9 images,
are NONDEPLOYABLE. No role/shared tag was used for these diagnostics.

Containers: CPU 0.5, memory+swap 512 MB, PID limit 128, network none, private IPC,
cap-drop ALL, no-new-privileges, isolated ROS domain 187, read-only test bind.
Only six Python modules were COPY-overlaid; no workspace/native rebuild. The
local legacy builder was needed because BuildKit attempted registry metadata
resolution for locally stored raw/digest FROM references and failed before any
image changed. These failed attempts are not role builds or acceptance.

## Unfinished gates / refusal

1. **Original 22 native scenarios: ZERO rerun.** The recovered archive has no
   `results/native-r6.json`, original 22-case harness or its manifest. A separate
   never-started r6 image container's `/tmp` was copied to `recovered-r6-tmp/`;
   only Nav2 patches/tarballs were present. Its image history contains controller
   source COPY steps but no native harness COPY. No original-controller result
   hashes can be compared because that original result is unavailable. No old
   acceptance was reconstructed, fabricated or inferred from these new tests.
2. **Exact role leaves: ZERO built.** Local Docker lacks both required bases:
   core `2d3df1c3c6bfe857f256be1a947892e78e942802c633f727b194331ffe73355f`
   and navigation `607d352411ed16c59ab16fb39323534f69d0b42068dad09b503e72f8bea845c8`.
   Evidence: `results/role-base-availability.stderr`. Available build CLI
   capabilities do not establish all required cap-drop/no-new-privileges/private
   IPC guarantees for a Pi build. Remote build path STOPPED; no SSH/Pi work was
   attempted. Do not substitute diagnostic ancestry or a shared tag.
3. `Dockerfile.role-core.TEST-UNBUILT` and
   `Dockerfile.role-navigation.TEST-UNBUILT` are pinned, COPY-only GATED recipes,
   NOT exact validated role artifacts. Role target import locations/symlink
   layout and role interface hashes remain unverified. Navigation must COPY the
   exact shared manual helper as well as JoyTeleop. No XML/native/interface/
   parameter bytes are copied or rebuilt, but preservation/ABI must still be
   measured on each exact role, not assumed from this diagnostic.
4. Restore durable original 22 scenarios/manifests and complete ALL without
   early abort; full retained contract/mandatory fixtures remain unavailable.
   Independently review current handshake and correlated ownership generation
   changes. Build/test exact two roles under verified resource limits; validate
   installed imports, interface hashes, unchanged healthy XML and native ABI.
5. Parent must perform fresh read-only hardware preflight and obtain explicit
   current operator authorization before any later manual/physical acceptance.
   This agent does not deploy or command a robot. Hardware acceptance remains
   false, and canonical project state remains 🟨, not Done.

**Do not deploy an unsafe provisional controller. Do not rerun or alter the
unrelated XML release. Do not guess physical values or relax refusal tests.**

## Restart-resistant archive

`patrolbot-evidence:joystick-handshake-blocked-20261015` is a separate FROM-scratch
evidence archive, NOT a runtime/role image. It contains this candidate at
`/evidence/`; recover using `docker create` (never start) then `docker cp`.
`HANDSHAKE-ARTIFACTS.sha256` pins the snapshot. Its post-build image ID and build
log are recorded in `results/handshake-evidence-image-id.txt` and
`results/handshake-evidence-build.log`, necessarily outside that snapshot.
The original recovery archive tag/image remains untouched.
