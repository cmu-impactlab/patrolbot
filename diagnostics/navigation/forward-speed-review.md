# Forward-speed review and rollback evidence

This is an offline review checklist, not a deployment or motion-test procedure.
The 1.8 m/s request and 2.8 m forward reach are study inputs. Neither is a
validated setting, and the shadow observer cannot authorize motion.

## Match the artifacts before review

Keep two distinct sets: **candidate review** and **known-good rollback**. For
each, record the full core and dashboard commits, archive checksums, and build
tool versions. A copy of `main` is a source baseline, not evidence of the
currently installed rollback. Confirm the latter separately before any change.

| Component | Evidence to attach |
| --- | --- |
| Pi ARM64 | Runtime image digest and architecture, source-tree digest, exact Nav2 upstream and patch hashes, hashes of launch/config files and active map revision |
| SBC | Binary digest and architecture, source commit, compiler and ARIA library versions, configuration digest and startup inventory |
| Dashboard | Server and Pi-side bridge image digests, frontend bundle digest, protocol/capability versions, configuration revision (no secrets) |
| Rollback | The corresponding *deployed* digests and verified state/config backups, plus a record of how the matched set was restored in rehearsal |

Label any missing binary, packaging step, or accepting party explicitly. A
source package, an ARIA-free test binary, and a local x86 Jazzy run do not
substitute for the Pi ARM64 or SBC runtime artifacts.

## Offline rehearsal

- Hash the candidate and source-baseline archives, extract into separate
  scratch directories, and confirm every commit independently of the filenames.
- Confirm the candidate leaves the checked-in motion settings unchanged and
  that no runtime launch starts the observer. Confirm the baseline restores
  the original UI and accepts only the original command contract.
- In an isolated test domain, verify that a monitor stop does not stand in for
  measured deceleration; record the source and whether each sample is
  simulated. Check stale data, ownership handoff, reconnect and default trims.
- Rehearse switching the scratch inputs back to the baseline as a set; verify
  checksums and compare baseline outputs. Do not label this a deployed rollback
  or change any live service.

## Evidence required for a separate live release decision

- Bind captures to this unit, its exact software set and configuration, and
  independently verified personnel protection approved for the facility.
  The existing navigation LiDAR and software monitor alone are not proof of
  personnel protection.
- Correlate owner and command at ingress, measured speed, detection, stop
  request, and observed stop on a common timeline. Record uncertainty, the
  leading-edge reference, conditions, and repeatability. Keep the longer
  field until **measured** speed permits shortening. A stop command, shadow
  output, or another unit's result is not this unit's physical evidence.
- Require coverage for Nav2, dashboard and physical controller on the
  same matched build. Include turns, restricted visibility, mixed-traffic
  protection, live trim changes, and transitions back to normal speed.
  Compare reverse, recovery and undocking against unchanged baseline bounds.
- Verify that setting changes cannot arm or resume motion. Manual recovery
  requires a new deadman press; there is no automatic resume after E-stop,
  fault or reconnect. Any Nav2 obstacle-clear resume needs separate safety
  and facility approval.
- Leave all three modes at the current limits unless all three pass and
  release is separately authorized. Missing ownership, stale evidence, and
  unverified rollback are blockers, not waivers.
