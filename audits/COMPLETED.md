# PatrolBot — accepted work, scoped

## October 15 source preservation and private archive migration

**Problem:** useful source was dirty or temporary; device rollback snapshots were
inconsistent and would be lost or leak credentials if blindly added to Git.

**Implementation:** preserve original work on named review branches, capture
running sources in separate deployment branches, add read-only hash verifiers,
and archive exact device snapshot paths off-device before retirement. The
dashboard host's existing dirty checkout was committed without changing its files.

**Evidence:** robot 136 source/config/patch files plus three core image identities
and running SBC executable match; dashboard/web 62 files plus their image IDs
match. Rebuilt frontend matches all 15 installed static files, entry SHA256
`92453189385baeb86d01b5100a1fbc9a0b5ad7af9dc6d950405591cbaf1aa664`.
Five verifier/legacy-apply regressions pass. Thirty-six snapshot directories (Pi11/SBC9/
dashboard16) and six stray Pi backup files were streamed, SHA256 checked,
content/path/mode/owner/mtime verified, rechecked and removed by exact path.
Private archives and manifests are under workstation
`backups/20261015T082506Z-clean-state-review/device-archives/` (not in Git).

**Residue/rollback:** restore historical snapshot paths from their verified tar
archives before using an old rollback script. Runtime release trees, credentials,
databases and image rollback artifacts remain untouched. SBC root-only process
inspection was unavailable; owned application/config/mount checks found no backup
dependencies. No service restart, behavior change or hardware acceptance occurred.
Only one off-device archive location is established; a second durable/encrypted
copy remains recommended. Complete build reproducibility stays PB020 🟨.

## Historical accepted substeps retained

- October 13: battery timestamp-index repair and live throughput acceptance;
  current backend source includes it. This does not recover older lost samples.
- October 13: 5,400.087676 s docked parked-validity endurance; moving accuracy,
  physical alignment and guarded undock were not accepted by this test.
- October 11: real link-only reconnect retained covariance/origin/seed.
- October 11: malformed dashboard requests received negative ACKs and durable audit.
- Bumper-group physical mapping and October 12 FLASH readbacks are accepted;
  enabled-motion cutoff remains open.
- Explicit motor-disable and historical undock successes remain valid for their
  recorded artifacts; later regressions do not erase them.
- Original single-velocity-path, journaling, smoother, protocol, odometry and
  interface-drift acceptance remain scoped to the original recorded releases.

The full previous narratives remain private in
`audits/history/20261015-pre-cleanup-COMPLETED.md` and dated campaign evidence.
These are historical acceptances, not assertions that current devices still run
every previously accepted artifact. Current state is in [STATUS.md](STATUS.md)
and [OPEN.md](OPEN.md).
