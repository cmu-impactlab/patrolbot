# Source, deployments and backups

## October 15 baseline

The intended integration branch remains `main` in both repositories. Current
running source is recorded separately on `deployed/robot-20261015` and dashboard
`deployed/dashboard-20261015`. These are observed baselines, **not approvals of
known defects** and not a claim that old images were rebuilt from those commits.
Do not replace a deployment with main or a review snapshot merely to align names.

Run from each corresponding checkout:

```bash
python3 -B docker/verify_release.py docker/deployments/observed-20261015.json --live
# In the dashboard repository:
python3 -B scripts/verify_release.py infrastructure/releases/observed-20261015.json --live
```

Without `--live` these only check repository files. With it they use read-only
SSH/container file reads, inspect image IDs/running SBC executable, and check
dashboard public asset hashes. They never issue ROS requests or restart services.
Passing proves recorded byte parity, **not** physical readiness or clean rebuilds.

The recorded Compose file is `docker/deployments/observed-20261015.compose.yaml`.
It deliberately preserves existing role-specific image/source mounts. It is
evidence, not permission to run `up`, build or recreate anything. Existing env,
reset override, named volumes, TLS and private credentials remain required.
The captured hardware profile is `hardware/patrolbot_hw_server/profiles/observed-20261015.conf`;
it is evidence only, not a default commissioning flag for another robot.

## Required future release workflow

1. Develop in a named branch/worktree; commit useful source/tests before ending
   a session. Publish review branches, never rely on `/tmp`, a device home or a
   directory named backups as the sole copy of source.
2. Review/merge only ready features to main. Keep incomplete override, manual
   ownership, undock and native-planning work separate until their own gates pass.
3. Build from clean, pinned commits. Record both repository SHAs, exact per-role
   image digests, native patch/interface/import hashes, map identities, SBC source
   and executable/toolchain, static assets and non-secret configuration.
4. Verify artifact retrieval and off-device rollback/config/state archives before
   any separately authorized activation. A shared old image is not a rollback
   for the present mixed-role release. Pin base-image/dependency inputs; current
   mutable recipes alone do not establish bit-for-bit rebuildability.
5. Test offline, then exact installed ARM/native artifacts in isolation. Perform
   fresh read-only quiescent admission; activate only explicitly approved roles.
6. Run source/image/interface/map/public-asset verification after activation.
   Reject unexplained drift; never silently overwrite it or relabel acceptance.
7. Update the deployment manifest and current audit in the same change. Hardware
   criteria require independently authorized physical acceptance.

The old `deploy_candidate.py --apply` refuses before any remote action: it
assumes a shared image and device-local keep-two rollback policy. A replacement
per-role transaction/rollback procedure must be reviewed/tested before use. Its
check-only evidence path remains available.

## Off-device private archives

Durable rollback snapshots belong off devices. The first migrated archive set is
on the workstation at `backups/20261015T082506Z-clean-state-review/device-archives/`.
Its sanitized [manifest](../../audits/2026-10-clean-state/archives.json) records
exact original paths, SHA256, bytes and entry counts. Raw tars may contain
credentials/DB/firmware state: never commit them. Current source belongs in Git;
images/binaries belong in artifact storage, not arbitrary Git source snapshots.

To recover a historical snapshot, first reverify the corresponding tar SHA256,
inspect its relative members, and deliberately restore it under the recorded
host home with ownership/modes preserved. Restore required prior images and
state before using any historical rollback helper. No automatic restore or
rollback is authorized by this document.

Device-local testing/transaction staging must be bounded and explicitly recorded.
Remove it only after terminal outcome, verified off-device archival and no active
mount/process/config dependency. Never treat runtime DBs, secrets, release
binds, locks with unknown outcomes or useful failed-test evidence as clutter.
A second independent encrypted archive copy is still recommended; this cleanup
establishes one off-device location, not a complete disaster-recovery service.
