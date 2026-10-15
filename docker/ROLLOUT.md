# Candidate rollout

> **Historical tool, October 15 correction:** check-only inspection remains
> available; `--apply` now refuses before access/mutation. Its shared-image and
> keep-two device-snapshot assumptions do not describe the current deployment.
> Use the per-role manifests and read-only checks in
> `docs/operations/source-and-release-state.md`. Do not execute the old apply or
> device-local rollback examples below without a separately reviewed procedure.

Run `deploy_candidate.py` from the workstation with the prescribed Pi SSHFS
mount available. The default mode checks the payload and completed eight-hour
soak only. It performs no deployment:

```sh
python3 docker/deploy_candidate.py \
  --manifest docker/deployments/issue60-candidate-f5ab152.json \
  --payload backups/20260929-canonical-readiness/payload-tooling-098cbaf.tar.gz \
  --soak-container patrolbot-canonical-soak-f5ab152-20260929
```

The manifest, payload and soak must refer to the same candidate. A completed
soak from an older image does not satisfy this check.

For an already-authorized deployment, add `--apply`. The entry point requires
successful terminal soak evidence on the pinned image and committed fixtures.
It then locks deployment, archives existing snapshots off-host, verifies both
copies, checks candidate and current image/source/health, and takes a fresh
subscriber-only stationary preflight. Only then does it replace the eight
declared source files and image selection and recreate bringup, bridge and
navigation. The dashboard service is outside this rollout.

Failed restart/postflight restores source and environment before recreating and
checking the previous image. A verified new snapshot is also copied off-host
before old snapshots are retired; the two newest remain on the Pi. Archive
directories on the workstation have mode 0700 because snapshots include the
deployment environment. Never publish or commit them.

The result identifies the backup, off-host archive and `rollback.py`. To restore
that snapshot deliberately, run its `rollback.py` on the Pi. It requires a fresh
stationary preflight and verifies the previous image and postflight. Successful
container health is not acceptance of physical localization.

An SSH failure is not evidence that the remote transaction stopped. Unknown
outcomes or incomplete snapshots retain `.deployment.lock`; inspect the
corresponding `.deployment-state-<snapshot>.json`, its recorded process and
actual container/source state before taking further action. Do not delete a
lock or start a competing apply/rollback based only on a quiet log or timeout.
No pose, velocity, goal, motor-enable, charge-release or undock command is issued
by these tools.
