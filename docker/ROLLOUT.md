# Candidate rollout

Run `deploy_candidate.py` from the workstation with the prescribed Pi SSHFS
mount available. The default mode checks the payload and completed eight-hour
soak only. It performs no deployment:

```sh
python3 docker/deploy_candidate.py \
  --manifest docker/deployments/issue60-candidate-f7fd2db.json \
  --payload backups/20260928-canonical-candidate/payload-tooling-221d78f.tar.gz \
  --soak-container patrolbot-canonical-soak-f7fd2db-20260928
```

For an already-authorized deployment, add `--apply`. The entry point requires
successful terminal soak evidence on the pinned image and committed fixtures.
It then locks deployment, archives existing snapshots off-host, verifies both
copies, checks candidate and current image/source/health, and takes a fresh
subscriber-only stationary preflight. Only then does it replace the four
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
