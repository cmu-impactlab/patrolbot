# PatrolBot — Agent Instructions

This is a **live robot**. Code here moves a 50 kg machine. Nothing that changes
robot behaviour is finished until it has been verified on the real hardware.

## Status tracking — the audit system

**`audits/` is the single source of truth for project status**, with exactly
three top-level files: `STATUS.md` (index), `COMPLETED.md` (done + how), and
`OPEN.md` (unfinished + why). `TODO.md` at the repo root is a pointer only.

**Read `SKILLS/AUDIT.md` before recording status anywhere.** It defines the
layout, the three states (✅ / 🟨 / 🔲), what each entry must answer, the
do-not-do warnings to carry forward, and how to handle a contradiction with an
existing record.

Keep the audit files current in the *same response* as the work — never leave
them stale, and never batch status updates for later.

## Reference

Architecture and access details live in `SKILLS/`:

| File | Covers |
|---|---|
| `SKILLS/AUDIT.md` | Status tracking, audit layout, entry rules |
| `SKILLS/rpi-architecture.md` | Raspberry Pi 5 stack, containers, services |
| `SKILLS/sbc-architecture.md` | Legacy SBC, ARIA, µARCS, hardware server |
| `SKILLS/SSH.md` | Remote access and SSHFS protocol |

## Mounting and access

Before working on Pi files, always mount if not already:

```bash
mountpoint -q /home/yousef/Projects/repos/patrolbot-repo/pi || sshfs robot-pi2:/home/ubuntu /home/yousef/Projects/repos/patrolbot-repo/pi -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3
```

SSH to Pi: `/usr/bin/ssh -o BatchMode=yes robot-pi2` (the old `robot-pi` host is
decommissioned — never use it)
SSH to SBC: `/usr/bin/ssh -o BatchMode=yes robot-sbc`
Filter noise: `| grep -v 'WARNING\|quantum\|vulnerable\|upgraded'`

The canonical source tree is this repository. Remote homes are deployment
copies, not source of truth.

## Backups

Back up any existing file before modifying it — revert must be cheap. Keep
private archives on the workstation under `backups/<UTC-timestamp>-<slug>/`,
mirroring the original paths, never in Git. Source work belongs on published
`review/*` branches; observed deployments belong on `deployed/*` branches with
checked manifests. Do not merge unfinished behavior just to clean a worktree.

Durable device-local backup snapshots are retired by the operator's October 15
policy. A device may stage a bounded testing/transaction copy only. Before
removing it, verify the off-device archive's SHA256, contents and metadata and
confirm no runtime dependency or uncertain transaction remains. Runtime databases,
secrets, logs and currently mounted release directories are not disposable backups.
The legacy shared-image rollout's `--apply` is retired; do not use it for the
current mixed-role deployment. See `docs/operations/source-and-release-state.md`.

## Working rules

- Do not guess a physical value to unblock software. Uncommissioned paths must
  keep failing closed.
- Do not send a velocity, goal, pose, charge-release, motor-enable or undock
  command without explicit operator authorization in the current session.
- Prefer the smallest change that closes the finding, through existing typed
  services and the existing single `/cmd_vel` path. Adding a second velocity
  publisher is almost always the wrong answer.
- Test offline first (contract tests run without ROS), then inside the exact
  image, then live.
