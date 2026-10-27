# Host software reset supervisor

This listener exposes one fixed operation over a Unix stream socket. It does
not expose the Docker socket to the dashboard container. Run the listener on
the Pi host as `ubuntu`, with the existing Docker group and a dedicated
`patrolbot-reset` socket group. Mount only
`/run/patrolbot-system-reset/` read-only into the web bridge
container, and grant that container's bridge process access to the socket's
group. Do not mount `/var/run/docker.sock` into the web bridge.

The host deployment must already contain `/home/ubuntu/patrolbot-repo`, its
configured `docker/.env`, and the dedicated `patrolbot-reset-sbc` SSH alias with a working
non-interactive key for the `ros` SBC account. Its fixed config path is
`/home/ubuntu/patrolbot-support/system-reset/ssh_config`. Provision that key and verify
its forced capabilities out of band; this service creates no SSH keys and
does not probe with a reset command. The remote `ros` user must be able to
restart its own user service and have a live user systemd manager.

Install `patrolbot-system-reset.service` under
`/etc/systemd/system/`, ensure `patrolbot-reset` exists, then reload systemd
and enable/start the service. `StateDirectory` stores the durable dedupe and
progress database under `/var/lib/patrolbot-system-reset/`; systemd creates
the runtime directory under `/run`. The service socket is mode `0660`; arrange
its group ownership as `patrolbot-reset` with the host's systemd runtime
directory policy before exposing it to the bridge. Restarting the listener
preserves request progress in SQLite. An interrupted accepted/running request becomes uncertain, never automatically resumes. A process lock prevents competing supervisor instances.

### Boot-time bind-mount prerequisite

Install `patrolbot-system-reset.tmpfiles.conf` as
`/etc/tmpfiles.d/patrolbot-system-reset.conf`, then run:

```sh
sudo systemd-tmpfiles --create /etc/tmpfiles.d/patrolbot-system-reset.conf
```

This creates only the restricted `0750 ubuntu:patrolbot-reset` **directory** during
sysinit, before Docker restores containers. It does not create a socket, start a
supervisor or request a reset. The supervisor remains responsible for the socket
and its `0660` permissions. Its existing `RuntimeDirectory` policy must preserve
the same directory inode during service restarts.

Without this policy, Docker may restore the web bridge before the supervisor
creates `/run/patrolbot-system-reset`. The strict bind mount then fails with an
OCI `no such file or directory` error, leaving the bridge exited (127) even after
the supervisor subsequently starts. Do not fix this by enabling automatic bind
directory creation or exposing Docker/SSH credentials to the container. After
checking the real supervisor socket exists, starting that exited web bridge is
sufficient; no core or SBC restart is needed. Boot-order acceptance requires a
separately authorized reboot; installation alone is not proof of that experiment.

## Socket protocol

Send one JSON object followed by newline; requests are limited to 4096 bytes.

```json
{"op":"current"}
{"op":"request","request_id":"<uuid>","operation":"software_reset"}
{"op":"status","request_id":"<uuid>"}
{"op":"status"}
```

`current` returns `idle` when configured and verified, `unavailable` when
local management tools or reset access configuration are missing, or the latest durable request. Use it after a
web bridge reconnect. Reusing a request UUID returns its stored state. While a
request is running, other requests return `busy`. After an ambiguous partial reset, its record remains `uncertain`. A new explicitly
confirmed UUID may start another reset; the previous UUID is never replayed.
The dashboard must not automatically retry an uncertain request. Motion remains
held while the current result is unknown.

## Emergency reset sequence

An authorized operator can reset without dock contact, pose, localization,
motion telemetry, idle navigation, or healthy ROS. An in-progress reset is
single-flight; no request is retried automatically. The fixed operation first
stops the Pi bridge, navigation and bringup containers, closing the old hardware
client session through the existing SBC stop/motor-disable path. It then
restarts the fixed SBC `patrolbot-server.service`, restarts the existing Pi core
containers, and restarts the web bridge. Every operation is bounded, and later
operations are attempted even if an earlier one fails.

The supervisor reads no ROS topics and waits for no dock, new-origin,
localization or health postcondition. `succeeded` means the fixed restart
commands returned successfully, not that the robot is ready to navigate.
Errors are recorded honestly as `uncertain`, retaining their request UUID.
A later reset requires a new explicit confirmation. No velocity, goal, pose,
motor-enable, charge-release or undock request is issued. Existing robot startup
and navigation holds remain independent of reset completion; old dashboard
intents are discarded, with no automatic resume.

Images, configuration, maps and audit history are preserved. This restarts
software services, not either computer or controller firmware. The dashboard
request still travels through its existing web bridge connection: recovery
when that process itself is dead needs an independent host management channel.
Offline fake-adapter tests do not contact a robot. No live reset is part of
installation or verification.

## Standard Docker deployment

Robot-side installation may precede the dashboard rollout: provision the restricted
SSH access, supervisor and matching web bridge together, then verify their read-only
status. The button requires the matching dashboard server and frontend. Their
deployment remains an operator task; installing the robot side does not update
the production website or establish end-to-end reset acceptance.

Reset socket/environment/group wiring is part of `docker/docker-compose.yml`.
No feature override is required: a normal web-bridge recreation retains it.
Set `PATROLBOT_RESET_GID` in the host's backed-up `docker/.env` to the actual
numeric `patrolbot-reset` group ID (`getent group patrolbot-reset | cut -d: -f3`).
Missing group configuration fails Compose validation rather than silently
removing reset. This is a standard deployment prerequisite even when selecting
only core services; do not guess a GID or default to the root group. The host
runtime directory must already exist; the bind mount will not create a fake
empty directory if supervisor installation is missing. The
supervisor restarts existing containers without recreating them, preserving socket
access across its own reset. The web bridge advertises `software_reset` only when
`PATROLBOT_RESET_SOCKET` is configured and the local management configuration is present.
Mount the directory rather than the socket inode so listener restarts remain
reachable. Preserve that runtime directory across systemd restarts with
`RuntimeDirectoryPreserve=restart`; replacing it leaves an existing container
bound to the old directory. Never mount the host Docker socket into the web bridge.

`compose.override.yaml` remains an empty compatibility shim for older commands;
it is neither required nor a separate place to add deployment features. Validate
the standard configuration without printing credentials before recreating the bridge:

```sh
docker compose --env-file /home/ubuntu/patrolbot-repo/docker/.env \
  -f /home/ubuntu/patrolbot-repo/docker/docker-compose.yml \
  --profile web-bridge config --quiet
```

Use the same options for an authorized web-bridge deployment. Both normal Compose
recreation and Docker restart preserve the reset integration. Core images, the
source image digests and unrelated services need not change for wiring-only fixes.

Before enabling: validate Compose without printing its environment, verify the
dedicated SBC SSH alias using the read-only `status` operation. `preflight.py`
remains a subscriber-only diagnostic and is never called by reset. Do not call the request operation to
check connectivity. End-to-end reset execution needs explicit operator authorization; navigation
acceptance is separate. Offline tests do not establish live reset acceptance.

Back up every existing deployment file under `~/backups/<UTC>-system-reset/`,
verify its off-host copy, and preserve the two-snapshot policy before changing
Compose, the environment, or services. Rollback restores the exact prior Compose,
environment and web image/config. Stop/disable the host supervisor only if its
removal is explicitly authorized and no reset is in progress. Never delete the
SQLite history or mark an uncertain request successful to make the button usable.
An uncertain record is retained. Another reset must be a newly confirmed
request; never rewrite the previous result or replay it automatically.

## Restricted SBC access

Use `ssh_config.example` as the Pi's dedicated config; pin the SBC host key from
an already verified workstation SSH connection, not an unauthenticated scan.
Generate a dedicated Ed25519 key on the Pi at the configured identity path; do
not copy any workstation private key. Install `sbc-command.sh` as executable
`/home/ros/patrolbot-support/system-reset/sbc-command.sh` on the SBC. Add only
the dedicated public key to its existing backed-up `authorized_keys`, with:

```
no-agent-forwarding,no-port-forwarding,no-X11-forwarding,no-pty,command="/home/ros/patrolbot-support/system-reset/sbc-command.sh" ssh-ed25519 PUBLIC_KEY patrolbot-system-reset
```

The wrapper permits exactly `status` and `restart`; it rejects any other remote
command. `status` uses `LoadState=loaded` output compatible with the SBC's older
systemd (`--value` is unsupported there). Keep private keys mode 0600 and do not
include them in source, PRs, or logs. Provisioning and live reset commissioning
are separate steps; checking `status` does not restart the robot.

## Removing dedicated access after rollback

First verify that the supervisor has no active or uncertain request and complete
the backed-up Pi rollback. Retain its SQLite history. From the workstation's
normal SBC SSH account, back up the current `~/.ssh/authorized_keys` before editing
it. Remove only the line whose Ed25519 key blob exactly matches the dedicated
Pi reset public key. Require exactly one match; stop for inspection if there are
zero or multiple matches. Preserve all other lines, ownership and mode 0600.
Do not overwrite the whole file with an older backup because other access keys
may have changed since installation.

Verify normal workstation SSH still works, then verify the dedicated reset key
can no longer authenticate. Only afterward remove the dedicated Pi private key
and config, and the SBC forced-command wrapper. Keep rollback evidence and
backups; revocation is a separate operator action, not part of a read-only check.
