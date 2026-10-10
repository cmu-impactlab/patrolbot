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

## Socket protocol

Send one JSON object followed by newline; requests are limited to 4096 bytes.

```json
{"op":"current"}
{"op":"request","request_id":"<uuid>","operation":"software_reset"}
{"op":"status","request_id":"<uuid>"}
{"op":"status"}
```

`current` returns `idle` when configured and verified, `unavailable` when
startup capability checks fail, or the latest durable request. Use it after a
web bridge reconnect. Reusing a request UUID returns its stored state. While a
request is running, other requests return `busy`. After an ambiguous partial
reset, the state remains `uncertain` and blocks new UUIDs until an operator
checks the robot locally. The dashboard must not automatically retry an
uncertain request.

## Interlocks and sequence

Before changing either machine, two independent five-second read-only ROS
captures must both prove fresh valid base telemetry and odometry, stationary
motion, `DOCKED_CONFIRMED`, motors disabled, no active undock, and idle
navigation actions. Unknown or stale state fails before the first restart.
The supervisor records the prior odometry epoch, restarts only the fixed SBC
`patrolbot-server.service`, then restarts the existing Pi `bringup`, `bridge`, and
`navigation` services plus the opt-in `web-bridge` service with fixed Docker arguments, preserving their exact images/configuration. It issues no velocity, pose, motor-enable, charge-release, or
undock command.

Success requires every core Compose service to report healthy, the web bridge
to be running, and a fresh read-only post-reset capture to show the robot
docked, stationary, motors disabled, navigation idle, a changed valid origin,
`localization_recovery_required`, and an empty seed timestamp. Success means
software recovery is up and a fresh operator pose seed is still required
before navigation. Any ambiguous command result or missing postcondition is
durably `uncertain`; it is never repeated automatically.

Offline fake-adapter tests live beside the supervisor and do not contact a
robot. No live reset is part of installation or verification.

## Opt-in deployment

Robot-side installation may precede the dashboard rollout: provision the restricted
SSH access, supervisor and matching web bridge together, then verify their read-only
status. The button requires the matching dashboard server and frontend. Their
deployment remains an operator task; installing the robot side does not update
the production website or establish end-to-end reset acceptance.

Use `compose.override.yaml` alongside the normal Compose file for **every**
subsequent Compose operation on the web bridge. Set `PATROLBOT_RESET_GID` in the
host's backed-up `docker/.env` to the numeric `patrolbot-reset` group ID. The
supervisor restarts existing containers without recreating them, preserving socket
access across its own reset. The web bridge advertises `software_reset` only when
`PATROLBOT_RESET_SOCKET` is configured and the host capability check succeeds.
Mount the directory rather than the socket inode so listener restarts remain
reachable. Never mount the host Docker socket into the web bridge.

The override is not automatically discovered in this subdirectory. Validate the
combined configuration without printing credentials before recreating the bridge:

```sh
docker compose --env-file /home/ubuntu/patrolbot-repo/docker/.env \
  -f /home/ubuntu/patrolbot-repo/docker/docker-compose.yml \
  -f /home/ubuntu/patrolbot-repo/docker/system-reset/compose.override.yaml \
  --profile web-bridge config --quiet
```

Use the same options for an authorized web-bridge deployment. Omitting the override
on recreation removes socket access and makes reset unavailable. A normal Docker
restart preserves the existing mount and environment.

Before enabling: validate Compose without printing its environment, verify the
dedicated SBC SSH alias using the read-only `status` operation, and run
`preflight.py` as a subscriber-only capture. Do not call the request operation to
check connectivity. End-to-end reset execution needs an operator window and
subsequent seed acceptance; offline tests do not commission the button.

Back up every existing deployment file under `~/backups/<UTC>-system-reset/`,
verify its off-host copy, and preserve the two-snapshot policy before changing
Compose, the environment, or services. Rollback removes the opt-in override and
socket environment, restores the prior web image/config, and stops/disables the
host supervisor only after checking no reset is in progress. Never delete the
SQLite history or mark an uncertain request successful to make the button usable.
An uncertain outcome requires local inspection and a reviewed recovery action;
this feature deliberately does not offer a browser bypass.

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
