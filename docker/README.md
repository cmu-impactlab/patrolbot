# PatrolBot Docker deployment

The Compose stack runs the three active Raspberry Pi services:

- `bringup`: `twist_mux` and the final velocity smoother
- `bridge`: TCP bridge to the SBC at `10.0.0.1:7272`
- `navigation`: Nav2, joystick teleop, safety watchdog, and static laser TF

The legacy `rosaria2` package is versioned in the monorepo but is not built or
run. Never run it alongside `patrolbot_bridge`.

## Configure and build

```bash
cd docker
cp .env.example .env
# Set PATROLBOT_IMAGE to a unique tag and PATROLBOT_REVISION to the Git SHA.
docker compose config
docker compose build navigation
```

If running Compose from the repository root instead of `docker/`, pass the env
file explicitly:

```bash
docker compose --env-file docker/.env -f docker/docker-compose.yml config
```

Only the `navigation` service declares the build because all three services
share the same immutable image tag. The build context is the repository root.
Source is also bind-mounted read-only from `PATROLBOT_ROOT`, so edits to
existing launch, parameter, map, or script files apply after the relevant
container is restarted. Adding files requires an image rebuild.

## Run and inspect

```bash
docker compose up -d
docker compose ps
./patrolbot-status
```

Docker health checks verify expected processes without continuously creating ROS
DDS participants. `patrolbot-status` performs the heavier ROS and hardware
readiness checks only when requested. It uses fresh `/odom` as the end-to-end
SBC data-path check instead of opening a probe connection to the single-client
hardware server. It also requires exactly one `/odom` publisher, exactly one
`/scan` publisher, and BEST_EFFORT reliability on the `/scan` and `/sonar`
publishers, preventing an accidentally re-enabled rollback stack or stale QoS
deployment from passing readiness:

The bridge alone sets `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`. Fast DDS
then sends DDS traffic from its background writer rather than blocking the
bridge's sensor publisher workers on a stalled reader. The status command
treats a missing/different bridge publication mode or an unattended
`ros2 topic hz` process as degraded.

- exit `0`, `OVERALL=ready`: containers, SBC, telemetry, lifecycle nodes, and TF
  are ready;
- exit `2`, `OVERALL=degraded`: containers are healthy but hardware or ROS
  readiness is incomplete;
- exit `1`, `OVERALL=unhealthy`: one or more containers failed liveness.

An unavailable SBC data path therefore produces a safe `degraded` result rather
than an unhealthy restart loop.

Compose shares the host IPC namespace for Fast DDS and stores each container's
ROS logs separately under `logs/{bringup,bridge,navigation}` in the repository.
Use the customizable viewer from the repository root to inspect one module or
all three together:

```bash
logs/patrolbot-logs.sh                         # all Docker logs, follow mode
logs/patrolbot-logs.sh bridge --tail 500      # bridge only
logs/patrolbot-logs.sh all --since 15m --no-follow
logs/patrolbot-logs.sh --source ros navigation
logs/patrolbot-logs.sh --health-only --readiness
```

Run `logs/patrolbot-logs.sh --help` for all filtering, health, timestamp, and
follow options.

On the Pi 5 deployment host, install an optional shell command:

```bash
sudo ln -sf /home/ubuntu/patrolbot-repo/docker/patrolbot-status /usr/local/bin/patrolbot-status
patrolbot-status
```

## Rollback

Keep the previous Compose directory and image tag until supervised hardware
acceptance succeeds:

```bash
docker compose down
cd ~/docker
docker compose up -d
```

Pi 5 repository snapshots created during deployment refreshes live under
`~/backups/patrolbot-repo-rollbacks/`. They are rollback aids, not active
runtime state. After supervised SBC-on acceptance passes, they may be removed to
free space:

```bash
du -sh ~/backups/patrolbot-repo-rollbacks
rm -rf ~/backups/patrolbot-repo-rollbacks
```

Do not run Docker and the old bare-metal services simultaneously. Joystick and
Nav2 goal tests require a clear robot area and an operator at the deadman/E-stop.
