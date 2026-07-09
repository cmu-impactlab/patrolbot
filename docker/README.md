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
readiness checks only when requested:

- exit `0`, `OVERALL=ready`: containers, SBC, telemetry, lifecycle nodes, and TF
  are ready;
- exit `2`, `OVERALL=degraded`: containers are healthy but hardware or ROS
  readiness is incomplete;
- exit `1`, `OVERALL=unhealthy`: one or more containers failed liveness.

An offline SBC therefore produces a safe `degraded` result rather than an
unhealthy restart loop.

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

Do not run Docker and the old bare-metal services simultaneously. Joystick and
Nav2 goal tests require a clear robot area and an operator at the deadman/E-stop.
