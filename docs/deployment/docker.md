---
title: Docker Deployment
description: Build, deploy, inspect, and roll back the Dockerized Raspberry Pi 5 stack.
---

# Docker Deployment

The Raspberry Pi 5 (`robot-pi2`, hostname `patrolbot-rpi5`) runs the three active
ROS 2 services from this monorepo. The SBC is currently off, so software liveness
is validated but hardware-connected readiness and motion acceptance remain pending.

## Services

| Service | Container | Role |
|---|---|---|
| `bringup` | `patrolbot-bringup` | `twist_mux` + final velocity smoother |
| `bridge` | `patrolbot-bridge` | TCP client to `10.0.0.1:7272` |
| `navigation` | `patrolbot-navigation` | Nav2 + joystick + safety watchdog + static TF |

All services share one image, host networking, `ROS_DOMAIN_ID=0`, and FastDDS.
Source is bind-mounted read-only from `PATROLBOT_ROOT`. The legacy `rosaria2`
package is versioned but deliberately excluded from the image.

## Relationship to the old ROS 1 Docker layout

The deprecated `~/Projects/repos/tamuq-patrolbot` ROS 1 stack is a useful
architectural ancestor, not a literal template. The part worth preserving is the
role split: separate long-running services for hardware/bridge, bringup, teleop,
and navigation, all using host networking so ROS discovery and device-facing
processes behave like native services.

The ROS 2 Pi 5 stack intentionally modernizes that shape:

- no ROS 1 `roscore`;
- no source cloning during image builds — the monorepo is the source of truth;
- no broad `privileged: true` or whole-`/dev` mounts;
- immutable image tags/revisions for rollback;
- Docker health checks plus a separate readiness/status command so an offline
  SBC is reported as degraded instead of forcing restart loops.

## Build and run

```bash
cd ~/patrolbot-repo/docker
cp .env.example .env
# Set PATROLBOT_IMAGE to a unique tag and PATROLBOT_REVISION to the Git SHA.
docker compose config
docker compose build navigation
docker compose up -d
docker compose ps
```

If running Compose from the repository root instead of `~/patrolbot-repo/docker`,
pass the env file explicitly:

```bash
docker compose --env-file docker/.env -f docker/docker-compose.yml config
```

Only the `navigation` service declares the build because all three services
share the same immutable image tag. The build context is the repository root.
Existing launch, parameter, map, and script files can be tuned through the bind
mounts and applied with a targeted restart. Adding files requires rebuilding the
image.

## Health and readiness

```bash
cd ~/patrolbot-repo/docker
./patrolbot-status
```

Docker health checks inspect expected processes without continuously creating
ROS DDS participants. The on-demand status command adds SBC reachability,
telemetry freshness, lifecycle, and TF checks:

- `OVERALL=ready` (exit 0): software and hardware data paths are ready.
- `OVERALL=degraded` (exit 2): containers are healthy but hardware or ROS
  readiness is incomplete.
- `OVERALL=unhealthy` (exit 1): container liveness failed.

With the SBC off, `degraded reason=sbc-unavailable` is the expected safe result.

## Rollback

Never run this stack and the old bare-metal services simultaneously. During the
migration, preserve both the previous Compose directory and its image tag:

```bash
cd ~/patrolbot-repo/docker
docker compose down
cd ~/docker
docker compose up -d
```

Short-lived repository snapshots made during Pi 5 refreshes are grouped under
`~/backups/patrolbot-repo-rollbacks/` on `robot-pi2`. They are rollback aids,
not active runtime state. Keep them until the supervised SBC-on acceptance test
passes; after that, they may be deleted to recover space:

```bash
du -sh ~/backups/patrolbot-repo-rollbacks
rm -rf ~/backups/patrolbot-repo-rollbacks
```

After the SBC returns, verify `/odom`, `/scan`, `/sonar`, `/battery`,
`/diagnostics`, `odom → base_link`, and `base_link → laser_frame`. Joystick and
Nav2 goal tests require a clear area and an operator at the deadman/E-stop.

## Operational safeguards

- Containers use `restart: unless-stopped`, graceful shutdown, an init process,
  and bounded JSON logs.
- Only the navigation container receives read-only `/dev/input` access and input
  device cgroup permission; the stack is not privileged.
- The bridge being unable to contact a powered-off SBC does not fail container
  liveness or create a restart loop.
