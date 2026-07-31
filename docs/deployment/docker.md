---
title: Docker Deployment
description: Build, deploy, inspect, and roll back the Dockerized Raspberry Pi 5 stack.
---

# Docker Deployment

The Raspberry Pi 5 (`robot-pi2`, hostname `patrolbot-rpi5`) is the main driver and
runs the three active ROS 2 services from this monorepo. On 2026-07-22 all three
containers were healthy at image `patrolbot:git-b98090a7796d`; the image revision,
embedded source digest, image label, and deployed tree matched. The dedicated SBC
socket and fresh stationary telemetry were also verified live.

## Services

| Service | Container | Role |
|---|---|---|
| `bringup` | `patrolbot-bringup` | `twist_mux` + final velocity smoother |
| `bridge` | `patrolbot-bridge` | TCP client to `10.0.0.1:7272` |
| `navigation` | `patrolbot-navigation` | Nav2 + joystick + safety watchdog + static TF |

All services share one image, host networking, `ROS_DOMAIN_ID=0`, Fast DDS, and
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
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
- Docker health checks plus a separate readiness/status command so a temporarily
  unavailable hardware endpoint is reported as degraded instead of forcing restart loops.

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
ROS DDS participants. The on-demand status command adds SBC data-path,
telemetry freshness, lifecycle, and TF checks:

- `OVERALL=ready` (exit 0): every software, telemetry, localization, and commissioned hardware gate is ready.
- `OVERALL=degraded` (exit 2): containers are healthy but hardware or ROS
  readiness is incomplete.
- `OVERALL=unhealthy` (exit 1): container liveness failed.

`degraded reason=sbc-unavailable` means fresh `/odom` was not observed through
the bridge; it does not mean the containers failed. The command intentionally
does not open a separate TCP probe to the single-client hardware server. It
retries lifecycle discovery to tolerate ROS graph startup delay. The 2026-07-22
stationary acceptance found the containers and telemetry path healthy, while
status correctly remained degraded on intentionally deferred physical readiness
such as joystick, localization, dock pose, and bumpers. The command also enforces one publisher for each hardware
topic and BEST_EFFORT reliability for `/scan` and `/sonar`, so a second bridge or
an outdated sensor QoS deployment cannot pass readiness unnoticed.
It also requires the bridge container's Fast DDS publication mode to be
`ASYNCHRONOUS`; the default synchronous mode can block the sensor publisher
worker on a stalled DDS reader. Readiness also rejects unattended
`ros2 topic hz` processes: wrapping `docker exec` in an outer timeout can leave
the ROS subscriber running with an unread output pipe, which eventually
back-pressures the sensor writer.

## Rollback

For rollback, preserve the exact prior image tag, protected environment file, deployed source tree,
container inspection, and generated checksums before cutover. Restore only the affected services;
do not use an old unrelated Compose directory:

```bash
cd /home/ubuntu/patrolbot-repo
# restore the verified prior source and docker/.env from the backup manifest
docker compose --env-file docker/.env -f docker/docker-compose.yml \
  up -d --no-deps bringup bridge navigation
```

Deployment snapshots are retained on the Pi under `/home/ubuntu/backups/`, in timestamped
directories named `<UTC-timestamp>-<slug>`, and off-host in the ignored `backups/` directory.
The Pi keeps the **two newest** snapshots; older ones are rotated away, so `~/backups/` is not
an archive.

The outgoing snapshot **MUST NOT** be pruned until its replacement has been confirmed working
on the robot — it is the only cheap revert path.

After any network or deployment change, verify `/odom`, `/scan`, `/sonar`, `/battery`,
`/diagnostics`, `odom → base_link`, and `base_link → laser_frame`. Joystick and
Nav2 goal tests require a clear area and an operator at the deadman/E-stop.

## Operational safeguards

- Containers use `restart: unless-stopped`, graceful shutdown, an init process,
  and bounded JSON logs.
- Containers share the host IPC namespace for Fast DDS. Their ROS logs persist
  separately under `logs/bringup`, `logs/bridge`, and `logs/navigation`.
- Only the navigation container receives read-only `/dev/input` access and input
  device cgroup permission; the stack is not privileged.
- The bridge being unable to contact the configured SBC endpoint does not fail container
  liveness or create a restart loop.
