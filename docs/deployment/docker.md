---
title: Docker Deployment
description: The Dockerized PatrolBot stack for the Raspberry Pi 5 migration target, including build, run, verification, and rollback.
---

# Docker Deployment

The Dockerized stack lives in the robot source repository under `docker/`. It packages the three
active Pi services into one `patrolbot:jazzy` image and runs them with Docker Compose.

!!! info "Migration state — 2026-07-07"
    The **Raspberry Pi 4** (`robot-pi`) remains the production navigation computer and still runs the
    bare-metal systemd services. The **Raspberry Pi 5** (`robot-pi2`, hostname `patrolbot-rpi5`,
    Ubuntu 24.04.4 LTS, `aarch64`) is provisioned as the Docker migration target. Read-only SSH
    verification on 2026-07-07 confirmed the expected `~/ros2_ws/src` packages, `second_map.yaml`
    at `0.075 m/px`, a Docker directory, and Docker `29.1.3`.

## What Compose Runs

| Compose service | Container | Command | Role |
|---|---|---|---|
| `bringup` | `patrolbot-bringup` | `ros2 launch patrolbot-launch bringup.xml` | `twist_mux` + final velocity smoother |
| `bridge` | `patrolbot-bridge` | `ros2 run patrolbot_bridge bridge_node` | TCP client to the SBC on `172.20.87.231:7272` |
| `navigation` | `patrolbot-navigation` | `ros2 launch patrolbot_navigation bringup.launch.py` | Nav2 + joystick teleop + safety watchdog + static TF |

All three share the same image, use `network_mode: host` for ROS 2 DDS multicast, set
`ROS_DOMAIN_ID=0`, and use `rmw_fastrtps_cpp`. The source trees are bind-mounted read-only from
`${PATROLBOT_WS:-/home/ubuntu/ros2_ws}/src` so launch, params, maps, and scripts can be tuned on the
host and picked up by restarting a container.

`rosaria2` is intentionally excluded. It is the legacy direct-ARIA path and must not run alongside
`patrolbot_bridge`.

## Host Configuration

Create `docker/.env` next to `docker-compose.yml`:

```bash
PATROLBOT_WS=/home/ubuntu/ros2_ws
```

The default is already `/home/ubuntu/ros2_ws`, so the `.env` file is only required if the workspace
lives elsewhere.

Install Docker and Compose on Ubuntu 24.04:

```bash
sudo apt-get install -y docker.io docker-compose-v2 docker-buildx
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

Log out and back in after adding the user to the `docker` group.

## Build

Build natively on the Pi 5 from the workspace `src` tree:

```bash
cd ~/docker
docker buildx build --load -f Dockerfile -t patrolbot:jazzy "$PATROLBOT_WS/src"
```

For a long SSH session, run the build detached:

```bash
nohup docker buildx build --load -f Dockerfile -t patrolbot:jazzy \
  "$PATROLBOT_WS/src" > build.log 2>&1 &
```

Building does not start containers and does not touch the running Pi 4 production stack.

## Cutover Runbook

Do not run the bare-metal systemd stack and Docker stack at the same time; that creates duplicate TF
publishers and competing velocity paths.

1. Snapshot the bare-metal service state:

   ```bash
   systemctl --user status patrolbot-bringup.service patrolbot-bridge.service patrolbot-navigation.service --no-pager
   ```

2. Stop and disable the bare-metal services only when deliberately cutting over:

   ```bash
   systemctl --user disable --now \
     patrolbot-navigation.service patrolbot-bridge.service patrolbot-bringup.service
   ```

3. Start Docker:

   ```bash
   cd ~/docker
   docker compose up -d
   docker compose ps
   ```

4. Verify before commanding motion:

   ```bash
   docker compose ps
   docker logs patrolbot-bridge
   ros2 topic hz /odom /scan /sonar /battery /diagnostics /cmd_vel
   ros2 run tf2_ros tf2_echo odom base_link
   ros2 run tf2_ros tf2_echo base_link laser_frame
   ```

Joystick and Nav2 goal tests require the robot in a clear space with someone at the local joystick
deadman / emergency stop.

## Rollback

Rollback is intentionally simple:

```bash
cd ~/docker
docker compose down
systemctl --user enable --now \
  patrolbot-bringup.service patrolbot-bridge.service patrolbot-navigation.service
```

After rollback, confirm `/odom`, `/scan`, TF, and `/cmd_vel` are flowing from the bare-metal stack.

## Notes

- Compose uses `/dev/input` only for the `navigation` container and grants input-subsystem major 13,
  so the gamepad can be hot-plugged without running the whole stack privileged.
- The first container start can be slow on SD cards because image layers are decompressed. A2-rated
  media or NVMe/USB-SSD boot is preferred.
- Adding a new source file requires rebuilding the image. Editing existing launch, params, maps, or
  scripts usually needs only `docker compose restart <service>` because the source trees are
  bind-mounted.
