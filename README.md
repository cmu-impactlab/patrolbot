# PatrolBot

Autonomous ROS 2 patrol robot platform: navigation, perception, device integration, and
remote operation on a Pioneer **PatrolBot-SH** mobile base.

PatrolBot is a **two-machine** system:

- An **SBC** (the robot's main PC) owns the physical hardware — the Pioneer base and the
  SICK LMS-200 laser — and streams telemetry to the Pi over a TCP text protocol.
- A **Raspberry Pi 5** (`robot-pi2`, hostname `patrolbot-rpi5`) runs the Dockerized
  ROS 2 Jazzy navigation stack (Nav2, the TCP bridge, and mobile-base launch). Its
  software-only migration is active; hardware-connected acceptance remains pending
  while the SBC is off. The former Raspberry Pi 4 path is retained for rollback.

The two never share a ROS graph: the SBC is not a ROS node. They meet only at a single
TCP socket. This split is the central fact of the architecture — see
[Communication (SBC ↔ Pi)](docs/architecture/communication-architecture.md).

## Documentation

The full documentation is built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/)
and lives under [`docs/`](docs/). Start at [`docs/index.md`](docs/index.md).

### Build the docs locally

```bash
pip install -r requirements.txt
mkdocs serve          # live-reload preview at http://127.0.0.1:8000
mkdocs build --strict # static site into ./site
```

The site deploys to GitHub Pages automatically on push to `main`
(see [`.github/workflows/deploy.yml`](.github/workflows/deploy.yml)).

## Repository layout

| Path | What it is |
|------|------------|
| `docs/` | Documentation source (Markdown + Mermaid) |
| `ros2_ws/src/` | Four ROS 2 packages; three active and legacy `rosaria2` |
| `docker/` | Image, Compose stack, liveness probes, and readiness status |
| `mkdocs.yml` | Site configuration and navigation tree |
| `requirements.txt` | Documentation build toolchain |
| `.github/workflows/deploy.yml` | GitHub Pages deploy workflow |

