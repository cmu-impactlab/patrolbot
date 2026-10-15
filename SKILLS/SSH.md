# PatrolBot Remote Access and SSHFS Protocol

> October 15, 2026 correction: `robot-pi` is decommissioned; do not use it.
> Current aliases are `robot-pi2`, `robot-sbc`, and `robot-dashboard`, all using
> `/usr/bin/ssh -o BatchMode=yes`. The Pi mount is the ignored repository `pi/`
> directory prescribed by AGENTS.md, not the historical `.remote/` layout below.
> Remote release trees must match the recorded `deployed/*` source/manifests;
> they are not independent development repositories. The remaining text is
> historical access evidence, not permission to revive old services.

## Canonical workspace

The canonical local repository is:

```text
/home/yousef/Projects/repos/patrolbot-repo
```

Remote homes are live deployment views, not source repositories. Never import
`build/`, `install/`, `log/`, credentials, caches, or nested `.git` directories.

## Machines

| Machine | SSH alias | Remote home | Role |
|---|---|---|---|
| Robot SBC | `robot-sbc` | `/home/ros` | hardware/ARIA TCP server |
| Raspberry Pi 4 | `robot-pi` | `/home/ubuntu` | powered bare-metal rollback board; isolate its ROS services during Pi 5 operation |
| Raspberry Pi 5 | `robot-pi2` / `ubuntu@patrolbot-ros2` | `/home/ubuntu` | main driver; active Docker board |

Use non-interactive SSH:

```bash
/usr/bin/ssh -o BatchMode=yes robot-sbc "echo SBC_OK"
/usr/bin/ssh -o BatchMode=yes robot-pi "echo PI4_OK"
/usr/bin/ssh -o BatchMode=yes ubuntu@patrolbot-ros2 "echo PI5_OK"
```

Live audit on 2026-07-15: all three aliases are reachable. Re-check at the start
of every task because power and network state can change.

## SSHFS mount points

All live mounts go under the ignored `.remote/` directory:

```text
/home/yousef/Projects/repos/patrolbot-repo/.remote/sbc
/home/yousef/Projects/repos/patrolbot-repo/.remote/pi4
/home/yousef/Projects/repos/patrolbot-repo/.remote/pi5
```

Example Pi 5 mount:

```bash
mkdir -p /home/yousef/Projects/repos/patrolbot-repo/.remote/pi5
sshfs ubuntu@patrolbot-ros2:/home/ubuntu \
  /home/yousef/Projects/repos/patrolbot-repo/.remote/pi5 \
  -o reconnect \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -o allow_other \
  -o compression=no
```

Use the same options for `robot-pi` and `robot-sbc`, changing the remote home and
mount point. Always validate immediately:

```bash
mount | grep '/home/yousef/Projects/repos/patrolbot-repo/.remote'
ls /home/yousef/Projects/repos/patrolbot-repo/.remote/pi5
```

Re-check the mount immediately before editing live files. If it breaks, unmount
and retry once with the same protocol; if that fails, stop and report it.

## Limitations

- SSHFS does not make the remote tree canonical or transaction-safe. Develop and
  commit locally, then deploy an exact revision.
- Never run Git recursively across `.remote/` or force-add it.
- A disconnected mount can appear stale; validate both `mount` output and a
  directory listing before relying on it.
- Pi 5 `eth0` was live at `10.0.0.2/24`, 100 Mbps/full duplex in the final
  2026-07-15 audit, and `10.0.0.1:7272` was reachable.
  `/etc/netplan/60-sbc-link.yaml` persistently assigns the address with DHCP
  disabled.
- Both Pis use `ROS_DOMAIN_ID=0` on Wi-Fi. Stop the Pi 4 bare-metal services
  while Pi 5 is the main driver to avoid duplicate ROS node names and command
  publishers.
