---
title: Maintenance
description: Keeping the robot serviceable — disk and image housekeeping, backup rotation, log limits, battery care, and the SBC clock.
---

# Maintenance

Routine upkeep for a deployed PatrolBot. Commands run **on the Pi** (`robot-pi2`) unless
stated otherwise.

The key words **MUST**, **MUST NOT**, and **SHOULD** are to be interpreted as described in
[RFC 2119](https://www.rfc-editor.org/rfc/rfc2119).

## Disk and Docker images

Image sprawl is the most likely thing to take this robot out of service. Every deployment tags
a new image, and old ones are not removed automatically.

Check current usage:

```bash
df -h /
docker system df
```

The root filesystem is a 58 GB SD card. Docker images and build cache together can consume more
than half of it, so headroom needs watching rather than assuming.

Reclaim build cache first — it is the cheapest space to recover and never contains a rollback
target:

```bash
docker builder prune
```

Only then consider removing images. The currently running images and the **immediately previous**
set **MUST** be kept: the previous image is the rollback path if a deployment misbehaves.
Identify what is in use before removing anything:

```bash
docker ps --format '{{.Names}}\t{{.Image}}'
docker images --format '{{.Repository}}:{{.Tag}}\t{{.CreatedSince}}'
```

`docker system prune -a` **MUST NOT** be run on this robot. It removes every image not backing a
running container, which includes the rollback target.

## Backups

Deployment snapshots live in `/home/ubuntu/backups/`, in directories named
`<UTC-timestamp>-<slug>`.

The policy is **keep the two newest snapshots**. Older ones are rotated away, so `~/backups/` is
a rollback buffer, not an archive — anything that needs keeping permanently belongs off-host.

The outgoing snapshot **MUST NOT** be removed until its replacement has been confirmed working
on the robot.

Off-host, the repository's `backups/` directory holds the same role and is git-ignored.

## Container logs

Log growth is already bounded in the Compose stack:

| Option | Value |
|---|---|
| `driver` | `json-file` |
| `max-size` | 10 MB |
| `max-file` | 5 |

That caps each container at roughly 50 MB. These limits **SHOULD NOT** be raised without
checking free space first — four services at a larger cap add up quickly on an SD card.

## Battery

The safety watchdog stops the robot at **23.0 V** (`battery_stop_voltage`). The bridge treats
**24.0 V** as the recharge threshold.

Sitting below the stop voltage is not a fault state the robot can recover from by itself — it
will hold still until the battery is charged. Return the robot to the dock rather than leaving
it stopped on the floor.

Battery telemetry arrives in the `AUX` frame's `BATT` section at about 5 Hz. If `/battery` is
silent, the SBC link is down; the battery itself is unaffected.

## The SBC clock

The SBC has no working RTC battery, so its clock does not survive a cold boot.

Consequences that matter in practice:

- SBC log timestamps **MUST NOT** be compared against Pi timestamps without accounting for the
  offset. Correlating the two by wall clock produces nonsense.
- Anything that reasons about SBC file modification times is unreliable.

Replacing the RTC battery is the only real fix; until then, treat the Pi as the authoritative
clock for any cross-machine reasoning.

## The SBC platform

The SBC runs Ubuntu 16.04 on i686 and receives no security updates. Access is key-only with no
root login. It cannot be upgraded in place, so maintenance is limited to keeping the exposure
surface small: the dedicated point-to-point link, no additional services, and no widening of the
ROS discovery range.

See [Limitations](../reference/limitations.md).

## Power loss

The serial-to-TCP topology does not expose the base's power-fail signal to Linux, so the SBC
cannot know it is losing power and cannot shut down gracefully. Cutting power is always an
abrupt stop.

Where practical, bring the stack down before removing power:

```bash
cd ~/patrolbot-repo
docker compose -f docker/docker-compose.yml --env-file docker/.env down
```

## Pi power and throttle state

The Pi kernel logs under-voltage events, but the live throttle state is held in a firmware
register that `vcgencmd` normally reads. `vcgencmd` reaches the VideoCore through `/dev/vcio`
and is **not available to the `ubuntu` user** on this robot, so a read-only scan can see
retained under-voltage log events while being unable to answer whether the Pi is throttling
right now.

The same information is world-readable through sysfs, so it needs neither `vcgencmd` nor root:

```bash
docker/pi-power-health.py          # human-readable report
docker/pi-power-health.py --json   # one JSON object
```

It reads `/sys/devices/platform/soc/soc:firmware/get_throttled` (the firmware word — low nibble
live, `0x1xxxx` nibble sticky since boot), the `rpi_volt` low-critical alarm under
`/sys/class/hwmon`, and `cpu0/cpufreq`. Reads only; it changes nothing.

- exit `0`, `OVERALL=clear` — no bit set now or since boot;
- exit `1`, `OVERALL=finding` — a bit is set, and the output names which;
- exit `2`, `OVERALL=unknown` — the firmware word could not be read.

An unreadable source reports `unknown` and never `clear`. Under-voltage **SHOULD** be treated
as a supply fault to be measured at the PSU and cable, not compensated for in software: a
capped clock from an unstable supply masquerades as application, NVMe, or timing problems.

## Routine checks

| Interval | Check |
|---|---|
| Each session | `./docker/status.sh` before commanding motion |
| Each session | `/scan` at 20–25 Hz; both TF links present |
| Each session | `./docker/pi-power-health.py` — no under-voltage or throttle bit set |
| Weekly | `df -h /` and `docker system df` |
| Weekly | `~/backups/` holds exactly the two newest snapshots |
| On redeploy | Revision and source-tree digest match the intended build |
| On redeploy | The previous image is still present as a rollback target |

<!-- SPDX-License-Identifier: Apache-2.0 -->
