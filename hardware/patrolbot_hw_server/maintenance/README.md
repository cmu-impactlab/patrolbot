# PatrolBot SBC Root Maintenance

These assets address the root-owned portions of PB-AUDIT-010, PB-AUDIT-013,
and PB-AUDIT-022 on the legacy Xenial/i686 SBC. They are deliberately not an
automatic installer: stepping the clock, restarting journald, or reloading SSH
must occur in a supervised maintenance window with rollback data already
verified.

PB-AUDIT-012 is not included as an executable change. The current
`/dev/ttyS0 -> socat -> TCP:7000 -> ARIA` chain transports serial bytes but does
not provide a demonstrated HOST-serial RI event to Linux. Installing a shutdown
daemon without a valid signal source would create false confidence. Select and
bench-test a real RI/modem-line or independent power-controller signal before
adding a shutdown service.

The operator confirmed that the earlier PB-AUDIT-016 outages were forced power
removals before the main PC/SBC completed shutdown. Until PB-AUDIT-012 has a
real automatic signal path, use this manual fallback every time:

1. Stop robot activity and confirm zero motion before beginning shutdown.
2. Request the documented main-PC shutdown, either through the PatrolBot LCD
   `PC Power OFF` workflow or with the site's approved SBC shutdown command.
3. Wait for the LCD to report `PC Power OFF` and for the SBC to finish shutting
   down. An SSH disconnect by itself is not proof that shutdown completed.
4. Only then press physical PWR OFF or remove external power.

If the main PC does not reach the documented finished state, treat that as a
shutdown fault and investigate it. Do not normalize forced power removal as the
recovery procedure; the observed log corruption shows why that is unsafe.

## Required gate

Before applying anything:

1. Confirm the robot is stationary and motors are disabled.
2. Confirm no hardware operation or navigation goal is active.
3. Verify the current host-local and off-host backup manifests.
4. Keep one working SSH session open and confirm key-only login in a second
   session.
5. Record the current checksums, owners, modes, and contents of
   `/etc/chrony/chrony.conf`, `/etc/systemd/journald.conf`,
   `/etc/ssh/sshd_config`, and the live PatrolBot user unit/drop-in.
6. Do not combine these changes with a velocity or dock/undock test.

The RTC currently reports 2008 while the system clock reports 2021 and Chrony
knows the real 2026 time. Replace the RTC battery as a physical maintenance
action; a software step alone will not make the correction survive power loss.

## 1. Correct the system clock

Chrony 2.1.1 on this SBC has no parse-only `chronyd` option. The directive was
checked against the installed `chrony.conf(5)` manual, but its deployment must
use the backup and live-service validation below rather than a newer-version
validation command.

With the robot still stationary and motors disabled, first authorize the
one-time forward step against the already-running configuration:

```bash
sudo chronyc makestep
date --iso-8601=seconds
timedatectl
chronyc tracking
```

Expected: current UTC/local time and a small system-time offset. If the date
does not converge immediately, stop; do not keep restarting Chrony or change
its source list during the robot maintenance window.

After the one-time step succeeds, merge the one active directive from
`chrony-makestep.directive` into `/etc/chrony/chrony.conf`. Do not copy the
explanatory comments blindly over the system file. Restart only Chrony and
verify that it remains active and synchronized:

```bash
sudo systemctl restart chrony.service
systemctl is-active chrony.service
chronyc tracking
journalctl -u chrony.service --since '-2 minutes' --no-pager
```

If the service fails, restore the backed-up `chrony.conf`, restart Chrony, and
retain the now-correct system time.

## 2. Enable bounded persistent journaling

Install the reviewed drop-in only after the clock is correct:

```bash
sudo install -d -o root -g root -m 0755 /etc/systemd/journald.conf.d
sudo install -o root -g root -m 0644 \
  hardware/patrolbot_hw_server/systemd/journald.conf.d/90-patrolbot.conf \
  /etc/systemd/journald.conf.d/90-patrolbot.conf
sudo install -d -o root -g systemd-journal -m 2755 /var/log/journal
sudo systemctl restart systemd-journald.service
sudo journalctl --flush
```

The drop-in caps persistent journal use at 128 MiB, keeps at least 2 GiB free,
compresses records, and limits retention to 30 days. Verify:

```bash
journalctl --disk-usage
stat -c '%U:%G %a %n' /var/log/journal
journalctl -b _SYSTEMD_USER_UNIT=patrolbot-server.service --no-pager -n 20
```

Do not restart `patrolbot-server.service` merely to generate a log. Its next
scheduled binary cutover is the appropriate verification point.

## 3. Install the compatible PatrolBot user unit

The canonical unit and drop-in under `../systemd/` are valid on systemd 229.
The drop-in sends ARIA stdout/stderr to the journal and contains none of the
unsupported direct-file append syntax.

Install them as user `ros` before the already-scheduled server cutover:

```bash
install -o ros -g ros -m 0644 \
  hardware/patrolbot_hw_server/systemd/patrolbot-server.service \
  /home/ros/.config/systemd/user/patrolbot-server.service
install -d -o ros -g ros -m 0755 \
  /home/ros/.config/systemd/user/patrolbot-server.service.d
install -o ros -g ros -m 0644 \
  hardware/patrolbot_hw_server/systemd/patrolbot-server.service.d/log.conf \
  /home/ros/.config/systemd/user/patrolbot-server.service.d/log.conf
systemctl --user daemon-reload
```

Do not restart the service until the binary-cutover safety gate passes.

## 4. Disable SSH password authentication

OpenSSH 7.2 requires merging `sshd-hardening.txt` into the main
`/etc/ssh/sshd_config`; do not install it as a drop-in. Remove or replace any
earlier active definitions of the same keys, then validate:

```bash
sudo sshd -t -f /etc/ssh/sshd_config
sudo systemctl reload ssh.service
```

Keep the original SSH session open. From a second terminal, require key-only
batch authentication through each management alias. If either connection
fails, restore the backed-up configuration and reload SSH from the original
session. This hardens the current appliance but does not close PB-AUDIT-022:
retiring Xenial/i686/kernel 4.4 still requires a platform migration.

## Acceptance and rollback

Accept only when:

- current time remains correct and Chrony synchronized;
- a reboot after RTC-battery replacement starts with a sane clock;
- persistent journal records survive a reboot and remain within the cap;
- PatrolBot server records are queryable by `_SYSTEMD_USER_UNIT` after the
  scheduled cutover;
- both key-only SSH aliases work and password login is refused;
- the hardware link, telemetry freshness, zero velocity, and single-client
  invariants remain unchanged.

Rollback the unit, drop-in, journald configuration, or SSH configuration only
from their exact verified backup copies. Correct time itself is not rolled
back. If SSH validation fails, restore and reload immediately without closing
the known-good session. If journal persistence causes disk pressure, remove the
PatrolBot journald drop-in, restart journald, and retain the collected logs for
diagnosis.
