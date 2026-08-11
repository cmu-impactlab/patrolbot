# Session 1 — Instrumentation: report

Date: 2026-08-11. Robot docked throughout. **No motion command was issued at any
point**, no FLASH write was performed, no AMCL parameter was changed.

Status: **changes 1a (SBC) and Step 1 (evidence) are complete and verified.
Changes 1b and 1c (Pi Python) are not yet made** — see "Where this stopped".

---

## 1. The µARCS CONFIG packet (change 1a)

Raw line, verbatim from the SBC journal (`diagnostics/session1/uarcs_config.txt`):

```
Aug 11 16:56:14 patrolbot1 patrolbot_server[7572]: uARCS CONFIG: TicksMM=189 RevCount=30127 DriftFactor=32 HasGyro=0 GyroType=2 GyroFW=0 GyroCW=1500 GyroCCW=1508
```

| Field | Value | Documented `researchPB` default | Note |
|---|---|---|---|
| `TicksMM` | **189** | 192 | Below the default, ~0.3% above the geometric prediction of 188.4 |
| `RevCount` | **30127** | 30700 | 1.87% below the default; non-default, so calibrated at some point |
| `DriftFactor` | **32** | 0 | Non-zero: a firmware drift correction is active |
| `HasGyro` | **0** | — | Derived, not independent — see below |
| `GyroType` | **2** | — | `GYRO_ANALOG_CONTROLLER`: firmware-fused |
| `GyroFWVersion` | **0** | — | Not reported by this firmware |
| `GyroCW` | **1500** | — | Per-direction gyro scale factor |
| `GyroCCW` | **1508** | — | Asymmetric with CW, i.e. genuinely calibrated |

All eight accessors exist on this ARIA version; nothing had to be dropped.
Robot identity from the same connect: `Name: TexasAMQ5100`, `Type: Pioneer`,
`Subtype: patrolbot-sh`, `Serial: CHJ_C05100`, params from
`/usr/local/Aria/params/patrolbot-sh.p`.

### Interpretation of the gyro result — and a correction to the session plan

`HasGyro=0` together with `GyroType=2` looks contradictory, and the session plan
treats them as two mutually exclusive branches of the decision table. They are not.
In `/usr/local/Aria/src/ArRobotConfigPacketReader.cpp:156-160`:

```cpp
myGyroType = packet->bufToUByte();
if (myGyroType == 1)
  myHasGyro = true;
else
  myHasGyro = false;
```

`HasGyro` is **not a field in the packet**. It is a boolean ARIA derives, and it is
true *only* when `GyroType == 1`. `GyroType=2` therefore forces `HasGyro=0` as a
matter of arithmetic. `HasGyro=0` carries no information here beyond "GyroType is
not 1", and the plan's first branch ("`HasGyro == 0` → no gyro configured anywhere")
does not apply and cannot be reached whenever GyroType >= 2.

The governing branch is the third one. `ArAnalogGyro::stabilizingCallback`
(`ArAnalogGyro.cpp:104-119`) makes the semantics explicit:

- `GyroType == 1` → `GYRO_ANALOG_COMPUTER`. ARIA installs an *encoder correction
  callback* and fuses the gyro **client-side**.
- `GyroType >= 2` → `GYRO_ANALOG_CONTROLLER`, logged by ARIA as "microcontroller
  gyro". It sets `myHasNoData = true` and installs **no** encoder correction
  callback, because the µARCS microcontroller fuses the gyro itself before the SIP
  is emitted. It only sets an odometry delay of 25 ms.

**Conclusions, per the plan's own mapping for `GyroType >= 2`:**

1. The firmware fuses the gyro before the SIP. `/odom` heading is **already
   gyro-corrected**, not pure encoder.
2. The `alpha1: 0.1` / `alpha4: 0.1` halving in `nav2_params.yaml` is **defensible**.
   Finding H-1 is *not* a live defect. It should not be reverted to 0.2.
3. **The EKF question is closed.** There is nothing independent left to fuse: ARIA
   reports `myHasNoData = true` for this gyro type, meaning no separate gyro data
   stream reaches the client at all. Adding `robot_localization` to fuse "the gyro"
   with `/odom` would be fusing the gyro with itself.

Corroborating evidence that this is a real, calibrated gyro rather than an absent
one: `GyroCW=1500` and `GyroCCW=1508` are non-default *and asymmetric*, which is the
signature of a per-direction calibration, and `DriftFactor=32` is a non-zero
firmware drift correction. A robot with no gyro would carry neither.

### The odometry constants

`TicksMM=189` against a geometric prediction of 188.4 — from
(4096 ticks/rev x 28.9 gear ratio) / (200 mm x pi) — is agreement to about 0.3%.

This **contradicts the prediction in the session plan**, which expected `TicksMM`
*above* 192 because the added SBC, Pi and LiDAR payload should compress the
foam-filled tires and so raise the ticks-per-mm. The measured value is not merely
below 192, it sits essentially on the uncompressed geometric value. Implied
effective wheel diameter is 199.3 mm against a nominal 200 mm.

Two readings, not distinguishable from this data alone:

- the constant was set from nominal geometry and **has never been re-calibrated**
  for the current payload, in which case there is an uncorrected translational
  scale error whose size is unknown; or
- tire compression under this payload is genuinely negligible.

`RevCount=30127` is 1.87% off the documented 30700 default. Being non-default, it
was calibrated at some point, by someone, against an unrecorded payload.

**Flagged as a finding for a later session:** these constants need a straight-line
and a rotation calibration run against ground truth to decide which reading is
right. That requires motion and so is out of scope here. No FLASH write was made;
client commands 88, 89 and 93 were never issued.

---

## 2. Pre-change evidence (Step 1)

Captured **before any restart** — the navigation container had been up 12 days, so
this is the original state. Full detail in `baseline_before.md`.

**The acceptance criterion is met without ambiguity.** Of 188 `/amcl_pose` samples
over 60 s, **188 have a non-positive diagonal entry**:

```
cov[0]  (xx)      =  8.253264e-10
cov[7]  (yy)      = -6.219768e-10    <- negative
cov[35] (yaw,yaw) = -6.128431e-14    <- negative
```

Every sample is byte-identical; the filter is frozen, not merely overconfident. It
had **not** self-recovered. A matrix with negative diagonal entries is not a
covariance matrix at all.

At the same instant `/patrolbot/dock_readiness` reported `localization_valid: true`
and `ready_for_undock: true`. That is the defect change 1b exists to fix,
demonstrated on the live robot rather than argued from the code.

Also recorded, not acted on: **`/odom` publishes an all-zero covariance** for both
the pose and the twist block. The bridge emits no uncertainty estimate whatsoever.

Baseline rates: `/odom` 22.71 Hz, `/scan` 22.76 Hz, `/tf` 69.6 Hz, `/amcl_pose`
3.13 Hz. Both TF links present.

---

## 3. Verification

| Step 7 item | Result |
|---|---|
| `uARCS CONFIG:` line in the SBC journal, all fields populated | **Pass** — all 8 fields present |
| Values recorded in `uarcs_config.txt` and referenced in report | **Pass** |
| `/odom` ~22.7 Hz, `/scan` ~22.7 Hz after restart | **Pass** — 22.728 / 22.727 Hz |
| No `nav2_params.yaml` values changed (`git diff` empty) | **Pass** — empty |
| Robot never moved; no motion command issued | **Pass** |
| Dock gate rejects synthetic negative variance | **Not yet** — change 1b not made |
| Dock gate still passes a healthy filter | **Not yet** — change 1b not made |
| `_tf_timer` stops broadcasting when `_pose_at` is stale | **Not yet** — change 1c not made |
| `map -> odom` present after re-seed | **Not applicable yet** — nav not restarted |

Post-restart state: `link_connected: true`, `dock_state: DOCKED_CONFIRMED`,
`blocking_reason: ready`, `fault_flags: 0`, `session_generation` incremented 3 -> 4,
which is a clean reconnect rather than a stall.

---

## 4. Things that did not go as described in the plan

1. **The plan's gyro decision table is wrong** where it treats `HasGyro == 0` as an
   independent branch. See section 1. The conclusion it leads to (revert the alphas
   to 0.2) is the *opposite* of the correct one.
2. **The `TicksMM` prediction was wrong in direction.** The plan expected > 192; the
   value is 189.
3. **The SBC network interface is not `eth0`.** It is `enp2s5`, and it holds
   **10.0.0.1/24**, not 10.0.0.2/24 as the plan's Step 0 expectation states. The
   link is up and healthy; only the plan's description is off.
4. **`journalctl --user -u patrolbot-server` returns "No entries"** on this box.
   systemd 229 on Ubuntu 16.04 does not serve the user-unit journal that way. The
   working invocation is by syslog identifier:
   `journalctl -t patrolbot_server`. The command given in Step 6.2 of the plan does
   not work as written. Journal storage is also still `auto` with an empty
   `/var/log/journal`, so **the journal is volatile** and this evidence would not
   survive a reboot; it is why the CONFIG line was copied into the repo immediately.
5. **`systemctl --user restart` warned "patrolbot-server.service changed on disk"**,
   i.e. a unit file was edited without a `daemon-reload` at some earlier point. Not
   caused by this session and not touched by it, but it means the running unit may
   not match the unit on disk. Worth resolving before the next restart.
6. **The server crashed once on restart and self-healed.** The first process could
   not reach the socat bridge on 127.0.0.1:7000 and exited; `Restart=always` brought
   it back 3 s later and it connected cleanly. A startup ordering race against
   `socat-boot.service`, pre-existing.
7. **`/scan` was down for about 47 s across the restart.** `lms2xx_1` failed its
   first configure handshake, ARIA power-cycled the LRF and connected on the second
   attempt. Pre-existing behaviour, not introduced here, but it means an SBC restart
   costs roughly a minute of laser outage rather than a few seconds.
8. **`urg_2` again did not connect**, consistent with the known dead low laser.
9. **Motors were enabled before the restart and are disabled after it**
   (`motors_enabled: true` -> `false`). This is the safe direction and matches the
   server's "Motors were NOT enabled at startup" policy. The robot did not move.
   Noted because anything that later assumes motors are enabled will now find they
   are not.

---

## 5. Behaviour change introduced by 1c

Not yet introduced — change 1c has not been made. When it is: `odom -> base_link`
will **disappear** rather than go stale when the SBC link drops, and downstream
consumers will begin reporting TF lookup failures during an outage instead of
silently consuming a confidently-wrong pose. That is the intended outcome, absorbed
by `patrolbot_safety_watchdog` and `transform_tolerance: 1.0`, but it is a real
behaviour change.

---

## 6. Where this stopped

Change 1a is built, deployed and verified on the SBC. Step 1's evidence is captured
and safe in the repo.

Changes 1b and 1c were not made, because deploying them requires restarting the
`patrolbot-bridge` and `patrolbot-navigation` containers, and a **concurrent agent's
Pi image build finished during this session** (`patrolbot:lan-dds-dabeb879-20260811t133349z`,
built 13:33 UTC) **without having been cut over** — all four containers are still
running the 12-day-old image. Restarting navigation un-localizes the robot
(`set_initial_pose: false`, `always_reset_initial_pose: false`,
`recovery_alpha_slow/fast: 0.0`) and would collide with that pending cutover.

## 7. Commits

| Change | Commit |
|---|---|
| 1a — log the uARCS CONFIG packet | `ae9e2b3` |
| 1b — reject invalid covariance in the dock gate | not yet made |
| 1c — gate the TF broadcast on pose freshness | not yet made |

Local only; nothing pushed. No AI attribution in any commit.

## 8. Backups

- Local: `backups/20260811T134643Z-session1-uarcs-config-log/` (original source,
  md5 verified against the pre-change file).
- SBC: `~/patrolbot-backups/20260811T134646Z-session1-uarcs-config-log/`
  (original source **and** the original binary, so revert is a single `cp`).
- SBC build tree: `~/patrolbot-builds/20260811T134708Z-session1-uarcs-config/`.
