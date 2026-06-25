---
title: Known Gaps
description: What this documentation does NOT verify — the SBC was unreachable when it was written, plus the places where the live Pi source contradicts older written notes.
---

# Known Gaps

This page is deliberately blunt about the limits of this documentation. It exists so no reader
mistakes an **unverified** claim for a confirmed one, and so the open questions are tracked in one
place rather than discovered by surprise.

There are three kinds of gap here:

1. **SBC access gaps** — the SBC was not reachable when this site was generated, so everything
   SBC-side is a snapshot, not a live audit.
2. **The laser transform orientation** — an unresolved, hardware-pending question.
3. **Contradictions** — places where the **live Pi source** disagrees with older written notes
   (the SKILLS architecture docs and the package READMEs). The live source is treated as
   authoritative.

---

## SBC access gaps

!!! info "SBC was verified 2026-06-25, not directly accessible now"
    The SBC was live-accessed during the 2026-06-25 session: `patrolbot_server.cpp` was read and
    modified to add AUX telemetry (sonar/battery/flags), and the systemd unit files were confirmed.
    The current documentation reflects that session. The SBC is not accessible right now, so the
    state below represents the 2026-06-25 verification, not a new live read.

Specifically unconfirmed since the last session:

- **SBC `linger` status.** `sudo loginctl enable-linger ros` is required once (interactively) for
  `patrolbot-server.service` to autostart at boot. This was listed as a remaining one-time action
  in the 2026-06-25 notes and may or may not have been run since.
- **SBC binary currency.** The `patrolbot_server` binary was last built during the 2026-06-25
  session after the AUX telemetry additions. If `patrolbot_server.cpp` has been modified since,
  the binary may not match the source.
- **SBC port assignments / baud rates** (`/dev/ttyS0` @ 9600, `/dev/ttyS2` @ 38400, socat → :7000,
  server :7272) are confirmed from the 2026-06-25 session and the ARIA `patrolbot-sh.p` profile.

What is **not** affected: everything Pi-side (the bridge, Nav2, the mobile base, the launch/systemd
setup, the maps, the FastDDS profile, `/sonar`, `/battery`, `/diagnostics`) was verified directly
against the live Pi.

---

## Laser transform orientation

!!! warning "Unresolved — needs a visual RViz check on hardware"
    The correct rotation of `base_link → laser_frame` is genuinely unsettled. Three different claims
    exist in the project's history:

    | Claim | Source | Rationale given |
    |---|---|---|
    | **`roll = π`** | **live `bringup.launch.py` (authoritative)** | un-mirror a left↔right-flipped scan (`LaserFlipped=true`, ARIA returns flipped order) |
    | `yaw = π` | SKILLS doc diagram/§5/Note 5, `patrolbot_navigation/README.md` | "SICK mounted facing rearward" |
    | identity (no rotation) | an even earlier note | — |

    **What runs today is `roll = π`** (the static TF args are `x=0.037 y=0 z=0.2 yaw=0 pitch=0
    roll=3.14159`). But `roll = π` and `yaw = π` are *different* rotations, so at most one is right.
    The resolution is a visual check: put RViz on the LAN, confirm scan dots align with real walls,
    then lock in the correct rotation and delete the conflicting notes. This is blocked on remote viz
    over the VPN (see [Remote Operation](deployment/remote-operation.md)).

This documentation reports the **live** value (`roll = π`) everywhere, with this caveat attached.

---

## Contradictions: live Pi source vs. written notes

These are cases where the live Pi source disagrees with the SKILLS architecture docs or the package
READMEs. **The live source is the more current and authoritative truth; the mismatch is flagged
rather than silently resolved.**

### 1. Laser TF: `roll = π` (live) vs. `yaw = π` (notes)

- **Live:** `bringup.launch.py` uses `roll = 3.14159` to un-mirror the scan.
- **Notes:** the SKILLS doc diagram/§5/Note 5 and `patrolbot_navigation/README.md` say `yaw = π`
  ("facing rearward"). (The SKILLS doc's *most recent* Note 2 does acknowledge `roll = π` set
  2026-06-25, so the doc is internally inconsistent.)
- **Resolution:** use `roll = π` (live); orientation correctness still pending hardware — see above.

### 2. Startup: systemd autostart (live) vs. "manual launch only" (notes)

- **Live:** three systemd **user** services (`patrolbot-bringup`, `patrolbot-bridge`,
  `patrolbot-navigation`) autostart at boot; the unit files exist on the Pi.
- **Notes:** the SKILLS doc §1 and §3 say "there is no robotics-specific systemd service; the
  operator launches the stack manually." (The doc's later Note 1 corrects this.)
- **Resolution:** the stack autostarts via systemd; the manual commands are the documented fallback.

### 3. Nav2 composition: `use_composition:=True` (live) vs. trailing comment says `False`

- **Live:** `bringup.launch.py` launches with `use_composition: 'True'` (single `nav2_container`)
  and applies `bond_timeout: 0.0` in the patched lifecycle managers.
- **Notes:** the trailing comment block in `nav2_params.yaml` claims bond starvation is avoided by
  launching with `use_composition:=False` (separate processes).
- **Resolution:** trust the launch file (composition is **mandatory** on this Pi). The yaml comment
  is stale and should be corrected or removed.

### 4. Map resolution: 0.2 m/px (live) vs. 0.1 m/px (README)

- **Live:** `maps/second_map.yaml` has `resolution: 0.2`; `global_costmap resolution` is 0.2 to
  match. The map was downsampled from 0.1 to 0.2 m/px to cut Nav2 startup.
- **Notes:** `patrolbot_navigation/README.md` still describes the active map as "0.1 m/px."
- **Resolution:** the active map is 0.2 m/px. (`local_costmap` is still 0.1 m by design.)

### 5. Scan `range_min`: 0.2 m (code) vs. 0.05 m (README)

- **Live:** `bridge_node.py` sets `range_min = 0.2` (the `SCAN_RANGE_MIN` self-occlusion filter).
- **Notes:** `patrolbot_bridge/README.md` says the scan range is "0.05–8 m."
- **Resolution:** `range_min` is 0.2 m.

---

## Code-hygiene observations

Not contradictions — minor cleanup items found while reading the live Pi source. Tracked here so a
contributor can pick them up (see [Contributing → good first contributions](contributing/contributing.md#good-first-contributions)):

- **Scaffold-default manifest metadata.** `patrolbot_bridge` and `patrolbot-launch` carry
  `maintainer: ubuntu@todo.todo`, `description: TODO: Package description`, `license: TODO`.
  `rosaria2` has `joao@todo.todo`. Only `patrolbot_navigation` has real metadata (MIT, a named
  maintainer).
- **Dead `ekf_config_file` reference.** `bringup.launch.py` defines `ekf_config_file =
  '.../ekf_test.yaml'` but never uses it, and `ekf_test.yaml` was already removed. Safe to delete
  the line.
- **Dead launch / editor temp files** in `patrolbot-launch/launch/` — see
  [Legacy Components](internals/legacy-components.md#known-dead-code--cleanup-candidates).
- **Committed colcon log** under `patrolbot_navigation/log/`.

---

## How to close these gaps

| Gap | What it needs |
|---|---|
| All SBC-side items | SSH/mount access to the SBC, then a live re-audit of `patrolbot_hw_server` + the units |
| Laser orientation | RViz on the LAN; align scan to walls; lock in `roll` vs `yaw`; delete the wrong notes |
| Contradictions 2–5 | Correct the SKILLS docs and package READMEs to match the live source |
| Code hygiene | The small PRs listed above |

When any of these is resolved, update the relevant page **and** remove its entry here — this page
should always reflect the *current* set of unknowns, not a historical one.
