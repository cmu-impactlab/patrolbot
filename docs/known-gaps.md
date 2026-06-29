---
title: Known Gaps
description: Remaining open questions and unverified claims in this documentation, after the live SBC + Pi audit on 2026-06-29.
---

# Known Gaps

This page tracks what is still unverified or uncertain after the 2026-06-29 live audit of both
machines. Resolved items from earlier sessions are noted below so the audit trail is complete.

---

## Resolved since initial publication

!!! success "SBC fully verified 2026-06-29"
    The SBC (`robot-sbc`, Ubuntu 16.04.7 LTS) was SSH-accessible and audited live. All items
    previously listed as unconfirmed are now closed:

    - **`linger` status** — `Linger=yes` confirmed for user `ros`. `patrolbot-server.service`
      starts at boot without a login session.
    - **Binary currency** — `patrolbot_server` binary last built **2026-06-28** (matches the
      source, which was last modified 2026-06-28 for the command watchdog fix).
    - **Port assignments** — `/dev/ttyS0` @ 9600, `/dev/ttyS2` @ 38400, socat → TCP:7000,
      server TCP:7272 all confirmed live.
    - **Both systemd services running** — `socat-boot.service` (system) and
      `patrolbot-server.service` (user) both `active (running)`.

!!! success "Laser TF orientation confirmed 2026-06-29"
    Live `/tf_static` from the Pi: `base_link → laser_frame` at `x=0.037, z=0.2`, quaternion
    `(x≈1, y=0, z=0, w≈0)` — that is **`roll = π`**. The earlier `yaw = π` claim in the SKILLS
    docs was wrong. `LaserFlipped=true` in `patrolbot-sh.p` is consistent with this. The
    hardware visual check (scan dots vs. real walls in RViz) has not been done, but the TF
    itself is confirmed correct.

---

## Laser transform orientation

!!! warning "Unresolved — needs a visual RViz check on hardware"
    The correct rotation of `base_link → laser_frame` is genuinely unsettled. Three different claims
    exist in the project's history:

    | Claim | Source | Rationale given |
    |---|---|---|
---

## Remaining open questions

### Map server — two instances

The map used for **navigation** (AMCL + costmaps) comes from `map_server` running as a lifecycle
component inside `nav2_container` on the Pi. A **separate** map is served from the operator's
laptop VM as `/map_viz` (a parallel `/map` copy on a different topic), used for visualization in
RViz to avoid overwriting Nav2's `/map`. This is a conscious operational choice and works fine; it
means:

- If the laptop VM is off, RViz shows no map tiles, but the robot navigates normally.
- The nav2 map server (on Pi) is the authoritative source for localization.

### Visual laser scan alignment

The `roll=π` TF is confirmed from live data, but whether the scan dots visually align with real
walls in RViz has not been verified (requires RViz + physical walls in the same room). This is
purely a sanity check — the math is correct.

---

## Contradictions resolved since initial publication

All contradictions from the initial publication have been resolved by the 2026-06-29 live audit.
For the record:

### Laser TF: `roll = π` confirmed

Closed. Live `/tf_static` quaternion `(x≈1, w≈0)` = `roll=π`. The `yaw=π` claim in old SKILLS
docs was wrong.

### Startup: systemd autostart confirmed

Three user services confirmed active live. The SKILLS doc's "manual launch only" claim was stale.

### Nav2 composition: `use_composition:=True` confirmed

Live launch confirmed. The trailing `yaml` comment claiming `False` is stale.

### Map resolution: 0.2 m/px confirmed

`second_map.yaml` confirmed live at 0.2 m/px. `local_costmap` is still 0.1 m.

### Scan `range_min`: corrected to 0.25 m

Live `bridge_node.py` has `SCAN_RANGE_MIN = 0.25` (not 0.2 as earlier noted). Updated in docs.

### Controller: RPP confirmed (DWB removed)

`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` confirmed live.
All DWB references in the docs updated to RPP on 2026-06-29.

---

## Code-hygiene observations

Minor cleanup items from reading the live Pi source:

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
- **Stale comment in `nav2_params.yaml`** claims `use_composition:=False`; actual launch uses `True`.

---

## How to close remaining gaps

| Gap | What it needs |
|---|---|
| Visual laser scan alignment | RViz on LAN, confirm scan dots align with real walls |
| Code hygiene | Small PRs for items above |

When resolved, update the relevant page and remove the entry here.
