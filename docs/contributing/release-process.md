---
title: Release Process
description: How to cut a known-good PatrolBot release — versioning the packages, the pre-release hygiene checklist, tagging, and capturing a reproducible robot state.
---

# Release Process

PatrolBot is a single-robot research platform, not a distributed package, so "release" means
**capturing a known-good, reproducible state of the whole robot** — both machines — and tagging it.
This page describes a lightweight, honest process for that.

## What a release is here

A release pins, together:

- The Pi packages (`patrolbot_bridge`, `patrolbot_navigation`, `patrolbot-launch`) at known commits
  — remembering `patrolbot_navigation` and `rosaria2` have their **own** git repos.
- The deployed `build_backup/patrolbot-launch` copy (it must match `src` at release time).
- The SBC `patrolbot_server` source + the binary build date.
- The active map and the matching costmap resolution.
- The systemd units on both machines.

## Versioning

Package manifests currently read `0.0.0`/`0.0.1`. For a real release, bump them deliberately and
consistently:

| Package | Action |
|---|---|
| `patrolbot_bridge` | set a real version in `package.xml` + `setup.py` (currently `0.0.0`) |
| `patrolbot_navigation` | bump from `0.0.1` |
| `patrolbot-launch` | set from `0.0.0` |
| `patrolbot_hw_server` | tag the SBC repo / record the binary build date |

## Pre-release hygiene checklist

Several of these are open items in [Known Gaps](../known-gaps.md) — a release is the natural time to
close them:

- [ ] Fix scaffold-default manifest metadata (`maintainer: ubuntu@todo.todo`, `description: TODO`,
      `license: TODO` in `patrolbot_bridge`/`patrolbot-launch`; `joao@todo.todo` in `rosaria2`).
- [ ] Remove [dead code / editor temp files](../internals/legacy-components.md#known-dead-code--cleanup-candidates).
- [ ] Reconcile doc/source mismatches (laser TF orientation, map resolution) — ideally after the
      hardware verification.
- [ ] Confirm `src/patrolbot-launch` and `build_backup/patrolbot-launch` are in sync.
- [ ] Confirm the stale `nav2_params.yaml` trailing comment is corrected or removed.
- [ ] `colcon test` clean; resilience matrix green.

## Cutting the release

```mermaid
flowchart LR
    A["hygiene checklist"] --> B["bump versions"]
    B --> C["sync build_backup ← src"]
    C --> D["full resilience + nav test"]
    D --> E["tag each repo"]
    E --> F["record SBC binary + map + units"]
    F --> G["update docs / changelog"]
```

1. Complete the hygiene checklist and bump versions.
2. Ensure `build_backup` matches `src`.
3. Run the full [test set](../development/testing.md): lint, a real navigate-to-goal, and the
   freeze/resume resilience matrix.
4. **Tag** each repository at the release commit (Pi workspace, `patrolbot_navigation`, `rosaria2`
   if changed, the SBC server repo).
5. **Record the environment** that isn't in git: the SBC binary build date, the active map file +
   its resolution, and the systemd unit contents on both machines.
6. Update this site and any changelog.

## Capturing reproducible state

Because key runtime facts live outside git (the `build_backup` copy, the SBC binary, the map, the
units), a release note should explicitly capture them:

```text
Release vX.Y.Z
- Pi workspace @ <commit>; patrolbot_navigation @ <commit>
- build_backup/patrolbot-launch synced to src @ <commit>
- SBC patrolbot_server built <date> from <commit/snapshot>
- Active map: second_map.{pgm,yaml} @ 0.2 m/px; global_costmap resolution 0.2
- systemd units: <list, both machines>
- Discovery server: <enabled/disabled>
```

## Documentation releases

The docs site deploys continuously from `main` (see
[Pull Request Process](pull-request-process.md#merge-and-deploy)); there is no separate docs release
step. Keep the docs honest at all times rather than batching doc updates into a release — the whole
point of [Known Gaps](../known-gaps.md) is that the site states what is verified and what isn't.
