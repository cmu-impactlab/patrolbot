---
title: Contributing
description: How to contribute to PatrolBot — the two-machine mental model, where code lives, the read-only-on-the-robot rule, and the documentation-in-sync expectation.
---

# Contributing

PatrolBot is a small, hardware-coupled project. Contributing well is less about process ceremony
and more about respecting two things: the **two-machine split** and the fact that you're often
working against a **live robot**.

## Before you start

1. Read the [Architecture Overview](../architecture/overview.md) and
   [Communication Architecture](../architecture/communication-architecture.md). Almost every change
   touches the SBC↔Pi seam in some way.
2. Know [where code lives](../internals/repository-structure.md): the monorepo is
   canonical and the Pi 5 is a deployment target.
3. Skim [Known Gaps](../known-gaps.md) so you don't "fix" something that is intentionally the way it
   is (or re-introduce a bug a comment is preventing).

## Ground rules

| Rule | Why |
|---|---|
| **Read-only on the robot unless the change is the task.** | Investigation and docs work must never modify the running robot. |
| **Don't run `rosaria2` alongside the bridge.** | TF and cmd_vel conflicts — see [rosaria2](../packages/rosaria2.md). |
| **Keep `base_shift_correction: False`, `bond_timeout: 0.0`, `MAGICK_THREAD_LIMIT=1`.** | Each prevents a specific crash/OOM. Don't remove "defaults that look removable." |
| **Restart the right service or container after changes.** | Otherwise the running robot keeps the old launch/params/scripts. |
| **Keep docs in sync.** | Doc drift is why [Known Gaps](../known-gaps.md) exists. |

## Workflow

```mermaid
flowchart LR
    A["Pick up an issue / gap"] --> B["Branch in the right repo"]
    B --> C["Change src / Docker files"]
    C --> D["colcon test (lint) + build"]
    D --> E["Manual + resilience tests"]
    E --> F["Update README + docs"]
    F --> G["Open PR"]
```

1. **Branch** in this monorepo.
2. **Make the change** following [Coding Standards](../development/coding-standards.md). Comment any
   value whose reason isn't obvious.
3. **Test** — at minimum `colcon test` (lint) and the relevant manual/resilience checks from
   [Testing](../development/testing.md). Anything near the seam re-runs the freeze/resume matrix.
4. **Document** — update the package `README.md` and this site for any structural or factual change.
5. **Open a PR** — see [Pull Request Process](pull-request-process.md).

## Good first contributions

The lowest-risk, highest-value tasks, several drawn straight from [Known Gaps](../known-gaps.md):

- Fix the scaffold-default package manifests (`maintainer`, `description`, `license` `TODO`s).
- Remove the [dead launch files and editor temp files](../internals/legacy-components.md#known-dead-code--cleanup-candidates).
- Add pytest coverage for the bridge's `_parse_telemetry` / `_parse_aux` (pure string functions).
- Clean up stale comments and dead files now that `patrolbot-bringup.service` launches by package
  name.
- Keep Docker migration docs current as the Pi 5 moves toward production.

## Things that need hardware

Some work can only be validated on the robot (and the SBC is sometimes unreachable — see
[Known Gaps](../known-gaps.md)):

- Changes to the SBC server need the SBC.
- Resilience tests need a live link to freeze/resume.

Coordinate access before starting these.
