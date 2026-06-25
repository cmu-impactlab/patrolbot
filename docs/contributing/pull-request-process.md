---
title: Pull Request Process
description: What a PatrolBot pull request should contain, the review checklist (especially for changes near the SBC↔Pi seam), and how the docs site deploys on merge.
---

# Pull Request Process

A PatrolBot PR is judged on three things: does it work on the hardware, does it keep the seam
robust, and does it leave the docs accurate. This page is the checklist.

## What a PR should contain

- **A focused change** in the correct repository (recall the nested `.git/` repos).
- **A description** stating: which machine(s) it affects, what was tested, and whether it touches
  the SBC↔Pi seam or the Nav2 lifecycle/launch.
- **Doc updates** for any structural or factual change (package `README.md` + this site).
- **`build_backup` note** if it changes the mobile base — say explicitly that the deployment copy
  was updated.

## Author checklist

- [ ] Builds: `colcon build` (Pi packages) / `make` (SBC server).
- [ ] Lints: `colcon test` passes (flake8/pep257/copyright, ament_lint).
- [ ] Manual check of the affected path (e.g. `/cmd_vel` flows, map appears, scan correct).
- [ ] **Resilience matrix re-run** if the change is near the bridge, SBC server, or
      lifecycle/launch (see [Testing](../development/testing.md#resilience-tests-the-important-ones)).
- [ ] Load-bearing config preserved: `base_shift_correction: False`, `bond_timeout: 0.0`,
      `use_composition: True`, `MAGICK_THREAD_LIMIT/OMP_NUM_THREADS=1` (unless the PR's purpose is to
      change one, with justification).
- [ ] Docs and READMEs updated; no new entries needed in [Known Gaps](../known-gaps.md).

## Reviewer checklist

- [ ] **Which machine?** The PR says, and the change lives on the right one.
- [ ] **Seam safety:** if it touches the bridge/server/launch, the self-healing story still holds
      (reconnect, re-accept, tear-down-not-respawn).
- [ ] **No dead code resurrected:** the change doesn't quietly re-enable `rosaria2`, a commented-out
      launch, or a `Maybe` component without promoting it properly.
- [ ] **Comments explain the why** for any non-obvious value.
- [ ] **Docs match the code** after this change (no new drift).

## Review focus by area

| Area touched | Look hard at |
|---|---|
| `bridge_node.py` | parsing isolation, the 3 s timeout/reconnect, TF decoupling |
| SBC `patrolbot_server.cpp` | the re-accept loop, EAGAIN guard, keepalive/user-timeout |
| `nav2_params.yaml` | the load-bearing flags; the stale trailing comment |
| `bringup.launch.py` / patched launches | composition, `bond_timeout: 0.0`, the OnProcessExit handler, the 20 s timer |
| mobile base | the cmd_vel remaps and that `build_backup` was updated |
| maps | costmap resolution matches the map resolution |

## Merge and deploy

- The docs site deploys automatically: a merge to `main` that changes `docs/`, `mkdocs.yml`, or
  `requirements.txt` triggers `.github/workflows/deploy.yml`, which builds with `mkdocs build
  --strict` and publishes to GitHub Pages. **A docs PR that fails `--strict` (e.g. a broken internal
  link) will fail the build** — preview locally with `mkdocs serve` first.
- Robot code does **not** auto-deploy. Code changes are applied to the machines per
  [Updates](../deployment/updates.md) — including the manual `build_backup` step for the mobile base.

## Commit and PR style

Write plain, descriptive commit messages and PR descriptions, as a team member would. State what
changed and why; reference the issue or the [Known Gaps](../known-gaps.md) item it addresses.
