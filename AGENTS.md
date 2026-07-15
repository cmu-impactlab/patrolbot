# PatrolBot Repository Instructions

These instructions apply to the entire repository.

## Source of Truth

- Treat this monorepo as the canonical source for the Raspberry Pi 5 ROS 2
  packages, Docker deployment, and documentation.
- Read the relevant files in `SKILLS/` before changing robot architecture or
  deployment documentation.
- Keep operational documentation aligned with verified runtime behavior. Do not
  restore obsolete Raspberry Pi 4 deployment instructions.

## Verification

- Run `git diff --check` before every commit.
- Run `.venv/bin/mkdocs build --strict` after documentation or MkDocs theme
  changes.
- Use focused tests appropriate to code changes and report exactly what ran.

## Commit Messages

Every commit must use a descriptive subject followed by a structured body with
both a short summary and a detailed explanation. Do not use a subject-only
commit.

Use this format:

```text
<concise imperative subject>

TL;DR:
<one or two sentences describing the outcome>

Detailed description:
- <specific change and why it was needed>
- <additional implementation or documentation details>
- <important compatibility, migration, or operational effects>

Verification:
- <command or check performed>
```

The detailed description must describe what changed, why it changed, and any
important operational impact. Include verification results rather than vague
statements such as "updated files" or "fixed docs."
