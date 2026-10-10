# Raspberry Pi home layout

Keep deployment trees and support artifacts separate under `/home/ubuntu`:

```text
patrolbot-repo/         # Robot deployment source, configuration and runtime logs
patrolbot-dashboard/    # Dashboard web bridge deployment source
patrolbot-support/      # Staging builds, experiments, diagnostics and backups
backups -> patrolbot-support/backups
```

Docker bind mounts use the two deployment repositories. Keep their paths stable.
Put new temporary builds, AMCL experiments, dock-pose captures and test workspaces
under `~/patrolbot-support/`, using dated descriptive directory names. Hidden
operational directories such as SSH configuration remain in the home directory.

Use `~/backups/` for robot rollback snapshots. The compatibility symlink keeps
existing deployment and rollback commands valid. Retain the two newest snapshots;
do not create additional backup roots inside deployment source trees.

The September 22 consolidation moved 29 existing entries without deleting their
contents or restarting services. The Pi retains its exact move list and original
Docker mounts in `~/patrolbot-support/layout-manifest.json`. Historical scripts
may still contain their former absolute paths; inspect those before reuse.

To restore the previous layout, first ensure no process uses the relocated
support directories, then run:

```bash
python3 ~/patrolbot-support/restore-layout.py
```

The helper checks for conflicting destinations and retains the manifest. It is
specific to this consolidation, not a general deployment rollback tool.
