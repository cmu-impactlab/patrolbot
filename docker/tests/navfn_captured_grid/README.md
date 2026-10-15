# Captured Navfn regression

This calls the installed production `NavfnPlanner::createPlan`, including its
coordinate rounding, start-cell clearing and 0.5 m goal tolerance. Only the
costmap and configuration are supplied by the fixture; no live ROS graph runs.
Build with assertions enabled (the target also undefines `NDEBUG`).

Mount these retained evidence files read-only; the binaries remain outside Git:

| Fixture path | Mount | Query time UTC | SHA256 |
|---|---|---|---|
| `backups/20261013-goal-spin-diagnosis/fresh-global-costmap.bin` | `/fixture/oct13.bin` | October 13, 18:16:33.935 | `3d40a9e560a01f510fe6fb9a716afcd322ee9df689f62afe401568eb1c8af9b0` |
| `backups/20261014-nav2-goal-diagnosis/fresh-costmap.bin` | `/fixture/oct14.bin` | October 14, 15:01:28.464 | `4f71356c2a9d54fb54b082f1852961b28d7d6df118444e098a304372aa4f4b98` |

Both full grids are 4235 by 3289, resolution `0.05000000074505806`, origin zero.
They are post-trial queries, not the original trials' dynamic scenes.

Inside an installed production container with `--network none --ipc private
--cap-drop ALL --security-opt no-new-privileges` and
`PATROLBOT_ISOLATED_TEST=1`, source `/entrypoint.sh` and run:

```sh
echo '3d40a9e560a01f510fe6fb9a716afcd322ee9df689f62afe401568eb1c8af9b0  /fixture/oct13.bin' | sha256sum --check
echo '4f71356c2a9d54fb54b082f1852961b28d7d6df118444e098a304372aa4f4b98  /fixture/oct14.bin' | sha256sum --check
cmake -S /validation/docker/tests/navfn_captured_grid -B /tmp/navfn-build
cmake --build /tmp/navfn-build -j2
/tmp/navfn-build/captured_grid /fixture/oct13.bin /fixture/oct14.bin
```

Bound the whole case to 120 seconds. A pass requires native paths from the
supplied free starts for October 13 audit273 and both October 14 goals
(audit346 / native UUID `6c9252e02f50480a911694b5e58b80bd`, audit348 /
`2cadba57dcec4719b2720582e378512e`). Each logged trapped start must throw
`nav2_core::StartOccupied` (planner action code 205 / `START_OCCUPIED`), with
the rounded start cell still occupied. Audit348 also checks its last logged start.
The exact coordinates come from retained command audit and planner logs.
Same-position start/goal requests must retain the free one-point path and reject
trapped starts. A small synthetic grid also requires normal paths from an occupied
start with one free or allowed-unknown exit, preserving Navfn's existing cost rules.

The check requires the installed patched Navfn library. A generic no-path
exception fails it. It does not establish physical localization, the historical
costmap, or motion; the scan/map/frame cause of the displaced estimate remains
unresolved. Physical acceptance is deferred by the operator.
