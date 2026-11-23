# Jazzy autonomous-speed runtime study

Companion evidence for the [physical commissioning plan](autonomous-speed-commissioning.md)
and offline apply contract. This is not a procedure for a running robot.

**Result:** One isolated native run passed: an active, idle Nav2 1.3.12 `controller_server` with the real RPP `FollowPath` plugin accepted and read back both speed parameters in one atomic request. The existing BT hold guards ordinary BT execution admission, not all motion admission. Neither result establishes readiness for rollout or a higher speed.

## Active RPP set/readback

| Request to `/controller_server/set_parameters_atomically` | Result | Both doubles from `/controller_server/get_parameters` (lifecycle `active`) |
| --- | --- | --- |
| Initial test configuration | — | `0.22 m/s`, `0.70 rad/s` |
| `0.20`, `0.60` | Success | `0.20`, `0.60` |
| `0.19`, string instead of angular double | Rejected: wrong type | Still `0.20`, `0.60` |
| Restore `0.22`, `0.70` | Success | `0.22`, `0.70` |

The probe used PatrolBot's [controller configuration](../ros2_ws/src/patrolbot_navigation/config/nav2_params.yaml#L126), except for lowering the test's initial angular target from `0.90` to `0.70`. It ran only the controller (including a minimal local costmap) with synthetic static TF, an unused velocity output, and no goals or motion consumers. No numeric request exceeded `0.26 m/s` or `0.70 rad/s`. The node deactivated, cleaned up and exited normally. Both package prefixes were under `/opt/scoped-native/install`; both versions were `1.3.12`. This is a cached validation overlay, not a stock or deployed image; it also has an extra RPP `tuning_cache` diagnostic not present in the study tree and not queried by this probe.

The image's exact-version source shows RPP's `ParameterHandler::dynamicParametersCallback` writes both linear fields (`desired_linear_vel` and `base_desired_linear_vel`) and `rotate_to_heading_angular_vel` (`parameter_handler.cpp:222–225,258–270`). The controller server delegates dotted plugin fields (`controller_server.cpp:849–854`); RPP uses those fields for path and rotate-to-heading commands (`regulated_pure_pursuit_controller.cpp:231,347`). This is source evidence of execution-field updates, not a measurement of those fields or of generated velocity. `setSpeedLimit()` can separately replace the effective linear target without changing parameter readback (`:493–507`); the shipped configuration enables the route server's `AdjustSpeedLimit` operation. Account for other writes to the effective target before displaying a confirmed speed.

The rejection case establishes that a bad type left both *stored* values unchanged; it does not prove rollback of side effects if a later on-set callback rejects, or atomicity across the hold, controller, smoothers and downstream policy. No goal execution, higher range, localization handoff, downstream caps, restart/reconciliation or physical motion was tested. The existing [general tuning backend](../ros2_ws/src/patrolbot_bridge/patrolbot_bridge/tuning_backend.py#L131) still uses a coordinated lifecycle reload; nothing here changes that.

## What the BT hold reserves

[Patch 0018](../docker/nav2-patches/0018-tuning-navigation-admission.patch) puts `setTuningHold(true)` and `tryStartNavigating()` under the same mux mutex. The first to reserve wins: if a navigator already owns the mux, the service refuses rather than cancelling it; if the hold wins, neither configured BT navigator (`/navigate_to_pose`, `/navigate_through_poses`) gets a new execution reservation. In the cached Jazzy source, `SimpleActionServer::handle_goal` can return `ACCEPT_AND_EXECUTE` before `BtActionServer::executeCallback` calls the gate; failure there terminates the goal without ticking its tree. Do not expect `goal_handle.accepted == false`. The same-navigator preempt handler is separate; the hold cannot be acquired over an owning navigator. The hold is an in-process `SetBool`, with no caller token, persistence or stop behavior. `~/navigation_idle` reports only that mux. This assessment is source-only: the cached 0018 matched the study copy byte-for-byte, but no BT action was sent. Existing [patch checks](../ros2_ws/src/patrolbot_bridge/test/test_tuning_runtime_contract.py#L29) are static and [backend reservation tests](../ros2_ws/src/patrolbot_bridge/test/test_tuning_backend.py#L263) use a fake transport.

Outside that mutex:

- **Direct `/follow_path`** is admitted by `controller_server` and can run the same RPP plugin. **Direct `/spin`, `/backup`, `/drive_on_heading`, and `/wait`** belong to `behavior_server`; blocking a new BT tree does not reserve those servers or prove any previously dispatched work has drained.
- **`/follow_waypoints`** has its own server. Its `NavigateToPose` child is gated, but the parent action/lifecycle and waypoint task are not. The route server and controller speed-limit subscription are independent of the mux.
- **Guarded undock and manual drive** have separate ownership: bridge/SBC reverse/turn and `/input/docking`, physical joystick, dashboard deadman, and hardware control. Some check the *bridge* `/patrolbot/tuning_hold` or other interlocks; those are not `/bt_navigator/tuning_hold`. The Nav2 `docking_server` is not launched. See the [command chain](../docs/safety/safety-chain.md#nav_command_mux--inside-nav2) and [undock operation](../docs/operations/controller-undocking.md#independence-and-safeguards). Independent ROS parameter/velocity writers are not reserved by the mux either.

For a future idle-only apply, treat the BT hold as one reservation. Direct controller/behavior ingress and independent motion/configuration owners still need explicit exclusion or enforceable admission policy. Status observations alone do not prevent a new admission. A lost set/read/hold response is not proof of failure or success; a future implementation needs bounded reconciliation before releasing its reservation.

## Requirements before wiring the picker

- Prove exclusive ownership of both cap writes and every ingress to the affected
  controller. `navigation_idle` followed by two independent parameter operations
  does not establish exclusivity. The existing SetBool hold is not a token-bound
  lease and is not the offline contract's adapter.
- Bind the authoritative commissioned bounds, both used caps, and the admission
  decision to one generation. Test wrong-owner and delayed replies, process
  restart while a write/release is unsettled, and loss of the release reply
  **after** a server opened its gate. The offline fake does not establish these
  guarantees for the distributed path.
- Inspect the controller's commanded output through a synthetic path and the
  guard/smoother/collision chain; parameter readback alone does not enforce the
  absolute angular cap on path curves or prevent a route speed-limit update
  from altering the linear target. Demonstrate fail-closed output using the
  current commissioned range before considering a wider one.
- Preserve the legacy active startup artifacts when retiring the general
  tuning UI. Some historical nominal targets are higher than their effective
  commissioned caps and are deliberately clamped on generation; replaying those
  values as a fresh patch is rejected. Do not regenerate from a pending draft.

## Reproduce the one probe

From this tree, with cached `patrolbot-ros-tests:localization-handoff-06d60c31f1fa` (ID `sha256:9d0deb9d528b165724c802d8c78dd197916dde63d270bf3db615284b02c03e0b`):

```bash
docker run --rm --pull=never --network=none --ipc=private \
  --user=65534:65534 --cap-drop=ALL --security-opt=no-new-privileges --read-only \
  --pids-limit=128 --memory=1g --cpus=2 --tmpfs /tmp:rw,nosuid,nodev,mode=1777,size=128m \
  --env HOME=/tmp --env ROS_LOG_DIR=/tmp/ros-log --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --env ROS_STATIC_PEERS= --env ROS_DOMAIN_ID=191 --env PATROLBOT_ISOLATED_TEST=1 \
  --mount "type=bind,src=$PWD,dst=/study,readonly" --workdir /tmp --entrypoint /bin/bash \
  patrolbot-ros-tests:localization-handoff-06d60c31f1fa -ec '
source /opt/ros/jazzy/setup.bash
source /opt/scoped-native/install/setup.bash
exec timeout -k 5s 100s python3 -u -B /study/proposals/autonomous-speed-native-probe.py
'
```

Context7 references: [ROS 2 parameter services and callback caveats](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Parameters.html), [Nav2's Kilted rename of `desired_linear_vel`](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/migration_guides/kilted/Kilted.md), [BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html). Context7 did not index Nav2 `1.3.12` specifically; the exact-version behavior above comes from the cached source and native run.
