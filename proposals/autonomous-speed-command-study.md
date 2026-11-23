# Jazzy RPP: observed commands versus parameter readback

**Passed.** The real Nav2 1.3.12 RPP plugin changed its straight-line command after an atomic parameter update. A subsequent route-style speed-limit message changed the command again without changing either parameter. An absolute message also raised the command above `desired_linear_vel` while staying below the current `0.26 m/s` envelope: parameter readback alone is not evidence of a hard output cap.

| Phase of one active `FollowPath` goal | ROS parameter readback (m/s, rad/s) | Controller output (m/s, rad/s) |
| --- | --- | --- |
| Initial | `0.22, 0.70` | `0.22, 0.00` |
| Atomic update confirmed | `0.20, 0.60` | `0.20, 0.00` |
| `SpeedLimit(percentage=true, speed_limit=75.0)` | `0.20, 0.60` | `0.15, 0.00` |
| `SpeedLimit(percentage=false, speed_limit=0.24)` | `0.20, 0.60` | `0.24, 0.00` |
| Zero/no-limit sentinel | `0.20, 0.60` | `0.20, 0.00` |

Each phase required three consecutive matching `Twist` samples and active-state readback of both doubles. The probe cancelled the goal, observed `CANCELED`, then deactivated and cleaned up the controller; the process exited normally. An initial run covered the absolute message; the final run also matched the route message form. The cached Jazzy `nav2_route/src/plugins/route_operations/adjust_speed_limit.cpp:63` sets `percentage = true`. The probe published the message itself; it did **not** run a route server or show that the route plugin sends absolute messages. See [Nav2's route operation reference](https://github.com/ros-navigation/navigation2/blob/main/nav2_route/README.md) for the topic contract.

**Fixture and limits.** `nav2_controller`, `nav2_regulated_pure_pursuit_controller`, and `nav2_msgs` resolved to the cached `/opt/scoped-native/install` overlay at `1.3.12`. Only the controller (with a minimal local costmap) and the probe ran, with identity static TF, stationary synthetic odometry, and a straight 1.2 m path. The remapped output had one publisher (`controller_server`) and one subscriber (the probe). No actuator, downstream smoother, collision monitor, BT hold, route graph, or robot was involved. Angular readback changed but the straight path did not exercise rotation. Applying during the synthetic goal tests plugin behavior, **not** whether in-goal updates should be allowed on the robot.

## Offline output guard

The probe replays each real captured command through `autonomous_speed_output.py`
without publishing the result. The 0.24 m/s samples are capped at 0.20 m/s;
the 0.15 m/s samples stay at 0.15 m/s. This is a model for a future
controller-only ingress, not a new node in the graph.

The model stops on missing/stale authority, an expired or wrong-epoch sample,
reverse input, or a raw value outside the existing envelope. Within the
envelope it uses a common scale for linear and angular output to avoid
silently changing requested curvature. A live integration still needs a
trusted source and independent receipt timestamp, owner/generation proof,
and closed-loop/stop acceptance downstream of smoothing and collision gating.
It must not be inserted on any topic shared with manual controls, behavior
recovery, or undock. Local replay does not approve a higher envelope.

## Reproduce

From this worktree, run [the separate probe](autonomous-speed-command-probe.py) using the cached image `patrolbot-ros-tests:localization-handoff-06d60c31f1fa` (`sha256:9d0deb9d528b165724c802d8c78dd197916dde63d270bf3db615284b02c03e0b`):

```bash
docker run --rm --pull=never --network=none --ipc=private \
  --user=65534:65534 --cap-drop=ALL --security-opt=no-new-privileges --read-only \
  --pids-limit=128 --memory=1g --cpus=2 --tmpfs /tmp:rw,nosuid,nodev,mode=1777,size=128m \
  --env HOME=/tmp --env ROS_LOG_DIR=/tmp/ros-log --env ROS_LOCALHOST_ONLY=0 \
  --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST --env ROS_STATIC_PEERS= \
  --env ROS_DOMAIN_ID=192 --env PATROLBOT_ISOLATED_TEST=1 \
  --mount "type=bind,src=$PWD,dst=/study,readonly" --workdir /tmp --entrypoint /bin/bash \
  patrolbot-ros-tests:localization-handoff-06d60c31f1fa -ec '
source /opt/ros/jazzy/setup.bash
source /opt/scoped-native/install/setup.bash
exec timeout -k 5s 75s python3 -u -B /study/proposals/autonomous-speed-command-probe.py
'
```
