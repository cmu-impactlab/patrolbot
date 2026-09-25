# Nav2 costmap stale warning patch

`0001-costmap-stale-warning-policy.patch` targets the extracted Navigation2
1.3.12 `nav2_costmap_2d` package. It changes warning cadence only; the existing
`isCurrent()` freshness predicate and return value are unchanged.

The per-buffer registry keeps the warning state outside `ObservationBuffer`, so
the public class layout and prebuilt plugin ABI remain unchanged. The first
stale check logs immediately, then stale checks log every 30 seconds by
`steady_clock`. Recovery logs once per stale episode. The logged counter counts
only checks while stale, so a recovery message reports the number of stale
checks accumulated over that buffer instance, including prior episodes.

At 5 checks per second for 8 hours, one continuously stale buffer performs
144,000 stale checks and emits 960 warnings during the interval, plus the
boundary warning at exactly 8 hours if checked. Healthy checks do not increase
the stale counter.

The canonical helper source is
`nav2_costmap_2d/src/stale_warning_policy.hpp`; the standalone C++ test
includes that same file. Validate the artifact and policy with:

```bash
python3 docker/nav2-patches/test_patch_artifact.py
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  docker/nav2-patches/test_stale_warning_policy.cpp \
  -Idocker/nav2-patches/nav2_costmap_2d/src \
  -o /tmp/nav2-stale-warning-policy-test
/tmp/nav2-stale-warning-policy-test
```

## Initial-pose timestamp retry

`0002-amcl-initial-pose-tf-retry.patch` queues a seed until the original
message time and fixed receipt time are covered by odometry TF. It retries
asynchronously every 20 ms, with one steady-clock deadline derived from
`transform_tolerance`. A newer seed supersedes the pending request;
deactivation, cleanup and shutdown cancel it. Timeout rejects the seed instead
of silently using an identity motion transform. No seed is republished and no
TF is future-dated by this patch.

Build the scoped overlay with `docker/Dockerfile.nav2-fixes`, supplying
`PATROLBOT_BASE_IMAGE` for the reviewed core image. The build checks the
installed AMCL and costmap versions are 1.3.12 and verifies the upstream
archive checksum. `PATROLBOT_NAV2_PATCH_SHA256` records the digest of sorted
patch basenames followed by NUL and each patch body. Candidate validation must
precede deployment.

Network-isolated integration fixtures in `docker/tests/` cover delayed TF,
timeout, supersession, lifecycle cancellation, costmap currentness throughout
an accelerated eight-hour outage, real cloud recovery, and safety-watchdog
hold/release. The accelerated outage is not eight hours of wall-clock resource
soak. The warning cadence's injected steady-clock test independently covers
960 warnings per eight hours at 5 Hz. Resource measurement uses `wait4` for the
test child alone to exclude compiler resource use.

## Operator-seed acceptance acknowledgement

`0003-amcl-seed-acceptance.patch` applies after patch 0002. It publishes the
original seed on `amcl/accepted_initial_pose` only after timestamped TF lookup
and particle initialization succeed. Timed-out, superseded and lifecycle-
canceled requests do not emit an acknowledgement. With
`always_reset_initial_pose=true`, configure also clears inactive seed reuse
and old transform validity.

`integration_amcl_seed_ack.py` checks acknowledgement correlation against the
compiled candidate. `Dockerfile.epoch-candidate` builds a development overlay
from the earlier two-patch candidate; it is not a deployment instruction.
The epoch coordinator and its consumers must be validated together before
rollout, including real lifecycle-manager pause/resume and old-goal termination.

## Behavior deactivation

`0004-behavior-deactivation.patch` makes `TimedBehavior::execute()` observe
its action server's active state. In Navigation2 1.3.12, deactivation first
marks the action server inactive, then waits for the execution callback.
Without this check an active Wait continues looping after deactivation,
preventing the lifecycle pause from completing. On callback return the
existing action worker terminates outstanding goals. New goals are rejected
by the inactive action server; no additional velocity publisher is introduced.

The overlay also builds `nav2_behaviors`. The isolated
`integration_epoch_navigation.py` exercises an active real Wait across an
origin change and requires its terminal result plus completed recovery.
Compilation or unit tests alone do not establish this acceptance.

## Pending-seed scheduling

`0005-amcl-pending-seed-priority.patch` skips laser processing while an
initial-pose request is waiting for timestamped TF. This prevents first-scanner
map preprocessing from occupying the callback group ahead of that request's
bounded retry. Pending requests already withdraw localization; after seed
acceptance, the next scan constructs or updates the scanner normally. The
original deadline, TF requirements and rejection behavior are unchanged.

Validate with the seed acknowledgement timeout/supersession tests and repeated
real-map startup/origin-recovery tests. A single successful startup does not
establish repeatable acceptance; retain failed-run evidence.

## Navigator plugin lifetime

`0006-bt-plugin-lifetime.patch` keeps the two navigator shared libraries mapped
until process exit on Linux using the ELF `NODELETE` linker flag. Executor-held
entity references can outlive the navigator plugin loader; their control-block
vtables must still be mapped when those references are released. This does not
retain active goals or navigator instances, change lifecycle acceptance, or
relax safety checks. The tradeoff is retaining the two code mappings for the
process lifetime rather than reclaiming them during plugin unload.

`Dockerfile.bt-lifetime` rebuilds only `nav2_bt_navigator` on an existing epoch
image, verifies the pinned upstream archive and package version, and checks both
ELF flags. The full Nav2 and epoch recipes also apply this patch and build the
package. Its separate patch digest is recorded in the scoped image label.

Validate `integration_component_teardown.py` with and without
`PATROLBOT_TEST_CLEANUP=1`, then composed epoch recovery/action termination and
scan-outage recovery through clean shutdown. All integration probes require a
network-isolated container. A passing component probe alone is insufficient.

Current validation: the scoped candidate passes all 12 component teardown cases
(six direct shutdown and six explicit cleanup). Composed scan-outage recovery
passes its recovery assertions, but the container still crashes during final
shutdown. This patch resolves the isolated navigator-plugin failure only; the
remaining composed failure is under investigation. Do not treat this candidate
as a complete shutdown fix or deploy it on the basis of component tests.

## Route-operation plugin lifetime

`0007-route-plugin-lifetime.patch` applies Linux `NODELETE` to the
`route_operations` library only. Configured RouteServer teardown otherwise
releases plugin code before retained weak references are destroyed. The native
trace faults at a control-block vtable inside the unloaded library. This retains
code mappings until process exit, not operation instances or active execution.

A diagnostic build passes all 16 component cases (eight direct shutdown and
eight with explicit cleanup), including RouteServer and the navigator controls.
Full-stack recovery and shutdown acceptance remain separately required before
production deployment.

The first composed test of the route diagnostic build still faults in lifecycle
node destruction. The standalone RouteServer regression is fixed in that build;
the remaining full-stack fault has not been attributed. No production rollout
is accepted on these results.
