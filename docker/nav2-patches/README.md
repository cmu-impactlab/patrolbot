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

## Regulated-pure-pursuit plugin lifetime

`0008-controller-plugin-lifetime.patch` retains the controller library's code
mappings on Linux until process exit. Configured ControllerServer otherwise
releases the plugin while lifecycle weak references still need control-block
vtables defined in it. This does not retain controller instances or enable motion.

The diagnostic build passes all 20 component cases without preload (ten normal,
ten explicit cleanup). Composed recovery and managed shutdown requests pass,
but final process exit still crashes during ROS Context destruction. The combined
candidate remains undeployed; this is not full-stack shutdown acceptance.

## Post-seed scan ordering

`0009-amcl-post-seed-scan.patch` records the timestamp of each successfully
accepted seed and rejects laser scans at or before that timestamp. A delayed
scan otherwise consumes AMCL's first forced update while the robot is stationary,
leaving the bridge correctly waiting for a newer estimate. The barrier is set
only after TF lookup and filter initialization succeed, follows each new seed,
and resets during lifecycle cleanup. It preserves the bridge freshness gate.

`integration_amcl_seed_scan_barrier.py` reproduces the deployed failure with a
scan 150 ms older than the seed. The AMCL-only candidate passes old/equal-stamp
rejection and produces a newer estimate without motion or a refresh request.
The five existing seed-acknowledgement cases also pass. Composed recovery and
live fresh-seed acceptance are still required; wall alignment remains a separate
physical acceptance gate. `Dockerfile.amcl-seed-scan` builds this scoped overlay
from the deployed image without the experimental teardown changes.

## Lifecycle-manager callback ownership

`0010-manager-callback-lifetime.patch` retains the Context callback handle and
removes the callback during manager destruction. Otherwise the Context keeps a
callback whose bound object and component library have already been destroyed,
causing a process-exit fault. No library-retention flag is needed for this repair.

A manager with zero managed nodes reproduced the fault twice. With the patch,
all 12 component cases pass and the composed Nav2 process exits cleanly after
recovery and managed shutdown. The composed test also exposed Python child
shutdown errors and a launch exception; complete stack shutdown is still pending.
The integration fixture now rejects nonzero child and launch exits as well as
SIGSEGV and forced termination.

## Shutdown during missing-TF startup

`0011-startup-shutdown-order.patch` prevents costmap activation from continuing
into publishers and the map-update thread when context shutdown ends its TF
wait. It also defers the lifecycle manager's service-thread join and state
release to destruction: joining from pre-shutdown prevents context shutdown
from interrupting an outstanding activation request. The existing destructor
joins the thread before its maps and timers are destroyed; callback ownership
from patch 0010 remains in force.

The production initial-transform wait is unchanged, preserving recovery after
SBC outages. The network-isolated `integration_startup_shutdown.py` requires
clean exit during missing-TF activation both directly and with
`PATROLBOT_STARTUP_MANAGER=1`. Baseline direct activation aborts; the manager
case requires forced termination. Both cases pass in the combined diagnostic
image. Full canonical-image and live acceptance are still required; the
prototype image inherits old provenance and must not be deployed.

## Map-localization transaction patches

Apply the full numbered series to Nav2 1.3.12 in order:

- `0013-map-consumption.patch`: read-only AMCL and static-layer acknowledgements
  identify the consumed map-load event after processing, not just receipt.
- `0014-navigation-idle.patch`: read-only navigator ownership acknowledgement;
  absence of action status messages is not evidence of an idle navigator.
- `0015-navigation-configure-first.patch`: optional configure-only autostart for
  the navigation manager. This avoids activation waiting for a pose while the
  localization transaction waits for the manager to pause. Pause from the
  configured state verifies every managed node is inactive. Existing defaults
  for other lifecycle managers remain unchanged.

The existing bridge recovery coordinator owns the transaction and motion hold.
These services neither authorize motion nor substitute for fresh localization.
`docker/tests/integration_floor_maps.py` exercises the patches with real Nav2,
canonical maps, synthetic scans and a loopback fake SBC in a network-none
container. Hardware acceptance remains separate under feature #66/#65.


## Applied-seed diagnostics

`0016-amcl-applied-seed-diagnostics.patch` logs one JSON record after each
successful `pf_init`, before sensor weighting. It records the original request
stamp, application time, actual TF-adjusted mean and 3-by-3 covariance passed to
`pf_init`, initialized sample mean/count, and the number inside 15 cm and 4 degrees
of that applied mean. It changes no particles or gates. The existing accepted
seed topic still echoes the original request, and the post-seed scan barrier
still uses the original request timestamp.

For the measured seed, the isolated scan regression sends covariance x/y
`0.0009` and yaw `0.00030461742`, requires sample mean within 1 cm / 0.3 degrees,
and at least 99% of samples inside 15 cm / 4 degrees, then requires old/equal
scans to be rejected and a fresh stationary scan to produce the first estimate.
The seed ACK regression additionally covers wrong frames, stale TF, bounded
retry, supersession and lifecycle reset without an extra seed refresh.

Offline check against the pinned Nav2 1.3.12 source with all patches applied:

```bash
python3 docker/tests/test_amcl_seed_diagnostics.py /path/to/navigation2-1.3.12
```

This compiles the actual upstream particle filter and added diagnostic block;
it does not establish ROS integration or physical wall alignment. Run both
`integration_amcl_seed_ack.py` and `integration_amcl_seed_scan_barrier.py` in the
exact candidate image with `--network none` and `PATROLBOT_ISOLATED_TEST=1` before
deployment. A live supervised measured seed and independent scan/map wall
alignment remain required; do not widen readiness gates or repeat seeds to
obtain acceptance.
