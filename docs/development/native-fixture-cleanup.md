# Native functional fixture cleanup

The bridge, manual-undock, dock-manager and Collision Monitor integration tests
share source-only support in `ros2_ws/src/patrolbot_bridge/test/native_fixture.py`.
It lives alongside bridge tests so existing native image package copies include
it; it is not installed in the production Python package. Every
successfully constructed resource is registered before the next setup step, so
readiness and test failures use the same cleanup path. Background thread errors
fail cleanup; cleanup errors are attached to an existing setup/test exception.

Cleanup stops dispatch and joins the spin thread, cancels queued worker calls
and drains running callbacks, disposes inactive rclpy tasks, then shuts down the
ROS executor before destroying nodes and the context. Bridge stop signals are set and its manual/transport
workers are joined before its node destructor can release handles.
The TEST ONLY worker-pool drain deliberately uses rclpy's private
`MultiThreadedExecutor._executor`, requiring a standard-library
`ThreadPoolExecutor`: work submitted to that pool may not
yet be counted by the executor's callback work tracker. Destroying task guards
before that queue settles can leave a worker accessing an invalid handle.
The TEST ONLY task adapter supports the legacy `_tasks` list of
`(task, entity, node)` tuples and the cached Jazzy 7.1.11 layout:
`_pending_tasks` dict, `_ready_tasks` deque, and MultiThreadedExecutor's `_futures`
list. It snapshots tasks before and after draining, including completed handlers
retained only in `_futures`, and reports completed exceptions once per task.
After the pool drains, it cancels inactive tasks and explicitly closes their
coroutine handlers, including already-cancelled tasks. The cached Jazzy
`Task.cancel()` closes pending coroutine handlers, but cancelling a queued
Python pool future never calls it. Other versions may only change task state.
The adapter requires the known `_handler`, `_executing` and `_task_lock` layout,
holds the task lock without waiting, and refuses to cancel or close an active
task. Cancellation/finalization can enqueue done callbacks; newly discovered
tasks are disposed too, without executing their callbacks. Coroutine finalizers
run while guards and node handles are still valid; their exceptions fail cleanup.
Idle or suspended async tasks are not resumed during teardown. Unknown storage,
task internals or pool shapes fail explicitly and retain handles until the
adapter is reviewed for the new rclpy version.
These private accesses are confined to fixture support; production rclpy is
unchanged. Pool draining and task disposal run in a daemon thread joined with
the fixture timeout, so a stuck coroutine finalizer also retains ROS handles.

The extracted cached executor bypasses its work-tracker wait in `shutdown()`.
The parent's controlled diagnostic returned True in 0.000096 seconds while a
callback remained blocked; it did not reproduce InvalidHandle. A later candidate
run passed 785 bridge tests but emitted never-awaited handler warnings and was
not accepted as clean. The native gate now promotes never-awaited coroutine
RuntimeWarnings to errors and rejects them in recorded evidence even if a local
warning filter overrides that promotion. The parent native ROS run must validate
this disposal against installed Jazzy; portable mocks alone do not prove that
the intermittent warning is fixed.

Each wait is bounded. A stuck dispatcher or callback fails teardown and retains
ROS handles/context rather than freeing handles still in use; external fake and
child-process cleanup is still attempted. A stuck Python thread cannot safely
be killed in process, so the enclosing CI process remains the final containment.
The owner relies on constructors to clean allocations if they raise internally;
it owns nodes once their constructors return.

Collision Monitor writes combined output to `ROS_LOG_DIR/collision_monitor.log`
when CI supplies the uploaded evidence directory, or pytest's retained temporary
`collision-monitor*/collision_monitor.log` otherwise (also exposed as `node.log_path`).
An early child exit fails teardown. Expected termination codes are 0 and SIGTERM;
a forced kill reaps the process and fails the fixture. Existing opt-ins and
sensor freshness/expiry assertions are unchanged.

Collision Monitor also records `collision_monitor_fixture.json` beside the child
log, including monotonic/ROS timestamps, DDS output source timestamps, all six
output velocity components, command identities and output barriers, lifecycle
transitions, graph counts, and synthetic sensor state/publication ticks. The
snapshot is written during external cleanup after executor draining, including
on setup/test failure; the existing child-process cleanup still runs if writing
evidence fails.

Before each scenario, the fixture waits for command, scan, sonar and output graph
connections and establishes actual clear-world nonzero output. Only this bounded
four-second precondition can stream clear commands. Each probe uses a distinct
velocity tag because production uses unstamped Twist; acknowledging that tag on
the ordered streams establishes motion independently of previous test order.
The fault is then applied, four sensor publication ticks settle its input, and
the scenario command is published exactly once. Its one-second output wait
rejects clear probes and DDS source timestamps at or before the acknowledged
probe. Missing output fails; STOP assertions require a received zero, and allowed
commands still require their actual nonzero velocity. No scenario command retry
or production parameter change is involved.

A timer keeps synthetic healthy sensors fresh. Stale-sonar scenarios disable
sonar under a separate producer lock before aging the source for 0.6 s. This
lock serializes complete sensor batches, world transitions and command
publication. `set_world(publish_sonar=False)` waits for an old batch's pending
or in-flight sonar publication to finish before returning; subsequent batches
continue scan only. No subsequent clear precondition runs until the scenario
has finished. The separate sonar-outage acceptance helper also disables
background sonar during its outage window; its strict 0.5–0.60 s onset and
30-second total limits remain unchanged.

DDS publication and graph queries never hold the observation lock. That lock
still protects paired output/source appends, evidence and source barriers. The
sensor timer has its own mutually exclusive callback group, separate from the
output subscription's default group, so a blocked sensor publish cannot exclude
output dispatch through rclpy's callback-group gate. The existing three-worker
executor and node lifetimes are unchanged. Lock order is producer then
observation; receive takes only observation.

The policy-only `c720374` native evidence showed correct rear-forward and final
clear-arc outputs sourced about 0.000140 s and 0.000142 s after their commands,
but received 1.275731 s and 1.544796 s later; graph counts were all one. Sensor
callbacks also stalled. This demonstrates observation delay rather than wrong
Collision Monitor output. The old shared lock and default callback group both
provided blocking paths; the evidence does not identify which path caused those
specific delays or exclude OS/DDS/executor scheduling. Portable event-controlled
regressions block scan, sonar, command publication and graph lookup and require
receipt before releasing the producer. They also require world transitions to
drain pending sonar and keep paired arrays atomic. They prove those fixture
blocking paths are removed, not that native scheduling delays are resolved.
No timeout, freshness guard, scenario retry or velocity assertion is relaxed.

Hosted PR85 evidence at `ab99323` showed all three STOP cases timing out waiting
for output, with the first polygon STOP logged about 0.13 s after activation.
Missing output discovery combined with Nav2 1.3.14's suppression of repeated
zeros after `stop_pub_timeout` is a plausible explanation, not a demonstrated
root cause. Portable simulations cover delayed output discovery/transport, a
single zero followed by suppression, scenario publication exactly once, stale
output rejection, preserved sonar staleness, and bounded readiness failures for
each graph connection. They do not replace the parent's native ROS validation
against the installed middleware and Nav2 version.

Offline regression: `python -m pytest docker/tests/test_native_fixture_cleanup.py`.
