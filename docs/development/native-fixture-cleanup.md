# Native functional fixture cleanup

The bridge, manual-undock, dock-manager and Collision Monitor integration tests
share source-only support in `ros2_ws/src/patrolbot_bridge/test/native_fixture.py`.
It lives alongside bridge tests so existing native image package copies include
it; it is not installed in the production Python package. Every
successfully constructed resource is registered before the next setup step, so
readiness and test failures use the same cleanup path. Background thread errors
fail cleanup; cleanup errors are attached to an existing setup/test exception.

Cleanup stops dispatch and joins the spin thread, cancels queued worker calls
and drains running callbacks, then shuts down the ROS executor before destroying
nodes and the context. Bridge stop signals are set and its manual/transport
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
Idle or suspended async tasks are not resumed during teardown. Unknown storage
or pool shapes fail explicitly and require an adapter review when rclpy changes.
These private accesses are confined to fixture support; production rclpy is
unchanged. Pool draining runs in a daemon thread joined with the fixture timeout.

The extracted cached executor bypasses its work-tracker wait in `shutdown()`.
The parent's controlled diagnostic returned True in 0.000096 seconds while a
callback remained blocked; it did not reproduce InvalidHandle. The full baseline
also passed once without the old intermittent warning. The parent native ROS run
must validate this ordering against installed Jazzy; portable mocks alone do not
prove that the intermittent warning is fixed.

Each wait is bounded. A stuck dispatcher or callback fails teardown and retains
ROS handles/context rather than freeing handles still in use; external fake and
child-process cleanup is still attempted. A stuck Python thread cannot safely
be killed in process, so the enclosing CI process remains the final containment.
The owner relies on constructors to clean allocations if they raise internally;
it owns nodes once their constructors return.

Collision Monitor writes combined output to pytest's retained temporary
`collision-monitor*/collision_monitor.log` (also exposed as `node.log_path`).
An early child exit fails teardown. Expected termination codes are 0 and SIGTERM;
a forced kill reaps the process and fails the fixture. Existing opt-ins and
sensor freshness/expiry assertions are unchanged.

Offline regression: `python -m pytest docker/tests/test_native_fixture_cleanup.py`.
