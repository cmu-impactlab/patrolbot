# PatrolBot Docker deployment

The Compose stack runs the three active Raspberry Pi services:

- `bringup`: `twist_mux` and the final velocity smoother
- `bridge`: TCP bridge to the SBC at `10.0.0.1:7272`
- `navigation`: Nav2, joystick teleop, alert-only controller feedback, safety
  watchdog, and static laser TF

The legacy `rosaria2` package is versioned in the monorepo but is not built or
run. Never run it alongside `patrolbot_bridge`.

## Local ROS tests (not a deployment image)

From the repository root, build the test environment once:

```bash
docker build -f docker/Dockerfile.ros-tests -t patrolbot-ros-tests .
docker run --rm --network=none patrolbot-ros-tests
```

The final image builds generated interfaces and runs bridge, dock-manager and
navigation functional suites in separate pytest processes from their source
package directories. Fake-SBC, dock-manager, manual-undock and installed Jazzy
Collision Monitor integration tests are required. The runner enables both
integration opt-ins, uses ROS domain 101 (valid on Linux) and localhost discovery.
Domain isolation is logical; hosted CI also requires `--network=none` and private
IPC. It never connects to the robot. Physical acceptance and full Nav2-stack
acceptance remain separate.

The required `native-test-image` job builds this strict rosdep image once, then
runs it non-root with a read-only root filesystem, private network/IPC, dropped
capabilities and no-new-privileges. Only its dedicated evidence directory is
bind-mounted writable, owned by UID/GID 65534; `/tmp` is tmpfs. It mounts no source,
credentials, devices, sockets or production volumes. The checkout's baked source
fixtures remain read-only. Job ID and branch-protection contexts are unchanged.

There are **no allowed skips**. Before collection, the runner explicitly excludes
`test_copyright.py`, `test_flake8.py` and `test_pep257.py` in each package. These
ament checks are outside this functional gate; their exclusion claims no lint or
copyright pass. All other collected tests must pass setup, call and teardown.
Required integration nodeids are checked without pinning a total test count.
Collection errors, empty suites, deselected tests, skipped/xfail outcomes, thread/unraisable
exceptions, missing JUnit/evidence, native pytest crashes, leaked child process
groups and deadlines fail the
gate. Failed suites do not prevent the remaining suites from running. There are
no retries or blanket warning filters.
Inherited `PYTEST_ADDOPTS`, `PYTEST_PLUGINS` and
`PYTEST_DISABLE_PLUGIN_AUTOLOAD` are cleared. Strict pytest configuration makes
unknown options (including timeout settings without their plugin) fatal.

Evidence defaults to `/tmp/native-test-evidence` (override with
`PATROLBOT_TEST_EVIDENCE`). Each suite keeps `pytest.log`, `junit.xml`, an
incremental `events.jsonl` (collection nodeids, deselection, skip/failure reasons,
all test phases and session finish), `summary.json` (command, exclusions, deadline,
exit and process-group cleanup), and a `ros/` log directory. An abnormal exit can
leave JUnit/session finish missing; that is itself a failure, with partial logs
retained. The root summary aggregates every suite. Hosted CI uploads build,
container and suite evidence with `if: always()` even on failure.

Portable gate regressions (no ROS or Docker):

```bash
python -B -m pytest -q docker/tests/test_native_ci_gate.py \
  docker/tests/test_native_test_image_workflow.py docker/tests/test_ros_test_image.py \
  docker/tests/test_main_protection.py
```

For rapid iteration, reuse one container with a read-only source mount and
incremental build output inside the container (no image rebuild per edit):

```bash
docker run -d --name patrolbot-ros-dev --network=none \
  --mount "type=bind,src=$PWD,dst=/source,readonly" \
  --entrypoint sleep patrolbot-ros-tests infinity
docker exec patrolbot-ros-dev bash -c '
  set -e
  source /opt/ros/jazzy/setup.bash
  cd /opt/patrolbot
  colcon build --base-paths /source/ros2_ws/src/patrolbot_interfaces \
    /source/ros2_ws/src/patrolbot_bridge /source/ros2_ws/src/patrolbot_dock_manager \
    /source/ros2_ws/src/patrolbot_navigation \
    --build-base /opt/dev-build --install-base /opt/dev-install \
    --cmake-args -DBUILD_TESTING=OFF
  PATROLBOT_TEST_WORKSPACE=/source PATROLBOT_TEST_INSTALL=/opt/dev-install \
    bash /source/docker/run-ros-tests.sh
'
```

Repeat only `docker exec` after edits. Use normal installation, not
`--symlink-install`: the map catalog deliberately rejects files resolving
outside its catalog directory. The `ros-test-env` Dockerfile target contains
only the reusable toolchain/dependencies; the final `ros-tests` target is
used by the hosted functional gate. Both return nonzero on failure.
Each pytest invocation is capped at 300 seconds (override with
`PATROLBOT_TEST_TIMEOUT`, maximum 600). Any remaining suite process group receives
SIGTERM with a five-second retirement bound, then SIGKILL with a one-second bound
if still present. Evidence records only signals actually sent and whether the
group retired; leaks, forced kills and incomplete retirement fail the gate.
Optional test paths select a diagnostic run, without
the default three-suite integration sentinel requirement. Hosted CI passes no
arguments and always runs the required selection.

## Configure and build

The standard deployment includes all commissioned web-bridge integrations,
including the software-reset supervisor socket. They must not be hidden in
optional feature overlays that disappear on recreation. Install/verify the host
supervisor as described in `system-reset/README.md` and set `PATROLBOT_RESET_GID`
to the actual host group before validating Compose. Missing prerequisites fail
configuration/startup instead of silently dropping the integration. This does
not enable commands, execute a reset, grant Docker/SSH access to the web bridge,
or bypass feature-specific authorization/commissioning gates. Tuning runtime
activation remains separately gated.

```bash
cd docker
cp .env.example .env
# Set PATROLBOT_IMAGE to a unique tag and PATROLBOT_REVISION to the Git SHA.
# Set PATROLBOT_RESET_GID to the host patrolbot-reset group's numeric ID.
docker compose config
docker compose build navigation
```

If running Compose from the repository root instead of `docker/`, pass the env
file explicitly:

```bash
docker compose --env-file docker/.env -f docker/docker-compose.yml config
```

Only the `navigation` service declares the build because all three services
share the same immutable image tag. The build context is the repository root.
Source is also bind-mounted read-only from `PATROLBOT_ROOT`, so edits to
existing launch, parameter, map, or script files apply after the relevant
container is restarted. Adding files requires an image rebuild.

## Run and inspect

```bash
docker compose up -d
docker compose ps
./patrolbot-status
```

Docker health checks verify expected processes without continuously creating ROS
DDS participants. `patrolbot-status` performs the heavier ROS and hardware
readiness checks only when requested. It uses fresh `/odom` as the end-to-end
SBC data-path check instead of opening a probe connection to the single-client
hardware server. It also requires exactly one `/odom` publisher, exactly one
`/scan` publisher, and BEST_EFFORT reliability on the `/scan` and `/sonar`
publishers, preventing an accidentally re-enabled rollback stack or stale QoS
deployment from passing readiness:

The bridge alone sets `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`. Fast DDS
then sends DDS traffic from its background writer rather than blocking the
bridge's sensor publisher workers on a stalled reader. The status command
treats a missing/different bridge publication mode or an unattended
`ros2 topic hz` process as degraded.

- exit `0`, `OVERALL=ready`: containers, SBC, telemetry, lifecycle nodes, and TF
  are ready;
- exit `2`, `OVERALL=degraded`: containers are healthy but hardware or ROS
  readiness is incomplete;
- exit `1`, `OVERALL=unhealthy`: one or more containers failed liveness.

An unavailable SBC data path therefore produces a safe `degraded` result rather
than an unhealthy restart loop.

Compose shares the host IPC namespace for Fast DDS and stores each container's
ROS logs separately under `logs/{bringup,bridge,navigation}` in the repository.
Use the customizable viewer from the repository root to inspect one module or
all three together:

```bash
logs/patrolbot-logs.sh                         # all Docker logs, follow mode
logs/patrolbot-logs.sh bridge --tail 500      # bridge only
logs/patrolbot-logs.sh all --since 15m --no-follow
logs/patrolbot-logs.sh --source ros navigation
logs/patrolbot-logs.sh --health-only --readiness
```

Run `logs/patrolbot-logs.sh --help` for all filtering, health, timestamp, and
follow options.

On the Pi 5 deployment host, install an optional shell command:

```bash
sudo ln -sf /home/ubuntu/patrolbot-repo/docker/patrolbot-status /usr/local/bin/patrolbot-status
patrolbot-status
```

## Rollback

Keep the previous Compose directory and image tag until supervised hardware
acceptance succeeds:

```bash
docker compose down
cd ~/docker
docker compose up -d
```

Pi 5 repository snapshots created during deployment refreshes live under
`~/backups/`, one timestamped directory per snapshot. That is the **only**
backup location in the home directory — do not create sibling `patrolbot-backups/`
or `patrolbot-repo-rollbacks/` trees. They are rollback aids, not active runtime
state. Keep the two newest snapshots and prune the rest once supervised SBC-on
acceptance passes:

```bash
du -sh ~/backups/*
ls -1dt ~/backups/*/ | tail -n +3 | xargs -r rm -rf   # keep the 2 newest
```

Do not run Docker and the old bare-metal services simultaneously. Joystick and
Nav2 goal tests require a clear robot area and an operator at the deadman/E-stop.
