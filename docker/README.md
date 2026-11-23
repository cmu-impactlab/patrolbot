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
exactly `test/test_copyright.py`, `test/test_flake8.py` and `test/test_pep257.py`
relative to each package. The runner supplies absolute `--ignore` paths and a
pytest 7-compatible `hookwrapper=True` collection hook matches those same absolute
paths before import. It yields to delegate hooks and calls `outcome.get_result()`
before forcing `True` only for those paths, preserving upstream errors even for
declared exclusions. Every other path retains the delegate result or exception;
matching basenames elsewhere are not excluded. Additional `--ignore` or
`--ignore-glob` options fail configuration. These
ament checks are outside this functional gate; their exclusion claims no lint or
copyright pass. All other collected tests must pass setup, call and teardown.
Required integration nodeids are checked without pinning a total test count.
For default full-suite runs, the parent enumerates `package/test` recursively
before starting pytest, using `test_*.py` and `*_test.py` and subtracting only
the three exact lint paths above. Successful file collectors (including empty
modules) and collected item paths must match that inventory. Missing or unexpected
files fail the gate, including files hidden by plugin/conftest hooks, ignored
parent directories, `collect_ignore`, or narrowed `python_files` configuration.
An empty module is allowed when successfully collected; the suite must still
contain tests and satisfy its sentinels. Explicit custom diagnostic paths disable
this full-suite inventory comparison and are marked `enforced: false`; they do
not claim full-suite coverage and may include sources outside the package.
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
incremental `events.jsonl` (pre-collection pytest/Python versions, cwd/rootdir,
config file, invocation, installed plugins and ignore-hook ordering/wrapper flags,
declared/observed absolute exclusions, collection files/nodeids, deselection, skip/failure reasons,
all test phases and session finish), `source-inventory.json` (the parent's
pre-child expected absolute source paths and enforcement flag),
`summary.json` (expected/collected/missing/unexpected source paths, inventory
enforcement and validation issues, command, exclusions, deadline,
exit and process-group cleanup), and a `ros/` log directory. An abnormal exit can
leave JUnit/session finish missing; that is itself a failure, with partial logs
retained. The root summary aggregates every suite. Hosted CI uploads build,
container and suite evidence with `if: always()` even on failure.
Evidence validation rejects collection of a declared excluded file, missing or
invalid inventory evidence, or a full-suite file inventory mismatch independently
of rootdir-relative nodeids.

Both failed PR 85 hosted runs are retained. The second run's pytest 7.4.4 metadata
records launch-testing 3.4.13 / launch-testing-ros 0.26.13 and a later-registered
`launch_testing` hook with `tryfirst=True`, competing with the gate's old
`tryfirst` hook. A `False` first result bypassed the gate despite matching source
paths and absolute exclusions. The passing cached image had launch-testing
3.4.11 without that competing hook. The wrapper runs around nonwrapper hooks
regardless of their priority or registration order. Portable regressions exercise
different rootdirs and an installed, autoloaded, later-registered `tryfirst`
plugin returning `False`, with import-time failures in all three excluded files,
functional files still collected, and delegate results/errors preserved.
Fresh hosted verification remains the parent's responsibility; no lint pass is claimed.

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

## Local pre-push verification

Run from a quiet checkout before every push (no Git hook is installed):

```bash
python3 -B docker/local-pre-push.py \
  --results /absolute/path/outside-checkout/pre-push-001 \
  --python /absolute/path/to/test-venv/bin/python \
  --image "$PATROLBOT_NATIVE_IMAGE"
```

Default `--gate both` runs the **full** existing portable runner and native
functional gate, continuing to native after a portable failure. `--gate portable`
or `--gate native` explicitly selects one; this is partial verification. The
portable interpreter needs `contracts/requirements-test.txt`, plus the host C++
toolchain/Make. `--python` defaults to the calling interpreter; `--image` defaults
to `PATROLBOT_NATIVE_IMAGE`. Missing tools/images/dependencies fail; nothing is
installed, pulled, retried, or skipped by this wrapper. No live cutover is included.

The snapshot contains current working-tree bytes of tracked and nonignored
untracked files, including unstaged edits and deletions, rather than just HEAD or
the index. Ignored paths are excluded. Unexpected credential paths, symlinks,
and special files fail closed. Keep credentials in ignored files: arbitrary
secrets inside source files cannot be detected.
Do not edit during capture; rerun after any source change. The normalized
`source.tar`, SHA-256 manifest and HEAD identify exactly what was tested. The
results directory must be new and outside the checkout; it is created with
owner-only access because evidence includes source and logs.

Portable compilation runs in a disposable source copy, with `MAKEFLAGS=-j2` and
a 30-minute deadline. Its combined log, exit status and hardware test artifacts
are retained. Native runs against the captured source mounted read-only, copies
it into disposable `/work`, and rebuilds interfaces, bridge, dock manager and
navigation before invoking `native_ci_gate.py` without diagnostic selectors.
There is no writable checkout mount. The cached image is resolved to its immutable
ID without pulling. It uses the standard local Unix socket
(`/var/run/docker.sock`), not a configured remote Docker context. The container
uses no network, private IPC, localhost DDS, no devices,
no capabilities, a read-only root, 2 CPUs, 4 GiB RAM/no extra swap, 512 PIDs, and
a 30-minute outer deadline. `/tmp` remains noexec; only the disposable 2 GiB
`/work` tmpfs permits writable execution for compiler probes/generated binaries.
The named container is removed on completion/timeout/interruption. Hard host or
process termination may require manual removal of the name in `native/result.json`.

`summary.json` records actual aggregate/gate exits (nonzero on failure), snapshot
identity and errors. Each gate has `result.json` with its exact command, exit and
time, plus `output.log`; native also retains image inspection, `build.log`, colcon
logs, per-suite pytest/JUnit/events/source inventories/summaries and ROS logs under
`native/tests/`. Build failure prevents native tests and remains a failure. Native
suite deadlines and evidence checks remain those of the existing gate. Disposable
build/install directories are removed; retained evidence is never overwritten.

Offline results depend on the cached image's ROS/Nav2/pytest/dependency versions.
Candidate package sources/interfaces are rebuilt, but dependency freshness and
hosted CI's current image are **not** verified: an old cache can differ from CI
(e.g. launch-testing collection hooks). Refresh the dependency image separately
when network access is authorized; this runner never performs that refresh and
does not claim lint, robot, deployment or live-stack validation.
