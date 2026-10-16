#!/bin/bash
# Run the Pi-side ROS pytest suites inside the test image.
# Usage: run-ros-tests.sh [extra pytest paths/args...]
# With no arguments bridge, dock-manager and offline navigation suites run.
# ROS setup scripts reference unset variables under `set -u`.
set -e
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${PATROLBOT_TEST_INSTALL:-/opt/patrolbot/install}/setup.bash"
set -u
workspace=${PATROLBOT_TEST_WORKSPACE:-/opt/patrolbot}
suite_timeout=${PATROLBOT_TEST_TIMEOUT:-300}
cd "$workspace"
if [ "$#" -gt 0 ]; then
    exec timeout --kill-after=15s "$suite_timeout" \
        python3 -B -m pytest -q -p no:cacheprovider "$@"
fi
run_suite() {
    timeout --kill-after=15s "$suite_timeout" \
        python3 -B -m pytest -q -p no:cacheprovider "$@"
}
# One invocation per suite, each from its package directory: both packages
# ship same-named helper modules (e.g. test_flake8.py) without packages,
# which collide in a single run — and the ament linter tests walk the
# working tree, which must be the package source (never build/ or install/
# copies). This matches colcon-test working directories.
status=0
cd "$workspace/ros2_ws/src/patrolbot_bridge"
run_suite test || status=1
cd "$workspace/ros2_ws/src/patrolbot_dock_manager"
run_suite test || status=1
# Navigation policy suites. test_collision_monitor_integration needs a live
# Nav2 stack and stays out of this image; the rest are offline policy checks.
cd "$workspace/ros2_ws/src/patrolbot_navigation"
run_suite test \
    --ignore=test/test_collision_monitor_integration.py \
    || status=1
exit $status
