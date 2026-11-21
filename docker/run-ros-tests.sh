#!/bin/bash
# Separate native functional suites; optional arguments select diagnostic tests.
set -e
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "${PATROLBOT_TEST_INSTALL:-/opt/patrolbot/install}/setup.bash"
set -u
export PATROLBOT_RUN_INTEGRATION=1
export PATROLBOT_RUN_MANUAL_UNDOCK_NATIVE=1
export ROS_DOMAIN_ID=101
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
runner_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec python3 -B "$runner_dir/native_ci_gate.py" "$@"
