#!/bin/bash
# Run the Pi-side ROS pytest suites inside the test image.
# Usage: run-ros-tests.sh [extra pytest paths/args...]
# With no arguments the full bridge + dock-manager suites run.
# ROS setup scripts reference unset variables under `set -u`.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /opt/patrolbot/install/setup.bash
set -u
cd /opt/patrolbot
if [ "$#" -gt 0 ]; then
    exec python3 -B -m pytest -q -p no:cacheprovider "$@"
fi
# One invocation per suite: both packages ship same-named helper modules
# (e.g. test_flake8.py) without packages, which collide in a single run.
status=0
python3 -B -m pytest -q -p no:cacheprovider \
    ros2_ws/src/patrolbot_bridge/test || status=1
python3 -B -m pytest -q -p no:cacheprovider \
    ros2_ws/src/patrolbot_dock_manager/test || status=1
# Navigation policy suites. test_collision_monitor_integration needs a live
# Nav2 stack and stays out of this image; the rest are offline policy checks.
python3 -B -m pytest -q -p no:cacheprovider \
    ros2_ws/src/patrolbot_navigation/test \
    --ignore=ros2_ws/src/patrolbot_navigation/test/test_collision_monitor_integration.py \
    || status=1
exit $status
