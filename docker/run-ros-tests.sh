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
exec python3 -B -m pytest -q -p no:cacheprovider \
    ros2_ws/src/patrolbot_bridge/test \
    ros2_ws/src/patrolbot_dock_manager/test
